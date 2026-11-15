#include <algorithm>
#include <cmath>
#include <cerrno>
#include <control/angle.h>
#include <robotics/gimbal/gimbal_axis.hpp>

namespace skywalker::robotics {

int GimbalAxis::validate() const {
    if (config_.topology > AxisTopology::Limited || !std::isfinite(config_.max_rate_rad_s) ||
        config_.max_rate_rad_s <= 0 || !std::isfinite(config_.min_angle_rad) || !std::isfinite(config_.max_angle_rad) ||
        config_.min_angle_rad >= config_.max_angle_rad || !std::isfinite(config_.max_lead_rad) ||
        config_.max_lead_rad < 0 || (config_.topology == AxisTopology::Continuous && config_.max_lead_rad != 0))
        return -EINVAL;
    if (config_.topology == AxisTopology::Continuous &&
        position_.reference() != control::PositionReference::AbsoluteNearest)
        return -ENOTSUP;
    if (config_.topology == AxisTopology::Limited &&
        position_.reference() != control::PositionReference::DriverContinuous)
        return -ENOTSUP;
    if (config_.reference_init > AxisReferenceInit::CalibratedFeedback ||
        (config_.topology == AxisTopology::Continuous && config_.reference_init != AxisReferenceInit::Preserve))
        return -EINVAL;
    return 0;
}

int GimbalAxis::begin() {
    if (configured_)
        return -EALREADY;
    int ret = validate();
    if (ret == 0)
        ret = position_.configure();
    configured_ = ret == 0;
    return ret;
}

int GimbalAxis::feedbackError(const motor::MotorSnapshot &snapshot) const {
    if (!snapshot.feedback_fresh)
        return -EAGAIN;
    const bool limited = config_.topology == AxisTopology::Limited;
    const std::uint32_t required = limited ? motor::FeedbackPosition : motor::FeedbackAbsolutePosition;
    if ((snapshot.feedback.valid & required) == 0 || (limited && !snapshot.position_reference_valid))
        return -ENODATA;
    const float angle = limited ? snapshot.feedback.position_rad : snapshot.feedback.absolute_position_rad;
    if (!std::isfinite(angle))
        return -EINVAL;
    // A valid measured angle outside the software range is still a usable
    // feedback. The target clamp brings it back without withdrawing the axis.
    return 0;
}

int GimbalAxis::prepareReference(const motor::MotorSnapshot &snapshot) {
    if (!snapshot.feedback_fresh)
        return -EAGAIN;
    double known;
    if ((snapshot.feedback.valid & motor::FeedbackAbsolutePosition) != 0 &&
        std::isfinite(snapshot.feedback.absolute_position_rad))
        known = snapshot.feedback.absolute_position_rad;
    else if (snapshot.native_position_valid && std::isfinite(snapshot.native_position_rad))
        known = snapshot.native_position_rad;
    else
        return -ENODATA;
    const int ret = drive_.reseedPosition(known);
    if (ret == 0)
        reference_seeded_ = true;
    return ret;
}

GimbalAxis::Status GimbalAxis::poll(std::uint64_t) {
    Status next{};
    if (!configured_)
        next.error = -EACCES;
    else {
        auto view = drive_.snapshot();
        if (config_.topology == AxisTopology::Limited &&
            config_.reference_init == AxisReferenceInit::CalibratedFeedback &&
            (!reference_seeded_ || !view.position_reference_valid) && view.feedback_fresh) {
            next.error = prepareReference(view);
            view = drive_.snapshot();
        }
        const int ret = feedbackError(view);
        next.feedback_healthy = !ret;
        if (!next.error)
            next.error = ret;
    }
    status_ = next;
    return next;
}
int GimbalAxis::seed(const motor::MotorSnapshot &snapshot) {
    const int ret = feedbackError(snapshot);
    if (ret < 0)
        return ret;
    target_angle_rad_ = config_.topology == AxisTopology::Continuous ? double(snapshot.feedback.absolute_position_rad)
                                                                     : double(snapshot.feedback.position_rad);
    if (config_.topology == AxisTopology::Limited)
        target_angle_rad_ = std::clamp(target_angle_rad_, double(config_.min_angle_rad), double(config_.max_angle_rad));
    observed_enable_generation_ = snapshot.enable_generation;
    observed_reference_generation_ = snapshot.reference_generation;
    initialized_ = true;
    return 0;
}
void GimbalAxis::withdraw() {
    initialized_ = false;
    target_angle_rad_ = 0;
    target_status_ = {};
    observed_enable_generation_ = observed_reference_generation_ = 0;
    previous_action_ = SafetyAction::Disable;
    previous_mode_ = GimbalMode::Disabled;
    if (configured_)
        (void)position_.reset();
}
int GimbalAxis::reset() {
    if (!configured_)
        return -EACCES;
    withdraw();
    // Reset the controller and acquire a Hold target only from the current reference.
    return seed(drive_.snapshot());
}
int GimbalAxis::update(const AxisCommand &command, SafetyAction action, float dt) {
    if (!configured_)
        return -EACCES;
    if (action > SafetyAction::Active || command.mode > GimbalMode::AbsoluteAngle || !std::isfinite(dt) || dt < 0)
        return -EINVAL;
    if (command.mode == GimbalMode::Rate && !std::isfinite(command.rate_rad_s))
        return -EINVAL;
    if (command.mode == GimbalMode::AbsoluteAngle && !std::isfinite(command.target_rad))
        return -EINVAL;
    if (action == SafetyAction::Disable || command.mode == GimbalMode::Disabled) {
        withdraw();
        return position_.update(target_angle_rad_, 0);
    }
    (void)poll(drive_.snapshot().feedback.timestamp_ms);
    const auto snapshot = drive_.snapshot();
    const int feedback_error = feedbackError(snapshot);
    if (feedback_error < 0 || snapshot.state != motor::MotorState::Active || !snapshot.output_permitted) {
        // Feedback loss/drive recovery must never accumulate a future Rate target.
        withdraw();
        target_status_.feedback_healthy = feedback_error == 0;
        target_status_.last_dt_s = dt;
        if (!feedback_error)
            target_status_.measured_rad = config_.topology == AxisTopology::Continuous
                                              ? snapshot.feedback.absolute_position_rad
                                              : snapshot.feedback.position_rad;
        const int ret = position_.update(target_angle_rad_, 0);
        return feedback_error < 0 ? feedback_error : ret;
    }
    if (initialized_ && (snapshot.enable_generation != observed_enable_generation_ ||
                         snapshot.reference_generation != observed_reference_generation_))
        withdraw();
    const bool hold = action == SafetyAction::Hold || command.mode == GimbalMode::Hold;
    const bool was_hold = previous_action_ == SafetyAction::Hold || previous_mode_ == GimbalMode::Hold;
    const bool rate_path = !hold && command.mode == GimbalMode::Rate;
    // A feedforward request edge means a new owner of the joint reference. The
    // explicit restart covers leaving/entering the vision path; the pending flag
    // covers a motion cycle whose output never reached the drive, so its target
    // debt is not replayed on the next executed cycle.
    const bool request_edge = command.use_rate_reference && !reference_request_previous_;
    reference_request_previous_ = command.use_rate_reference;
    AxisRestartReason restart_reason = AxisRestartReason::None;
    if (rate_path) {
        if (command.restart_rate_session)
            restart_reason = AxisRestartReason::CommandSession;
        else if (request_edge)
            restart_reason = AxisRestartReason::RequestEdge;
        else if (rate_restart_pending_)
            restart_reason = explicit_reseed_ ? AxisRestartReason::ExplicitReseed : AxisRestartReason::UnconsumedCycle;
    }
    const bool was_initialized = initialized_;
    if (restart_reason != AxisRestartReason::None) {
        // Drop the old target and control history. The seeding below rebuilds the
        // target from this cycle's feedback, so the previous session cannot be replayed.
        withdraw();
        explicit_reseed_ = false;
    }
    target_status_ = {};
    target_status_.feedback_healthy = true;
    target_status_.last_dt_s = dt;
    target_status_.measured_rad = config_.topology == AxisTopology::Continuous ? snapshot.feedback.absolute_position_rad
                                                                               : snapshot.feedback.position_rad;
    // Do not integrate across a recovery interval. The current input starts on
    // the next valid execution interval after the feedback target is established.
    const float integration_dt = was_initialized && restart_reason == AxisRestartReason::None && dt <= 0.02f ? dt : 0;
    if (!initialized_) {
        if (command.mode == GimbalMode::AbsoluteAngle && !hold) {
            target_angle_rad_ = command.target_rad;
            observed_enable_generation_ = snapshot.enable_generation;
            observed_reference_generation_ = snapshot.reference_generation;
            initialized_ = true;
        }
        else
            (void)seed(snapshot);
    }
    if (hold && !was_hold)
        (void)seed(snapshot);
    const double q_before = target_angle_rad_; // 本周期积分起点（必要时已重新播种）
    double executed_delta = 0;                 // 实际写入目标的增量，连续轴取 wrap 前值
    if (initialized_ && !hold) {
        double delta = 0;
        if (command.mode == GimbalMode::Rate) {
            if (command.rate_rad_s == 0 && !config_.hold_on_zero_rate)
                (void)seed(snapshot);
            target_status_.requested_rate_rad_s = command.rate_rad_s;
            target_status_.limited_rate_rad_s = std::clamp(command.rate_rad_s, -config_.max_rate_rad_s,
                                                           config_.max_rate_rad_s);
            target_status_.rate_limited = target_status_.limited_rate_rad_s != command.rate_rad_s;
            delta = target_status_.limited_rate_rad_s * integration_dt;
        }
        else if (command.mode == GimbalMode::AbsoluteAngle) {
            if (config_.topology == AxisTopology::Continuous) {
                float error = 0;
                const int ret = control_shortest_angle_error(command.target_rad, float(target_angle_rad_), &error);
                if (ret < 0)
                    return ret;
                delta = error;
            }
            else
                delta = double(std::clamp(command.target_rad, config_.min_angle_rad, config_.max_angle_rad)) -
                        target_angle_rad_;
            const double step = config_.max_rate_rad_s * integration_dt;
            delta = std::clamp(delta, -step, step);
        }
        target_angle_rad_ += delta;
        executed_delta = delta;
    }
    if (config_.topology == AxisTopology::Limited) {
        const auto candidate = target_angle_rad_;
        target_angle_rad_ = std::clamp(candidate, double(config_.min_angle_rad), double(config_.max_angle_rad));
        target_status_.mechanical_limited = target_angle_rad_ != candidate;
        if (!hold && command.mode == GimbalMode::Rate && config_.max_lead_rad > 0) {
            // Do not apply the lead guard to an inertial AbsoluteAngle target
            // or to a wrapped continuous axis. The measured value may be outside
            // the software range; the final target clamp below still applies.
            const auto before_lead = target_angle_rad_;
            target_angle_rad_ = std::clamp(before_lead, target_status_.measured_rad - double(config_.max_lead_rad),
                                           target_status_.measured_rad + double(config_.max_lead_rad));
            target_status_.lead_limited = target_angle_rad_ != before_lead;
            target_angle_rad_ = std::clamp(target_angle_rad_, double(config_.min_angle_rad),
                                           double(config_.max_angle_rad));
        }
    }
    else
        target_angle_rad_ = std::remainder(target_angle_rad_, 6.283185307179586);
    // 有限轴没有 wrap，重算后的目标差才是本周期真正写入的增量（含限位投影）。
    if (config_.topology == AxisTopology::Limited)
        executed_delta = target_angle_rad_ - q_before;
    target_status_.target_rad = target_angle_rad_;
    target_status_.target_valid = initialized_;
    target_status_.integration_dt_s = integration_dt;
    target_status_.restart_reason = restart_reason;
    target_status_.reference_requested = command.use_rate_reference && rate_path;
    const bool projected = target_status_.mechanical_limited || target_status_.lead_limited;
    // 参考速度必须和本次真正施加的目标增量同源：恢复周期、未积分周期、被限位
    // 投影或超出轴限速时一律不传前馈，而不是用 clamp 掩盖错误的差分。
    bool reference_accepted = target_status_.reference_requested &&
                              restart_reason == AxisRestartReason::None && integration_dt > 0.0f && !projected;
    float reference_velocity = 0.0f;
    if (reference_accepted) {
        const double candidate = executed_delta / static_cast<double>(integration_dt);
        if (std::isfinite(candidate) && std::fabs(candidate) <= static_cast<double>(config_.max_rate_rad_s) * 1.001)
            reference_velocity = static_cast<float>(candidate);
        else
            reference_accepted = false;
    }
    target_status_.reference_accepted = reference_accepted;
    target_status_.reference_velocity_rad_s = reference_velocity;
    previous_action_ = action;
    previous_mode_ = command.mode;
    const int ret = reference_accepted ? position_.update(target_angle_rad_, reference_velocity, 0.0f, dt)
                                       : position_.update(target_angle_rad_, initialized_ ? dt : 0);
    if (ret < 0 || !position_.telemetry().output_valid) {
        // 下层没有真正执行本周期（等待/失败）：丢弃本次新增目标，下一有效周期
        // 从测量重新播种，避免未执行的参考位移变成追赶债务或前馈。
        target_angle_rad_ = q_before;
        target_status_.target_rad = target_angle_rad_;
        target_status_.reference_accepted = false;
        target_status_.reference_velocity_rad_s = 0.0f;
        if (rate_path)
            rate_restart_pending_ = true;
    }
    else
        rate_restart_pending_ = false;
    return ret;
}
int GimbalAxis::updateRate(float rate_rad_s, float dt_s) {
    return update({GimbalMode::Rate, 0, rate_rad_s}, SafetyAction::Active, dt_s);
}
int GimbalAxis::updateTrajectory(const GimbalJointTrajectory &trajectory, SafetyAction action, float dt) {
    if (!configured_)
        return -EACCES;
    if (action > SafetyAction::Active || !std::isfinite(trajectory.position_rad) ||
        !std::isfinite(trajectory.rate_rad_s) || !std::isfinite(trajectory.acceleration_rad_s2) || !std::isfinite(dt) ||
        dt < 0)
        return -EINVAL;
    if (action == SafetyAction::Disable) {
        withdraw();
        return position_.update(target_angle_rad_, 0);
    }
    (void)poll(drive_.snapshot().feedback.timestamp_ms);
    const auto snapshot = drive_.snapshot();
    const int feedback_error = feedbackError(snapshot);
    if (feedback_error < 0 || snapshot.state != motor::MotorState::Active || !snapshot.output_permitted) {
        withdraw();
        target_status_ = {};
        target_status_.feedback_healthy = feedback_error == 0;
        target_status_.last_dt_s = dt;
        if (!feedback_error)
            target_status_.measured_rad = config_.topology == AxisTopology::Continuous
                                              ? snapshot.feedback.absolute_position_rad
                                              : snapshot.feedback.position_rad;
        const int ret = position_.update(target_angle_rad_, 0);
        return feedback_error < 0 ? feedback_error : ret;
    }
    if (initialized_ && (snapshot.enable_generation != observed_enable_generation_ ||
                         snapshot.reference_generation != observed_reference_generation_))
        withdraw();
    target_status_ = {};
    target_status_.feedback_healthy = true;
    target_status_.last_dt_s = dt;
    target_status_.measured_rad = config_.topology == AxisTopology::Continuous ? snapshot.feedback.absolute_position_rad
                                                                               : snapshot.feedback.position_rad;
    const float integration_dt = initialized_ && dt <= 0.02f ? dt : 0;
    double target = config_.topology == AxisTopology::Continuous
                        ? std::remainder(double(trajectory.position_rad), 6.283185307179586)
                        : std::clamp(double(trajectory.position_rad), double(config_.min_angle_rad),
                                     double(config_.max_angle_rad));
    if (config_.topology == AxisTopology::Limited) {
        const auto clamped = target;
        target_status_.mechanical_limited = clamped != double(trajectory.position_rad);
        if (config_.max_lead_rad > 0) {
            const auto before_lead = target;
            target = std::clamp(target, target_status_.measured_rad - double(config_.max_lead_rad),
                                target_status_.measured_rad + double(config_.max_lead_rad));
            target_status_.lead_limited = target != before_lead;
            target = std::clamp(target, double(config_.min_angle_rad), double(config_.max_angle_rad));
        }
    }
    target_angle_rad_ = target;
    // The feedforward must describe the same motion as the applied target. When
    // the soft limit or the lead guard projected it, or the rate exceeds the axis
    // capability, the inconsistent part is dropped instead of being handed to the
    // position loop.
    float reference_rate = std::clamp(trajectory.rate_rad_s, -config_.max_rate_rad_s, config_.max_rate_rad_s);
    float reference_acceleration = trajectory.acceleration_rad_s2;
    const bool projected = target_status_.mechanical_limited || target_status_.lead_limited;
    if (projected) {
        reference_rate = 0;
        reference_acceleration = 0;
    }
    target_status_.target_rad = target_angle_rad_;
    target_status_.target_valid = true;
    target_status_.requested_rate_rad_s = reference_rate;
    target_status_.limited_rate_rad_s = reference_rate;
    target_status_.rate_limited = reference_rate != trajectory.rate_rad_s;
    target_status_.integration_dt_s = integration_dt;
    if (!initialized_) {
        observed_enable_generation_ = snapshot.enable_generation;
        observed_reference_generation_ = snapshot.reference_generation;
        initialized_ = true;
    }
    previous_action_ = action;
    previous_mode_ = GimbalMode::AbsoluteAngle;
    // Position, rate and acceleration all come from the same feasible plan; the
    // position loop only has to supply the tracking correction.
    return position_.update(target_angle_rad_, reference_rate, reference_acceleration, dt);
}
int GimbalAxis::reseedTargetFromFeedback() {
    if (!configured_)
        return -EACCES;
    target_status_ = {};
    const int ret = seed(drive_.snapshot());
    if (ret == 0) {
        // 显式重播种后，下一个有效运动周期只重建会话：不把本周期的速率积分到
        // 新种子上，也不把播种前的目标债带进前馈。
        rate_restart_pending_ = true;
        explicit_reseed_ = true;
    }
    return ret;
}
} // namespace skywalker::robotics
