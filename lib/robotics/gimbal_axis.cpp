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
    if (limited && (angle < config_.min_angle_rad || angle > config_.max_angle_rad))
        return -ERANGE;
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
    if (action > SafetyAction::Active || command.mode > GimbalMode::AbsoluteAngle || !std::isfinite(dt) || dt < 0 ||
        !std::isfinite(command.rate_rad_s) || !std::isfinite(command.target_rad))
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
    target_status_ = {};
    target_status_.feedback_healthy = true;
    target_status_.last_dt_s = dt;
    target_status_.measured_rad = config_.topology == AxisTopology::Continuous
                                     ? snapshot.feedback.absolute_position_rad
                                     : snapshot.feedback.position_rad;
    const bool hold = action == SafetyAction::Hold || command.mode == GimbalMode::Hold;
    const bool was_hold = previous_action_ == SafetyAction::Hold || previous_mode_ == GimbalMode::Hold;
    // Do not integrate across a recovery interval. The current input starts on
    // the next valid execution interval after the feedback target is established.
    const float integration_dt = initialized_ && dt <= 0.02f ? dt : 0;
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
    if (initialized_ && !hold) {
        double delta = 0;
        if (command.mode == GimbalMode::Rate) {
            if (command.rate_rad_s == 0 && !config_.hold_on_zero_rate)
                (void)seed(snapshot);
            target_status_.requested_rate_rad_s = command.rate_rad_s;
            target_status_.limited_rate_rad_s =
                std::clamp(command.rate_rad_s, -config_.max_rate_rad_s, config_.max_rate_rad_s);
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
    }
    if (config_.topology == AxisTopology::Limited) {
        const auto candidate = target_angle_rad_;
        target_angle_rad_ = std::clamp(candidate, double(config_.min_angle_rad), double(config_.max_angle_rad));
        target_status_.mechanical_limited = target_angle_rad_ != candidate;
        if (!hold && command.mode == GimbalMode::Rate && config_.max_lead_rad > 0) {
            // The measured joint coordinate was validated above for freshness,
            // reference validity and mechanical range. Do not apply this to an
            // inertial AbsoluteAngle target or to a wrapped continuous axis.
            const auto before_lead = target_angle_rad_;
            target_angle_rad_ = std::clamp(before_lead,
                                           target_status_.measured_rad - double(config_.max_lead_rad),
                                           target_status_.measured_rad + double(config_.max_lead_rad));
            target_status_.lead_limited = target_angle_rad_ != before_lead;
            target_angle_rad_ = std::clamp(target_angle_rad_, double(config_.min_angle_rad),
                                           double(config_.max_angle_rad));
        }
    }
    else
        target_angle_rad_ = std::remainder(target_angle_rad_, 6.283185307179586);
    target_status_.target_rad = target_angle_rad_;
    target_status_.target_valid = initialized_;
    target_status_.integration_dt_s = integration_dt;
    previous_action_ = action;
    previous_mode_ = command.mode;
    return position_.update(target_angle_rad_, initialized_ ? dt : 0);
}
int GimbalAxis::updateRate(float rate_rad_s, float dt_s) {
    return update({GimbalMode::Rate, 0, rate_rad_s}, SafetyAction::Active, dt_s);
}
} // namespace skywalker::robotics
