#include <algorithm>
#include <cmath>
#include <cerrno>
#include <control/angle.h>
#include <robotics/gimbal/gimbal_axis.hpp>

namespace skywalker::robotics {

int GimbalAxis::validate() const {
    if (config_.topology > AxisTopology::Limited || !std::isfinite(config_.max_rate_rad_s) ||
        config_.max_rate_rad_s <= 0 || !std::isfinite(config_.min_angle_rad) || !std::isfinite(config_.max_angle_rad) ||
        config_.min_angle_rad >= config_.max_angle_rad)
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

GimbalAxis::Status GimbalAxis::poll(std::uint64_t now_ms) {
    Status next{};
    next.ready_since_ms = status_.ready_since_ms;
    if (!configured_) {
        next.error = -EACCES;
    }
    else {
        auto snapshot = drive_.snapshot();
        const bool ready = drive_.ready();
        if (ready && config_.topology == AxisTopology::Limited &&
            config_.reference_init == AxisReferenceInit::CalibratedFeedback &&
            (!reference_seeded_ || !snapshot.position_reference_valid)) {
            next.error = prepareReference(snapshot);
            snapshot = drive_.snapshot();
        }
        const int feedback_error = feedbackError(snapshot);
        next.feedback_healthy = feedback_error == 0;
        if (next.error == 0)
            next.error = feedback_error;
        next.ready_for_enable = ready && next.error == 0 && drive_.ready();
    }
    if (next.ready_for_enable && !status_.ready_for_enable)
        next.ready_since_ms = now_ms;
    status_ = next;
    return next;
}

int GimbalAxis::seed(const motor::MotorSnapshot &snapshot) {
    const int ret = feedbackError(snapshot);
    if (ret < 0)
        return ret;
    target_angle_rad_ = config_.topology == AxisTopology::Continuous ? double(snapshot.feedback.absolute_position_rad)
                                                                     : double(snapshot.feedback.position_rad);
    previous_action_ = SafetyAction::Disable;
    previous_mode_ = GimbalMode::Disabled;
    generation_ = snapshot.enable_generation;
    initialized_ = true;
    return 0;
}

int GimbalAxis::reset() {
    if (!configured_)
        return -EACCES;
    const auto snapshot = drive_.snapshot();
    if (snapshot.state == motor::MotorState::Active || snapshot.state == motor::MotorState::Enabling)
        return -EBUSY;
    int ret = feedbackError(snapshot);
    if (ret < 0)
        return ret;
    ret = position_.reset();
    return ret < 0 ? ret : seed(drive_.snapshot());
}

int GimbalAxis::update(const AxisCommand &command, SafetyAction action, float dt) {
    if (!configured_ || action == SafetyAction::Disable || command.mode == GimbalMode::Disabled)
        return -EACCES;
    if (action > SafetyAction::Active || command.mode > GimbalMode::AbsoluteAngle || !std::isfinite(dt) || dt <= 0 ||
        dt > 0.02f || !std::isfinite(command.rate_rad_s) || !std::isfinite(command.target_rad))
        return -EINVAL;

    const auto snapshot = drive_.snapshot();
    if (snapshot.state != motor::MotorState::Active || !snapshot.output_permitted)
        return -EACCES;
    const int feedback_error = feedbackError(snapshot);
    if (feedback_error < 0)
        return feedback_error;
    if (!initialized_ || generation_ != snapshot.enable_generation) {
        const int ret = seed(snapshot);
        if (ret < 0)
            return ret;
    }

    const bool hold = action == SafetyAction::Hold || command.mode == GimbalMode::Hold;
    const bool was_hold = previous_action_ == SafetyAction::Hold || previous_mode_ == GimbalMode::Hold;
    if (hold && !was_hold) {
        const int ret = seed(snapshot);
        if (ret < 0)
            return ret;
    }
    if (!hold) {
        double delta = 0;
        if (command.mode == GimbalMode::Rate) {
            if (command.rate_rad_s == 0 && !config_.hold_on_zero_rate) {
                const int ret = seed(snapshot);
                if (ret < 0)
                    return ret;
            }
            delta = std::clamp(command.rate_rad_s, -config_.max_rate_rad_s, config_.max_rate_rad_s) * dt;
        }
        else if (command.mode == GimbalMode::AbsoluteAngle) {
            if (config_.topology == AxisTopology::Continuous) {
                float error = 0;
                const int ret = control_shortest_angle_error(command.target_rad, static_cast<float>(target_angle_rad_),
                                                             &error);
                if (ret < 0)
                    return ret;
                delta = error;
            }
            else
                delta = double(std::clamp(command.target_rad, config_.min_angle_rad, config_.max_angle_rad)) -
                        target_angle_rad_;
            const double step = config_.max_rate_rad_s * dt;
            delta = std::clamp(delta, -step, step);
        }
        target_angle_rad_ += delta;
    }
    if (config_.topology == AxisTopology::Limited) {
        target_angle_rad_ = std::clamp(target_angle_rad_, double(config_.min_angle_rad), double(config_.max_angle_rad));
    }
    else
        target_angle_rad_ = std::remainder(target_angle_rad_, 6.283185307179586);

    previous_action_ = action;
    previous_mode_ = command.mode;
    return position_.update(target_angle_rad_, dt);
}

int GimbalAxis::updateRate(float rate_rad_s, float dt_s) {
    return update({GimbalMode::Rate, 0, rate_rad_s}, SafetyAction::Active, dt_s);
}

} // namespace skywalker::robotics
