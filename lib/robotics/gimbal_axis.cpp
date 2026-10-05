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
    target_angle_rad_ = (config_.topology == AxisTopology::Continuous ? double(snapshot.feedback.absolute_position_rad)
                                                                      : double(snapshot.feedback.position_rad)) +
                        pending_rate_delta_rad_;
    pending_rate_delta_rad_ = 0;
    initialized_ = true;
    return 0;
}
int GimbalAxis::reset() {
    if (!configured_)
        return -EACCES;
    // Explicit business operation: acquire a Hold target. Driver recovery never calls it.
    return seed(drive_.snapshot());
}
int GimbalAxis::update(const AxisCommand &command, SafetyAction action, float dt) {
    if (!configured_)
        return -EACCES;
    if (action > SafetyAction::Active || command.mode > GimbalMode::AbsoluteAngle || !std::isfinite(dt) || dt < 0 ||
        !std::isfinite(command.rate_rad_s) || !std::isfinite(command.target_rad))
        return -EINVAL;
    if (action == SafetyAction::Disable || command.mode == GimbalMode::Disabled) {
        previous_action_ = action;
        previous_mode_ = command.mode;
        return position_.update(target_angle_rad_, 0);
    }
    (void)poll(drive_.snapshot().feedback.timestamp_ms);
    const auto snapshot = drive_.snapshot();
    const bool measured = feedbackError(snapshot) == 0;
    const bool hold = action == SafetyAction::Hold || command.mode == GimbalMode::Hold;
    const bool was_hold = previous_action_ == SafetyAction::Hold || previous_mode_ == GimbalMode::Hold;
    const float integration_dt = dt <= 0.02f ? dt : 0;
    if (!initialized_) {
        if (command.mode == GimbalMode::AbsoluteAngle && !hold) {
            target_angle_rad_ = command.target_rad;
            initialized_ = true;
        }
        else if (measured)
            (void)seed(snapshot);
        else if (command.mode == GimbalMode::Rate && !hold)
            pending_rate_delta_rad_ += std::clamp(command.rate_rad_s, -config_.max_rate_rad_s, config_.max_rate_rad_s) *
                                       integration_dt;
    }
    if (hold && !was_hold && measured)
        (void)seed(snapshot);
    if (initialized_ && !hold) {
        double delta = 0;
        if (command.mode == GimbalMode::Rate) {
            if (command.rate_rad_s == 0 && !config_.hold_on_zero_rate && measured)
                (void)seed(snapshot);
            delta = std::clamp(command.rate_rad_s, -config_.max_rate_rad_s, config_.max_rate_rad_s) * integration_dt;
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
    if (config_.topology == AxisTopology::Limited)
        target_angle_rad_ = std::clamp(target_angle_rad_, double(config_.min_angle_rad), double(config_.max_angle_rad));
    else
        target_angle_rad_ = std::remainder(target_angle_rad_, 6.283185307179586);
    previous_action_ = action;
    previous_mode_ = command.mode;
    return position_.update(target_angle_rad_, initialized_ ? dt : 0);
}
int GimbalAxis::updateRate(float rate_rad_s, float dt_s) {
    return update({GimbalMode::Rate, 0, rate_rad_s}, SafetyAction::Active, dt_s);
}
} // namespace skywalker::robotics
