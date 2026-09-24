#include <algorithm>
#include <cmath>
#include <cerrno>
#include <control/angle.h>
#include <robotics/gimbal/yaw_gimbal.hpp>

namespace skywalker::robotics {

int YawGimbal::validate() const {
    if (config_.topology > YawTopology::Limited || !std::isfinite(config_.max_rate_rad_s) ||
        config_.max_rate_rad_s <= 0 || !std::isfinite(config_.min_angle_rad) || !std::isfinite(config_.max_angle_rad) ||
        config_.min_angle_rad >= config_.max_angle_rad)
        return -EINVAL;
    if (config_.topology == YawTopology::Continuous && axis_.reference() != control::PositionReference::AbsoluteNearest)
        return -ENOTSUP;
    if (config_.topology == YawTopology::Limited && axis_.reference() != control::PositionReference::DriverContinuous)
        return -ENOTSUP;
    return 0;
}

int YawGimbal::begin() {
    const int ret = validate();
    return ret < 0 ? ret : axis_.configure();
}

int YawGimbal::seed(const motor::MotorSnapshot &snapshot) {
    if (!snapshot.feedback_fresh)
        return -EAGAIN;
    const std::uint32_t required = config_.topology == YawTopology::Continuous ? motor::FeedbackAbsolutePosition
                                                                               : motor::FeedbackPosition;
    if ((snapshot.feedback.valid & required) == 0 ||
        (config_.topology == YawTopology::Limited && !snapshot.position_reference_valid))
        return -ENODATA;
    target_angle_rad_ = config_.topology == YawTopology::Continuous ? double(snapshot.feedback.absolute_position_rad)
                                                                    : double(snapshot.feedback.position_rad);
    previous_action_ = SafetyAction::Disable;
    previous_mode_ = GimbalMode::Disabled;
    generation_ = snapshot.enable_generation;
    initialized_ = true;
    return 0;
}

int YawGimbal::reset() {
    const auto snapshot = drive_.snapshot();
    if (snapshot.state == motor::MotorState::Active || snapshot.state == motor::MotorState::Enabling)
        return -EBUSY;
    const int ret = axis_.reset();
    return ret < 0 ? ret : seed(drive_.snapshot());
}

int YawGimbal::update(const GimbalCommand &command, SafetyAction action, float dt) {
    if (action == SafetyAction::Disable || command.mode == GimbalMode::Disabled)
        return -EACCES;
    if (action > SafetyAction::Active || command.mode > GimbalMode::AbsoluteAngle || !std::isfinite(dt) || dt <= 0 ||
        dt > 0.02f || !std::isfinite(command.yaw_rate_rad_s) || !std::isfinite(command.yaw_target_rad))
        return -EINVAL;

    const auto snapshot = drive_.snapshot();
    if (snapshot.state != motor::MotorState::Active || !snapshot.output_permitted)
        return -EACCES;
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
            if (command.yaw_rate_rad_s == 0 && !config_.hold_on_zero_rate) {
                const int ret = seed(snapshot);
                if (ret < 0)
                    return ret;
            }
            delta = std::clamp(command.yaw_rate_rad_s, -config_.max_rate_rad_s, config_.max_rate_rad_s) * dt;
        }
        else if (command.mode == GimbalMode::AbsoluteAngle) {
            if (config_.topology == YawTopology::Continuous) {
                float error = 0;
                const int ret = control_shortest_angle_error(command.yaw_target_rad,
                                                             static_cast<float>(target_angle_rad_), &error);
                if (ret < 0)
                    return ret;
                delta = error;
            }
            else
                delta = double(std::clamp(command.yaw_target_rad, config_.min_angle_rad, config_.max_angle_rad)) -
                        target_angle_rad_;
            const double step = config_.max_rate_rad_s * dt;
            delta = std::clamp(delta, -step, step);
        }
        target_angle_rad_ += delta;
    }
    if (config_.topology == YawTopology::Limited) {
        const double actual = snapshot.feedback.position_rad;
        if (actual < double(config_.min_angle_rad) || actual > double(config_.max_angle_rad))
            return -ERANGE;
        target_angle_rad_ = std::clamp(target_angle_rad_, double(config_.min_angle_rad), double(config_.max_angle_rad));
    }
    else
        target_angle_rad_ = std::remainder(target_angle_rad_, 6.283185307179586);

    previous_action_ = action;
    previous_mode_ = command.mode;
    return axis_.update(target_angle_rad_, dt);
}

} // namespace skywalker::robotics
