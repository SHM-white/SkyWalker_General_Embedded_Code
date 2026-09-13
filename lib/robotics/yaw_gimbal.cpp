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
    if (config_.topology == YawTopology::Continuous &&
        motor_.reference() != control::PositionReference::AbsoluteNearest)
        return -ENOTSUP;
    if (config_.topology == YawTopology::Limited && motor_.reference() != control::PositionReference::DriverContinuous)
        return -ENOTSUP;
    return 0;
}
int YawGimbal::begin() {
    int ret = validate();
    return ret < 0 ? ret : motor_.configure();
}
void YawGimbal::seed() {
    const auto &m = motor_.telemetry().measurement;
    target_angle_rad_ = config_.topology == YawTopology::Continuous ? double(m.feedback.absolute_position_rad)
                                                                    : m.position_rad;
    previous_action_ = SafetyAction::Disable;
    previous_mode_ = GimbalMode::Disabled;
    initialized_ = true;
}
int YawGimbal::poll(std::uint64_t now) {
    int ret = motor_.poll(now);
    if (ret == 0 && (!initialized_ || generation_ != motor_.status().resume_generation)) {
        seed();
        generation_ = motor_.status().resume_generation;
    }
    return ret;
}
int YawGimbal::suspend(PauseReason reason) {
    initialized_ = false;
    previous_action_ = SafetyAction::Disable;
    return motor_.suspend(reason);
}
int YawGimbal::update(const GimbalCommand &c, SafetyAction action, float dt) {
    if (action == SafetyAction::Disable || c.mode == GimbalMode::Disabled)
        return suspend(PauseReason::OperatorDisabled);
    if (action > SafetyAction::Active || c.mode > GimbalMode::AbsoluteAngle || !std::isfinite(dt) || dt <= 0 ||
        dt > 0.02f || !std::isfinite(c.yaw_rate_rad_s) || !std::isfinite(c.yaw_target_rad)) {
        suspend(PauseReason::InvalidCycle);
        return -EINVAL;
    }
    if (!initialized_)
        return -EAGAIN;
    if (motor_.state() == ExecutionState::Ready) {
        seed();
        const int ret = motor_.resume();
        if (ret < 0)
            return ret;
        previous_action_ = action;
        previous_mode_ = c.mode;
        return 0; // First armed cycle is zero output.
    }
    if (motor_.state() != ExecutionState::Active)
        return -EAGAIN;
    const bool hold = action == SafetyAction::Hold || c.mode == GimbalMode::Hold;
    const bool was_hold = previous_action_ == SafetyAction::Hold || previous_mode_ == GimbalMode::Hold;
    if (hold && !was_hold)
        seed();
    if (!hold) {
        double delta = 0;
        if (c.mode == GimbalMode::Rate) {
            if (c.yaw_rate_rad_s == 0 && !config_.hold_on_zero_rate)
                seed();
            delta = std::clamp(c.yaw_rate_rad_s, -config_.max_rate_rad_s, config_.max_rate_rad_s) * dt;
        }
        else if (c.mode == GimbalMode::AbsoluteAngle) {
            if (config_.topology == YawTopology::Continuous) {
                float error = 0;
                int ret = control_shortest_angle_error(c.yaw_target_rad, static_cast<float>(target_angle_rad_), &error);
                if (ret < 0)
                    return ret;
                delta = error;
            }
            else
                delta = double(std::clamp(c.yaw_target_rad, config_.min_angle_rad, config_.max_angle_rad)) -
                        target_angle_rad_;
            const double step = config_.max_rate_rad_s * dt;
            delta = std::clamp(delta, -step, step);
        }
        target_angle_rad_ += delta;
    }
    if (config_.topology == YawTopology::Limited) {
        const double actual = motor_.telemetry().measurement.position_rad;
        if (actual < double(config_.min_angle_rad) || actual > double(config_.max_angle_rad)) {
            suspend(PauseReason::InvalidCycle);
            return -ERANGE;
        }
        target_angle_rad_ = std::clamp(target_angle_rad_, double(config_.min_angle_rad), double(config_.max_angle_rad));
    }
    else
        target_angle_rad_ = std::remainder(target_angle_rad_, 6.283185307179586);
    previous_action_ = action;
    previous_mode_ = c.mode;
    return motor_.update(target_angle_rad_);
}
int YawGimbal::stop() {
    initialized_ = false;
    return motor_.stop();
}
}
