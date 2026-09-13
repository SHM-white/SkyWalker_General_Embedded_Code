#include <algorithm>
#include <cmath>
#include <cerrno>
#include <robotics/command/manual_command_mapper.hpp>
#include <robotics/command/command_manager.hpp>
namespace skywalker::robotics {
int ManualCommandMapper::map(const RemoteState &r, OperatorIntent &out) const {
    if (!std::isfinite(config_.channel_range) || config_.channel_range <= 0 ||
        !std::isfinite(config_.analog_deadband) || config_.analog_deadband < 0 || config_.analog_deadband >= 1 ||
        !std::isfinite(config_.mouse_yaw_scale) || !std::isfinite(config_.mouse_pitch_scale))
        return -EINVAL;
    OperatorIntent n{};
    n.stamp = r.stamp;
    if (!r.online || !r.stamp.valid) {
        out = n;
        return 0;
    }
    if (r.left_switch == RcSwitch::Unknown || r.right_switch == RcSwitch::Unknown)
        return -EINVAL;
    n.mode = r.left_switch == RcSwitch::Middle ? OperatorMode::Manual
             : r.left_switch == RcSwitch::Up   ? OperatorMode::Auto
                                               : OperatorMode::Safe;
    n.source = r.right_switch == RcSwitch::Up ? ControlSource::KeyboardMouse : ControlSource::Remote;
    auto norm = [&](float x) {
        x = std::clamp(x / config_.channel_range, -1.0f, 1.0f);
        return std::fabs(x) <= config_.analog_deadband
                   ? 0.0f
                   : std::copysign((std::fabs(x) - config_.analog_deadband) / (1 - config_.analog_deadband), x);
    };
    if (n.source == ControlSource::Remote) {
        n.chassis_vx_norm = norm(r.analog.left_y);
        n.chassis_vy_norm = -norm(r.analog.left_x);
        n.chassis_wz_norm = norm(r.analog.wheel);
        n.gimbal_yaw_rate_norm = -norm(r.analog.right_x);
        n.gimbal_pitch_rate_norm = norm(r.analog.right_y);
    }
    else {
        auto key = [&](unsigned bit) { return (r.keyboard.bits >> bit) & 1u; };
        n.chassis_vx_norm = float(key(0)) - float(key(1));
        n.chassis_vy_norm = float(key(2)) - float(key(3));
        n.gimbal_yaw_rate_norm = std::clamp(-r.mouse.x * config_.mouse_yaw_scale, -1.0f, 1.0f);
        n.gimbal_pitch_rate_norm = std::clamp(-r.mouse.y * config_.mouse_pitch_scale, -1.0f, 1.0f);
        n.friction_requested = r.mouse.right;
        n.fire_requested = r.mouse.left;
    }
    out = n;
    return 0;
}
int CommandManager::reset(std::uint64_t) {
    sequence_ = 0;
    return 0;
}
int CommandManager::step(const OperatorIntent &i, const GlobalSafetyDecision &s, std::uint64_t now, RobotCommand &out) {
    const float values[] = {i.chassis_vx_norm, i.chassis_vy_norm, i.chassis_wz_norm, i.gimbal_yaw_rate_norm,
                            i.gimbal_pitch_rate_norm};
    for (float v : values)
        if (!std::isfinite(v))
            return -EINVAL;
    const float limits[] = {config_.max_chassis_vx_m_s, config_.max_chassis_vy_m_s, config_.max_chassis_wz_rad_s,
                            config_.max_gimbal_yaw_rate_rad_s, config_.max_gimbal_pitch_rate_rad_s};
    for (float v : limits)
        if (!std::isfinite(v) || v < 0)
            return -EINVAL;
    if (i.mode > OperatorMode::Auto || i.source > ControlSource::Autonomous || s.chassis > SafetyAction::Active ||
        s.gimbal > SafetyAction::Active || s.shooter > SafetyAction::Active)
        return -EINVAL;
    RobotCommand n{};
    n.stamp = {now, sequence_ + 1, true};
    n.chassis.stamp = n.gimbal.stamp = n.shooter.stamp = n.stamp;
    const bool fresh = isFresh(i.stamp, now, config_.input_timeout_ms) &&
                       isFresh(s.stamp, now, config_.input_timeout_ms);
    // Autonomous and discrete shooter events require a separate future producer.
    if (fresh && i.mode == OperatorMode::Manual) {
        auto scale = [](float v, float limit) { return std::clamp(v, -1.0f, 1.0f) * limit; };
        if (s.chassis == SafetyAction::Active) {
            n.chassis.mode = ChassisMode::BodyVelocity;
            n.chassis.source = i.source;
            n.chassis.vx_m_s = scale(i.chassis_vx_norm, limits[0]);
            n.chassis.vy_m_s = scale(i.chassis_vy_norm, limits[1]);
            n.chassis.wz_rad_s = scale(i.chassis_wz_norm, limits[2]);
        }
        if (s.gimbal == SafetyAction::Active) {
            n.gimbal.mode = GimbalMode::Rate;
            n.gimbal.source = i.source;
            n.gimbal.yaw_rate_rad_s = scale(i.gimbal_yaw_rate_norm, limits[3]);
            n.gimbal.pitch_rate_rad_s = scale(i.gimbal_pitch_rate_norm, limits[4]);
        }
    }
    if (fresh && s.gimbal == SafetyAction::Hold)
        n.gimbal.mode = GimbalMode::Hold;
    out = n;
    ++sequence_;
    return 0;
}
}
