#include <algorithm>
#include <cmath>
#include <cerrno>
#include <robotics/command/manual_command_mapper.hpp>
#include <robotics/command/command_manager.hpp>
namespace skywalker::robotics {
namespace {
void fillManualMotion(const OperatorIntent &i, const CommandManager::Config &c, RobotCommand &out) {
    const auto scale = [](float v, float limit) { return std::clamp(v, -1.0f, 1.0f) * limit; };
    out.chassis.mode = ChassisMode::BodyVelocity;
    out.chassis.source = i.source;
    out.chassis.vx_m_s = scale(i.chassis_vx_norm, c.max_chassis_vx_m_s);
    out.chassis.vy_m_s = scale(i.chassis_vy_norm, c.max_chassis_vy_m_s);
    out.chassis.wz_rad_s = scale(i.chassis_wz_norm, c.max_chassis_wz_rad_s);
    out.gimbal.mode = GimbalMode::Rate;
    out.gimbal.source = i.source;
    out.gimbal.yaw_rate_rad_s = scale(i.gimbal_yaw_rate_norm, c.max_gimbal_yaw_rate_rad_s);
    out.gimbal.pitch_rate_rad_s = scale(i.gimbal_pitch_rate_norm, c.max_gimbal_pitch_rate_rad_s);
}
}
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
    previous_mode_ = OperatorMode::Safe;
    auto_entry_us_ = last_step_us_ = override_quiet_since_us_ = 0;
    auto_baseline_sequence_ = last_seen_vision_sequence_ = 0;
    have_vision_sequence_ = have_step_time_ = manual_override_ = have_override_quiet_time_ = false;
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
        fillManualMotion(i, config_, n);
        if (s.chassis != SafetyAction::Active)
            n.chassis = {};
        if (s.gimbal != SafetyAction::Active)
            n.gimbal = {};
    }
    if (fresh && s.gimbal == SafetyAction::Hold)
        n.gimbal.mode = GimbalMode::Hold;
    n.chassis.stamp = n.gimbal.stamp = n.shooter.stamp = n.stamp;
    out = n;
    ++sequence_;
    return 0;
}
}

namespace skywalker::robotics {
namespace {
std::uint32_t permissionReason(const OutputPermission &p, std::uint64_t now, std::uint32_t timeout) {
    if (!p.valid || !p.stamp.valid)
        return PermissionMissing;
    if (!isFresh(p.stamp, now, timeout))
        return PermissionStale;
    return p.enabled ? 0u : PermissionDenied;
}
}
int CommandManager::validate() const {
    const float nonnegative[] = {config_.max_chassis_vx_m_s, config_.max_chassis_vy_m_s, config_.max_chassis_wz_rad_s,
                                 config_.max_gimbal_yaw_rate_rad_s, config_.max_gimbal_pitch_rate_rad_s};
    for (float v : nonnegative)
        if (!std::isfinite(v) || v < 0)
            return -EINVAL;
    const float positive[] = {config_.max_vision_yaw_acceleration_rad_s2, config_.max_vision_pitch_acceleration_rad_s2,
                              config_.requested_fire_rate_hz};
    for (float v : positive)
        if (!std::isfinite(v) || v <= 0)
            return -EINVAL;
    if (!config_.input_timeout_ms || !config_.permission_timeout_ms || !config_.vision_timeout_us ||
        !config_.expected_vision_reference.frame_id || !config_.expected_vision_reference.epoch ||
        !config_.override_release_us || !std::isfinite(config_.override_enter_norm) ||
        !std::isfinite(config_.override_exit_norm) || config_.override_exit_norm < 0 ||
        config_.override_exit_norm >= config_.override_enter_norm || config_.override_enter_norm > 1)
        return -EINVAL;
    OperatorIntent unused{};
    return ManualCommandMapper(config_.mapper).map(RemoteState{}, unused);
}
void CommandManager::updateAutoOverride(OperatorIntent &i, const CommandInputs &in, core::TimeUs now) {
    const float magnitude = std::max(std::fabs(i.gimbal_yaw_rate_norm), std::fabs(i.gimbal_pitch_rate_norm));
    if (!manual_override_ && magnitude >= config_.override_enter_norm) {
        manual_override_ = true;
        have_override_quiet_time_ = false;
    }
    if (!manual_override_)
        return;
    if (magnitude > config_.override_exit_norm) {
        have_override_quiet_time_ = false;
        return;
    }
    i.gimbal_yaw_rate_norm = i.gimbal_pitch_rate_norm = 0;
    if (!have_override_quiet_time_) {
        have_override_quiet_time_ = true;
        override_quiet_since_us_ = now;
    }
    if (now - override_quiet_since_us_ >= config_.override_release_us) {
        manual_override_ = have_override_quiet_time_ = false;
        auto_entry_us_ = now;
        auto_baseline_sequence_ = in.vision.stamp.valid ? in.vision.stamp.sequence : 0;
    }
}
int CommandManager::step(const CommandInputs &in, core::TimeUs now, CommandDecision &out) {
    CommandDecision n{};
    const MessageStamp stamp{now / 1000, sequence_ + 1u, true};
    const auto finish = [&](int ret) {
        n.requested.stamp = n.requested.chassis.stamp = n.requested.gimbal.stamp = n.requested.shooter.stamp = stamp;
        n.command.stamp = n.command.chassis.stamp = n.command.gimbal.stamp = n.command.shooter.stamp = stamp;
        sequence_ = stamp.sequence;
        out = n;
        return ret;
    };
    const auto disable = [&](std::uint32_t reason, int ret) {
        previous_mode_ = OperatorMode::Safe;
        manual_override_ = have_override_quiet_time_ = false;
        n.chassis_reasons = n.gimbal_reasons = n.shooter_reasons = reason;
        return finish(ret);
    };
    if (validate() < 0)
        return disable(InvalidManagerConfig, -EINVAL);
    if (have_step_time_ && now < last_step_us_)
        return disable(ClockRegression, -ESTALE);
    last_step_us_ = now;
    have_step_time_ = true;
    bool regressed = false;
    if (in.vision.stamp.valid) {
        regressed = have_vision_sequence_ && in.vision.stamp.sequence < last_seen_vision_sequence_;
        if (!regressed)
            last_seen_vision_sequence_ = in.vision.stamp.sequence;
        have_vision_sequence_ = true;
    }
    const auto ms = now / 1000;
    if (!in.remote.online || !isFresh(in.remote.stamp, ms, config_.input_timeout_ms))
        return disable(RcUnavailable, 0);
    OperatorIntent i{};
    if (ManualCommandMapper(config_.mapper).map(in.remote, i) < 0)
        return disable(InvalidInputs, -EINVAL);
    n.operator_mode = i.mode;
    if (i.mode == OperatorMode::Safe)
        return disable(SafeRequested, 0);
    if (i.mode == OperatorMode::Auto) {
        if (previous_mode_ != OperatorMode::Auto) {
            auto_entry_us_ = now;
            auto_baseline_sequence_ = in.vision.stamp.valid ? in.vision.stamp.sequence : 0;
        }
        updateAutoOverride(i, in, now);
    }
    else
        manual_override_ = have_override_quiet_time_ = false;
    previous_mode_ = i.mode;
    fillManualMotion(i, config_, n.requested);
    n.manual_override = manual_override_;
    n.override_quiet = have_override_quiet_time_;
    bool visual_control = false;
    if (i.mode == OperatorMode::Auto && manual_override_) {
        n.gimbal_reasons |= ManualOverride;
        if (n.override_quiet)
            n.gimbal_reasons |= OverrideQuiet;
    }
    else if (i.mode == OperatorMode::Auto) {
        n.requested.gimbal = {};
        n.requested.gimbal.mode = GimbalMode::Hold;
        const auto &v = in.vision.value;
        const float values[] = {v.yaw.angle_rad,    v.pitch.angle_rad,         v.yaw.rate_rad_s,
                                v.pitch.rate_rad_s, v.yaw.acceleration_rad_s2, v.pitch.acceleration_rad_s2};
        bool finite = true;
        for (float value : values)
            finite = finite && std::isfinite(value);
        if (!in.vision.stamp.valid)
            n.gimbal_reasons |= VisionMissing;
        else if (regressed)
            n.gimbal_reasons |= InvalidVision;
        else if (!core::fresh(in.vision.stamp, now, config_.vision_timeout_us))
            n.gimbal_reasons |= VisionStale;
        else if (!v.control_requested)
            n.gimbal_reasons |= VisionStopped;
        else if (v.reference != config_.expected_vision_reference)
            n.gimbal_reasons |= VisionReferenceMismatch;
        else if (!finite)
            n.gimbal_reasons |= InvalidVision;
        else if (in.vision.stamp.sequence <= auto_baseline_sequence_ || in.vision.stamp.time_us < auto_entry_us_)
            n.gimbal_reasons |= WaitNewVision;
        else {
            visual_control = true;
            n.selected_vision = in.vision;
            auto &target = n.selected_vision.value;
            const auto limit = [&](float value, float maximum) {
                const float bounded = std::clamp(value, -maximum, maximum);
                if (bounded != value)
                    n.gimbal_reasons |= ValueLimited;
                return bounded;
            };
            target.yaw.rate_rad_s = limit(target.yaw.rate_rad_s, config_.max_gimbal_yaw_rate_rad_s);
            target.pitch.rate_rad_s = limit(target.pitch.rate_rad_s, config_.max_gimbal_pitch_rate_rad_s);
            target.yaw.acceleration_rad_s2 = limit(target.yaw.acceleration_rad_s2,
                                                   config_.max_vision_yaw_acceleration_rad_s2);
            target.pitch.acceleration_rad_s2 = limit(target.pitch.acceleration_rad_s2,
                                                     config_.max_vision_pitch_acceleration_rad_s2);
            auto &g = n.requested.gimbal;
            g.mode = GimbalMode::AbsoluteAngle;
            g.source = ControlSource::Vision;
            g.yaw_target_rad = target.yaw.angle_rad;
            g.pitch_target_rad = target.pitch.angle_rad;
            g.yaw_rate_rad_s = target.yaw.rate_rad_s;
            g.pitch_rate_rad_s = target.pitch.rate_rad_s;
        }
    }
    if (!i.friction_requested)
        n.shooter_reasons |= ShooterNotArmed;
    else {
        auto &s = n.requested.shooter;
        s.mode = ShooterMode::Ready;
        s.source = i.source;
        const bool fire = i.mode == OperatorMode::Manual || manual_override_
                              ? i.fire_requested
                              : visual_control && in.vision.value.fire_requested;
        if (fire) {
            s.mode = ShooterMode::FireContinuous;
            s.source = visual_control ? ControlSource::Vision : i.source;
            s.fire_rate_hz = config_.requested_fire_rate_hz;
        }
    }
    n.command = n.requested;
    const auto &robot = in.referee.robot;
    const auto cr = permissionReason(robot.chassis_output, ms, config_.permission_timeout_ms);
    const auto gr = permissionReason(robot.gimbal_output, ms, config_.permission_timeout_ms);
    const auto sr = permissionReason(robot.shooter_output, ms, config_.permission_timeout_ms);
    n.chassis_reasons |= cr;
    n.gimbal_reasons |= gr;
    n.shooter_reasons |= sr;
    if (cr)
        n.command.chassis = {};
    if (gr)
        n.command.gimbal = {};
    if (sr)
        n.command.shooter = {};
    if (n.command.gimbal.source != ControlSource::Vision)
        n.selected_vision = {};
    if (n.command.shooter.mode == ShooterMode::FireContinuous &&
        (n.command.gimbal.mode == GimbalMode::Hold || n.command.gimbal.mode == GimbalMode::Disabled)) {
        n.command.shooter.mode = ShooterMode::Ready;
        n.command.shooter.source = i.source;
        n.command.shooter.fire_rate_hz = 0;
        n.shooter_reasons |= AimNotControlling;
    }
    return finish(0);
}
}
