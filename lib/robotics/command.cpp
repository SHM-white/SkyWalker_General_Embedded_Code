#include <algorithm>
#include <cmath>
#include <cerrno>
#include <robotics/command/manual_command_mapper.hpp>
#include <robotics/command/command_arbiter.hpp>
namespace skywalker::robotics {
namespace {
MouseShooterGesture::Config gestureConfig(const CommandArbiter::Config &config) {
    auto gesture = config.mouse_gesture;
    gesture.input_timeout_ms = config.input_timeout_ms;
    return gesture;
}
void fillManualMotion(const OperatorIntent &i, const CommandArbiter::Config &c, RobotCommand &out) {
    const auto scale = [](float v, float limit) { return std::clamp(v, -1.0f, 1.0f) * limit; };
    out.chassis.mode = ChassisMode::BodyVelocity;
    out.chassis.source = i.source;
    out.chassis.vx_m_s = scale(i.chassis_vx_norm, c.max_chassis_vx_m_s);
    out.chassis.vy_m_s = scale(i.chassis_vy_norm, c.max_chassis_vy_m_s);
    out.chassis.wz_rad_s = scale(i.chassis_wz_norm, c.max_chassis_wz_rad_s);
    out.gimbal.mode = GimbalMode::Rate;
    out.gimbal.source = i.source;
    out.gimbal.yaw_rate_rad_s = i.mouse_rate_mapping
                                  ? std::clamp(i.mouse_yaw_rate_rad_s, -c.max_gimbal_yaw_rate_rad_s,
                                               c.max_gimbal_yaw_rate_rad_s)
                                  : scale(i.gimbal_yaw_rate_norm, c.max_gimbal_yaw_rate_rad_s);
    out.gimbal.pitch_rate_rad_s = i.mouse_rate_mapping
                                    ? std::clamp(i.mouse_pitch_rate_rad_s, -c.max_gimbal_pitch_rate_rad_s,
                                                 c.max_gimbal_pitch_rate_rad_s)
                                    : scale(i.gimbal_pitch_rate_norm, c.max_gimbal_pitch_rate_rad_s);
}
}
int ManualCommandMapper::map(const RemoteState &r, OperatorIntent &out) const {
    if (!std::isfinite(config_.channel_range) || config_.channel_range <= 0 ||
        !std::isfinite(config_.analog_deadband) || config_.analog_deadband < 0 || config_.analog_deadband >= 1 ||
        !std::isfinite(config_.mouse_yaw_scale) || !std::isfinite(config_.mouse_pitch_scale) ||
        !std::isfinite(config_.mouse_yaw_rate_per_unit) || !std::isfinite(config_.mouse_pitch_rate_per_unit) ||
        (config_.input_profile != RemoteInputProfile::PhysicalRemote &&
         config_.input_profile != RemoteInputProfile::KeyboardMouseSelectable &&
         config_.input_profile != RemoteInputProfile::ShooterSelectable))
        return -EINVAL;
    OperatorIntent n{};
    n.stamp = r.stamp;
    if (!r.online || !r.stamp.valid) {
        out = n;
        return 0;
    }
    if (r.left_switch == RcSwitch::Unknown || r.right_switch == RcSwitch::Unknown)
        return -EINVAL;
    const bool shooter_selectable = config_.input_profile == RemoteInputProfile::ShooterSelectable;
    n.mode = shooter_selectable ? (r.left_switch == RcSwitch::Down ? OperatorMode::Safe : OperatorMode::Manual)
             : r.left_switch == RcSwitch::Middle ? OperatorMode::Manual
             : r.left_switch == RcSwitch::Up   ? OperatorMode::Auto
                                               : OperatorMode::Safe;
    const bool physical = config_.input_profile == RemoteInputProfile::PhysicalRemote;
    n.source = shooter_selectable
                   ? (r.left_switch == RcSwitch::Up ? ControlSource::KeyboardMouse : ControlSource::Remote)
                   : (!physical && r.right_switch == RcSwitch::Up ? ControlSource::KeyboardMouse
                                                                 : ControlSource::Remote);
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
        if (physical || shooter_selectable) {
            n.friction_requested = r.right_switch != RcSwitch::Down;
            n.fire_requested = r.right_switch == RcSwitch::Up;
        }
    }
    else {
        auto key = [&](unsigned bit) { return (r.keyboard.bits >> bit) & 1u; };
        n.chassis_vx_norm = float(key(0)) - float(key(1));
        n.chassis_vy_norm = float(key(2)) - float(key(3));
        n.gimbal_yaw_rate_norm = std::clamp(-r.mouse.x * config_.mouse_yaw_scale, -1.0f, 1.0f);
        n.gimbal_pitch_rate_norm = std::clamp(-r.mouse.y * config_.mouse_pitch_scale, -1.0f, 1.0f);
        if (shooter_selectable) {
            // 速度只由云台 Rate 通路使用真实 dt 积分，鼠标上游不乘采样周期。
            n.mouse_rate_mapping = true;
            n.mouse_yaw_rate_rad_s = -r.mouse.x * config_.mouse_yaw_rate_per_unit;
            n.mouse_pitch_rate_rad_s = -r.mouse.y * config_.mouse_pitch_rate_per_unit;
        }
        n.friction_requested = r.mouse.right;
        n.fire_requested = r.mouse.left;
    }
    out = n;
    return 0;
}
CommandArbiter::CommandArbiter(const Config &config)
    : config_(config), config_error_(validateConfig()), mouse_gesture_(gestureConfig(config)) {
}
void CommandArbiter::reset() {
    sequence_ = 0;
    previous_mode_ = OperatorMode::Safe;
    auto_entry_us_ = last_step_us_ = override_quiet_since_us_ = 0;
    auto_baseline_sequence_ = last_seen_vision_sequence_ = 0;
    have_vision_sequence_ = have_step_time_ = manual_override_ = have_override_quiet_time_ = false;
    withdrawMouseShooter();
}
void CommandArbiter::withdrawMouseShooter() {
    mouse_gesture_.withdraw();
    mouse_fire_ = {};
}
int CommandArbiter::observeRemoteFrame(const RemoteState &remote, bool run_allowed, std::uint64_t now_ms) {
    OperatorIntent mapped{};
    const int ret = ManualCommandMapper(config_.mapper).map(remote, mapped);
    if (ret < 0) {
        withdrawMouseShooter();
        return ret;
    }
    const bool selected = mapped.mode != OperatorMode::Safe && mapped.source == ControlSource::KeyboardMouse;
    return mouse_gesture_.update(remote, selected, run_allowed, now_ms, mouse_fire_);
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
int CommandArbiter::validateConfig() const {
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
    const int ret = ManualCommandMapper(config_.mapper).map(RemoteState{}, unused);
    return ret < 0 ? ret : MouseShooterGesture(gestureConfig(config_)).configError();
}
void CommandArbiter::updateAutoOverride(OperatorIntent &i, const CommandInputs &in, core::TimeUs now) {
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
CommandDecision CommandArbiter::update(const CommandInputs &in) {
    const auto now = in.now_us;
    CommandDecision n{};
    const MessageStamp stamp{now / 1000, sequence_ + 1u, true};
    const auto finish = [&](int ret) {
        n.requested.stamp = n.requested.chassis.stamp = n.requested.gimbal.stamp = n.requested.shooter.stamp = stamp;
        n.command.stamp = n.command.chassis.stamp = n.command.gimbal.stamp = n.command.shooter.stamp = stamp;
        sequence_ = stamp.sequence;
        n.error = ret;
        return n;
    };
    const auto disable = [&](std::uint32_t reason, int ret) {
        withdrawMouseShooter();
        n.mouse_fire = {};
        previous_mode_ = OperatorMode::Safe;
        manual_override_ = have_override_quiet_time_ = false;
        n.chassis_reasons = n.gimbal_reasons = n.shooter_reasons = reason;
        return finish(ret);
    };
    if (config_error_ < 0)
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
    if (!in.run_allowed)
        return disable(OperatorGateDenied, 0);
    OperatorIntent i{};
    if (ManualCommandMapper(config_.mapper).map(in.remote, i) < 0)
        return disable(InvalidInputs, -EINVAL);
    const auto &robot = in.referee.robot;
    const auto cr = config_.require_referee_for_motion
                        ? permissionReason(robot.chassis_output, ms, config_.permission_timeout_ms)
                        : 0u;
    const auto gr = config_.require_referee_for_motion
                        ? permissionReason(robot.gimbal_output, ms, config_.permission_timeout_ms)
                        : 0u;
    const auto sr = config_.require_referee_for_motion
                        ? permissionReason(robot.shooter_output, ms, config_.permission_timeout_ms)
                        : 0u;
    if (observeRemoteFrame(in.remote, !gr && !sr, ms) < 0)
        return disable(InvalidInputs, -ESTALE);
    n.mouse_fire = mouse_fire_;
    if (i.source == ControlSource::KeyboardMouse) {
        i.friction_requested = mouse_fire_.friction_requested;
        i.fire_requested = mouse_fire_.kind != MouseFeedKind::Idle;
    }
    n.operator_mode = i.mode;
    if (i.mode == OperatorMode::Safe)
        return disable(SafeRequested, 0);
    if (i.mode == OperatorMode::Auto && !config_.allow_auto)
        return disable(AutoUnavailable, 0);
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
                              : visual_control && in.vision.value.fire_requested &&
                                    (config_.mapper.input_profile != RemoteInputProfile::PhysicalRemote ||
                                     i.fire_requested);
        if (fire) {
            const bool mouse_single = i.source == ControlSource::KeyboardMouse &&
                                      (i.mode == OperatorMode::Manual || manual_override_) &&
                                      mouse_fire_.kind == MouseFeedKind::ClickSingle;
            s.mode = mouse_single ? ShooterMode::FireSingle : ShooterMode::FireContinuous;
            s.source = visual_control ? ControlSource::Vision : i.source;
            s.fire_rate_hz = config_.requested_fire_rate_hz;
            if (mouse_single) {
                // 此戳只来自实际按下帧；仲裁 finish 仅刷新普通命令戳。
                s.fire_event_id = mouse_fire_.click_event_id;
                s.fire_event_stamp = mouse_fire_.click_stamp;
            }
        }
    }
    n.command = n.requested;
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
    if ((n.command.shooter.mode == ShooterMode::FireContinuous ||
         n.command.shooter.mode == ShooterMode::FireSingle) &&
        (n.command.gimbal.mode == GimbalMode::Hold || n.command.gimbal.mode == GimbalMode::Disabled)) {
        n.command.shooter.mode = ShooterMode::Ready;
        n.command.shooter.source = i.source;
        n.command.shooter.fire_rate_hz = 0;
        n.command.shooter.fire_event_id = 0;
        n.command.shooter.fire_event_stamp = {};
        n.shooter_reasons |= AimNotControlling;
        withdrawMouseShooter();
    }
    if (gr || sr)
        withdrawMouseShooter();
    return finish(0);
}
}
