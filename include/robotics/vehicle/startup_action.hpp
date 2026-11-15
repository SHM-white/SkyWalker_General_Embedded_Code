#pragma once

#include <cmath>
#include <cstdint>

#include <core/measurement.hpp>
#include <robotics/chassis/spin_speed_profile.hpp>
#include <robotics/command/command_arbiter.hpp>
#include <robotics/command/command_inputs.hpp>
#include <robotics/command/manual_command_mapper.hpp>
#include <robotics/messages/remote.hpp>

namespace skywalker::robotics::vehicle {

// 上电快捷解锁的动作语义判定原因位。仅用于判定与诊断，不是故障锁存。
enum StartupActionReason : std::uint32_t {
    StartupMappingInvalid = 1u << 0,
    StartupChassisMotion = 1u << 1,
    StartupGimbalMotion = 1u << 2,
    StartupSpinRequest = 1u << 3,
    StartupMechanism = 1u << 4,
    StartupCruiseRequest = 1u << 5,
    StartupAutoUnknown = 1u << 6,
    StartupAutoMotion = 1u << 7
};

struct StartupActionAssessment {
    // false：输入不可用或映射无法判定，本次不允许使用快捷解锁。
    bool known = false;
    std::uint32_t reasons = 0;
    bool noAction() const {
        return known && reasons == 0;
    }
};

// 只读、无 I/O、无状态：判断“解锁后当前输入是否会主动产生动作”。
//
// 判定使用解锁后的候选意图，发生在运行门控、裁判裁剪和驱动就绪之前，
// 因此不会因为“尚未解锁所以实际输出为零”或“电机掉线”而误判为无动作。
// observed 为管理线程最近一次发布前的输入快照；nullptr 表示尚无快照。
inline StartupActionAssessment assessStartupAction(const RemoteState &remote, const CommandInputs *observed,
                                                   const CommandArbiter::Config &config,
                                                   const SpinSpeedProfile::Config &spin_config,
                                                   core::TimeUs now_us) {
    StartupActionAssessment result{};
    const auto now_ms = now_us / 1000;
    // 输入必须在线、戳有效且新鲜；两拨杆必须已知。
    if (!remote.online || !remote.stamp.valid || remote.left_switch == RcSwitch::Unknown ||
        remote.right_switch == RcSwitch::Unknown || !isFresh(remote.stamp, now_ms, config.input_timeout_ms))
        return result;
    OperatorIntent i{};
    if (ManualCommandMapper(effectiveMapperConfig(config)).map(remote, i) < 0) {
        result.reasons |= StartupMappingInvalid;
        return result;
    }
    if (i.mode == OperatorMode::Safe) {
        // 安全档启动瞬间无动作。它不授予运行权，离开左 Down 时会再次判定。
        result.known = true;
        return result;
    }
    if ((i.mode == OperatorMode::Navigation || i.mode == OperatorMode::Auto) && config.enable_cruise &&
        isCruise(i.autonomous)) {
        // 巡航任一子模式本身就是扫描意图：即使没有导航、视觉或电机未就绪。
        result.known = true;
        result.reasons |= StartupCruiseRequest;
        return result;
    }
    if (i.mode == OperatorMode::Manual) {
        std::uint32_t reasons = 0;
        if (i.chassis_vx_norm != 0 || i.chassis_vy_norm != 0)
            reasons |= StartupChassisMotion;
        if (i.gimbal_yaw_rate_norm != 0 || i.gimbal_pitch_rate_norm != 0 ||
            (i.mouse_rate_mapping &&
             (i.mouse_yaw_rate_rad_s != 0 || i.mouse_pitch_rate_rad_s != 0)))
            reasons |= StartupGimbalMotion;
        // 合并键鼠档的附加动作输入：Shift 即将锁存小陀螺，Q/E 代替拨轮方向。
        // 只对正式应用的合并配置生效，不改变 PhysicalRemote 或其他样例。
        const bool merged_keys = config.mapper.merge_keyboard_mouse;
        const auto bits = remote.keyboard.bits;
        const bool shift = merged_keys && (((bits >> 4) & 1u) != 0);
        const bool q = merged_keys && (((bits >> 6) & 1u) != 0);
        const bool e = merged_keys && (((bits >> 7) & 1u) != 0);
        const float wheel =
            ManualCommandMapper::normalizeChannel(remote.analog.wheel, 660.0f, 0.03f);
        const float dial = e ? 1.0f : q ? -1.0f : wheel;
        if (remote.right_switch == RcSwitch::Down || shift ||
            std::fabs(dial) >= spin_config.dial_deadband)
            reasons |= StartupSpinRequest;
        if (i.friction_requested || i.fire_requested)
            reasons |= StartupMechanism;
        result.known = true;
        result.reasons = reasons;
        return result;
    }
    // 自主档（普通导航/视觉）：只有新鲜有效且明确为零的源才算“无动作”。
    // 缺帧、过期、非法、参考不匹配或没有快照都属于未知，不授予快捷解锁。
    bool known = true;
    bool motion = false;
    if (config.allow_navigation) {
        const auto &nav = observed ? observed->navigation : communication::navigation::Command{};
        if (!observed || !nav.stamp.valid)
            known = false;
        else if (!core::fresh(nav.stamp, now_us, config.navigation_timeout_us))
            known = false;
        else if (!std::isfinite(nav.value.vx_m_s) || !std::isfinite(nav.value.vy_m_s) ||
                 !std::isfinite(nav.value.wz_rad_s))
            known = false;
        else if (nav.value.vx_m_s != 0 || nav.value.vy_m_s != 0 || nav.value.wz_rad_s != 0)
            motion = true;
    }
    if (config.allow_auto) {
        const auto &vision = observed ? observed->vision : core::Measurement<communication::vision::AimCommand>{};
        const auto &v = vision.value;
        if (!observed || !vision.stamp.valid)
            known = false;
        else if (!core::fresh(vision.stamp, now_us, config.vision_timeout_us))
            known = false;
        else if (v.reference != config.expected_vision_reference)
            known = false;
        else if (!std::isfinite(v.yaw.angle_rad) || !std::isfinite(v.pitch.angle_rad) ||
                 !std::isfinite(v.yaw.rate_rad_s) || !std::isfinite(v.pitch.rate_rad_s) ||
                 !std::isfinite(v.yaw.acceleration_rad_s2) || !std::isfinite(v.pitch.acceleration_rad_s2))
            known = false;
        else if (v.control_requested)
            // 不在此推测“角误差为零”：有效跟踪请求即视为动作。
            motion = true;
    }
    result.known = known;
    if (motion)
        result.reasons |= StartupAutoMotion;
    if (!known)
        result.reasons |= StartupAutoUnknown;
    return result;
}

} // namespace skywalker::robotics::vehicle
