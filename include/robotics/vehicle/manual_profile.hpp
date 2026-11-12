#pragma once

#include <robotics/chassis/spin_speed_profile.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/gimbal/inertial_gimbal.hpp>
#include <robotics/gimbal/yaw_centering.hpp>
#include <robotics/vehicle/gimbal_profile.hpp>

namespace skywalker::robotics::vehicle {
inline constexpr std::uint32_t manual_command_timeout_ms = gimbal_command_timeout_ms;
// Old physical joint-direction hint retained for standalone benches;
// reference-v4 world-rate commands must NOT apply this factor.
inline constexpr float big_yaw_wire_direction = -1.0f;
inline constexpr CommandManager::Config manualCommandConfig() {
    auto c = gimbalCommandConfig();
    c.require_external_run_gate = true;
    c.max_chassis_vx_m_s = manual_max_vx_m_s;
    c.max_chassis_vy_m_s = manual_max_vy_m_s;
    c.max_chassis_wz_rad_s = manual_max_wz_rad_s;
    return c;
}
static_assert(manualCommandConfig().max_chassis_wz_rad_s > 0);
inline InertialGimbalAdapter::Config manualInertialConfig() {
    InertialGimbalAdapter::Config c{};
    c.yaw = yawAxisConfig();
    c.pitch = pitchAxisConfig();
    c.yaw_direction = small_yaw.direction;
    c.pitch_direction = pitch.direction;
    c.pitch_locked = false;
    c.allow_unknown_quality = true;
    return c;
}
// 整车头部 CAN IMU 姿态外环；机械限位与电机 PID 仍由共享两轴配置负责。
inline InertialGimbalAdapter::Config sentryInertialConfig() {
    auto c = manualInertialConfig();
    // 姿态误差转关节速度的增益，单位 1/s；Yaw/Pitch 独立于电机位置环 P。
    c.yaw_kp = 3.0f;
    c.pitch_kp = 3.0f;
    // 头部实测角速度误差的阻尼系数，抑制惯性外环超调。
    c.gyro_damping = 0.1f;
    c.pitch.max_rate_rad_s = manual_max_gimbal_pitch_rate_rad_s;
    // 世界 Yaw 目标最大超前角，单位 rad；失能/断流另外通过应用清除旧目标。
    c.max_goal_lead_rad = dualYawAllocatorConfig().head_angle_lead_max_rad;
    // Pitch 不套用同一超前角：Rate 模式的 Pitch 目标是积分器，钳制会把它
    // 钉在“实测+超前角”上，松杆后持续朝限位爬行并在端点锁死。绝对视觉目标
    // 也先保持不钳制，等 Pitch 方向与机械端点实测后再单独评估。
    c.pitch_max_goal_lead_rad = 0;
    return c;
}
// 调参：变速小陀螺。曲线本身在 calibration.hpp 的 spinRateCurveRadS()。
inline constexpr SpinSpeedProfile::Config spinProfileConfig() {
    SpinSpeedProfile::Config c{};
    c.max_rate_rad_s = spin_rate_max_rad_s;
    c.acceleration_limit_rad_s2 = spin_acceleration_limit_rad_s2;
    c.deceleration_rad_s2 = spin_deceleration_rad_s2;
    c.stop_epsilon_rad_s = spin_stop_epsilon_rad_s;
    return c;
}
static_assert(spinProfileConfig().max_rate_rad_s > 0);
inline YawCenteringController::Config manualCenteringConfig() {
    YawCenteringController::Config c{};
    c.center_rad = yaw_center_rad;
    c.follow_direction = small_yaw.direction;
    c.max_rate_rad_s = 0.6f;
    c.acceleration_rad_s2 = 0.8f;
    return c;
}
} // namespace skywalker::robotics::vehicle
