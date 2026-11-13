#pragma once

#include <robotics/chassis/spin_speed_profile.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/gimbal/dual_yaw_motion_allocator.hpp>
#include <robotics/gimbal/inertial_gimbal.hpp>
#include <robotics/gimbal/yaw_centering.hpp>
#include <robotics/vehicle/gimbal_profile.hpp>

namespace skywalker::robotics::vehicle {
inline constexpr std::uint32_t manual_command_timeout_ms = gimbal_command_timeout_ms;
// Old physical joint-direction hint retained for standalone benches;
// reference-v4 world-rate commands must NOT apply this factor.
inline constexpr float big_yaw_wire_direction = -1.0f;
// 调参：整车合并档位（摇杆 + 键鼠）的鼠标灵敏度，单位 1/原始透传单位。
// 鼠标增量先乘该比例转成归一化角速度，再乘 max_gimbal_yaw/pitch_rate_rad_s，
// 所以它只改变未饱和区的灵敏度，不改变最大角速度。
// 原值 0.002（x=500 原始单位即到满幅）；现在放大 3 倍，鼠标手感更跟手。
inline constexpr float manual_mouse_yaw_scale = 0.08f, manual_mouse_pitch_scale = 0.04f;
inline constexpr CommandManager::Config manualCommandConfig() {
    auto c = gimbalCommandConfig();
    c.require_external_run_gate = true;
    c.max_chassis_vx_m_s = manual_max_vx_m_s;
    c.max_chassis_vy_m_s = manual_max_vy_m_s;
    c.max_chassis_wz_rad_s = manual_max_wz_rad_s;
    c.mapper.mouse_yaw_scale = manual_mouse_yaw_scale;
    c.mapper.mouse_pitch_scale = manual_mouse_pitch_scale;
    return c;
}
static_assert(manualCommandConfig().max_chassis_wz_rad_s > 0);
static_assert(manual_mouse_yaw_scale > 0 && manual_mouse_pitch_scale > 0, "mouse sensitivity must be positive");
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
    // 保留视觉完整世界目标；大偏差由回中链路逐步转交大 Yaw。
    // 这里只限制执行速度，不能截断目标角。
    c.max_goal_lead_rad = 0;
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
// 【已作废】旧的大 Yaw 回中参数：只按小 Yaw 偏角产生一个速度请求。
// 正式云台已改用 dualYawAllocatorConfig() 的主动分配（持续前馈 + 实测残差 +
// 行程预测），本工厂仅保留给历史 bringup 与契约测试，不要在新代码中引用。
inline YawCenteringController::Config manualCenteringConfig() {
    YawCenteringController::Config c{};
    c.center_rad = yaw_center_rad;
    c.follow_direction = small_yaw.direction;
    c.max_rate_rad_s = 0.6f;
    c.acceleration_rad_s2 = 0.8f;
    return c;
}
// 双 Yaw 分配器：正式云台唯一的大小 Yaw 分工点。
//
// 所有几何量都取自 calibration.hpp 已标定的编码器端点/中心，能力量取自电机标定；
// 这里只给“保守预算”，不代表实测加速度能力。调参顺序见
// docs/dev/双Yaw速度加速度分配与视觉轨迹控制方案.md 第 10 节：先标定方向与参考，
// 再单独测两轴响应，最后才改这里的持续/快变分配。
inline DualYawAllocatorConfig dualYawAllocatorConfig() {
    DualYawAllocatorConfig c{};
    // 小 Yaw 关节几何（编码器坐标）。方向只在这里给出一次。
    c.joint_center_rad = yaw_center_rad;
    c.joint_min_rad = yaw_min_rad;
    c.joint_max_rad = yaw_max_rad;
    c.joint_direction = small_yaw.direction;
    // 端点前保留的机械/测量余量，约 1.7 deg；标定端点本身仍待实测。
    c.joint_margin_rad = 0.03f;
    // 与 yawMotorConfig() 的位置环死区一致，作为位置偏差允许量。
    c.joint_tolerance_rad = 0.012f;
    c.joint_rate_max_rad_s = small_yaw.velocity_limit_rad_s;
    // 头部世界速率预算：手动输入上限就是操纵杆语义上限。
    c.head_rate_max_rad_s = manual_max_gimbal_yaw_rate_rad_s;
    c.head_acceleration_max_rad_s2 = 8.0f;
    c.head_jerk_max_rad_s3 = 0;
    c.head_steady_rate_max_rad_s = manual_max_gimbal_yaw_rate_rad_s;
    // 大 Yaw 世界意图：与世界头部速率同量级，不直接使用关节速度上限。
    c.big_intent_max_rad_s = manual_max_gimbal_yaw_rate_rad_s;
    c.big_acceleration_max_rad_s2 = 8.0f;
    c.big_jerk_max_rad_s3 = 0;
    c.big_track_gain_s_inv = 4.0f;
    // 手动持续运动提取时间常数；视觉让大 Yaw 承担更多，单独更长。
    c.slow_tau_s = 0.12f;
    c.vision_slow_tau_s = 0.24f;
    c.vision_angle_gain_s_inv = 8.0f;
    // 小 Yaw 关节加减速能力，用于在载体加速度新鲜时收紧头部加速度窗。
    c.joint_acceleration_max_pos_rad_s2 = 20.0f;
    c.joint_acceleration_max_neg_rad_s2 = 20.0f;
    // 回中：只做附加的慢调节，持续运动仍由前馈承担。
    c.center_deadband_rad = 0.0087f;
    c.center_kp_rad_s_per_rad = 2.0f;
    c.center_rate_max_rad_s = 0.8f;
    // 小 Yaw 外向制动包络；能力值保守，未在负载下实测前不放大。
    c.brake_delay_s = 0.03f;
    c.brake_outward_acceleration_rad_s2 = 6.0f;
    c.brake_deceleration_rad_s2 = 12.0f;
    // 载体跟不动时收缩头部预算，而不是继续把误差堆给小 Yaw。
    c.tracking_error_rad_s = 0.35f;
    c.tracking_budget_scale_min = 0.4f;
    c.tracking_budget_slew_per_s = 0.5f;
    c.still_rate_epsilon_rad_s = 0.01f;
    c.still_acceleration_epsilon_rad_s2 = 0.05f;
    c.feedback_timeout_us = 50000;
    c.source_timeout_us = 100000;
    c.max_cycle_us = 20000;
    // 计划角最多领先实测头部角 0.5 rad：小 Yaw 被行程卡住时不允许无限累积，
    // 但最终目标本身仍由输入每周期重新给出，不会被截短。
    c.head_angle_lead_max_rad = 0.5f;
    return c;
}
} // namespace skywalker::robotics::vehicle
