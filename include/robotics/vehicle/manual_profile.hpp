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
// ============================================================================
// 双 Yaw 调参入口索引（按“先标定方向/参考 → 再测单轴能力 → 最后调分工”的顺序改，
// 不要一次同时改多个分组，否则无法判断改善或恶化来自哪一层）
//
//   1) 几何、方向与电机标定：include/robotics/vehicle/calibration.hpp
//        MotorCalibration 字段顺序 = {id, master_id, gear_ratio, direction,
//        effort_limit, velocity_limit_rad_s, encoder_zero_ticks}
//        small_yaw / pitch / big_yaw / reference_big_yaw 四组标定；
//        yaw_low_ticks/yaw_high_ticks = 小 Yaw 软件端点，yaw_center_rad = 小 Yaw 中心，
//        reference_big_yaw_forward_zero_rad = 大 Yaw 相对底盘前向零点，
//        big_yaw_native_acceleration_rad_s2 = 大 Yaw 执行器目标加速度。
//   2) 小 Yaw 电机环与机械限位：include/robotics/vehicle/gimbal_profile.hpp
//        yawHardware()（GM6020 ID/电流/编码器零点/超时）、
//        yawMotorConfig()（位置环 + 速度内环 PID、参考斜率、速度/电流上限）、
//        yawAxisConfig()（软限位端点、限速、限位停发策略）。
//   3) 大 Yaw 底盘侧执行：include/robotics/vehicle/big_yaw_profile.hpp
//        bigYawHardware()（J4310 后端与 PMAX/VMAX/TMAX）、
//        bigYawMotorConfig()（速度内环）、bigYawExecutionConfig()（关节限速/传动比/方向）、
//        bigYawHeadingFollowerConfig()（世界航向闭环、零速捕获驻留）。
//   4) 大小 Yaw 分工：本文件 dualYawAllocatorConfig()，只有这里决定“谁承担多少”。
//   5) 头部姿态外环：本文件 sentryInertialConfig()（yaw_kp / gyro_damping）。
//   6) 输入灵敏度：本文件 manual_mouse_yaw/pitch_scale 与 calibration.hpp 的
//        manual_max_gimbal_yaw_rate_rad_s / manual_max_gimbal_pitch_rate_rad_s。
// ============================================================================

// 双 Yaw 分配器：正式云台唯一的大小 Yaw 分工点。
//
// 所有几何量都取自 calibration.hpp 已标定的编码器端点/中心，能力量取自电机标定；
// 这里只给“保守预算”，不代表实测加速度能力。调参顺序见
// docs/dev/双Yaw速度加速度分配与视觉轨迹控制方案.md 第 10 节：先标定方向与参考，
// 再单独测两轴响应，最后才改这里的持续/快变分配。
inline DualYawAllocatorConfig dualYawAllocatorConfig() {
    DualYawAllocatorConfig c{};
    // ---- 小 Yaw 关节几何（编码器坐标，全部来自 calibration.hpp）----
    // 小 Yaw 工作中心：回中要停住的位置，不是编码器零点；改端点时必须同步改它。
    c.joint_center_rad = yaw_center_rad;
    // 软件行程两端（rad，编码器坐标）：小 Yaw 允许的最小/最大关节角。
    c.joint_min_rad = yaw_min_rad;
    c.joint_max_rad = yaw_max_rad;
    // 编码器 → 头部相对角的正负号，只取 ±1，且只在这一处应用一次。
    c.joint_direction = small_yaw.direction;
    // 端点前保留的机械/测量余量（rad，约 1.7°）；调大只会更早把运动交给大 Yaw。
    c.joint_margin_rad = 0.03f;
    // 位置偏差允许量（rad）：与 yawMotorConfig() 的位置环死区 0.012 一致，
    // 用于判定“已经贴边”，不是再算一次死区。
    c.joint_tolerance_rad = 0.012f;
    // 小 Yaw 可执行关节速度上限（rad/s，编码器坐标），直接取 GM6020 输出轴标定。
    c.joint_rate_max_rad_s = small_yaw.velocity_limit_rad_s;
    // ---- 头部世界速率预算（大小 Yaw 合计能产生的世界角速度）----
    // 头部世界速率硬上限（rad/s）：手动满幅就是它，也是可行域的外层上限。
    c.head_rate_max_rad_s = manual_max_gimbal_yaw_rate_rad_s;
    // 头部世界加速度上限（rad/s²）：决定推杆后多久到满速、反向时多久停住。
    c.head_acceleration_max_rad_s2 = 8.0f;
    // 头部 jerk 上限（rad/s³）：0 = 关闭 jerk 整形，只做每周期加速度钳制。
    c.head_jerk_max_rad_s3 = 0;
    // 持续运动预算（rad/s）：长期可交给大 Yaw 的世界速率，必须 ≤ head_rate_max。
    // 调小 → 大 Yaw 更保守、小 Yaw 承担更多；调大 → 大 Yaw 更主动。
    c.head_steady_rate_max_rad_s = manual_max_gimbal_yaw_rate_rad_s;
    // ---- 大 Yaw 世界意图（发给底盘的 world rate，不是关节速度）----
    // 大 Yaw 世界意图上限（rad/s）：与头部速率同量级，不直接使用关节速度上限。
    c.big_intent_max_rad_s = manual_max_gimbal_yaw_rate_rad_s;
    // 大 Yaw 世界意图加速度上限（rad/s²）：越大越早跟上意图，越容易过冲。
    c.big_acceleration_max_rad_s2 = 8.0f;
    // 大 Yaw jerk 上限（rad/s³）：0 = 关闭。
    c.big_jerk_max_rad_s3 = 0;
    // 大 Yaw 意图收敛增益（1/s）：把“持续分量 + 回中”拉向世界意图的速度，
    // 越大越硬、越小越柔。
    c.big_track_gain_s_inv = 4.0f;
    // ---- 快慢分离（决定高频由谁承担）----
    // 手动持续运动提取时间常数（s）：越大 → 大 Yaw 越晚参与，小 Yaw 承担更多快变。
    c.slow_tau_s = 0.12f;
    // 视觉持续运动时间常数（s）：比手动更长，让大 Yaw 承担更多慢变跟踪。
    c.vision_slow_tau_s = 0.24f;
    // 视觉角度误差 → 头部速率增益（1/s）：决定视觉目标收敛快慢。
    c.vision_angle_gain_s_inv = 8.0f;
    // ---- 小 Yaw 关节加减速能力（用来收紧头部加速度窗）----
    // 正/负向关节加速度能力（rad/s²）：载体加速度新鲜时，头部加速度窗被夹到
    // 载体加速度 ± 该值，避免要求小 Yaw 输出它做不到的相对加速度。未实测前保持保守。
    c.joint_acceleration_max_pos_rad_s2 = 20.0f;
    c.joint_acceleration_max_neg_rad_s2 = 20.0f;
    // ---- 回中（附加的慢调节，不替代持续前馈）----
    // 回中死区（rad，约 0.5°）：中心附近不动，避免来回微抖。
    c.center_deadband_rad = 0.0087f;
    // 回中比例增益（rad/s per rad）：偏角每 1 rad 产生多少大 Yaw 世界速率。
    c.center_kp_rad_s_per_rad = 2.0f;
    // 回中速率上限（rad/s）：限制回中这条慢调节自身的速度。
    c.center_rate_max_rad_s = 0.8f;
    // ---- 小 Yaw 外向制动包络（行程保护）----
    // 制动反应延迟（s）：从发出命令到真正开始减速的整链延迟估计。
    c.brake_delay_s = 0.03f;
    // 延迟期间最坏的外向加速度（rad/s²）：越保守，越早开始限制外向速度。
    c.brake_outward_acceleration_rad_s2 = 6.0f;
    // 有保证的制动减速度（rad/s²）：必须是负载下能实现的保守值，不是理想值。
    c.brake_deceleration_rad_s2 = 12.0f;
    // ---- 大 Yaw 实际跟不上时的预算收缩 ----
    // 判定“大 Yaw 没跟上”的速率误差阈值（rad/s）。
    c.tracking_error_rad_s = 0.35f;
    // 预算收缩下限（0..1）：最差情况下头部速率预算保留的比例。
    c.tracking_budget_scale_min = 0.4f;
    // 预算收缩/恢复斜率（1/s）：决定收缩快慢，太小会滞后、太大会抖。
    c.tracking_budget_slew_per_s = 0.5f;
    // ---- 静止锁存（防止残余速率持续发给底盘）----
    // 速率静止阈值（rad/s）：所有分量都在此以内才锁存为精确 0。
    c.still_rate_epsilon_rad_s = 0.01f;
    // 加速度静止阈值（rad/s²）：与上面一起判定静止。
    c.still_acceleration_epsilon_rad_s2 = 0.05f;
    // ---- 时间与新鲜度 ----
    // 小 Yaw 关节反馈超时（us）：超过按 Feedback 拒绝分配。
    c.feedback_timeout_us = 50000;
    // 输入源超时（us）：超过走退化本地路径（仍带同一套行程保护）。
    c.source_timeout_us = 100000;
    // 允许的最大控制周期（us）：超过按 Cycle 处理，不跨周期积分。
    c.max_cycle_us = 20000;
    // ---- 计划角超前保护 ----
    // 计划角最多领先实测头部角 0.5 rad：小 Yaw 被行程卡住时不允许无限累积，
    // 但最终目标本身仍由输入每周期重新给出，不会被截短。
    c.head_angle_lead_max_rad = 0.5f;
    return c;
}
} // namespace skywalker::robotics::vehicle
