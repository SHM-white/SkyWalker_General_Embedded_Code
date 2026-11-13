#pragma once
#include <robotics/execution/big_yaw_executor.hpp>
#include <robotics/execution/big_yaw_heading_follower.hpp>
#include <robotics/vehicle/calibration.hpp>
#include <robotics/vehicle/motion_profile.hpp>
namespace skywalker::robotics::vehicle {
inline motor::dm::Config bigYawHardware() {
    // TODO(hardware): confirm MIT mode and PMAX/VMAX/TMAX stored in the drive.
    const auto &cal = big_yaw_native_velocity_profile_confirmed ? reference_big_yaw : big_yaw;
    const motor::dm::J4310Options options{.id = cal.id,
                                          .master_id = cal.master_id,
                                          .position_max_rad = 12.5f,
                                          .velocity_max_rad_s = 30,
                                          .torque_max_nm = 10,
                                          .torque_limit_nm = cal.effort_limit,
                                          .timing = {.feedback_timeout_ms = 50,
                                                     .command_timeout_ms = 20,
                                                     .enable_timeout_ms = 3000,
                                                     .retry_interval_ms = 100}};
    return big_yaw_native_velocity_profile_confirmed ? motor::dm::j4310Velocity(options) : motor::dm::j4310Mit(options);
}
inline control::VelocityMotor::Config bigYawMotorConfig() {
    control::VelocityMotor::Config c{};
    c.effort_unit = control::EffortUnit::NewtonMeter;
    // TODO(tuning): identify the loaded big-Yaw inertia and tune the velocity loop.
    // kp/ki/kd 作用于“目标关节速度 - 实测关节速度”，输出 N·m；
    // 积分限幅 0.3 N·m，输出限幅取标定 effort_limit。
    c.loop.regulator.feedback = {.kp = 0.02f,
                                 .ki = 0.15f,
                                 .kd = 0,
                                 .derivative_tau_s = 0,
                                 .integral_min = -0.3f,
                                 .integral_max = 0.3f,
                                 .output_min = -big_yaw.effort_limit,
                                 .output_max = big_yaw.effort_limit,
                                 .deadband = 0,
                                 .dt_min_s = 0.001f,
                                 .dt_max_s = 0.020f};
    // 速度参考升/降斜率（电机轴 rad/s²）：底层再兜一次加速度，避免阶跃。
    c.loop.reference_slew = {0.5f, 0.5f};
    // 速度反馈滤波时间常数（s）：抑制 CAN 反馈量化噪声，过大会拖慢响应。
    c.loop.measurement_filter_tau_s = 0.02f;
    // 速度软死区（电机轴 rad/s）：小速度不输出力矩，避免静止时来回顶。
    c.loop.soft_deadband_rad_s = 0.02f;
    // 速度指令绝对上限（电机轴 rad/s）= 关节标定上限 × 传动比。
    c.loop.requested_velocity_abs_max_rad_s = big_yaw.velocity_limit_rad_s * big_yaw.gear_ratio;
    // 力矩绝对上限（N·m）。
    c.loop.effort_abs_max = big_yaw.effort_limit;
    return c;
}
inline constexpr BigYawExecutor::Config bigYawExecutionConfig() {
    BigYawExecutor::Config c{};
    // 是否启用执行器内部的航向锁（另一条大 Yaw 路径）；当前后端为原生速度。
    c.heading_lock_enabled = big_yaw_heading_lock_enabled;
    // 原生速度后端是否已确认可用；未确认为 false 时执行器拒绝运行。
    c.native_velocity_mode_confirmed = big_yaw_native_velocity_profile_confirmed;
    // 原生速度模式的目标加速度（电机轴 rad/s²）：来自 calibration.hpp 的调参常量。
    c.native_velocity_acceleration_rad_s2 = big_yaw_native_acceleration_rad_s2;
    // MIT 速度后端是否已确认可用（与原生速度二选一）。
    c.mit_mode_confirmed = big_yaw_mit_mode_confirmed;
    // 启动时大 Yaw 相对底盘的前向对齐是否已确认。
    c.heading_reference_confirmed = startup_heading_alignment_confirmed;
    // 航向锁路径的力矩上限（N·m）。
    c.heading.torque_max_nm = big_yaw.effort_limit;
    const auto &cal = big_yaw_native_velocity_profile_confirmed ? reference_big_yaw : big_yaw;
    // 最终关节速率上限（rad/s）：应用侧与执行器都按它钳制。
    c.rate_abs_max_rad_s = cal.velocity_limit_rad_s;
    // 电机轴 → 关节的传动比，只在这里以及执行器里各用一次。
    c.motor_to_joint_ratio = cal.gear_ratio;
    // 电机正方向 → 关节/世界正方向（从上方看逆时针）的符号，只取 ±1。
    c.direction = cal.direction;
    return c;
}
// World-heading follower for the FollowCenter request. Output is a physical
// joint rate; BigYawExecutor applies direction and ratio exactly once.
inline BigYawWorldHeadingFollower::Config bigYawHeadingFollowerConfig() {
    const auto &cal = big_yaw_native_velocity_profile_confirmed ? reference_big_yaw : big_yaw;
    BigYawWorldHeadingFollower::Config c{};
    // 航向 P 增益（1/s）：把“捕获目标 - 实测航向”的误差转成额外世界速率。
    c.heading_kp = big_yaw_heading_kp;
    // 航向修正量上限（rad/s）：P 项最多加多少世界速率。
    c.heading_rate_max_rad_s = big_yaw_heading_rate_max_rad_s;
    // 捕获目标最多领先实测航向多少（rad）：防止跟随失败后误差绕过 ±π 反向修正。
    c.heading_error_max_rad = 1.5707963f;
    // 角速度 PI 的 P / I：作用于“目标世界角速度 - 实测头部角速度”。
    c.rate_kp = big_yaw_heading_rate_kp;
    c.rate_ki = big_yaw_heading_rate_ki;
    // 角速度积分上限（rad/s），同时参与抗饱和判断。
    c.rate_integral_max_rad_s = big_yaw_heading_integral_max_rad_s;
    // 最终关节速率输出上限（rad/s），与执行器 rate_abs_max 一致。
    c.output_max_rad_s = cal.velocity_limit_rad_s;
    // 零速判定的浮点噪声容差（rad/s）。
    c.release_epsilon_rad_s = big_yaw_heading_release_epsilon_rad_s;
    // 双 Yaw 分配器的大 Yaw 世界意图是连续轨迹：反向点和“持续前馈 + 回中”抵消
    // 都会自然穿过零速。只有零请求持续 100 ms 才按松杆处理，避免每次过零都丢弃
    // 已积累的航向目标；0 = 旧行为（第一个零速周期就捕获）。
    c.stop_capture_dwell_s = 0.1f;
    return c;
}
} // namespace skywalker::robotics::vehicle
