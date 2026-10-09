#pragma once
#include <algorithm>
#include <drivers/motor/dji_motor.hpp>
#include <robotics/shooter/shooter_executor.hpp>
#include <robotics/vehicle/calibration.hpp>
namespace skywalker::robotics::vehicle {
// 调参：鼠标快速连点、长按及 RC 连发共用射频，Hz；5 是保守初值，须无弹实机标定。
// 拨盘速度仍按 fire_rate_hz × dial_step_rad 换算，并受 dial_speed_rad_s 与硬件速度上限约束。
inline constexpr float requested_fire_rate_hz = 5.0f;
inline motor::dji::Config frictionHardware(std::size_t i) {
    const auto &c = friction[i];
    return motor::dji::m3508({.id = c.id,
                              .current_limit_a = c.effort_limit,
                              .gear_ratio = c.gear_ratio,
                              .timing = {.feedback_timeout_ms = 30,
                                         .command_timeout_ms = 20,
                                         .enable_timeout_ms = 100,
                                         .retry_interval_ms = 100}});
}
inline motor::dji::Config dialHardware() {
    return motor::dji::m2006({.id = dial.id,
                              .current_limit_a = dial.effort_limit,
                              .gear_ratio = dial.gear_ratio,
                              .timing = {.feedback_timeout_ms = 30,
                                         .command_timeout_ms = 20,
                                         .enable_timeout_ms = 100,
                                         .retry_interval_ms = 100}});
}
inline control::VelocityMotor::Config frictionMotorConfig() {
    control::VelocityMotor::Config c{};
    c.effort_unit = control::EffortUnit::Ampere;
    // 调参：摩擦速度 PID 的输出为 A，误差为输出轴 rad/s；先无负载调内环，再标定前馈。
    // 保留积分 ±0.5 A、输出 ±1 A 与 dt 范围；增益过大会造成振荡或电流饱和。
    c.loop.regulator.feedback = {0.3f, 0.05f, 0.01f, 0, -0.5f, 0.5f, -1, 1, 0, .001f, .02f};
    c.loop.reference_slew = {20, 20}; // 调参：升/降速斜率，rad/s²；过小启动慢，过大冲击和电流增加。
    c.loop.requested_velocity_abs_max_rad_s = std::min(friction[0].velocity_limit_rad_s,
                                                       friction[1].velocity_limit_rad_s);
    // 调参：软件摩擦电流上限，A；仍不能超过 calibration 中的电机保护限值。
    c.loop.effort_abs_max = std::min({1.0f, friction[0].effort_limit, friction[1].effort_limit});
    return c;
}
inline control::PositionMotor::Config dialMotorConfig() {
    control::PositionMotor::Config c{};
    c.effort_unit = control::EffortUnit::Ampere;
    // M2006/C610 has no supported temperature feedback in this driver.
    c.reference = control::PositionReference::DriverContinuous;
    // 调参：拨盘位置外环输入误差 rad、输出速度 rad/s；死区 0.01 rad 应结合分辨率和噪声缩小。
    // 保留原有 PID 标定值；只用于单发/静止保持，连发直接复用下方速度内环。
    c.loop.position = {6.0, 2.0, 0, 0, -4.0, 4.0, -20, 20, .01f, .001f, .02f};
    // 调参：拨盘电流软上限，A；与摩擦轮独立，不能放大以掩盖机构卡滞。
    const float current_limit = std::min(3.0f, dial.effort_limit);
    // 调参：速度内环误差 rad/s、输出 A；kp/ki 与积分限幅控制跟随、静摩擦补偿及饱和。
    c.loop.velocity.regulator.feedback = {.kp = 2.0f,
                                          .ki = 1.0f,
                                          .kd = 0.0f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -current_limit,
                                          .integral_max = current_limit,
                                          .output_min = -20,
                                          .output_max = 20,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    // 调参：拨盘速度参考升/降斜率，rad/s²；同时影响连发停止距离和提前热量预留。
    // 必须保留正值并无弹标定实际刹停；执行器只在正常停连发时使用此受控减速。
    c.loop.velocity.reference_slew = {30, 30};
    c.loop.velocity.requested_velocity_abs_max_rad_s = dial.velocity_limit_rad_s;
    c.loop.velocity.effort_abs_max = current_limit;
    // 调参：gear_ratio、direction、每发分度在 calibration.hpp；装弹前确认真实归零和机械相位。
    return c;
}
inline ShooterExecutor::Config shooterExecutionConfig(bool unloaded_relative = false) {
    ShooterExecutor::Config c{};
    c.friction_speed_rad_s = friction_speed_rad_s;
    c.friction_direction = {friction[0].direction, friction[1].direction};
    c.dial_direction = dial.direction;
    c.dial_step_rad = dial_step_rad;
    c.dial_speed_rad_s = dial_speed_rad_s;
    c.heat_per_round = heat_per_round;
    // 调参：实际位置误差/静止速度/连续稳定时间，rad、rad/s、ms；共同判定单发机械完成。
    c.dial_tolerance_rad = 0.03f;
    c.dial_settle_velocity_rad_s = 0.2f;
    c.dial_settle_ms = 30;
    // 调参：下一分度相位容差 rad；先测无弹停转偏差，过大可能跨过尚未到达的分度。
    c.dial_phase_tolerance_rad = 0.03f;
    // 调参：单发完成/连发无进展及正常减速保护期限，ms；保留超时停机，不自动补发。
    c.jam_timeout_ms = 1500;
    c.dial_brake_timeout_ms = 1500;
    c.allow_relative_dial_reseed = unloaded_relative;
    return c;
}
} // namespace skywalker::robotics::vehicle
