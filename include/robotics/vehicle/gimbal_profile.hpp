#pragma once

#include <cstdint>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <control/position_motor.hpp>
#include <drivers/imu/dm_imu_can.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dm_motor.hpp>
#if defined(CONFIG_SKYWALKER_COMMAND_SERVICE)
#include <robotics/command/command_manager.hpp>
#endif
#include <robotics/gimbal/gimbal_axis.hpp>
#include <robotics/gimbal/gimbal_executor.hpp>
#include <robotics/vehicle/calibration.hpp>

namespace skywalker::robotics::vehicle {
// Values and side-effect-free configuration factories only. Applications own
// their devices, receivers, motor instances, startup order and execution loops.
inline constexpr std::uint32_t gimbal_command_timeout_ms = 100;
inline constexpr core::TimeUs gimbal_period_us = 5000;
// 视觉 Rate 桥接的速度前馈增益（docs/dev/云台MPC轨迹跟踪响应优化实施指南.md 第 8.2 节）。
// 0 只接通数据、不改变响应；0.25/0.5 为逐轮实验值，先确认 gain=0 记录中的参考连续性与
// 合成速度限幅未频繁介入，再按轴、按轮提高。1.0 不是必须达到的目标，yaw 与 pitch 分开评估。
inline constexpr float yaw_velocity_feedforward_gain = 0.5f;
// Pitch 几何追赶参考尚无实机连续性基线；先保留桥接观测，关闭额外速度前馈。
inline constexpr float pitch_velocity_feedforward_gain = 0.0f;
static_assert(small_yaw.gear_ratio == 1 && pitch.gear_ratio == 1,
              "Native joint profiles require an explicit external gearbox transform otherwise");
static_assert(yaw_low_ticks < yaw_high_ticks && yaw_high_ticks < 8192 && small_yaw.encoder_zero_ticks < 8192);

inline motor::dji::Config yawHardware() {
    return motor::dji::gm6020({.id = small_yaw.id,
                               .current_limit_a = small_yaw.effort_limit,
                               .encoder_zero_ticks = small_yaw.encoder_zero_ticks,
                               .current_mode_confirmed = connections_confirmed,
                               .timing = {.feedback_timeout_ms = 20,
                                          .command_timeout_ms = 20,
                                          .enable_timeout_ms = 100,
                                          .retry_interval_ms = 100}});
}
inline motor::dm::Config pitchHardware() {
    return motor::dm::j4310Mit({.id = pitch.id,
                                .master_id = pitch.master_id,
                                .position_max_rad = 12.5f,
                                .velocity_max_rad_s = 30.0f,
                                .torque_max_nm = 10.0f,
                                .torque_limit_nm = pitch.effort_limit,
                                .timing = {.feedback_timeout_ms = 50,
                                           .command_timeout_ms = 20,
                                           .enable_timeout_ms = 3000,
                                           .retry_interval_ms = 100}});
}
// 小 Yaw 机械/执行配置，字段顺序与 GimbalAxisConfig 一致：
//   {topology, min_angle_rad, max_angle_rad, max_rate_rad_s, hold_on_zero_rate, reference_init}
//   - min/max_angle_rad：软限位端点，来自 calibration.hpp 的 yaw_min/max_rad（由 ticks 换算）
//   - max_rate_rad_s：关节限速，取 GM6020 输出轴标定
//   - hold_on_zero_rate：Rate 目标为 0 时保持当前位置，而不是松开
//   - reference_init：用标定反馈重建编码器参考（上电即按绝对反馈对齐）
inline constexpr GimbalAxisConfig yawAxisConfig() {
    return {AxisTopology::Limited,          yaw_min_rad, yaw_max_rad,
            small_yaw.velocity_limit_rad_s, true,        AxisReferenceInit::CalibratedFeedback};
}
// Pitch 同理：限位/限速取自 calibration.hpp 的 pitch_min/max_rad 与 pitch 标定。
inline constexpr GimbalAxisConfig pitchAxisConfig() {
    return {AxisTopology::Limited,      pitch_min_rad, pitch_max_rad,
            pitch.velocity_limit_rad_s, true,          AxisReferenceInit::CalibratedFeedback};
}

inline skywalker::control::PositionMotor::Config yawMotorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = skywalker::control::EffortUnit::Ampere;
    c.reference = skywalker::control::PositionReference::DriverContinuous;
    // 调参：先确认速度内环，再改位置外环增益/积分限幅；保持原有初值，
    // 不通过提高 kp 掩盖速度、电流饱和。deadband 单位 rad，0.012约0.69°；
    // 根据静止噪声与机械间隙逐步缩小，不能直接清零来消除微调台阶。
    c.loop.position = {.kp = 35.0f,
                       .ki = 0.5f,
                       .kd = 3.6f,
                       .derivative_tau_s = 0.01f,
                       .integral_min = -0.6f,
                       .integral_max = 0.6f,
                       .output_min = -4.0f,
                       .output_max = 4.0f,
                       .deadband = 0.012f,
                       .dt_min_s = 0.001f,
                       .dt_max_s = 0.020f};
    // 调参：Yaw速度内环。误差单位 rad/s，输出/积分限幅单位 A；
    // 增益、微分滤波和限幅按电流/转速反馈分别调，勿同时改多组参数。
    c.loop.velocity.regulator.feedback = {.kp = 0.35f,
                                          .ki = 0.4f,
                                          .kd = 0.0f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -0.6f,
                                          .integral_max = 0.6f,
                                          .output_min = -3.0f,
                                          .output_max = 3.0f,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    // 调参：速度参考升/降斜率（rad/s²），决定启动、停转和反向响应。
    c.loop.velocity.reference_slew = {40.0f, 40.0f};
    // 调参：速度反馈滤波时间常数（s），0直通；软死区单位rad/s。
    c.loop.velocity.measurement_filter_tau_s = 0.0f;
    c.loop.velocity.soft_deadband_rad_s = 0.0f;
    // 调参：内环速度/电流上限；与硬件能力协调，鼠标灵敏度独立设置。
    c.loop.velocity.requested_velocity_abs_max_rad_s = 15.0f;
    c.loop.velocity.effort_abs_max = 3.0f;
    // 视觉 Rate 桥接：只在本轴确实由视觉 Rate 路径驱动时生效，默认增益见文件顶部。
    c.loop.velocity_feedforward_gain = yaw_velocity_feedforward_gain;
    // 视觉模式外置头部 IMU 陀螺速度内环（绕过 GM6020 1-RPM 量化与偏置）。
    c.has_external_velocity = true;
    c.external_velocity.regulator.feedback = {.kp = 1.2f,
                                              .ki = 1.5f,
                                              .kd = 0.0f,
                                              .derivative_tau_s = 0.0f,
                                              .integral_min = -0.45f,
                                              .integral_max = 0.45f,
                                              .output_min = -3.0f,
                                              .output_max = 3.0f,
                                              .deadband = 0.0f,
                                              .dt_min_s = 0.001f,
                                              .dt_max_s = 0.020f};
    c.external_velocity.reference_slew = {80.0f, 80.0f};
    c.external_velocity.measurement_filter_tau_s = 0.003f;
    c.external_velocity.soft_deadband_rad_s = 0.0f;
    c.external_velocity.requested_velocity_abs_max_rad_s = 10.0f;
    c.external_velocity.effort_abs_max = small_yaw.effort_limit;
    c.external_acceleration_effort_gain = 0.0f;
    return c;
}
inline skywalker::control::PositionMotor::Config pitchMotorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = skywalker::control::EffortUnit::NewtonMeter;
    c.reference = skywalker::control::PositionReference::DriverContinuous;
    // 调参：Pitch位置外环，保持既有PID；deadband=0.005 rad约0.29°。
    // 第一轮只缩小死区，其余参数与轨迹保持基线，先确认平台残差与静止噪声再动 D。
    c.loop.position = {.kp = 30.0f,
                       .ki = 0.6f,
                       .kd = 2.0f,
                       .derivative_tau_s = 0.01f,
                       .integral_min = -1.2f,
                       .integral_max = 1.2f,
                       .output_min = -5.0f,
                       .output_max = 5.0f,
                       .deadband = 0.005f,
                       .dt_min_s = 0.001f,
                       .dt_max_s = 0.020f};
    // 调参：Pitch速度内环输出单位N·m；先看速度跟踪与力矩饱和，
    // 再改增益，避免用外环增益补偿内环或机构不足。
    c.loop.velocity.regulator.feedback = {.kp = 0.6f,
                                          .ki = 0.8f,
                                          .kd = 0.005f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -2.0f,
                                          .integral_max = 2.0f,
                                          .output_min = -6.0f,
                                          .output_max = 6.0f,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    // 调参：Pitch升/降斜率（rad/s²）与Yaw独立，过小会增加停转拖尾。
    c.loop.velocity.reference_slew = {10.0f, 10.0f};
    // 调参：反馈滤波时间常数（s），此处0.02不是鼠标输入固定延迟；
    // 软死区（rad/s）影响小速度跟踪，须结合噪声分别调整。
    c.loop.velocity.measurement_filter_tau_s = 0.02f;
    c.loop.velocity.soft_deadband_rad_s = 0.02f;
    // 调参：机构速度/力矩上限，保留原值并按安全机械能力逐步确认。
    c.loop.velocity.requested_velocity_abs_max_rad_s = 6.0f;
    c.loop.velocity.effort_abs_max = 5.0f;
    // 视觉 Rate 桥接：Pitch 参考由几何追赶生成，先以零增益取得连续性基线；
    // 记录中若 v_ref 反复跳变，保持增益为 0 并补受限关节参考。
    c.loop.velocity_feedforward_gain = pitch_velocity_feedforward_gain;
    // 视觉模式外置头部 IMU 陀螺速度内环 + 水平余弦重力前馈（+0.92 N·m 对应电机正方向抬头）。
    c.has_external_velocity = true;
    c.external_velocity.regulator.feedback = {.kp = 1.8f,
                                              .ki = 2.5f,
                                              .kd = 0.0f,
                                              .derivative_tau_s = 0.0f,
                                              .integral_min = -1.2f,
                                              .integral_max = 1.2f,
                                              .output_min = -5.0f,
                                              .output_max = 5.0f,
                                              .deadband = 0.0f,
                                              .dt_min_s = 0.001f,
                                              .dt_max_s = 0.020f};
    c.external_velocity.regulator.feedforward = {.k_bias = 0.0f,
                                                 .k_static = 0.0f,
                                                 .k_velocity = 0.0f,
                                                 .k_acceleration = 0.0f,
                                                 .k_gravity = 0.92f,
                                                 .velocity_epsilon = 0.0f,
                                                 .acceleration_epsilon = 0.0f,
                                                 .gravity_model = CONTROL_GRAVITY_COS};
    c.external_velocity.reference_slew = {40.0f, 40.0f};
    c.external_velocity.measurement_filter_tau_s = 0.005f;
    c.external_velocity.soft_deadband_rad_s = 0.0f;
    c.external_velocity.requested_velocity_abs_max_rad_s = 6.0f;
    c.external_velocity.effort_abs_max = 5.0f;
    c.external_acceleration_effort_gain = 0.0f;
    return c;
}

#if defined(CONFIG_SKYWALKER_COMMAND_SERVICE)
inline constexpr CommandManager::Config gimbalCommandConfig() {
    CommandManager::Config c{};
    c.mapper.input_profile = RemoteInputProfile::PhysicalRemote;
    c.max_gimbal_yaw_rate_rad_s = manual_max_gimbal_yaw_rate_rad_s;
    c.max_gimbal_pitch_rate_rad_s = manual_max_gimbal_pitch_rate_rad_s;
    c.input_timeout_ms = gimbal_command_timeout_ms;
    c.require_referee_for_chassis = false;
    c.require_referee_for_gimbal = false;
    c.require_referee_for_shooter = false;
    // 发射机构先接线：手动档允许键鼠产生摩擦轮/发射请求，仲裁器据此产出
    // ShooterCommand（含单发事件戳）。摩擦轮/拨盘执行端与热量门控另行接入。
    c.enable_shooter = true;
    c.allow_auto = vision_execute_confirmed && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER) &&
                   DT_HAS_ALIAS(vision_uart);
    c.expected_vision_reference = head_reference;
    return c;
}
#endif
inline GimbalExecutor::Config gimbalExecutionConfig() {
    return {.command_timeout_ms = gimbal_command_timeout_ms,
            .source_timeout_us = gimbal_command_timeout_ms * 1000ULL,
            .permission_timeout_ms = 300,
            .max_cycle_us = 20000};
}
inline imu::DmImuCanSource::Config headSensorConfig() {
    imu::DmImuCanSource::Config c{};
#if DT_NODE_HAS_STATUS(DT_ALIAS(imu_can), okay)
    c.can = DEVICE_DT_GET(DT_ALIAS(imu_can));
#else
    c.can = DEVICE_DT_GET(DT_NODELABEL(can3));
#endif
    // The installed head IMU is a dedicated CAN3 quaternion reporter.
    // Changing transport/mode is a hardware-baseline change, not a stage option.
    c.feedback_id = 0x11;
    c.slave_id = 1;
    c.mode = imu::DmImuCanSource::Mode::ActiveReport;
    c.attitude = imu::DmImuCanSource::Attitude::Quaternion;
    c.reference = head_reference;
    c.sensor_to_body = head_sensor_to_body;
    return c;
}
} // namespace skywalker::robotics::vehicle
