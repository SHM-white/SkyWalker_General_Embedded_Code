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
static_assert(small_yaw.gear_ratio == 1 && pitch.gear_ratio == 1,
              "Native joint profiles require an explicit external gearbox transform otherwise");
static_assert(yaw_low_ticks < yaw_high_ticks && yaw_high_ticks < 8192 &&
              small_yaw.encoder_zero_ticks < 8192);

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
inline constexpr GimbalAxisConfig yawAxisConfig() {
    return {AxisTopology::Limited, yaw_min_rad, yaw_max_rad,
            small_yaw.velocity_limit_rad_s, true, AxisReferenceInit::CalibratedFeedback};
}
inline constexpr GimbalAxisConfig pitchAxisConfig() {
    return {AxisTopology::Limited, pitch_min_rad, pitch_max_rad,
            pitch.velocity_limit_rad_s, true, AxisReferenceInit::CalibratedFeedback};
}

inline skywalker::control::PositionMotor::Config yawMotorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = skywalker::control::EffortUnit::Ampere;
    c.reference = skywalker::control::PositionReference::DriverContinuous;
    // 调参：先确认速度内环，再改位置外环增益/积分限幅；保持原有初值，
    // 不通过提高 kp 掩盖速度、电流饱和。deadband 单位 rad，0.012约0.69°；
    // 根据静止噪声与机械间隙逐步缩小，不能直接清零来消除微调台阶。
    c.loop.position = {.kp = 20.0f,
                       .ki = 0.5f,
                       .kd = 1.48f,
                       .derivative_tau_s = 0.0f,
                       .integral_min = -0.3f,
                       .integral_max = 0.3f,
                       .output_min = -4.0f,
                       .output_max = 4.0f,
                       .deadband = 0.012f,
                       .dt_min_s = 0.001f,
                       .dt_max_s = 0.020f};
    // 调参：Yaw速度内环。误差单位 rad/s，输出/积分限幅单位 A；
    // 增益、微分滤波和限幅按电流/转速反馈分别调，勿同时改多组参数。
    c.loop.velocity.regulator.feedback = {.kp = 0.33f,
                                          .ki = 0.55f,
                                          .kd = 0.00005f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -1.2f,
                                          .integral_max = 1.2f,
                                          .output_min = -1.2f,
                                          .output_max = 1.2f,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    // 调参：速度参考升/降斜率（rad/s²），决定启动、停转和反向响应。
    c.loop.velocity.reference_slew = {20.0f, 20.0f};
    // 调参：速度反馈滤波时间常数（s），0直通；软死区单位rad/s。
    c.loop.velocity.measurement_filter_tau_s = 0.0f;
    c.loop.velocity.soft_deadband_rad_s = 0.0f;
    // 调参：内环速度/电流上限；与硬件能力协调，鼠标灵敏度独立设置。
    c.loop.velocity.requested_velocity_abs_max_rad_s = 4.0f;
    c.loop.velocity.effort_abs_max = 1.2f;
    return c;
}
inline skywalker::control::PositionMotor::Config pitchMotorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = skywalker::control::EffortUnit::NewtonMeter;
    c.reference = skywalker::control::PositionReference::DriverContinuous;
    // 调参：Pitch位置外环，保持既有PID；deadband=0.01 rad约0.57°。
    // 先记录重力负载、静止噪声和跟踪误差，再逐项调整增益/死区/积分。
    c.loop.position = {.kp = 10.0f,
                       .ki = 0.1f,
                       .kd = 0.0f,
                       .derivative_tau_s = 0.0f,
                       .integral_min = -0.3f,
                       .integral_max = 0.3f,
                       .output_min = -5.0f,
                       .output_max = 5.0f,
                       .deadband = 0.01f,
                       .dt_min_s = 0.001f,
                       .dt_max_s = 0.020f};
    // 调参：Pitch速度内环输出单位N·m；先看速度跟踪与力矩饱和，
    // 再改增益，避免用外环增益补偿内环或机构不足。
    c.loop.velocity.regulator.feedback = {.kp = 0.25f,
                                          .ki = 0.20f,
                                          .kd = 0.0f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -0.7f,
                                          .integral_max = 0.7f,
                                          .output_min = -1.8f,
                                          .output_max = 1.8f,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    // 调参：Pitch升/降斜率（rad/s²）与Yaw独立，过小会增加停转拖尾。
    c.loop.velocity.reference_slew = {8.0f, 8.0f};
    // 调参：反馈滤波时间常数（s），此处0.02不是鼠标输入固定延迟；
    // 软死区（rad/s）影响小速度跟踪，须结合噪声分别调整。
    c.loop.velocity.measurement_filter_tau_s = 0.02f;
    c.loop.velocity.soft_deadband_rad_s = 0.02f;
    // 调参：机构速度/力矩上限，保留原值并按安全机械能力逐步确认。
    c.loop.velocity.requested_velocity_abs_max_rad_s = 5.0f;
    c.loop.velocity.effort_abs_max = 0.8f;
    return c;
}

#if defined(CONFIG_SKYWALKER_COMMAND_SERVICE)
inline CommandManager::Config gimbalCommandConfig() {
    CommandManager::Config c{};
    c.mapper.input_profile = RemoteInputProfile::PhysicalRemote;
    c.max_gimbal_yaw_rate_rad_s = 1.0f;
    c.max_gimbal_pitch_rate_rad_s = 0.8f;
    c.input_timeout_ms = gimbal_command_timeout_ms;
    c.require_referee_for_motion = false;
    c.allow_auto = false;
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
