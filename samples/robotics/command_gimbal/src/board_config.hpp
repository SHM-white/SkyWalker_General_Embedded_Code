#pragma once
#include <cstdint>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <control/position_motor.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dm_motor.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>
#include <robotics/gimbal/gimbal_executor.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/vehicle/calibration.hpp>
#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
#include <drivers/imu/dm_imu_rs485.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <robotics/gimbal/inertial_gimbal.hpp>
#endif

namespace board_config {
// Set true only after checking BOTH motors, units, direction, zero and limits.
namespace calibration = skywalker::robotics::vehicle;
inline constexpr bool connections_configured = calibration::connections_confirmed;
inline constexpr std::uint32_t command_timeout_ms = 100;
inline constexpr std::uint16_t yaw_encoder_zero_ticks = calibration::small_yaw.encoder_zero_ticks;
inline constexpr std::uint16_t yaw_travel_low_ticks = calibration::yaw_low_ticks;
inline constexpr std::uint16_t yaw_travel_high_ticks = calibration::yaw_high_ticks;
inline constexpr float dji_encoder_ticks_per_turn = 8192.0f;
inline constexpr float two_pi = 6.2831853071795864769f;
inline constexpr float yaw_min_angle_rad = calibration::yaw_min_rad;
inline constexpr float yaw_max_angle_rad = calibration::yaw_max_rad;
// Independently calibrated joint center for the future big-yaw follower.
// This initial value is the safe travel midpoint, NOT the driver encoder zero.
inline constexpr float yaw_center_rad = calibration::yaw_center_rad;
static_assert(yaw_travel_low_ticks < yaw_travel_high_ticks && yaw_travel_high_ticks < 8192 &&
              yaw_encoder_zero_ticks < 8192);
// These native-shaft driver profiles do not implement an external gearbox.
// TODO(hardware): add an explicit output-shaft transform if either joint has one.
static_assert(calibration::small_yaw.gear_ratio == 1 && calibration::pitch.gear_ratio == 1);
#if DT_NODE_HAS_STATUS(DT_ALIAS(remote_uart), okay)
inline const device *remote_uart = DEVICE_DT_GET(DT_ALIAS(remote_uart));
#else
inline const device *remote_uart = nullptr;
#endif
// Attach BOTH motors before starting either bus. One execution-thread owner
// stages all mechanisms and commits each physical bus once per cycle.
inline const device *yaw_can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline const device *pitch_can = DEVICE_DT_GET(DT_NODELABEL(can2));

// Confirm these settings against the installed motors before enabling them.
inline skywalker::motor::dji::Config yawHardware() {
    return skywalker::motor::dji::gm6020({.id = calibration::small_yaw.id,
                                          .current_limit_a = calibration::small_yaw.effort_limit,
                                          .encoder_zero_ticks = yaw_encoder_zero_ticks,
                                          .current_mode_confirmed = connections_configured,
                                          .timing = {.feedback_timeout_ms = 20,
                                                     .command_timeout_ms = 20,
                                                     .enable_timeout_ms = 100,
                                                     .retry_interval_ms = 100}});
}
inline skywalker::motor::dm::Config pitchHardware() {
    return skywalker::motor::dm::j4310Mit({.id = calibration::pitch.id,
                                           .master_id = calibration::pitch.master_id,
                                           .position_max_rad = 12.5f,
                                           .velocity_max_rad_s = 30.0f,
                                           .torque_max_nm = 10.0f,
                                           .torque_limit_nm = calibration::pitch.effort_limit,
                                           .timing = {.feedback_timeout_ms = 50,
                                                      .command_timeout_ms = 20,
                                                      .enable_timeout_ms = 3000,
                                                      .retry_interval_ms = 100}});
}

// ManualCommandMapper already maps right_x to negative yaw and right_y to
// positive pitch, matching the source gimbal_control calibration. These signs
// map the resulting mechanical command into the calibrated driver coordinates.
inline constexpr float yaw_command_sign = calibration::small_yaw.direction,
                       pitch_command_sign = calibration::pitch.direction;
// Limited axes use calibrated driver coordinates, not a startup-relative zero.
// Yaw and pitch use the coordinates calibrated in gimbal_control.
inline constexpr skywalker::robotics::GimbalAxisConfig yaw{skywalker::robotics::AxisTopology::Limited,
                                                           yaw_min_angle_rad,
                                                           yaw_max_angle_rad,
                                                           calibration::small_yaw.velocity_limit_rad_s,
                                                           true,
                                                           skywalker::robotics::AxisReferenceInit::CalibratedFeedback};
// Pitch range is synchronized with the current gimbal_control bench configuration.
inline constexpr skywalker::robotics::GimbalAxisConfig
    pitch{skywalker::robotics::AxisTopology::Limited,
          calibration::pitch_min_rad,
          calibration::pitch_max_rad,
          calibration::pitch.velocity_limit_rad_s,
          true,
          skywalker::robotics::AxisReferenceInit::CalibratedFeedback};

// 调参：仅鼠标/发射台架启用的关节目标领先上限（rad）。初值约为
// Yaw 8.6°、Pitch 5.7°，需结合目标-反馈误差及停止拖尾逐轴标定。
// 超出上限的鼠标输入会被丢弃；过小会减弱跟踪/保持，0 表示关闭。
// 不修改上面的通用轴配置，也不将这项关节限幅用于惯性角目标。
inline constexpr float shooter_yaw_max_lead_rad = 0.15f;
inline constexpr float shooter_pitch_max_lead_rad = 0.10f;
inline skywalker::robotics::GimbalAxisConfig shooterYawAxisConfig() {
    auto c = yaw;
    c.max_lead_rad = shooter_yaw_max_lead_rad;
    return c;
}
inline skywalker::robotics::GimbalAxisConfig shooterPitchAxisConfig() {
    auto c = pitch;
    c.max_lead_rad = shooter_pitch_max_lead_rad;
    return c;
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

inline const skywalker::robotics::CommandManager::Config command_policy = [] {
    skywalker::robotics::CommandManager::Config c{};
    c.mapper.input_profile = skywalker::robotics::RemoteInputProfile::PhysicalRemote;
    c.max_gimbal_yaw_rate_rad_s = 1.0f;
    c.max_gimbal_pitch_rate_rad_s = 0.8f;
    c.input_timeout_ms = command_timeout_ms;
    c.require_referee_for_motion = IS_ENABLED(CONFIG_COMMAND_GIMBAL_REFEREE);
    c.allow_auto = IS_ENABLED(CONFIG_COMMAND_GIMBAL_VISION_EXECUTE);
    // TODO(reference): AB carries no epoch metadata; negotiate a new reference
    // session with the vision producer after IMU reset before enabling Auto.
    c.expected_vision_reference = calibration::head_reference;
    return c;
}();
inline skywalker::robotics::CommandManager::Config shooterCommandPolicy() {
    auto c = command_policy;
    // 台架专用：左拨杆 Down=Safe、Middle=RC、Up=键鼠；不改其他应用Auto。
    c.mapper.input_profile = skywalker::robotics::RemoteInputProfile::ShooterSelectable;
    c.require_external_run_gate = true;
    c.allow_auto = false;
    c.require_referee_for_motion = false;
    // 调参：连点间隔严格小于500 ms；达到500 ms无新点击则撤销连发。
    // 长按250 ms是未实机确认的初值，与连点门槛独立。
    c.mouse_gesture.rapid_click_gap_ms = 500;
    c.mouse_gesture.hold_to_auto_ms = 250;
    // 调参：按DR16速度量合同映射，单位为(rad/s)/原始输入单位。
    // 透传量尚未确认是真实鼠标counts；固定DPI/客户端/视场角后标定。
    // 保留旧未饱和区比例，不随下面的机构限速变化，不叠加鼠标滤波。
    c.mapper.mouse_yaw_rate_per_unit = 0.002f;
    c.mapper.mouse_pitch_rate_per_unit = 0.0016f;
    // 调参：独立的操作角速度上限（rad/s）；提高前先确认两轴机械能力。
    c.max_gimbal_yaw_rate_rad_s = 1.0f;
    c.max_gimbal_pitch_rate_rad_s = 0.8f;
    // 统一射频由发射台架用vehicle配置赋值；RC、快速连点与长按共用。
    return c;
}
inline const skywalker::robotics::GimbalExecutor::Config execution_policy{
    .command_timeout_ms = command_timeout_ms,
    .source_timeout_us = command_timeout_ms * 1000ULL,
    .permission_timeout_ms = 300,
    .max_cycle_us = 20000,
};

#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
inline const device *head_uart = DEVICE_DT_GET(DT_ALIAS(rs485_2));
inline constexpr skywalker::imu::ImuReceiver::Config head_receiver{.poll_interval_us = 1000, .priority = 6};
inline skywalker::imu::DmImuRs485Source::Config headSensor() {
    skywalker::imu::DmImuRs485Source::Config c{};
    c.protocol = {1, 20000};
    c.reference = calibration::head_reference;
    c.sensor_to_body = calibration::head_sensor_to_body;
    // TODO(IMU): confirm quaternion direction/scales and vendor quality policy.
    return c;
}
inline skywalker::robotics::InertialGimbalAdapter::Config inertialPolicy() {
    skywalker::robotics::InertialGimbalAdapter::Config c{};
    c.yaw = yaw;
    c.pitch = pitch;
    c.yaw_direction = yaw_command_sign;
    c.pitch_direction = pitch_command_sign;
    c.pitch_locked = false;
    c.allow_unknown_quality = true;
    // TODO(control): validate inertial_gimbal before opening vision execution.
    return c;
}
#endif

// TODO(hardware): connect physical estop/reset. RC disable is recoverable.
inline bool emergencyStopRequested() {
    return false;
}
inline bool takeEmergencyResetRequest() {
    return false;
}
} // namespace board_config
