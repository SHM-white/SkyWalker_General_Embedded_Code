#pragma once

#include <robotics/vehicle/gimbal_profile.hpp>

#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
#include <drivers/imu/dm_imu_rs485.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <robotics/gimbal/inertial_gimbal.hpp>
#endif

namespace board_config {
namespace calibration = skywalker::robotics::vehicle;
inline constexpr bool connections_configured = calibration::connections_confirmed;
inline constexpr std::uint32_t command_timeout_ms = calibration::gimbal_command_timeout_ms;
inline constexpr std::uint16_t yaw_encoder_zero_ticks = calibration::small_yaw.encoder_zero_ticks;
inline constexpr std::uint16_t yaw_travel_low_ticks = calibration::yaw_low_ticks;
inline constexpr std::uint16_t yaw_travel_high_ticks = calibration::yaw_high_ticks;
inline constexpr float dji_encoder_ticks_per_turn = 8192.0f;
inline constexpr float two_pi = calibration::two_pi;
inline constexpr float yaw_min_angle_rad = calibration::yaw_min_rad;
inline constexpr float yaw_max_angle_rad = calibration::yaw_max_rad;
inline constexpr float yaw_center_rad = calibration::yaw_center_rad;
#if DT_NODE_HAS_STATUS(DT_ALIAS(remote_uart), okay)
inline const device *remote_uart = DEVICE_DT_GET(DT_ALIAS(remote_uart));
#else
inline const device *remote_uart = nullptr;
#endif
inline const device *yaw_can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline const device *pitch_can = DEVICE_DT_GET(DT_NODELABEL(can2));
using calibration::yawHardware;
using calibration::pitchHardware;
using calibration::yawMotorConfig;
using calibration::pitchMotorConfig;
inline constexpr auto yaw = calibration::yawAxisConfig();
inline constexpr auto pitch = calibration::pitchAxisConfig();
inline constexpr float yaw_command_sign = calibration::small_yaw.direction;
inline constexpr float pitch_command_sign = calibration::pitch.direction;

// Connect physical estop/reset inputs here. RC disable remains recoverable.
inline bool emergencyStopRequested() { return false; }
inline bool takeEmergencyResetRequest() { return false; }
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

inline const skywalker::robotics::CommandManager::Config command_policy = [] {
    auto c = calibration::gimbalCommandConfig();
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
inline const auto execution_policy = calibration::gimbalExecutionConfig();

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

} // namespace board_config
