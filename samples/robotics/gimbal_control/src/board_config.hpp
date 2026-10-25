#pragma once

#include <robotics/vehicle/gimbal_profile.hpp>

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

inline constexpr float yaw_direction = -yaw_command_sign;
inline constexpr float pitch_direction = pitch_command_sign;
} // namespace board_config
