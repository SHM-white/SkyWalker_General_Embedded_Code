#pragma once
namespace skywalker::robotics::vehicle::bringup {
// Conservative stage policy; installed hardware/PID/calibration stay shared.
inline constexpr float yaw_encoder_rate_rad_s = 0.2f;
inline constexpr float pitch_encoder_rate_rad_s = 0.15f;
inline constexpr float big_yaw_rate_rad_s = 0.1f;
} // namespace skywalker::robotics::vehicle::bringup
