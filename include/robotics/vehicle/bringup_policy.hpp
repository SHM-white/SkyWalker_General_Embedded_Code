#pragma once
namespace skywalker::robotics::vehicle::bringup {
// Conservative stage policy; installed hardware/PID/calibration stay shared.
inline constexpr float yaw_encoder_rate_rad_s = 4.0f;
inline constexpr float pitch_encoder_rate_rad_s = 1.20f;
inline constexpr float big_yaw_rate_rad_s = 2.0f;
} // namespace skywalker::robotics::vehicle::bringup
