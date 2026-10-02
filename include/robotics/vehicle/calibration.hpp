#pragma once

#include <array>
#include <cstdint>
#include <core/measurement.hpp>

namespace skywalker::robotics::vehicle {
// One calibration source for the staged samples and the two vehicle roles.
// TODO(hardware): replace template values using the installed robot's records.
// Keep confirmation flags false until the corresponding bench has passed.
inline constexpr bool connections_confirmed = false;
inline constexpr bool imu_mounting_confirmed = false;
inline constexpr bool power_model_calibrated = false;
inline constexpr bool shooter_constraints_confirmed = false;

struct MotorCalibration {
    std::uint8_t id = 0;
    std::uint16_t master_id = 0;
    float gear_ratio = 1, direction = 1;
    float effort_limit = 0.5f, velocity_limit_rad_s = 10;
    std::uint16_t encoder_zero_ticks = 0;
};

// TODO(calibration): ID/mode, current-mode firmware, output-shaft units and signs.
inline constexpr MotorCalibration small_yaw{7, 0, 1, 1, 1.5f, 4, 5670};
inline constexpr MotorCalibration pitch{1, 0x11, 1, 1, 0.5f, 5, 0};
inline constexpr MotorCalibration big_yaw{2, 0x12, 1, 1, 0.5f, 1, 0};
inline constexpr std::array<MotorCalibration, 2> friction{{
    {1, 0, 19, 1, 2, 60, 0}, {2, 0, 19, -1, 2, 60, 0},
}};
inline constexpr MotorCalibration dial{3, 0, 36, 1, 1, 10, 0};
inline constexpr std::array<MotorCalibration, 4> steer{{
    {1, 0, 1, 1, 1, 6, 0}, {2, 0, 1, 1, 1, 6, 0},
    {3, 0, 1, 1, 1, 6, 0}, {4, 0, 1, 1, 1, 6, 0},
}};
inline constexpr std::array<MotorCalibration, 4> wheel{{
    {1, 0, 19, 1, 2, 60, 0}, {2, 0, 19, 1, 2, 60, 0},
    {3, 0, 19, 1, 2, 60, 0}, {4, 0, 19, 1, 2, 60, 0},
}};

inline constexpr float two_pi = 6.2831853071795864769f;
inline constexpr std::uint16_t yaw_low_ticks = 5770, yaw_high_ticks = 7340;
inline constexpr float yaw_min_rad = (yaw_low_ticks - small_yaw.encoder_zero_ticks) * two_pi / 8192;
inline constexpr float yaw_max_rad = (yaw_high_ticks - small_yaw.encoder_zero_ticks) * two_pi / 8192;
// TODO(calibration): measure this independently; encoder zero is not joint center.
inline constexpr float yaw_center_rad = (yaw_min_rad + yaw_max_rad) * 0.5f;
inline constexpr float pitch_min_rad = -0.5f, pitch_max_rad = 0.5f;
inline constexpr float wheel_radius_m = 0.05f, wheelbase_m = 0.4f, track_m = 0.4f;

// TODO(IMU): measure sensor-to-mechanical-body rotations, then establish epochs
// on each boot/reference reset; equal frame IDs do not synchronize two clocks.
inline constexpr core::Quaternion chassis_sensor_to_body{}, carrier_sensor_to_body{}, head_sensor_to_body{};
inline constexpr core::OrientationReference chassis_reference{1, 1}, carrier_reference{2, 1}, head_reference{3, 1};
inline constexpr float power_idle_w = 5, power_per_abs_amp_w = 8;
// TODO(shooter): calibrate wheel speed -> projectile speed, dial angle per round,
// speed tolerance, dwell, jam current/time, heat per round and cooling data.
inline constexpr float friction_speed_rad_s = 40, dial_step_rad = two_pi / 8;
inline constexpr float heat_per_round = 10;
} // namespace skywalker::robotics::vehicle
