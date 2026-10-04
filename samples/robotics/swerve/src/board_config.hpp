#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <drivers/motor/dji_motor.hpp>
#include <robotics/vehicle/swerve_profile.hpp>

namespace bench {
// This single-module harness is independent of the full chassis calibration.
inline const device *steer_can = DEVICE_DT_GET(DT_NODELABEL(can2));
inline const device *drive_can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline constexpr bool hardware_confirmed = true; // CAN2 GM6020 ID2, CAN1 M3508 ID2.
// Assumes GM6020 current-mode firmware; this driver does not support voltage mode.
inline constexpr bool steer_current_mode = true;
inline constexpr bool capture_startup_zero = false;
// h7_framework-main app_robot.c @ 05e53b3: original ID2 (front right).
inline constexpr std::uint16_t steer_zero_ticks = 2421;
inline constexpr float steer_direction = 1.0f, drive_direction = 1.0f;
inline constexpr float drive_gear_ratio = 3591.0f / 187.0f;
// Reference uses rotor_speed = command_speed * 1 / 0.01 (empirical scale).
// This driver reports output-shaft speed = rotor_speed / drive_gear_ratio.
// The equivalent radius preserves that scale WITHOUT applying the reduction twice.
// This is not a measured physical wheel radius. Replace with the real radius
// when changing from reference command units to calibrated metres per second.
inline constexpr float wheel_radius_m = 0.01f * drive_gear_ratio;
inline constexpr float steer_current_limit_a = 0.8f, drive_current_limit_a = 0.5f;
inline constexpr float test_speed_m_s = 0.1f;
// Match the working dji_speed_control sample without changing other chassis samples.
inline constexpr skywalker::core::TimeUs control_period_us = 5000;
static_assert(control_period_us >= 1000 && control_period_us < 20000);

inline skywalker::motor::dji::Config steerHardware() {
    return skywalker::motor::dji::gm6020({.id = 2, .current_limit_a = steer_current_limit_a,
        .encoder_zero_ticks = capture_startup_zero ? std::uint16_t{0} : steer_zero_ticks,
        .current_mode_confirmed = steer_current_mode, .timing = {20, 20, 30, 100}});
}
inline skywalker::motor::dji::Config driveHardware() {
    return skywalker::motor::dji::m3508({.id = 2, .current_limit_a = drive_current_limit_a,
        .gear_ratio = drive_gear_ratio, .timing = {20, 20, 30, 100}});
}
inline skywalker::robotics::SwerveModule::Config moduleConfig() {
    // Reuse the existing controller tuning, overriding this bench's mechanics.
    auto c = skywalker::robotics::vehicle::swerveConfig().modules[0];
    c.wheel_radius_m = wheel_radius_m;
    c.steer.velocity.effort_abs_max = steer_current_limit_a;
    c.steer.velocity.regulator.feedback.output_min = -steer_current_limit_a;
    c.steer.velocity.regulator.feedback.output_max = steer_current_limit_a;
    c.drive.effort_abs_max = drive_current_limit_a;
    c.drive.regulator.feedback.output_min = -drive_current_limit_a;
    c.drive.regulator.feedback.output_max = drive_current_limit_a;
    c.drive.regulator.feedback.kp = skywalker::robotics::vehicle::drive_rotor_velocity_kp * drive_gear_ratio;
    return c;
}
inline skywalker::robotics::SwerveKinematics::Config kinematicsConfig() {
    skywalker::robotics::SwerveKinematics::Config c{};
    // A single module has no chassis lever arm; only the stick's XY vector matters.
    c.locations = {};
    c.max_wheel_velocity_m_s = test_speed_m_s;
    c.stationary_epsilon_m_s = 0.01f;
    c.resume_velocity_m_s = 0.02f;
    return c;
}
} // namespace bench
