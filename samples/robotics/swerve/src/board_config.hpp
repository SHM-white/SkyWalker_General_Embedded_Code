#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <drivers/motor/dji_motor.hpp>
#include <robotics/swerve/swerve_module.hpp>
namespace bench {
inline const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline skywalker::motor::dji::Config steerHardware() {
    return skywalker::motor::dji::gm6020({.id = 1,
                                          .current_limit_a = 0.5f,
                                          .encoder_zero_ticks = 0,
                                          .current_mode_confirmed = true,
                                          .timing = {20, 20, 30, 100}});
}
inline skywalker::motor::dji::Config driveHardware() {
    return skywalker::motor::dji::m3508(
        {.id = 1, .current_limit_a = 0.5f, .gear_ratio = 3591.0f / 187.0f, .timing = {20, 20, 30, 100}});
}
inline constexpr float steer_direction = 1, drive_direction = 1;
inline constexpr float test_speed_m_s = 0.1f;
inline skywalker::robotics::SwerveModule::Config moduleConfig() {
    skywalker::robotics::SwerveModule::Config c{};
    c.wheel_radius_m = 0.05f;
    c.drive.regulator.feedback = {0.03f, 0.1f, 0, 0, -0.3f, 0.3f, -0.3f, 0.3f, 0, .001f, .02f};
    c.drive.reference_slew = {10, 10};
    c.drive.requested_velocity_abs_max_rad_s = 10;
    c.drive.effort_abs_max = .3f;
    c.steer.velocity = c.drive;
    c.steer.position = {3, 0, 0, 0, -10, 10, -10, 10, 0.01f, .001f, .02f};
    return c;
}
}
