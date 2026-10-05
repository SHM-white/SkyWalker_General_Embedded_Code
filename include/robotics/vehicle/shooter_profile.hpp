#pragma once
#include <algorithm>
#include <drivers/motor/dji_motor.hpp>
#include <robotics/shooter/shooter_executor.hpp>
#include <robotics/vehicle/calibration.hpp>
namespace skywalker::robotics::vehicle {
inline motor::dji::Config frictionHardware(std::size_t i) {
    const auto &c = friction[i];
    return motor::dji::m3508({.id = c.id,
                              .current_limit_a = c.effort_limit,
                              .gear_ratio = c.gear_ratio,
                              .timing = {.feedback_timeout_ms = 30,
                                         .command_timeout_ms = 20,
                                         .enable_timeout_ms = 100,
                                         .retry_interval_ms = 100}});
}
inline motor::dji::Config dialHardware() {
    return motor::dji::m2006({.id = dial.id,
                              .current_limit_a = dial.effort_limit,
                              .gear_ratio = dial.gear_ratio,
                              .timing = {.feedback_timeout_ms = 30,
                                         .command_timeout_ms = 20,
                                         .enable_timeout_ms = 100,
                                         .retry_interval_ms = 100}});
}
inline control::VelocityMotor::Config frictionMotorConfig() {
    control::VelocityMotor::Config c{};
    c.effort_unit = control::EffortUnit::Ampere;
    // TODO(control): identify feedforward and tune gains on unloaded wheels.
    c.loop.regulator.feedback = {0.03f, 0.05f, 0, 0, -0.5f, 0.5f, -1, 1, 0, .001f, .02f};
    c.loop.reference_slew = {20, 20};
    c.loop.requested_velocity_abs_max_rad_s = std::min(friction[0].velocity_limit_rad_s,
                                                       friction[1].velocity_limit_rad_s);
    c.loop.effort_abs_max = std::min({1.0f, friction[0].effort_limit, friction[1].effort_limit});
    return c;
}
inline control::PositionMotor::Config dialMotorConfig() {
    control::PositionMotor::Config c{};
    c.effort_unit = control::EffortUnit::Ampere;
    // M2006/C610 has no supported temperature feedback in this driver.
    c.reference = control::PositionReference::DriverContinuous;
    c.loop.position = {3, 0, 0, 0, -2, 2, -2, 2, .01f, .001f, .02f};
    c.loop.velocity = frictionMotorConfig().loop;
    c.loop.velocity.requested_velocity_abs_max_rad_s = dial.velocity_limit_rad_s;
    c.loop.velocity.effort_abs_max = std::min(0.5f, dial.effort_limit);
    // TODO(dial): establish indexing/home and gear ratio before feeding rounds.
    return c;
}
inline ShooterExecutor::Config shooterExecutionConfig(bool unloaded_relative = false) {
    ShooterExecutor::Config c{};
    c.friction_speed_rad_s = friction_speed_rad_s;
    c.friction_direction = {friction[0].direction, friction[1].direction};
    c.dial_direction = dial.direction;
    c.dial_step_rad = dial_step_rad;
    c.heat_per_round = heat_per_round;
    c.allow_relative_dial_reseed = unloaded_relative;
    return c;
}
} // namespace skywalker::robotics::vehicle
