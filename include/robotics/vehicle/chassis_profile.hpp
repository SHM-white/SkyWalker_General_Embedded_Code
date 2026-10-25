#pragma once

#include <robotics/chassis/chassis_executor.hpp>
#include <robotics/vehicle/swerve_profile.hpp>

namespace skywalker::robotics::vehicle {
// Pure installed profiles. The application owns every motor and bus.
inline motor::dji::Config steerHardware(std::size_t index) {
    const auto &c = steer[index];
    return motor::dji::gm6020({.id = c.id,
                              .current_limit_a = c.effort_limit,
                              .encoder_zero_ticks = c.encoder_zero_ticks,
                              .current_mode_confirmed = connections_confirmed,
                              .timing = {.feedback_timeout_ms = 20,
                                         .command_timeout_ms = 20,
                                         .enable_timeout_ms = 100,
                                         .retry_interval_ms = 100}});
}
inline motor::dji::Config wheelHardware(std::size_t index) {
    const auto &c = wheel[index];
    return motor::dji::m3508({.id = c.id,
                             .current_limit_a = c.effort_limit,
                             .gear_ratio = c.gear_ratio,
                             .timing = {.feedback_timeout_ms = 20,
                                        .command_timeout_ms = 20,
                                        .enable_timeout_ms = 100,
                                        .retry_interval_ms = 100}});
}
inline SwerveHardware::Config swerveHardwareConfig() {
    SwerveHardware::Config c{};
    c.hardware_confirmed = connections_confirmed;
    for (std::size_t i = 0; i < 4; ++i) {
        c.directions[i] = steer[i].direction;
        c.directions[i + 4] = wheel[i].direction;
    }
    return c;
}
inline ChassisExecutor::Config chassisExecutionConfig() {
    ChassisExecutor::Config c{};
    // Manual low-current operation; measured power remains a separate capability.
    c.require_power_budget = false;
    c.power_model_calibrated = power_model_calibrated;
    c.power_control_calibrated = false;
    c.allow_estimated_power = false;
    c.estimate_idle_power_w = power_idle_w;
    c.estimate_w_per_abs_amp = power_per_abs_amp_w;
    c.bench_effort_scale = chassis_drive_scale;
    c.bench_steer_effort_scale = chassis_steer_scale;
    c.bench_drive_current_limit_a = drive_bench_current_a;
    c.bench_steer_current_limit_a = steer_bench_current_a;
    return c;
}
} // namespace skywalker::robotics::vehicle
