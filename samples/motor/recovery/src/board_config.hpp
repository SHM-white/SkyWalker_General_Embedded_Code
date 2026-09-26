#pragma once

#include <control/velocity_motor.hpp>

namespace bench {
#ifdef SKYWALKER_RECOVERY_DM
inline constexpr bool dm = true;
#else
inline constexpr bool dm = false;
#endif
inline constexpr float target_velocity_rad_s = 2.0f;

inline skywalker::control::VelocityMotor::Config motorConfig() {
    skywalker::control::VelocityMotor::Config config{};
    config.effort_unit = dm ? skywalker::control::EffortUnit::NewtonMeter : skywalker::control::EffortUnit::Ampere;
    config.safety = {12.0f, 70.0f};
    config.loop.regulator.feedback = {.kp = .03f,
                                      .ki = .1f,
                                      .kd = 0.0f,
                                      .derivative_tau_s = 0.0f,
                                      .integral_min = -.3f,
                                      .integral_max = .3f,
                                      .output_min = -.3f,
                                      .output_max = .3f,
                                      .deadband = 0.0f,
                                      .dt_min_s = .001f,
                                      .dt_max_s = .02f};
    config.loop.reference_slew = {5.0f, 5.0f};
    config.loop.requested_velocity_abs_max_rad_s = 6.0f;
    config.loop.effort_abs_max = .3f;
    return config;
}
}
