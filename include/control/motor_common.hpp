#pragma once

namespace skywalker::control {

enum class EffortUnit { Unspecified, Ampere, NewtonMeter };

struct MotorSafety {
    float velocity_abs_max_rad_s = 0.0f;
    // Zero disables temperature cutoff. A positive value requires temperature feedback.
    float temperature_max_c = 0.0f;
};

} // namespace skywalker::control
