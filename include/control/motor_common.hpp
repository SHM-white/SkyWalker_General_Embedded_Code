#pragma once

namespace skywalker::control {

enum class EffortUnit { Unspecified, Ampere, NewtonMeter };

// Legacy controllers stop their motor; sessions own the stop decision instead.
enum class ControlFailurePolicy { StopMotor, ReportOnly };
enum class ControlIssue {
    None, NotConfigured, NotActive, FeedbackStale, MissingFeedback, ReferenceLost,
    InvalidMeasurement, SpeedLimit, TemperatureLimit, InvalidTarget, InvalidPeriod,
    OutputRejected,
};
struct ControlCheck {
    ControlIssue issue = ControlIssue::None;
    int error = 0;
};

struct MotorSafety {
    float velocity_abs_max_rad_s = 0.0f;
    // Zero disables temperature cutoff. A positive value requires temperature feedback.
    float temperature_max_c = 0.0f;
};

} // namespace skywalker::control
