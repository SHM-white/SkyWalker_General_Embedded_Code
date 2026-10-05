#pragma once
namespace skywalker::control {
enum class EffortUnit { Unspecified, Ampere, NewtonMeter };
// A diagnostic describes this axis only; it never revokes a Motor request.
enum class ControlIssue {
    None,
    NotConfigured,
    NotActive,
    FeedbackStale,
    MissingFeedback,
    ReferenceLost,
    InvalidMeasurement,
    InvalidTarget,
    InvalidPeriod,
    OutputRejected,
};
struct ControlCheck {
    ControlIssue issue = ControlIssue::None;
    int error = 0;
};
}
