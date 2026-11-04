#pragma once

#include <drivers/led/led_pattern.hpp>
#include <robotics/vehicle/diagnostics.hpp>

namespace skywalker::robotics::vehicle {

// Presentation only: do not grant motion permission based on LED state.
// A real emergency must be supplied explicitly; "disarmed" is not EStop.
inline led::LedStatus statusLedFor(DiagnosticReason reason, bool active,
                                   bool emergency_stop = false, int execution_error = 0) {
    using led::LedStatus;
    if (emergency_stop)
        return LedStatus::EStop;
    if (execution_error < 0 || reason == DiagnosticReason::InitFailed ||
        reason == DiagnosticReason::Configuration)
        return LedStatus::Fault;
    switch (reason) {
    case DiagnosticReason::Ready:
        return active ? LedStatus::Running : LedStatus::Ready;
    case DiagnosticReason::RemoteStale:
    case DiagnosticReason::PeerStale:
        return LedStatus::LinkLost;
    case DiagnosticReason::FeedbackStale:
    case DiagnosticReason::ImuStale:
    case DiagnosticReason::CommandStale:
        return LedStatus::Recovering;
    case DiagnosticReason::Disarmed:
    case DiagnosticReason::Permission:
        return LedStatus::Disarmed;
    case DiagnosticReason::Boot:
        return LedStatus::Boot;
    default:
        return LedStatus::Initializing;
    }
}

} // namespace skywalker::robotics::vehicle
