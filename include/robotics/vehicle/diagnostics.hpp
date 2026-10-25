#pragma once
#include <cstdint>
#include <robotics/vehicle/diagnostics_config.hpp>

namespace skywalker::robotics::vehicle {
enum class DiagnosticReason : std::uint8_t {
    Boot, Ready, InitFailed, RemoteStale, Disarmed, FeedbackStale, ImuStale,
    PeerStale, Configuration, Permission, CommandStale
};
struct DiagnosticStatus {
    DiagnosticReason reason = DiagnosticReason::Boot;
    bool valid = false, enabled = false;
    float target = 0, feedback = 0;
    std::uint32_t age_ms = UINT32_MAX;
    float dt_us = 0, effort = 0, detail = 0;
    int error = 0;
    float extra0 = 0, extra1 = 0;
};
// One application/telemetry thread owns each instance. Output copies into a
// fixed queue; UART transmission runs in a separate low-priority worker.
class Diagnostics {
public:
    int start(const char *application, const char *role, const char *limits = "see application profile");
    void event(const char *step, int result);
    void status(const DiagnosticStatus &);
private:
    const char *application_ = "boot";
    std::uint32_t sequence_ = 0;
    std::uint64_t next_sample_ms_ = 0, next_log_ms_ = 0, transition_ms_ = 0;
    DiagnosticReason last_reason_ = DiagnosticReason::Boot;
    int first_error_ = 0;
};
} // namespace skywalker::robotics::vehicle
