#pragma once
#include <cstdint>
#include <array>
#include <limits>
#include <robotics/vehicle/diagnostics_config.hpp>

namespace skywalker::robotics::vehicle {
enum class DiagnosticReason : std::uint8_t {
    Boot,
    Ready,
    InitFailed,
    RemoteStale,
    Disarmed,
    FeedbackStale,
    ImuStale,
    PeerStale,
    Configuration,
    Permission,
    CommandStale,
    Aligning,
    CycleInvalid,
    TransportError
};
struct DiagnosticStatus {
    DiagnosticReason reason = DiagnosticReason::Boot;
    bool valid = false, enabled = false;
    float target = 0, feedback = 0;
    std::uint32_t age_ms = UINT32_MAX;
    float dt_us = 0, effort = 0, detail = 0;
    int error = 0;
    float extra0 = 0, extra1 = 0;
    // I19..I48. I19: 101/105 chassis, 102 legacy attitude gimbal, 103 legacy
    // planar-yaw gimbal, 104 dual-Yaw planar gimbal, 106 head-IMU inertial dual-Yaw.
    // Legacy callers keep the original 19-channel frame. The extension length is
    // a multiple of three so the text output can group three channels per line.
    bool extended = false;
    std::array<float, 30> extension = [] {
        std::array<float, 30> values{};
        values.fill(std::numeric_limits<float>::quiet_NaN());
        return values;
    }();
};
// One application/telemetry thread owns each instance. Output copies into a
// fixed queue; UART transmission runs in a separate low-priority worker.
#if defined(CONFIG_SKYWALKER_VEHICLE_DIAGNOSTICS)
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
#else
class Diagnostics {
public:
    int start(const char *, const char *, const char * = "see application profile") {
        return 0;
    }
    void event(const char *, int) {
    }
    void status(const DiagnosticStatus &) {
    }
};
#endif
} // namespace skywalker::robotics::vehicle
