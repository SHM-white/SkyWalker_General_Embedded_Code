#pragma once
#include <variant>
#include <robotics/command/command_inputs.hpp>

namespace skywalker::robotics {
enum class SourceRole : std::uint8_t { Operator, Aim };
using SourceValue = std::variant<RemoteState, core::Measurement<communication::vision::AimCommand>>;
struct SourceDiagnostics {
    int error = 0, sample_error = 0;
    std::uint32_t state = 0, dropped = 0;
    bool dropped_available = false;
};
struct SourceSample {
    SourceValue value{};
    SourceDiagnostics diagnostics{};
};
// Static lifetime: sources/receivers must outlive the manager, even on errors.
// Startup thread calls start once; only the manager worker calls sample.
class ICommandSource {
public:
    virtual ~ICommandSource() = default;
    virtual SourceRole role() const = 0; // Fixed for this object's lifetime.
    virtual int start() = 0; // 0 schedules reception, not necessarily online.
    // Bounded, preserve original timestamps. 0 writes a complete sample (which
    // may be offline/invalid); -EAGAIN retains cache; other errors invalidate it.
    virtual int sample(SourceSample &out) = 0;
};
// Permission is a constraint, not a competing motion source.
class IPermissionSource {
public:
    virtual ~IPermissionSource() = default;
    virtual int start() = 0;
    // Same sample/cache contract; now_ms is local monotonic time.
    virtual int sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics) = 0;
};
struct CommandSnapshot {
    CommandInputs observed{};
    CommandDecision decision{};
    SourceDiagnostics remote{}, vision{}, permission{};
};
}
