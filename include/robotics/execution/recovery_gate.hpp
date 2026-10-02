#pragma once

#include <cerrno>
#include <limits>
#include <core/measurement.hpp>
#include <robotics/command/command_source.hpp>
#include <robotics/execution/run_status.hpp>

namespace skywalker::robotics {

// A command's arbitration sequence is not evidence of a newly produced input.
// Preserve the selected producer's time/sequence across cache and EAGAIN reads.
inline core::Stamp sourceStamp(const CommandSnapshot &frame, ControlSource source) {
    if (source == ControlSource::Remote || source == ControlSource::KeyboardMouse) {
        const auto &s = frame.observed.remote.stamp;
        return {s.timestamp_ms * 1000, s.sequence, s.valid && frame.observed.remote.online};
    }
    if (source == ControlSource::Vision)
        return frame.decision.selected_vision.stamp;
    return {};
}

// Single execution-thread owner. Each mechanism owns a separate gate, including
// mechanisms sharing a physical CAN. This class authorizes a lifecycle; it never
// writes a motor, clears a target, prepares a reference or requests an enable.
class RecoveryGate {
public:
    struct Config {
        std::uint32_t command_timeout_ms = 100;
        core::TimeUs source_timeout_us = 100000;
    };
    enum class Stage : std::uint8_t {
        WaitingPrerequisites, WaitingCommand, Enabling, Active, Blocked
    };

    explicit RecoveryGate(const Config &config) : config_(config) {}

    // The caller must suspend output and erase its old target on every revoke.
    // Clearing a fault does not erase this mechanism's recovery generation.
    void withdraw(WaitReason reason, int error = 0, bool blocked = false) {
        stage_ = blocked ? Stage::Blocked : Stage::WaitingPrerequisites;
        reason_ = reason;
        error_ = error;
        boundary_us_ = 0;
    }

    // Call only after prerequisites, reference preparation and control-history
    // reset have all completed. Repeated waiting cycles must not call this again.
    void prepared(std::uint64_t now_ms) {
        if (stage_ != Stage::WaitingPrerequisites) return;
        if (!config_.command_timeout_ms || !config_.source_timeout_us) {
            withdraw(WaitReason::Configuration, -EINVAL, true);
            return;
        }
        if (generation_ == std::numeric_limits<std::uint32_t>::max()) {
            withdraw(WaitReason::Configuration, -EOVERFLOW, true);
            return;
        }
        ++generation_;
        boundary_us_ = now_ms * 1000;
        stage_ = Stage::WaitingCommand;
        reason_ = WaitReason::Command;
        error_ = 0;
    }

    // Run in every cycle, also while the drive's enable handshake is pending.
    // Both the manager publication and original selected input must remain fresh.
    // Millisecond source resolution intentionally rejects a same-tick input at
    // the recovery boundary. A genuinely later input authorizes a new target.
    bool accept(const MessageStamp &command, const core::Stamp &source, std::uint64_t now_ms) {
        if (stage_ == Stage::WaitingPrerequisites || stage_ == Stage::Blocked) return false;
        const bool fresh = config_.command_timeout_ms && config_.source_timeout_us &&
            isFresh(command, now_ms, config_.command_timeout_ms) &&
            core::fresh(source, now_ms * 1000, config_.source_timeout_us) &&
            command.timestamp_ms > boundary_us_ / 1000 &&
            source.time_us > boundary_us_;
        if (!fresh) {
            if (stage_ == Stage::Enabling || stage_ == Stage::Active)
                withdraw(WaitReason::Command, -ESTALE);
            else {
                reason_ = WaitReason::Command;
                error_ = -ESTALE;
            }
            return false;
        }
        if (stage_ == Stage::WaitingCommand) stage_ = Stage::Enabling;
        reason_ = WaitReason::None;
        error_ = 0;
        return true;
    }

    void enabled() {
        if (stage_ == Stage::Enabling) stage_ = Stage::Active;
    }

    Stage stage() const { return stage_; }
    std::uint32_t generation() const { return generation_; }
    core::TimeUs boundaryUs() const { return boundary_us_; }
    WaitReason reason() const { return reason_; }
    int error() const { return error_; }

private:
    Config config_;
    Stage stage_ = Stage::WaitingPrerequisites;
    WaitReason reason_ = WaitReason::Command;
    int error_ = 0;
    std::uint32_t generation_ = 0;
    core::TimeUs boundary_us_ = 0;
};

} // namespace skywalker::robotics
