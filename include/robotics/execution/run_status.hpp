#pragma once
#include <robotics/messages/feedback.hpp>
namespace skywalker::robotics {
enum class RunState : std::uint8_t { Disabled, Recovering, Active, Blocked };
enum class WaitReason : std::uint8_t { None, Command, Transport, Feedback, Reference, Configuration, Drive, Power, Cycle };
struct RunStatus {
    RunState state = RunState::Disabled;
    WaitReason reason = WaitReason::None;
    int error = 0;
    bool ready = false;
    std::uint32_t generation = 0;
    std::uint32_t last_command_sequence = 0;
    // Produced by the execution owner, never refreshed by a reader or sender.
    MessageStamp stamp{};
};
// Explicit mapping preserves the V1 wire enum values. Diagnostics never request an estop.
inline ChassisFeedbackSummary wireFeedback(const RunStatus &s) {
    ChassisFeedbackSummary f{};
    f.execution_state = s.state == RunState::Active ? ExecutionState::Active
                      : s.state == RunState::Blocked ? ExecutionState::ConfigBlocked
                      : s.ready ? ExecutionState::Ready : ExecutionState::Recovering;
    f.safety_state = s.state == RunState::Active ? SafetyState::Active
                   : s.state == RunState::Blocked ? SafetyState::ConfigBlocked
                   : s.ready ? SafetyState::Ready : SafetyState::Waiting;
    f.ready = s.ready;
    f.armed = s.state == RunState::Active;
    switch (s.reason) {
    case WaitReason::Command: f.active_reasons = CommandStale; break;
    case WaitReason::Transport: f.active_reasons = TransportUnavailable; break;
    case WaitReason::Feedback: f.active_reasons = FeedbackStale; break;
    case WaitReason::Reference: f.active_reasons = RecoveryBoundary; break;
    case WaitReason::Cycle: f.active_reasons = RecoveryBoundary; break;
    case WaitReason::Configuration: case WaitReason::Drive: f.active_reasons = InvalidConfiguration; break;
    case WaitReason::Power: f.active_reasons = PowerBudgetStale; break;
    default: break;
    }
    f.last_command_sequence = s.last_command_sequence;
    f.stamp = s.stamp;
    return f;
}
// A live communication thread cannot keep a stopped execution owner ready.
// V1 bytes/enums remain unchanged: stale production maps to its existing
// Waiting state and FeedbackStale reason instead of forwarding old authority.
inline ChassisFeedbackSummary wireFeedback(const RunStatus &s, std::uint64_t now_ms,
                                          std::uint32_t timeout_ms) {
    if (isFresh(s.stamp, now_ms, timeout_ms)) return wireFeedback(s);
    ChassisFeedbackSummary f{};
    f.active_reasons = FeedbackStale;
    return f;
}
}
