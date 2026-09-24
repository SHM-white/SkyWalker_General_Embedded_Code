#pragma once
#include <cstdint>
#include <control/motor_lifecycle.hpp>

namespace skywalker::robotics {
enum class ControlSource : std::uint8_t { None, Remote, KeyboardMouse, Vision, Autonomous };
struct MessageStamp {
    // 消息产生或接收时的系统时间，单位 ms。
    std::uint64_t timestamp_ms = 0;
    // 消息序号，用于识别新消息和重复消息。
    std::uint32_t sequence = 0;
    // 时间戳和消息内容是否可用。
    bool valid = false;
};
inline bool isFresh(const MessageStamp &s, std::uint64_t now_ms, std::uint32_t timeout_ms) {
    return s.valid && now_ms >= s.timestamp_ms && now_ms - s.timestamp_ms <= timeout_ms;
}
// RFC-style serial arithmetic; exactly half a sequence space is ambiguous.
inline bool sequenceAfter(std::uint32_t next, std::uint32_t previous) {
    const auto delta = next - previous;
    return delta != 0 && delta < 0x80000000u;
}
enum class SafetyAction : std::uint8_t { Disable, Hold, Active };
enum class SafetyState : std::uint8_t { Boot, Waiting, Ready, Active, Degraded, EmergencyStop, ConfigBlocked };
using ExecutionState = control::ExecutionState;
using PauseReason = control::PauseReason;
enum SafetyReason : std::uint32_t {
    NoReason = 0,
    OperatorDisabled = 1u << 0,
    CommandStale = 1u << 1,
    PowerDisabled = 1u << 2,
    PowerStale = 1u << 3,
    FeedbackStale = 1u << 4,
    TransportUnavailable = 1u << 5,
    EmergencyStop = 1u << 6,
    InvalidConfiguration = 1u << 7,
    RecoveryBoundary = 1u << 8,
    PowerBudgetStale = 1u << 9
};
struct OutputPermission {
    // 输出许可数据是否有效。
    bool valid = false;
    // 对应机构的电源输出是否获准开启。
    bool enabled = false;
    // 输出许可的时间戳与序号。
    MessageStamp stamp{};
};
} // namespace skywalker::robotics
