#pragma once
#include <robotics/messages/command.hpp>
#include <robotics/messages/feedback.hpp>
namespace skywalker::robotics {
enum class BoardRole : std::uint8_t { Unknown, GimbalController, ChassisController };
struct BoardHeartbeat {
    BoardRole role = BoardRole::Unknown;
    SafetyState safety_state = SafetyState::Waiting;
    std::uint32_t active_reasons = 0, sender_uptime_ms = 0;
    std::uint64_t sender_boot_id = 0;
    std::uint32_t resume_generation = 0;
    bool ready = false, sync_requested = false;
    MessageStamp stamp{};
};
struct RemoteChassisControl {
    ChassisCommand command{};
    SafetyAction global_action = SafetyAction::Disable;
    std::uint32_t active_reasons = 0;
    std::uint64_t receiver_boot_id = 0;
    std::uint32_t resume_generation = 0;
    MessageStamp stamp{};
};
struct ChassisConstraint {
    OutputPermission output{};
    bool power_valid = false;
    float power_limit_w = 0, buffer_energy_j = 0;
    std::uint32_t output_age_ms = 0, power_age_ms = 0;
    MessageStamp stamp{};
};
inline bool forwardedFresh(const MessageStamp &s, std::uint32_t age_ms, std::uint64_t now_ms,
                           std::uint32_t timeout_ms) {
    return isFresh(s, now_ms, timeout_ms) && age_ms <= timeout_ms && now_ms - s.timestamp_ms <= timeout_ms - age_ms;
}
}
