#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
struct GlobalSafetyInputs {
    std::uint64_t now_ms = 0;
    bool command_source_fresh = false, operator_motion_enabled = false, emergency_stop_requested = false;
    OutputPermission gimbal_power{}, chassis_power{}, shooter_power{};
    MessageStamp chassis_heartbeat_stamp{}, chassis_feedback_stamp{};
    ExecutionState chassis_execution_state = ExecutionState::Waiting;
    std::uint32_t chassis_active_reasons = 0;
};
struct GlobalSafetyDecision {
    SafetyAction gimbal = SafetyAction::Disable, chassis = SafetyAction::Disable, shooter = SafetyAction::Disable;
    SafetyState state = SafetyState::Waiting;
    std::uint32_t active_reasons = 0;
    MessageStamp stamp{};
};
struct LocalSafetyInputs {
    std::uint64_t now_ms = 0, receiver_boot_id = 0, command_boot_id = 0;
    std::uint32_t resume_generation = 0, command_generation = 0;
    MessageStamp peer_heartbeat_stamp{}, command_stamp{};
    SafetyAction global_action = SafetyAction::Disable;
    bool power_allowed = false, feedback_fresh = false, hardware_ready = false, armed = false;
    bool config_valid = true, emergency_stop_requested = false;
};
struct LocalSafetyDecision {
    SafetyAction action = SafetyAction::Disable;
    ExecutionState state = ExecutionState::Waiting;
    std::uint32_t active_reasons = 0;
};
}
