#pragma once
#include <cstdint>
namespace skywalker::control {
enum class ExecutionState : std::uint8_t { Waiting, Recovering, Ready, Active, EStopLatched, ConfigBlocked };
enum class PauseReason : std::uint8_t {
    OperatorDisabled,
    RefereeDisabled,
    CommandTimeout,
    FeedbackTimeout,
    TransportTemporary,
    InvalidCycle,
    EmergencyStop
};
}
