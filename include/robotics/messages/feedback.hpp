#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
struct ChassisFeedbackSummary {
    ExecutionState execution_state = ExecutionState::Waiting;
    SafetyState safety_state = SafetyState::Waiting;
    bool ready = false, armed = false;
    std::uint32_t active_reasons = 0, last_command_sequence = 0, valid_fields = 0;
    float vx_m_s = 0, vy_m_s = 0, wz_rad_s = 0, power_w = 0;
    MessageStamp stamp{};
};
}
