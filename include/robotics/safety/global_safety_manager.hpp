#pragma once
#include <robotics/messages/safety.hpp>
namespace skywalker::robotics {
class GlobalSafetyManager {
public:
    struct Config {
        bool require_referee_for_motion = false;
        std::uint32_t permission_timeout_ms = 300;
        std::uint32_t chassis_heartbeat_timeout_ms = 100, chassis_feedback_timeout_ms = 100;
    };
    explicit GlobalSafetyManager(const Config &config) : config_(config) {
    }
    int evaluate(const GlobalSafetyInputs &input, GlobalSafetyDecision &out);
    int clearEmergencyStop(bool estop_input_released);
    bool estopLatched() const {
        return estop_latched_;
    }

private:
    Config config_;
    bool estop_latched_ = false;
    std::uint32_t sequence_ = 0;
};
}
