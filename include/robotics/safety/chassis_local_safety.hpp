#pragma once
#include <robotics/messages/safety.hpp>
namespace skywalker::robotics {
class ChassisLocalSafety {
public:
    struct Config {
        std::uint32_t command_timeout_ms = 100, heartbeat_timeout_ms = 100, stable_command_count = 3;
    };
    explicit ChassisLocalSafety(const Config &config) : config_(config) {
    }
    int evaluate(const LocalSafetyInputs &, LocalSafetyDecision &out);
    int clearEmergencyStop(bool released);

private:
    Config config_;
    bool estop_latched_ = false, have_sequence_ = false;
    std::uint32_t sequence_ = 0, count_ = 0, generation_ = 0;
    std::uint64_t boot_id_ = 0;
    MessageStamp last_command_{};
};
}
