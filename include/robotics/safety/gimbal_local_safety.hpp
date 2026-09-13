#pragma once
#include <robotics/messages/safety.hpp>
namespace skywalker::robotics {
class GimbalLocalSafety {
public:
    struct Config {
        std::uint32_t command_timeout_ms = 100;
    };
    explicit GimbalLocalSafety(const Config &config) : config_(config) {
    }
    int evaluate(const LocalSafetyInputs &, LocalSafetyDecision &out);
    int clearEmergencyStop(bool released);

private:
    Config config_;
    bool estop_latched_ = false;
};
}
