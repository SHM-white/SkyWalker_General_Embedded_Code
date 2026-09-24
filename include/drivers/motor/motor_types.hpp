#pragma once

#include <cstdint>

namespace skywalker::motor {

struct Timing {
    std::uint32_t feedback_timeout_ms = 20;
    std::uint32_t command_timeout_ms = 10;
    std::uint32_t recovery_stable_ms = 20;
    std::uint32_t enable_timeout_ms = 100;
};

} // namespace skywalker::motor
