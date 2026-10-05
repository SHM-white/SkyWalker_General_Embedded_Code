#pragma once

#include <cstdint>

namespace skywalker::motor {

struct Timing {
    // Maximum age of actual device feedback, in milliseconds.
    std::uint32_t feedback_timeout_ms = 20;
    // A publication never refreshes the age of its producer's command.
    std::uint32_t command_timeout_ms = 10;
    // Maximum duration of one enable/clear operation.
    std::uint32_t enable_timeout_ms = 100;
    // Per-endpoint interval between protocol retries.
    std::uint32_t retry_interval_ms = 100;
};

} // namespace skywalker::motor
