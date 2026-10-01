#pragma once
#include "input_sources.hpp"
#include <lib/vofa/vofa.h>

namespace bench {
class Telemetry {
public:
    int start(const device *vofa_uart);
    void emit(const InputFrame &, const skywalker::robotics::CommandDecision &);

private:
    Vofa vofa_{};
    bool vofa_ready_ = false;
    std::uint64_t next_log_ms_ = 0, next_vofa_ms_ = 0;
    std::uint32_t vofa_rejected_ = 0;
    int last_vofa_error_ = 0;
};
}
