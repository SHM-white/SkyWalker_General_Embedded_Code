#pragma once

#include <cstdint>

namespace skywalker::motor {

struct Timing {
    // 最后一次反馈后允许的最长间隔，单位 ms；超时后反馈视为过期。
    std::uint32_t feedback_timeout_ms = 20;
    // 已发布运动命令的最长有效时间，单位 ms；超时后请求安全输出。
    std::uint32_t command_timeout_ms = 10;
    // 反馈恢复后须持续稳定的时间，单位 ms，之后电机才可进入就绪状态。
    std::uint32_t recovery_stable_ms = 20;
    // 从请求使能到完成使能流程的最长等待时间，单位 ms。
    std::uint32_t enable_timeout_ms = 100;
};

} // namespace skywalker::motor
