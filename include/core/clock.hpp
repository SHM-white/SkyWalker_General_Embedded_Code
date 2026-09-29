#pragma once
#include <core/measurement.hpp>
#include <zephyr/kernel.h>
namespace skywalker::core {
// Microsecond units; actual resolution is the configured kernel tick period.
inline TimeUs monotonicTimeUs() {
    return k_ticks_to_us_floor64(k_uptime_ticks());
}
}
