#pragma once
#include <robotics/vehicle/calibration.hpp>
#include <zephyr/kernel.h>
namespace skywalker::samples::chassis {
// Deadline is in kernel ticks, as is core::monotonicTimeUs. Late iterations
// skip missed periods instead of issuing a burst of near-zero-dt updates.
class PeriodicDeadline {
public:
    explicit PeriodicDeadline(core::TimeUs period_us = robotics::vehicle::chassis_period_us)
        : deadline_(k_uptime_ticks()), period_(k_us_to_ticks_ceil64(period_us)) {
    }
    void wait() {
        deadline_ += period_;
        const auto now = k_uptime_ticks();
        if (deadline_ <= now)
            deadline_ += ((now - deadline_) / period_ + 1) * period_;
        k_sleep(K_TIMEOUT_ABS_TICKS(deadline_));
    }

private:
    std::int64_t deadline_, period_;
};
}
