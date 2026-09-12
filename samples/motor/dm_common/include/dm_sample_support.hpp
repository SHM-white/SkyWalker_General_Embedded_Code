#pragma once

#include <zephyr/device.h>

#include <drivers/motor/dm_bus.hpp>
#include <drivers/motor/dm_motor.hpp>
#include <drivers/motor/motor.hpp>

namespace skywalker::samples::dm {

struct Session {
    skywalker::motor::dm::Bus bus{};
    const struct device *motor = nullptr;
    skywalker::motor::dm::Descriptor descriptor{};
};

int prepare(Session &session, const struct device *motor);

int arm(Session &session);

// Allows fresh Disabled feedback while the bus is Safe (controller startup).
// Once armed, requires Enabled feedback; freshness and limits always apply.
int readSafeFeedback(const Session &session, float velocity_abs_max_rad_s, float temperature_max_c,
                     skywalker::motor::Feedback &feedback, skywalker::motor::dm::RawFeedback &raw);

int flush(Session &session);

int stop(Session &session);

int stopAfterFailure(Session &session, int original_error);

} // namespace skywalker::samples::dm
