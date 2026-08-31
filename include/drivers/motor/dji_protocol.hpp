#pragma once

#include <cstdint>
#include <zephyr/drivers/can.h>

namespace skywalker::motor::dji {

struct RawFeedback {
    std::uint16_t encoder = 0;
    std::int16_t speed_rpm = 0;
    std::int16_t current_raw = 0;
    std::uint8_t temperature_raw = 0;
    std::uint64_t timestamp_ms = 0;
};

bool decodeFeedback(const struct can_frame &frame, RawFeedback &out);

int buildCommandFrame(struct can_frame &frame,
                      std::uint16_t command_id,
                      const std::int16_t command_raw[4]);

} // namespace skywalker::motor::dji
