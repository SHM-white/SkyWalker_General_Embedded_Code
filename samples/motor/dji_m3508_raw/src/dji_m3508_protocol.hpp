#pragma once

#include <cstdint>
#include <zephyr/drivers/can.h>

namespace skywalker::motor::dji
{
    struct M3508Feedback
    {
        std::uint16_t encoder = 0;
        std::int16_t rpm = 0;
        std::int16_t current_raw = 0;
        std::uint8_t temperature = 0;
    };

    bool decodeM3508Feedback(const struct can_frame &frame, M3508Feedback &out);
    void buildGroupCurrentFrame(struct can_frame &frame, std::uint16_t command_id, const std::int16_t current[4]);
}