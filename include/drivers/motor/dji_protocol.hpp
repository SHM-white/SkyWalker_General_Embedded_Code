#pragma once

#include <cstdint>
#include <zephyr/drivers/can.h>

namespace skywalker::motor::dji
{
    enum class Model : std::uint8_t {
        M3508C620 = 0,
        M2006C610,
        GM6020Current,
    };

    struct RawFeedback {
        std::uint16_t encoder = 0;
        std::int16_t speed_rpm = 0;
        std::int16_t current_raw = 0;
        std::uint8_t temperature_raw = 0;
        std::uint64_t timestamp_ms = 0;
    };

    struct Descriptor {
        Model model = Model::M3508C620;
        const struct device *can = nullptr;
        std::uint8_t motor_id = 0;
        std::uint16_t feedback_id = 0;
        std::uint16_t command_id = 0;
        std::uint8_t command_slot = 0;
        float protocol_current_max_a = 0.0f;
        float configured_current_limit_a = 0.0f;
        float gear_ratio = 1.0f;
        bool temperature_valid = false;
    };

    int describe(const struct device *dev, Descriptor &out);
    int readRawFeedback(const struct device *dev, RawFeedback &out);


}