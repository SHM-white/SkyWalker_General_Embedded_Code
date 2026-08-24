#pragma once

#include <cstdint>
#include <zephyr/device.h>

#include <drivers/motor/dji_protocol.hpp>

namespace skywalker::motor::dji::m3508
{
    int setCurrentRaw(const struct device *dev, std::int16_t current_raw);
    int getCurrentRaw(const struct device *dev, std::int16_t &out);
    int readRawFeedback(const struct device *dev, M3508Feedback &out);
    int getCommandInfo(const struct device *dev, std::uint16_t &command_id, std::uint8_t &command_slot);
} // namespace skywalker::motor::dji::m3508
