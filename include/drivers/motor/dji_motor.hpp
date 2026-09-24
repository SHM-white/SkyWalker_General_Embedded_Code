#pragma once

#include <cstdint>
#include <zephyr/device.h>

#include <drivers/motor/dji_protocol.hpp>
#include <drivers/motor/motor_types.hpp>

namespace skywalker::motor::dji {

enum class Model : std::uint8_t {
    M3508C620 = 0,
    M2006C610,
    GM6020Current,
};

struct Gm6020Options {
    std::uint8_t id = 0;
    float current_limit_a = 0.0f;
    std::uint16_t encoder_zero_ticks = 0;
    bool current_mode_confirmed = false;
    Timing timing{};
};

struct M3508Options {
    std::uint8_t id = 0;
    float current_limit_a = 0.0f;
    float gear_ratio = 19.0f;
    Timing timing{};
};

struct M2006Options {
    std::uint8_t id = 0;
    float current_limit_a = 0.0f;
    float gear_ratio = 36.0f;
    Timing timing{};
};

struct Config {
    Model model = Model::M3508C620;
    std::uint8_t id = 0;
    float current_limit_a = 0.0f;
    float gear_ratio = 1.0f;
    std::uint16_t encoder_zero_ticks = 0;
    bool current_mode_confirmed = false;
    Timing timing{};
};

Config gm6020(const Gm6020Options &options);
Config m3508(const M3508Options &options);
Config m2006(const M2006Options &options);

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
int describe(const Config &config, Descriptor &out);
// Non-armed only. Re-seed continuous coordinates from a fresh encoder sample.
// Fixed-zero 1:1 axes start at calibrated absolute position, other axes at zero.
int resetMeasurementReference(const struct device *dev);
int readRawFeedback(const struct device *dev, RawFeedback &out);

} // namespace skywalker::motor::dji
