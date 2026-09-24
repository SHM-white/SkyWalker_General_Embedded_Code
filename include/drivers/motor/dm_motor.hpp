#pragma once

#include <cstdint>

#include <drivers/motor/dm_protocol.hpp>
#include <drivers/motor/motor_types.hpp>

namespace skywalker::motor::dm {

enum class Model : std::uint8_t {
    J4310_2EC_V1_1 = 0,
};

struct J4310Options {
    std::uint8_t id = 0;
    std::uint16_t master_id = 0;
    float position_max_rad = 0.0f;
    float velocity_max_rad_s = 0.0f;
    float torque_max_nm = 0.0f;
    float torque_limit_nm = 0.0f;
    Timing timing{50, 20, 50, 3000};
};

struct Config {
    Model model = Model::J4310_2EC_V1_1;
    ControlMode mode = ControlMode::Mit;
    std::uint8_t id = 0;
    std::uint16_t master_id = 0;
    Limits limits{};
    float torque_limit_nm = 0.0f;
    Timing timing{50, 20, 50, 3000};
};

Config j4310Mit(const J4310Options &options);
Config j4310Velocity(const J4310Options &options);
Config j4310PositionVelocity(const J4310Options &options);

struct Descriptor {
    Model model = Model::J4310_2EC_V1_1;
    ControlMode mode = ControlMode::Mit;
    std::uint16_t motor_id = 0;
    std::uint16_t master_id = 0;
    std::uint16_t control_id = 0;
    Limits limits{};
    float torque_limit_nm = 0.0f;
};

int describe(const Config &config, Descriptor &out);

} // namespace skywalker::motor::dm
