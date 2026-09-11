#pragma once

#include <cstdint>

#include <zephyr/device.h>

#include <drivers/motor/dm_protocol.hpp>

namespace skywalker::motor::dm {

enum class Model : std::uint8_t {
    J4310_2EC_V1_1 = 0,
};

struct Descriptor {
    Model model = Model::J4310_2EC_V1_1;
    const struct device *can = nullptr;
    ControlMode mode = ControlMode::Mit;
    std::uint16_t motor_id = 0;
    std::uint16_t master_id = 0;
    std::uint16_t control_id = 0;
    Limits limits{};
    float torque_limit_nm = 0.0f;
};

int describe(const struct device *dev, Descriptor &out);

int setMitCommand(const struct device *dev, const MitCommand &command);

int setPositionVelocity(const struct device *dev, float position_rad, float velocity_limit_rad_s);

int setVelocity(const struct device *dev, float velocity_rad_s);

int readRawFeedback(const struct device *dev, RawFeedback &out);

int getDriveStatus(const struct device *dev, DriveStatus &out);

} // namespace skywalker::motor::dm
