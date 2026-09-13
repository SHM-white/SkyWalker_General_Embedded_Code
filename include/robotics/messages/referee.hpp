#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
struct RefereePowerState {
    float chassis_power_w = 0, chassis_power_limit_w = 0, buffer_energy_j = 0;
    MessageStamp stamp{}; // Measured power/buffer, never refreshed by robot status.
    MessageStamp limit_stamp{};
};
struct RefereeRobotState {
    std::uint8_t robot_id = 0;
    OutputPermission chassis_output{}, gimbal_output{}, shooter_output{};
};
struct RefereeState {
    RefereeRobotState robot{};
    RefereePowerState power{};
    MessageStamp stamp{};
    bool online = false;
};
}
