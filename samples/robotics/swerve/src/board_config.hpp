#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "../../common/chassis_can.hpp"
#include <drivers/motor/dji_motor.hpp>
#include <robotics/swerve/swerve_module.hpp>
#include "../../common/chassis_bench.hpp"
namespace bench {
// TODO(wiring): confirm actual split topology; steer and wheel use separate CAN.
inline const device *steer_can = skywalker::samples::chassis::steer_can;
inline const device *drive_can = skywalker::samples::chassis::drive_can;
inline constexpr bool hardware_confirmed = skywalker::robotics::vehicle::connections_confirmed;
inline skywalker::motor::dji::Config steerHardware() {
    return skywalker::samples::chassis::steerConfig(0);
}
inline skywalker::motor::dji::Config driveHardware() {
    return skywalker::samples::chassis::driveConfig(0);
}
inline constexpr float steer_direction = skywalker::robotics::vehicle::steer[0].direction;
inline constexpr float drive_direction = skywalker::robotics::vehicle::wheel[0].direction;
inline constexpr float test_speed_m_s = 0.1f;
inline skywalker::robotics::SwerveModule::Config moduleConfig() {
    return skywalker::samples::chassis::controllerConfig().modules[0];
}
}
