#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "../../common/chassis_can.hpp"
namespace bench {
// TODO(wiring): confirm MC02 CAN connector numbering and both terminations.
inline const device *steer_can = skywalker::samples::chassis::steer_can;
inline const device *drive_can = skywalker::samples::chassis::drive_can;
}
