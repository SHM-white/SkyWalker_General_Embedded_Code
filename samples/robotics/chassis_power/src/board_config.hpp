#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "../../common/chassis_can.hpp"
namespace bench {
inline const device *steer_can = skywalker::samples::chassis::steer_can;
inline const device *drive_can = skywalker::samples::chassis::drive_can;
// TODO(wiring/profile): confirm referee User UART and RM2026 protocol version.
inline const device *referee_uart = DEVICE_DT_GET(DT_ALIAS(referee_uart));
}
