#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
namespace bench {
// TODO(wiring): confirm MC02 CAN connector numbering and both terminations.
inline const device *steer_can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline const device *drive_can = DEVICE_DT_GET(DT_NODELABEL(can2));
}
