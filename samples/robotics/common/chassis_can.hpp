#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
namespace skywalker::samples::chassis {
// Reference chassis harness: one CanBus owner per physical controller.
inline const device *const steer_can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline const device *const drive_can = DEVICE_DT_GET(DT_NODELABEL(can3));
inline const device *const big_yaw_can = DEVICE_DT_GET(DT_NODELABEL(can2));
}
