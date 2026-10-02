#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
namespace bench {
inline const device *steer_can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline const device *drive_can = DEVICE_DT_GET(DT_NODELABEL(can2));
// TODO(wiring/profile): confirm referee User UART and RM2026 protocol version.
inline const device *referee_uart = DEVICE_DT_GET(DT_ALIAS(referee_uart));
}
