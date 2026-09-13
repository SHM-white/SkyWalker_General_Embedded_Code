#pragma once
#include <zephyr/device.h>
#include <communication/referee/referee_protocol.hpp>
namespace bench {
inline const device *referee_uart = DEVICE_DT_GET(DT_ALIAS(referee_uart));
// Confirm the actual referee firmware. Select Unspecified to observe frame counters only.
inline constexpr auto version = skywalker::communication::RefereeVersion::Rm2026V1_3;
inline constexpr std::uint32_t offline_timeout_ms = 500, permission_timeout_ms = 300;
}
