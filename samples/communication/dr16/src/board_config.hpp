#pragma once
#include <zephyr/device.h>
#include <communication/remote/remote_service.hpp>
namespace bench {
static_assert(DT_NODE_HAS_COMPAT(DT_ALIAS(telemetry_uart), zephyr_cdc_acm_uart),
              "DR16 USB telemetry requires telemetry-uart = &cdc_acm_uart0; check overlays and rebuild pristine");
inline const device *remote_uart = DEVICE_DT_GET(DT_ALIAS(remote_uart));
inline const device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
inline constexpr skywalker::communication::Dr16Decoder::Config decoder{
    .channel_center = 1024,
    .channel_min = 364,
    .channel_max = 1684,
    .center_deadband = 0,
    .decode_wheel = true,
};
inline constexpr skywalker::communication::RemoteService::Config remote{100, 10};
}
