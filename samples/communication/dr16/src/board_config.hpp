#pragma once
#include <zephyr/device.h>
#include <communication/remote/remote_service.hpp>
namespace bench {
inline const device *remote_uart = DEVICE_DT_GET(DT_ALIAS(remote_uart));
inline constexpr skywalker::communication::Dr16Decoder::Config decoder{
    .channel_center = 1024,
    .channel_min = 364,
    .channel_max = 1684,
    .center_deadband = 0,
    .decode_wheel = false,
};
inline constexpr skywalker::communication::RemoteService::Config remote{100, 10};
}
