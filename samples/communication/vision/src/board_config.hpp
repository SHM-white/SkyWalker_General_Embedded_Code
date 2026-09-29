#pragma once
#include <communication/vision/ab_protocol.hpp>
#include <communication/vision/vision_receiver.hpp>
#include <zephyr/device.h>
namespace bench {
inline const device *vision_uart = DEVICE_DT_GET(DT_ALIAS(vision_uart));
inline const device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
inline constexpr skywalker::communication::vision::AbProtocol::Config protocol{.command_reference = {1, 1},
                                                                               .assembly_timeout_us = 20000};
inline constexpr skywalker::communication::vision::VisionReceiver::Config receiver{};
}
