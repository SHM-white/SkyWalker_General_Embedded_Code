#pragma once
#include <robotics/command/receiver_sources.hpp>
#include <communication/vision/ab_protocol.hpp>
#include <robotics/command/command_manager.hpp>
#include <zephyr/devicetree.h>

namespace bench {
struct InputConfig {
    const device *remote_uart = nullptr, *vision_uart = nullptr, *referee_uart = nullptr;
    skywalker::communication::RemoteReceiver::Config remote{};
    skywalker::communication::vision::AbProtocol::Config protocol{};
    skywalker::communication::vision::VisionReceiver::Config vision{};
    skywalker::communication::RefereeVersion referee_version = skywalker::communication::RefereeVersion::Rm2026V1_3;
    std::uint32_t referee_timeout_ms = 500;
};
inline const InputConfig inputs{
    .remote_uart = DEVICE_DT_GET(DT_ALIAS(remote_uart)),
    .vision_uart = DEVICE_DT_GET(DT_ALIAS(vision_uart)),
    .referee_uart = DEVICE_DT_GET(DT_ALIAS(referee_uart)),
};
#ifdef CONFIG_COMMAND_MANAGER_VOFA
inline const device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
#else
inline const device *telemetry_uart = nullptr;
#endif
inline const skywalker::robotics::CommandManager::Config manager = [] {
    skywalker::robotics::CommandManager::Config c{};
    c.max_chassis_vx_m_s = 0.5f;
    c.max_chassis_vy_m_s = 0.5f;
    c.max_chassis_wz_rad_s = 1.0f;
    c.max_gimbal_yaw_rate_rad_s = 1.0f;
    c.max_gimbal_pitch_rate_rad_s = 0.8f;
    c.input_timeout_ms = 100;
    c.permission_timeout_ms = 300;
    c.vision_timeout_us = 100000;
    c.expected_vision_reference = {1, 1};
    c.override_enter_norm = 0.15f;
    c.override_exit_norm = 0.05f;
    c.override_release_us = 200000;
    return c;
}();
}
