#pragma once
#include <zephyr/device.h>
#include <robotics/messages/interboard.hpp>
namespace bench {
inline const device *uart=DEVICE_DT_GET(DT_ALIAS(interboard_uart));
#ifdef CONFIG_BENCH_CHASSIS_ROLE
inline constexpr auto role=skywalker::robotics::BoardRole::ChassisController;
#else
inline constexpr auto role=skywalker::robotics::BoardRole::GimbalController;
#endif
inline constexpr float target_vx_m_s=0.2f; // Sent as a semantic command; this sample owns no motors.
}
