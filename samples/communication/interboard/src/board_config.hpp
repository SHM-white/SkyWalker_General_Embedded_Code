#pragma once
#include <zephyr/device.h>
#include <robotics/messages/interboard.hpp>
#include <communication/interboard/configured_interboard_transport.hpp>
namespace bench {
inline const device *uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart));
#ifdef CONFIG_BENCH_CHASSIS_ROLE
inline constexpr auto role = skywalker::robotics::BoardRole::ChassisController;
#else
inline constexpr auto role = skywalker::robotics::BoardRole::GimbalController;
#endif
inline constexpr float target_vx_m_s = 0.2f; // Sent as a semantic command; this sample owns no motors.
inline const skywalker::communication::ConfiguredInterBoardTransport::Config transport = [] {
    using namespace skywalker::communication;
    ConfiguredInterBoardTransport::Config c{};
#if defined(CONFIG_BENCH_TRANSPORT_RS485)
    c.kind = InterBoardTransportKind::Rs485;
#elif defined(CONFIG_BENCH_TRANSPORT_CAN)
    c.kind = InterBoardTransportKind::Can;
#endif
    c.uart = uart;
    c.rs485.uart = DEVICE_DT_GET(DT_ALIAS(interboard_rs485));
    const bool coordinator = role == skywalker::robotics::BoardRole::GimbalController;
    c.rs485.role = coordinator ? Rs485InterBoardTransport::Role::Coordinator
                               : Rs485InterBoardTransport::Role::Responder;
    c.can.can = DEVICE_DT_GET(DT_ALIAS(interboard_can));
    c.can.tx_id = coordinator ? 0x600 : 0x601;
    c.can.rx_id = coordinator ? 0x601 : 0x600;
    return c;
}();
}
