#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/calibration.hpp>
#include <core/clock.hpp>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
using namespace skywalker;
using namespace skywalker::robotics;
namespace installed = skywalker::robotics::vehicle;
using installed::DiagnosticReason;
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include "../../../robotics/common/operator_controls.hpp"
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::ConfiguredInterBoardTransport transport(
    {.kind = communication::InterBoardTransportKind::Uart,
     .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))}, &link_dma);

communication::InterBoardEndpoint endpoint(transport, {BoardRole::ChassisController});
int main() {
    installed::Diagnostics diagnostics;
    diagnostics.start("chassis_link_observe", "chassis", "observe only");
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        endpoint.poll(now);
        const auto peer = endpoint.snapshot();
        diagnostics.status({.reason = peer.online ? DiagnosticReason::Ready : DiagnosticReason::PeerStale,
                            .valid = peer.online, .enabled = false,
                            .target = peer.big_yaw_request.target_rate_rad_s,
                            .feedback = peer.big_yaw_feedback.actual_rate_rad_s,
                            .age_ms = peer.peer.stamp.valid && now >= peer.peer.stamp.timestamp_ms
                                          ? std::uint32_t(now-peer.peer.stamp.timestamp_ms) : UINT32_MAX,
                            .detail = float(peer.operator_control_valid), .error = peer.error,
                            .extra0 = float(peer.rejected_frames),
                            .extra1 = float(peer.big_yaw_feedback.valid)});
        k_sleep(K_MSEC(2));
    }
}
