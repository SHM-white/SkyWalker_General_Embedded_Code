#include <communication/remote/remote_receiver.hpp>
#include "../../../robotics/common/rc_controls.hpp"
#include <algorithm>
#include <cerrno>
#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/bringup_policy.hpp>
#include <robotics/vehicle/calibration.hpp>
#include <core/clock.hpp>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include "../../../robotics/common/operator_controls.hpp"
#include <zephyr/kernel.h>
using namespace skywalker;
using namespace skywalker::robotics;
namespace installed = skywalker::robotics::vehicle;
using installed::DiagnosticReason;
namespace {
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::ConfiguredInterBoardTransport transport(
    {.kind = communication::InterBoardTransportKind::Uart,
     .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))}, &link_dma);
communication::InterBoardEndpoint endpoint(transport, {BoardRole::GimbalController});
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                      samples::control::receiverConfig());
} // namespace

int main() {
    installed::Diagnostics diagnostics;
    diagnostics.start("gimbal_big_yaw_command", "gimbal", "big-yaw joint=0.1rad/s");
    const int setup = remote.start();
    diagnostics.event("remote.start", setup);
    samples::control::RcControlAdapter controls;
    samples::control::OperatorControlPublisher authority;
    communication::RemoteReceiver::Snapshot received{};
    std::uint32_t sequence = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        endpoint.poll(now);
        const auto peer = endpoint.snapshot();
        (void)remote.snapshot(received);
        const auto &state = controls.update(received.remote, now);
        const bool revoked = authority.publish(endpoint, state.run_allowed, false, received.remote.stamp,
                                              state.clear_event_id, state.clear_stamp, now);
        if (revoked) controls.withdraw();
        const bool allowed = setup == 0 && state.run_allowed && !revoked && peer.online;
        const auto age = received.remote.stamp.valid && now >= received.remote.stamp.timestamp_ms
                             ? std::uint32_t(std::min<std::uint64_t>(now-received.remote.stamp.timestamp_ms, UINT32_MAX))
                             : UINT32_MAX;
        BigYawRequest request{};
        request.mode = allowed ? BigYawMode::FollowCenter : BigYawMode::Disabled;
        // Preserve the current v4 peer convention; do not silently flip one end.
        request.target_rate_rad_s = allowed ? -samples::control::RcControlAdapter::normalize(
            received.remote.analog.right_x) * installed::bringup::big_yaw_rate_rad_s : 0;
        request.source_sequence = received.remote.stamp.sequence;
        request.source_age_ms = age;
        request.receiver_boot_id = peer.peer.sender_boot_id;
        request.permission = {state.fresh, allowed, received.remote.stamp};
        request.permission_age_ms = age;
        request.stamp = {now, ++sequence, true};
        endpoint.submitBigYaw(request);
        diagnostics.status({.reason = setup ? DiagnosticReason::InitFailed :
                            !state.fresh ? DiagnosticReason::RemoteStale :
                            !peer.online ? DiagnosticReason::PeerStale :
                            !allowed ? DiagnosticReason::Disarmed : DiagnosticReason::Ready,
                            .valid = peer.online && peer.big_yaw_feedback.valid &&
                                     forwardedFresh(peer.big_yaw_feedback.stamp,
                                                    peer.big_yaw_feedback.production_age_ms, now, 100),
                            .enabled = allowed, .target = request.target_rate_rad_s,
                            .feedback = peer.big_yaw_feedback.actual_rate_rad_s,
                            .age_ms = age, .detail = float(unsigned(request.mode)),
                            .error = setup ? setup : peer.error,
                            .extra0 = float(peer.big_yaw_feedback.armed),
                            .extra1 = float(peer.big_yaw_feedback.ready)});
        k_sleep(K_MSEC(5));
    }
}
