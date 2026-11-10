#include <algorithm>
#include <cerrno>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <communication/remote/remote_receiver.hpp>
#include <robotics/vehicle/diagnostics.hpp>
#include <zephyr/kernel.h>
#include "../../../robotics/common/operator_controls.hpp"
#include "../../../robotics/common/rc_controls.hpp"

using namespace skywalker;
using namespace skywalker::robotics;
namespace installed = skywalker::robotics::vehicle;
using installed::DiagnosticReason;

namespace {
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::ConfiguredInterBoardTransport transport({.kind = communication::InterBoardTransportKind::Uart,
                                                        .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))},
                                                       &link_dma);
communication::InterBoardEndpoint endpoint(transport, {BoardRole::GimbalController});
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver::Config driveReceiverConfig() {
    auto config = samples::control::receiverConfig();
    // Decode the four stick channels; reserved DBUS bytes or an uncentered
    // optional wheel must not prevent decoding/arming this stage.
    config.decoder.decode_wheel = false;
    return config;
}
communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma, driveReceiverConfig());
K_THREAD_STACK_DEFINE(link_stack, 4096);
k_thread link_thread;
void linkTask(void *, void *, void *) {
    for (;;) {
        endpoint.poll(static_cast<std::uint64_t>(k_uptime_get()));
        k_sleep(K_MSEC(1));
    }
}
} // namespace

int main() {
    installed::Diagnostics diagnostics;
    diagnostics.start("gimbal_drive_command", "gimbal", "vx,vy=0.1m/s; wz=0.2rad/s; no wheel channel");
    const int setup = remote.start();
    diagnostics.event("remote.start", setup);
    k_thread_create(&link_thread, link_stack, K_THREAD_STACK_SIZEOF(link_stack), linkTask, nullptr, nullptr, nullptr, 6,
                    0, K_NO_WAIT);
    samples::control::RcControlAdapter controls({.require_right_middle_to_arm = false});
    samples::control::OperatorControlPublisher authority;
    communication::RemoteReceiver::Snapshot received{};
    std::uint64_t previous_ms = 0;
    for (;;) {
        // Preserve switch edges if several real frames arrive between iterations.
        // Reading/retransmitting a cached input never renews its production time.
        RemoteState frame{};
        for (unsigned budget = 0; budget < 32; ++budget) {
            const int result = remote.nextFrame(frame);
            if (result == -EAGAIN)
                break;
            if (result < 0) {
                controls.withdraw();
                break;
            }
            (void)controls.update(frame, static_cast<std::uint64_t>(k_uptime_get()));
        }
        (void)remote.snapshot(received);
        const auto peer = endpoint.snapshot();
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const auto &state = controls.update(received.remote, now);
        const bool revoked = authority.publish(endpoint, setup == 0 && state.run_allowed, false, received.remote.stamp,
                                               state.clear_event_id, state.clear_stamp, now);
        if (revoked)
            controls.withdraw();
        const bool allowed = setup == 0 && state.run_allowed && !revoked && peer.online;
        const auto age = received.remote.stamp.valid && now >= received.remote.stamp.timestamp_ms
                             ? static_cast<std::uint32_t>(
                                   std::min<std::uint64_t>(now - received.remote.stamp.timestamp_ms, UINT32_MAX))
                             : UINT32_MAX;
        ChassisCommand request{};
        request.mode = allowed ? ChassisMode::BodyVelocity : ChassisMode::Disabled;
        request.source = allowed ? ControlSource::Remote : ControlSource::None;
        request.vx_m_s = allowed ? samples::control::RcControlAdapter::normalize(received.remote.analog.left_y) * 0.1f
                                 : 0;
        request.vy_m_s = allowed ? -samples::control::RcControlAdapter::normalize(received.remote.analog.left_x) * 0.1f
                                 : 0;
        request.wz_rad_s = allowed && state.spin_requested ? 0.2f : 0;
        request.stamp = received.remote.stamp;
        endpoint.submit(request);
        const bool feedback_fresh = peer.online && isFresh(peer.feedback.stamp, now, 100);
        const unsigned flags = unsigned(state.fresh) | (unsigned(controls.controlsCentered()) << 1) |
                               (unsigned(controls.armReady()) << 2) | (unsigned(state.armed) << 3) |
                               (unsigned(state.run_allowed) << 4) | (unsigned(peer.online) << 5) |
                               (unsigned(feedback_fresh) << 6) |
                               (unsigned(feedback_fresh && peer.feedback.armed) << 7) |
                               (unsigned(state.spin_requested) << 8);
        diagnostics.status(
            {.reason = setup || received.uart_error ? DiagnosticReason::InitFailed
                       : !state.fresh               ? DiagnosticReason::RemoteStale
                       : !peer.online               ? DiagnosticReason::PeerStale
                       : !allowed                   ? DiagnosticReason::Disarmed
                                                    : DiagnosticReason::Ready,
             .valid = state.fresh,
             .enabled = allowed,
             .target = request.vx_m_s,
             .feedback = float(received.remote.analog.left_y),
             .age_ms = age,
             .dt_us = previous_ms ? float(now - previous_ms) * 1000 : 0,
             .effort = float(controls.neutralHeldMs()),
             .detail = float(flags),
             .error = setup                 ? setup
                      : received.uart_error ? received.uart_error
                                            : peer.error,
             .extra0 = float(unsigned(received.remote.left_switch) * 4 + unsigned(received.remote.right_switch)),
             .extra1 = float(received.remote.stamp.sequence & 0xffff)});
        previous_ms = now;
        k_sleep(K_MSEC(2));
    }
}
