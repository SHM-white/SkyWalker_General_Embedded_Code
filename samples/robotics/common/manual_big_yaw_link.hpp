#pragma once
#include "operator_controls.hpp"
#include "rc_controls.hpp"
#include <communication/interboard/uart_interboard_transport.hpp>
#include <zephyr/sys/printk.h>

namespace skywalker::samples::manual_big_yaw {
using namespace robotics;
inline communication::AsyncUart::DmaBuffers link_dma __nocache;
inline communication::UartInterBoardTransport transport(DEVICE_DT_GET(DT_ALIAS(interboard_uart)), link_dma);
inline communication::InterBoardEndpoint endpoint(transport, {BoardRole::GimbalController});
// Only the execution thread owns these controls and the outgoing request.
// An independent gate lets a peer disconnect stop big Yaw without withdrawing
// the local gimbal/shooter gate. Both consume the existing receiver snapshot.
inline control::RcControlAdapter controls({false, true});
inline control::OperatorControlPublisher operator_controls;
inline BigYawRequest request{};
inline constexpr float manual_rate_max_rad_s = 0.1f;

inline void communicationTask(void *, void *, void *) {
    for (;;) {
        endpoint.poll(k_uptime_get());
        k_sleep(K_MSEC(1));
    }
}

inline void update(const control::RcControlState &local, bool emergency, bool paused, std::uint64_t now_ms) {
    const auto &rc = controls.update(local.remote, now_ms);
    const bool allowed = local.run_allowed && rc.run_allowed && !paused && !emergency;
    if (operator_controls.publish(endpoint, allowed, emergency, rc.remote.stamp,
                                  rc.clear_event_id, rc.clear_stamp, now_ms))
        controls.withdraw();
    const auto peer = endpoint.snapshot();
    const bool run = allowed && rc.run_allowed && rc.fresh && peer.online;
    request = {};
    // v4 calls its velocity request FollowCenter; no centering or IMU runs here.
    request.mode = run ? BigYawMode::FollowCenter : BigYawMode::Disabled;
    request.target_rate_rad_s = run ? manual_rate_max_rad_s *
        control::RcControlAdapter::normalize(rc.remote.analog.wheel) : 0;
    request.source_sequence = rc.remote.stamp.sequence;
    request.stamp = rc.remote.stamp;
    request.receiver_boot_id = peer.peer.sender_boot_id;
    request.permission = {rc.fresh, run, rc.remote.stamp};
    // Original RC stamps survive repeated snapshots; the endpoint accumulates
    // transmission age. Manual bench permission is independent of the shooter.
    endpoint.submitBigYaw(request);
}

inline void printStatus() {
    const auto peer = endpoint.snapshot();
    const auto &feedback = peer.big_yaw_feedback;
    printk("bigYaw online=%d run=%d source=%u target=%.3f actual=%.3f ready=%d valid=%d link=%d rejected=%u\n",
           peer.online, request.mode != BigYawMode::Disabled, request.source_sequence,
           double(request.target_rate_rad_s), double(feedback.actual_rate_rad_s),
           feedback.ready, feedback.valid, peer.error, peer.rejected_frames);
}
} // namespace skywalker::samples::manual_big_yaw
