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
#include <communication/remote/remote_receiver.hpp>
#include "../../../robotics/common/rc_controls.hpp"
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                      samples::control::receiverConfig());

int main() {
    installed::Diagnostics diagnostics;
    diagnostics.start("gimbal_remote", "gimbal", "observe only");
    const int setup = remote.start();
    diagnostics.event("remote.start", setup);
    communication::RemoteReceiver::Snapshot received{};
    samples::control::RcControlAdapter controls;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        (void)remote.snapshot(received);
        const auto &state = controls.update(received.remote, now);
        const auto &r = received.remote;
        diagnostics.status({.reason = setup < 0 ? DiagnosticReason::InitFailed :
                            !state.fresh ? DiagnosticReason::RemoteStale : DiagnosticReason::Ready,
                            .valid = state.fresh, .enabled = false,
                            .target = float(r.analog.right_x), .feedback = float(r.analog.right_y),
                            .age_ms = r.stamp.valid && now >= r.stamp.timestamp_ms
                                          ? std::uint32_t(now-r.stamp.timestamp_ms) : UINT32_MAX,
                            .effort = float(r.analog.wheel),
                            .detail = float(unsigned(r.left_switch) * 4 + unsigned(r.right_switch)),
                            .error = setup < 0 ? setup : received.uart_error,
                            .extra0 = float(r.analog.left_x), .extra1 = float(r.analog.left_y)});
        k_sleep(K_MSEC(20));
    }
}
