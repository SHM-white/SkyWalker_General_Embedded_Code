#include <cmath>
#include <robotics/vehicle/big_yaw_profile.hpp>
#include <drivers/motor/can_bus.hpp>
#include <zephyr/drivers/regulator.h>
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
communication::InterBoardEndpoint endpoint(transport, {BoardRole::ChassisController});
motor::Motor drive(installed::bigYawHardware());
motor::CanBus bus(DEVICE_DT_GET(DT_NODELABEL(can3)));
BigYawExecutor executor(drive, installed::bigYawMotorConfig(), installed::bigYawExecutionConfig());
} // namespace
int main() {
    installed::Diagnostics diagnostics;
    diagnostics.start("chassis_big_yaw", "chassis", "big-yaw joint=0.1rad/s");
    int setup = installed::connections_confirmed ? bus.attach(drive) : -EACCES;
    if (!setup) setup = bus.start();
    const bool started = setup == 0;
    if (!setup) setup = executor.begin();
    if (!setup) setup = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
    diagnostics.event("big_yaw.setup", setup);
    samples::control::OperatorControlConsumer controls;
    std::uint64_t peer_boot = 0;
    auto previous = core::monotonicTimeUs();
    for (;;) {
        const auto now = core::monotonicTimeUs(), now_ms = now/1000;
        const auto dt = now-previous;
        previous = now;
        endpoint.poll(now_ms);
        const auto peer = endpoint.snapshot();
        const auto authority = controls.update(peer, now_ms);
        if (peer.peer.stamp.valid && peer_boot != peer.peer.sender_boot_id) {
            peer_boot = peer.peer.sender_boot_id;
            (void)executor.suspend(now, WaitReason::Reference, -ESTALE);
        }
        BigYawExecutionInputs input{};
        input.request = peer.big_yaw_request;
        // Receiver enforces the stage ceiling as well as the sender.
        if (std::abs(input.request.target_rate_rad_s) > installed::bringup::big_yaw_rate_rad_s)
            input.request.mode = BigYawMode::Disabled;
        if (!authority.run_allowed) input.request.mode = BigYawMode::Disabled;
        input.local_boot_id = peer.local_boot_id;
        input.peer_online = peer.online;
        input.transport_ready = setup == 0 && peer.online;
        input.emergency_stop = authority.emergency_stop;
        input.clear_estop = authority.clear_estop;
        const auto status = setup ? executor.suspend(now, WaitReason::Configuration, setup, true)
                                  : executor.update(input, now);
        int error = setup;
        if (started) {
            const int commit = bus.commit().error;
            if (!error) error = commit;
        }
        const auto feedback = executor.feedback();
        endpoint.setBigYawFeedback(feedback);
        endpoint.setStatus(status);
        const auto snapshot = drive.snapshot();
        diagnostics.status({.reason = setup ? DiagnosticReason::InitFailed :
                            !peer.online ? DiagnosticReason::PeerStale :
                            !authority.run_allowed ? DiagnosticReason::Disarmed :
                            !feedback.valid ? DiagnosticReason::FeedbackStale :
                            status.reason == WaitReason::Command ? DiagnosticReason::CommandStale :
                            DiagnosticReason::Ready,
                            .valid = feedback.valid, .enabled = feedback.armed,
                            .target = input.request.target_rate_rad_s, .feedback = feedback.actual_rate_rad_s,
                            .age_ms = peer.big_yaw_request.stamp.valid && now_ms >= peer.big_yaw_request.stamp.timestamp_ms
                                          ? std::uint32_t(now_ms-peer.big_yaw_request.stamp.timestamp_ms) : UINT32_MAX,
                            .dt_us = float(dt), .effort = snapshot.feedback.torque_nm,
                            .detail = float(unsigned(status.reason)), .error = error ? error : status.error,
                            .extra0 = float(unsigned(snapshot.state)), .extra1 = float(peer.operator_control_valid)});
        k_sleep(K_USEC(installed::chassis_period_us));
    }
}
