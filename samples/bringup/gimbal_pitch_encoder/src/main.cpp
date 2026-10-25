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
#include <robotics/vehicle/gimbal_profile.hpp>
#include <robotics/vehicle/bringup_policy.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>
#include <zephyr/drivers/regulator.h>
#include <cerrno>
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                      samples::control::receiverConfig());

namespace {
motor::Motor drive(installed::pitchHardware());
motor::CanBus bus(DEVICE_DT_GET(DT_NODELABEL(can2)));
GimbalAxis axis(drive, installed::pitchMotorConfig(), installed::pitchAxisConfig());
}
int main() {
    installed::Diagnostics diagnostics;
    diagnostics.start("gimbal_pitch_encoder", "gimbal", "pitch=0.15rad/s; installed effort/PID/limits");
    diagnostics.event("remote.start", remote.start());
    int setup = installed::connections_confirmed ? bus.attach(drive) : -EACCES;
    if (!setup) setup = bus.start();
    const bool started = setup == 0;
    if (!setup) setup = axis.begin();
    if (!setup) setup = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
    diagnostics.event("axis.setup", setup);
    samples::control::RcControlAdapter controls;
    communication::RemoteReceiver::Snapshot received{};
    auto previous = core::monotonicTimeUs();
    for (;;) {
        const auto now = core::monotonicTimeUs();
        const auto now_ms = now / 1000;
        const float dt = float(now - previous) * 1e-6f;
        previous = now;
        (void)remote.snapshot(received);
        const auto &state = controls.update(received.remote, now_ms);
        const bool cycle_valid = dt > 0 && dt <= 0.020f;
        const bool requested = setup == 0 && cycle_valid && state.run_allowed;
        const float rate = requested ? installed::pitch.direction *
            samples::control::RcControlAdapter::normalize(received.remote.analog.right_y) *
            installed::bringup::pitch_encoder_rate_rad_s : 0;
        int error = setup;
        if (!requested) {
            drive.disable();
            axis.withdraw();
        } else {
            error = drive.enable();
            if (!error) error = axis.updateRate(rate, dt);
        }
        if (started) {
            const int commit_error = bus.commit().error;
            if (!error) error = commit_error;
        }
        const auto feedback = drive.snapshot();
        const auto telemetry = axis.telemetry();
        const bool valid = feedback.feedback_fresh &&
                           (feedback.feedback.valid & motor::FeedbackPosition);
        const bool enabled = requested && telemetry.output_valid && feedback.output_permitted &&
                             feedback.state == motor::MotorState::Active;
        diagnostics.status({.reason = setup ? DiagnosticReason::InitFailed :
                            !state.fresh ? DiagnosticReason::RemoteStale :
                            !state.run_allowed ? DiagnosticReason::Disarmed :
                            !valid ? DiagnosticReason::FeedbackStale : DiagnosticReason::Ready,
                            .valid = valid, .enabled = enabled,
                            .target = float(axis.targetAngleRad()), .feedback = feedback.feedback.position_rad,
                            .age_ms = received.remote.stamp.valid && now_ms >= received.remote.stamp.timestamp_ms
                                          ? std::uint32_t(now_ms-received.remote.stamp.timestamp_ms) : UINT32_MAX,
                            .dt_us = dt * 1e6f, .effort = telemetry.effort_command,
                            .detail = float(unsigned(feedback.state)), .error = error,
                            .extra0 = rate, .extra1 = feedback.feedback.velocity_rad_s});
        k_sleep(K_USEC(installed::gimbal_period_us));
    }
}
