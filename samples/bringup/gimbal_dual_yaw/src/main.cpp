#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <communication/remote/remote_receiver.hpp>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
#include <robotics/command/receiver_sources.hpp>
#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/manual_profile.hpp>
#include <robotics/vehicle/bringup_policy.hpp>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include "../../../robotics/common/rc_controls.hpp"
#include <drivers/imu/imu_receiver.hpp>
#include <communication/interboard/uart_interboard_transport.hpp>
#include "../../../robotics/common/operator_controls.hpp"
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                     samples::control::receiverConfig());
RemoteSource source(remote);
CommandManager commands(vehicle::manualCommandConfig());
vehicle::Diagnostics diagnostics;
imu::DmImuCanSource head_source(vehicle::headSensorConfig());
imu::ImuReceiver head(head_source, {.poll_interval_us = 1000, .priority = 6});
motor::Motor yaw(vehicle::yawHardware()), pitch(vehicle::pitchHardware());
motor::Group motors(yaw, pitch);
motor::CanBus yaw_bus(DEVICE_DT_GET(DT_NODELABEL(can1)));
motor::CanBus pitch_bus(DEVICE_DT_GET(DT_NODELABEL(can2)));
GimbalExecutor gimbal(yaw, pitch, motors, vehicle::yawMotorConfig(), vehicle::yawAxisConfig(),
                      vehicle::pitchMotorConfig(), vehicle::pitchAxisConfig(), vehicle::gimbalExecutionConfig());
InertialGimbalAdapter::Config inertialConfig() {
    auto c = vehicle::manualInertialConfig();
    c.yaw.max_rate_rad_s = 1.0f;
    c.pitch.max_rate_rad_s = 0.8f;
    return c;
}
InertialGimbalAdapter adapter(inertialConfig());
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::UartInterBoardTransport transport(DEVICE_DT_GET(DT_ALIAS(interboard_uart)), link_dma);
communication::InterBoardEndpoint endpoint(transport, {BoardRole::GimbalController});
YawCenteringController::Config centeringConfig() {
    auto c = vehicle::manualCenteringConfig();
    c.max_rate_rad_s = vehicle::bringup::big_yaw_rate_rad_s;
    return c;
}
YawCenteringController centering(centeringConfig());
void communicationTask(void *, void *, void *) {
    for (;;) {
        endpoint.poll(k_uptime_get());
        k_sleep(K_MSEC(1));
    }
}
} // namespace
K_THREAD_DEFINE(dual_yaw_link_worker, 4096, communicationTask, nullptr, nullptr, nullptr, 6, K_FP_REGS, 0);

int main() {
    diagnostics.start("gimbal_dual_yaw", "gimbal", "yaw<=1rad/s,pitch<=0.8rad/s,big_yaw<=0.1rad/s,yaw_effort<=1.2A,pitch_effort<=0.8Nm");
    const int imu_error = head.start();
    diagnostics.event("head.start", imu_error);
    const int power_error = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
    diagnostics.event("motor_power", power_error);
    int source_error = commands.registerSource(source);
    if (!source_error)
        source_error = commands.start();
    diagnostics.event("remote_commands", source_error);
    int setup_error = !vehicle::connections_confirmed ? -EACCES : power_error;
    if (!setup_error)
        setup_error = yaw_bus.attach(yaw);
    if (!setup_error)
        setup_error = pitch_bus.attach(pitch);
    bool yaw_started = false, pitch_started = false;
    if (!setup_error) {
        setup_error = yaw_bus.start();
        yaw_started = setup_error == 0;
    }
    if (!setup_error) {
        setup_error = pitch_bus.start();
        pitch_started = setup_error == 0;
    }
    if (!setup_error)
        setup_error = gimbal.begin();
    diagnostics.event("gimbal.begin", setup_error);
    const bool configured = setup_error == 0;
    samples::control::RcControlAdapter controls;
    communication::RemoteReceiver::Snapshot rc{};
    CommandSnapshot frame{};
    core::TimeUs previous_us = 0;
    samples::control::OperatorControlPublisher operator_publisher;
    bool peer_feedback_was_valid = false;
    for (;;) {
        const auto now = core::monotonicTimeUs(), ms = now / 1000;
        const auto dt_us = previous_us && now > previous_us ? now - previous_us : 0;
        previous_us = now;
        (void)remote.snapshot(rc);
        const auto &op = controls.update(rc.remote, ms);
        if (operator_publisher.publish(endpoint, op.run_allowed, false, op.remote.stamp,
                                       op.clear_event_id, op.clear_stamp, ms))
            controls.withdraw();
        commands.setOperatorGate(op.run_allowed, op.clear_event_id);
        (void)commands.snapshot(frame);
        const auto measured_head = head.snapshot();
        const auto fields = imu::Orientation | imu::Gyro;
        const bool imu_fresh = !imu_error && measured_head.state == imu::State::Running &&
                               (measured_head.fresh_mask & fields) == fields &&
                               core::fresh(measured_head.sample.orientation.stamp, now, 20000) &&
                               core::fresh(measured_head.sample.gyro_rad_s.stamp, now, 20000);
        InertialGimbalInputs input{};
        input.head = measured_head;
        input.yaw = yaw.snapshot();
        input.pitch = pitch.snapshot();
        input.command = frame.decision.command.gimbal;
        input.source_stamp = sourceStamp(frame, input.command.source);
        input.prerequisites_ready = configured && vehicle::imu_mounting_confirmed;
        if (!op.run_allowed || !imu_fresh || source_error)
            input.command.mode = GimbalMode::Disabled;
        const auto inertial = adapter.update(input, now);
        GimbalExecutionInputs request{};
        request.command = inertial.command;
        request.source_stamp = inertial.source_stamp;
        request.yaw_output_valid = inertial.yaw_output_valid;
        request.pitch_output_valid = inertial.pitch_output_valid;
        request.transport_ready = configured && !source_error && !imu_error;
        request.clear_estop = op.clear_estop;
        const auto run = configured ? gimbal.update(request, now)
                                    : gimbal.suspend(now, WaitReason::Configuration, setup_error, true);
        const int yaw_error = yaw_started ? yaw_bus.commit().error : 0;
        const int pitch_error = pitch_started ? pitch_bus.commit().error : 0;
        const auto ys = yaw.snapshot(), ps = pitch.snapshot();
        const auto age = measured_head.sample.orientation.stamp.valid &&
                         now >= measured_head.sample.orientation.stamp.time_us
                             ? std::min<std::uint64_t>((now - measured_head.sample.orientation.stamp.time_us) / 1000,
                                                       UINT32_MAX) : UINT32_MAX;
        const auto peer = endpoint.snapshot();
        const auto &feedback = peer.big_yaw_feedback;
        const bool peer_feedback_valid = peer.online && feedback.valid &&
                                         forwardedFresh(feedback.stamp, feedback.production_age_ms, ms, 50) &&
                                         std::isfinite(feedback.actual_rate_rad_s) &&
                                         std::fabs(feedback.actual_rate_rad_s) <= vehicle::big_yaw.velocity_limit_rad_s;
        if (peer_feedback_valid != peer_feedback_was_valid)
            centering.reset();
        peer_feedback_was_valid = peer_feedback_valid;
        YawCenteringInputs follow{};
        follow.joint_yaw_rad = ys.feedback.position_rad;
        follow.joint_stamp = {ys.feedback.timestamp_ms * 1000, ys.feedback.timestamp_ms, ys.feedback_fresh};
        follow.source_stamp = input.source_stamp;
        follow.enabled = op.run_allowed && peer.online && input.command.mode != GimbalMode::Disabled;
        follow.head_stable = inertial.stabilization_valid && run.state == RunState::Active;
        follow.permission_valid = vehicle::connections_confirmed && vehicle::imu_mounting_confirmed &&
                                  op.run_allowed && !op.clear_estop;
        const auto centered = centering.update(follow, now);
        BigYawRequest big_request{};
        big_request.mode = centered.enabled ? BigYawMode::FollowCenter : BigYawMode::Disabled;
        // A disabled drive can need a zero-speed enable to start reporting.
        // Missing real feedback never permits a nonzero carrier request.
        big_request.target_rate_rad_s = peer_feedback_valid
                                           ? vehicle::big_yaw_wire_direction * centered.velocity_rad_s : 0;
        big_request.source_sequence = static_cast<std::uint32_t>(input.source_stamp.sequence);
        big_request.source_age_ms = input.source_stamp.valid && now >= input.source_stamp.time_us
                                       ? static_cast<std::uint32_t>(std::min<std::uint64_t>(
                                             (now - input.source_stamp.time_us) / 1000, UINT32_MAX)) : UINT32_MAX;
        big_request.command_age_ms = input.command.stamp.valid && ms >= input.command.stamp.timestamp_ms
                                        ? static_cast<std::uint32_t>(std::min<std::uint64_t>(
                                              ms - input.command.stamp.timestamp_ms, UINT32_MAX)) : UINT32_MAX;
        big_request.stamp = centered.stamp;
        big_request.permission = {op.fresh, follow.permission_valid, op.remote.stamp};
        endpoint.submitBigYaw(big_request);
        endpoint.setStatus(run);
        const auto reason = setup_error || source_error || imu_error ? vehicle::DiagnosticReason::InitFailed
                            : !op.fresh ? vehicle::DiagnosticReason::RemoteStale
                            : !op.run_allowed ? vehicle::DiagnosticReason::Disarmed
                            : !imu_fresh ? vehicle::DiagnosticReason::ImuStale
                            : !peer.online || !peer_feedback_valid ? vehicle::DiagnosticReason::PeerStale
                            : !inertial.yaw_output_valid || !inertial.pitch_output_valid
                                ? vehicle::DiagnosticReason::FeedbackStale
                            : vehicle::DiagnosticReason::Ready;
        diagnostics.status({.reason = reason,
                            .valid = imu_fresh && ys.feedback_fresh && ps.feedback_fresh && peer_feedback_valid,
                            .enabled = centered.enabled && peer_feedback_valid,
                            .target = big_request.target_rate_rad_s,
                            .feedback = peer_feedback_valid ? feedback.actual_rate_rad_s :
                                                             std::numeric_limits<float>::quiet_NaN(),
                            .age_ms = feedback.valid ? feedback.production_age_ms : UINT32_MAX,
                            .dt_us = float(dt_us),
                            .effort = gimbal.yawTelemetry().effort_command,
                            .detail = float(unsigned(centered.reason)),
                            .error = setup_error ? setup_error : source_error ? source_error : imu_error ? imu_error :
                                     yaw_error ? yaw_error : pitch_error ? pitch_error : run.error,
                            .extra0 = centered.center_error_rad,
                            .extra1 = inertial.head_yaw_rad});
        k_sleep(K_USEC(vehicle::gimbal_period_us));
    }
}
