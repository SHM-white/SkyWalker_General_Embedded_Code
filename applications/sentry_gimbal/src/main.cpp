#include <algorithm>
#include <cerrno>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <core/clock.hpp>
#include <drivers/imu/dm_imu_can.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/command/receiver_sources.hpp>
#include <robotics/gimbal/gimbal_executor.hpp>
#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/manual_profile.hpp>
#include <zephyr/drivers/regulator.h>
#include "../../../samples/robotics/common/operator_controls.hpp"
#include "../../../samples/robotics/common/rc_controls.hpp"

namespace {
using namespace skywalker;
using namespace robotics;
namespace installed = robotics::vehicle;
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::AsyncUart::DmaBuffers remote_dma __nocache;
std::uint32_t age(const MessageStamp &stamp, std::uint64_t now_ms) {
    return stamp.valid && now_ms >= stamp.timestamp_ms
               ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now_ms - stamp.timestamp_ms, UINT32_MAX))
               : UINT32_MAX;
}
std::uint32_t age(const core::Stamp &stamp, core::TimeUs now_us) {
    return stamp.valid && now_us >= stamp.time_us
               ? static_cast<std::uint32_t>(std::min<std::uint64_t>((now_us - stamp.time_us) / 1000, UINT32_MAX))
               : UINT32_MAX;
}
} // namespace

int main() {
    static installed::Diagnostics diagnostics;
    diagnostics.start("sentry_gimbal", "gimbal", "yaw=1rad/s; pitch=0.8rad/s; vx,vy=0.3m/s; wz=0.5rad/s");
    static communication::ConfiguredInterBoardTransport transport(
        {.kind = communication::InterBoardTransportKind::Uart,
         .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))}, &link_dma);
    static communication::InterBoardEndpoint endpoint(transport, {BoardRole::GimbalController});
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                                 samples::control::receiverConfig());
    static RemoteSource remote_source(remote);
    static CommandManager commands(installed::manualCommandConfig());
    static imu::DmImuCanSource head_source(installed::headSensorConfig());
    static imu::ImuReceiver head(head_source, {.poll_interval_us = 1000, .priority = 6});
    static motor::Motor yaw(installed::yawHardware()), pitch(installed::pitchHardware());
    static motor::CanBus yaw_bus(DEVICE_DT_GET(DT_NODELABEL(can1)));
    static motor::CanBus pitch_bus(DEVICE_DT_GET(DT_NODELABEL(can2)));
    static motor::Group group(yaw, pitch);
    static GimbalExecutor gimbal(yaw, pitch, group,
                                 installed::yawMotorConfig(), installed::yawAxisConfig(),
                                 installed::pitchMotorConfig(), installed::pitchAxisConfig(), installed::gimbalExecutionConfig());
    static InertialGimbalAdapter inertial(installed::manualInertialConfig());
    static YawCenteringController centering(installed::manualCenteringConfig());
    int command_error = commands.registerSource(remote_source);
    if (command_error == 0)
        command_error = commands.start();
    const int head_error = head.start();
    int yaw_error = installed::connections_confirmed ? yaw_bus.attach(yaw) : -ENODEV;
    int pitch_error = installed::connections_confirmed ? pitch_bus.attach(pitch) : -ENODEV;
    if (yaw_error == 0)
        yaw_error = yaw_bus.start();
    if (pitch_error == 0)
        pitch_error = pitch_bus.start();
    const bool yaw_started = yaw_error == 0, pitch_started = pitch_error == 0;
    const int control_error = yaw_started && pitch_started ? gimbal.begin() : -ENODEV;
    const int power_error = installed::connections_confirmed
                                ? regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1))) : -ENODEV;
    diagnostics.event("command", command_error);
    diagnostics.event("head_imu", head_error);
    diagnostics.event("yaw_bus", yaw_error);
    diagnostics.event("pitch_bus", pitch_error);
    diagnostics.event("gimbal", control_error);
    diagnostics.event("motor_rail", power_error);
    const bool setup_ready = command_error == 0 && head_error == 0 && yaw_error == 0 &&
                             pitch_error == 0 && control_error == 0 && power_error == 0;
    samples::control::RcControlAdapter controls;
    samples::control::OperatorControlPublisher operator_publisher;
    communication::RemoteReceiver::Snapshot remote_frame{};
    CommandSnapshot frame{};
    std::uint64_t next_send_ms = 0;
    std::uint32_t request_sequence = 0;
    std::int64_t deadline = k_uptime_ticks();
    const auto period = k_us_to_ticks_ceil64(installed::gimbal_period_us);
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        endpoint.poll(now_ms);
        (void)remote.snapshot(remote_frame);
        auto operator_state = controls.update(remote_frame.remote, now_ms);
        if (operator_publisher.publish(endpoint, operator_state.run_allowed, false,
                                       operator_state.remote.stamp, operator_state.clear_event_id,
                                       operator_state.clear_stamp, now_ms)) {
            controls.withdraw();
            operator_state.run_allowed = false;
        }
        commands.setOperatorGate(operator_state.run_allowed, operator_state.clear_event_id);
        (void)commands.snapshot(frame);
        const auto head_frame = head.snapshot();
        InertialGimbalInputs adaptation{};
        adaptation.command = frame.decision.command.gimbal;
        if (!operator_state.run_allowed)
            adaptation.command.mode = GimbalMode::Disabled;
        adaptation.source_stamp = sourceStamp(frame, adaptation.command.source);
        adaptation.yaw = yaw.snapshot();
        adaptation.pitch = pitch.snapshot();
        adaptation.head = head_frame;
        adaptation.prerequisites_ready = setup_ready && installed::imu_mounting_confirmed &&
                                         !operator_state.clear_estop;
        const auto adapted = inertial.update(adaptation, now_us);
        GimbalExecutionInputs input{};
        input.command = adapted.command;
        input.source_stamp = adapted.source_stamp;
        input.yaw_output_valid = adapted.yaw_output_valid;
        input.pitch_output_valid = adapted.pitch_output_valid;
        input.transport_ready = setup_ready;
        input.clear_estop = operator_state.clear_estop;
        auto status = control_error == 0 ? gimbal.update(input, now_us)
                                         : gimbal.suspend(now_us, WaitReason::Configuration, control_error, true);
        YawCenteringInputs follow{};
        follow.joint_yaw_rad = adaptation.yaw.feedback.position_rad;
        follow.joint_stamp = {adaptation.yaw.feedback.timestamp_ms * 1000,
                              adaptation.yaw.feedback.timestamp_ms,
                              adaptation.yaw.feedback_fresh && adaptation.yaw.position_reference_valid &&
                                  bool(adaptation.yaw.feedback.valid & motor::FeedbackPosition)};
        follow.source_stamp = adaptation.source_stamp;
        follow.enabled = operator_state.run_allowed && input.command.mode != GimbalMode::Disabled &&
                         !operator_state.clear_estop;
        follow.head_stable = adapted.yaw_output_valid && adapted.pitch_output_valid &&
                             status.state == RunState::Active;
        const OutputPermission movement_permission{
            operator_state.fresh, operator_state.run_allowed && installed::connections_confirmed,
            operator_state.remote.stamp};
        follow.permission_valid = movement_permission.valid && movement_permission.enabled &&
                                  isFresh(movement_permission.stamp, now_ms, 300);
        const auto centered = centering.update(follow, now_us);
        if (now_ms >= next_send_ms) {
            next_send_ms = now_ms + 10;
            BigYawRequest request{};
            request.source_sequence = static_cast<std::uint32_t>(adaptation.source_stamp.sequence);
            request.source_age_ms = age(adaptation.source_stamp, now_us);
            request.command_age_ms = age(adaptation.command.stamp, now_ms);
            request.permission = movement_permission;
            if (centered.enabled && operator_state.run_allowed &&
                request.source_age_ms <= installed::manual_command_timeout_ms &&
                request.command_age_ms <= installed::manual_command_timeout_ms) {
                request.mode = BigYawMode::FollowCenter;
                request.target_rate_rad_s = installed::big_yaw_wire_direction * centered.velocity_rad_s;
            }
            // Stop remains newly produced even when IMU/motor initialization failed.
            request.stamp = {now_ms, ++request_sequence, true};
            endpoint.submitBigYaw(request);
            auto wheel_command = frame.decision.command.chassis;
            if (!operator_state.run_allowed)
                wheel_command.mode = ChassisMode::Disabled;
            endpoint.submit(wheel_command);
        }
        if (yaw_started) {
            const int error = yaw_bus.commit().error;
            if (error < 0)
                status.error = error;
        }
        if (pitch_started) {
            const int error = pitch_bus.commit().error;
            if (error < 0)
                status.error = error;
        }
        endpoint.setStatus(status);
        const bool imu_fresh = core::fresh(head_frame.sample.orientation.stamp, now_us, 20000);
        auto reason = !setup_ready ? installed::DiagnosticReason::InitFailed
                    : !operator_state.fresh ? installed::DiagnosticReason::RemoteStale
                    : !operator_state.run_allowed ? installed::DiagnosticReason::Disarmed
                    : !installed::imu_mounting_confirmed ? installed::DiagnosticReason::Configuration
                    : !imu_fresh ? installed::DiagnosticReason::ImuStale
                    : !adaptation.yaw.feedback_fresh || !adaptation.pitch.feedback_fresh
                        ? installed::DiagnosticReason::FeedbackStale
                    : status.state == RunState::Active ? installed::DiagnosticReason::Ready
                                                      : installed::DiagnosticReason::CommandStale;
        diagnostics.status({.reason = reason,
                            .valid = adaptation.yaw.feedback_fresh,
                            .enabled = status.state == RunState::Active,
                            .target = static_cast<float>(gimbal.yawTargetRad()),
                            .feedback = adaptation.yaw.feedback.position_rad,
                            .age_ms = age(head_frame.sample.orientation.stamp, now_us)});
        deadline += period;
        const auto ticks = k_uptime_ticks();
        if (deadline <= ticks)
            deadline += ((ticks - deadline) / period + 1) * period;
        k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
    }
}
