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
#include <robotics/vehicle/led_status.hpp>
#include <drivers/led/status_led.hpp>
#include <robotics/vehicle/manual_profile.hpp>
#include <robotics/gimbal/world_yaw_window_follow.hpp>
#include <robotics/vehicle/motion_profile.hpp>
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
    static led::StatusLed status_led;
    const int status_led_error = status_led.start();
    if constexpr (led::StatusLed::available)
        diagnostics.event("status_led", status_led_error);
    (void)status_led.setStatus(led::LedStatus::Initializing);
    static communication::ConfiguredInterBoardTransport transport({.kind = communication::InterBoardTransportKind::Uart,
                                                                   .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))},
                                                                  &link_dma);
    static communication::InterBoardEndpoint endpoint(transport, {.role = BoardRole::GimbalController});
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
    static GimbalExecutor gimbal(yaw, pitch, group, installed::yawMotorConfig(), installed::yawAxisConfig(),
                                 installed::pitchMotorConfig(), installed::pitchAxisConfig(),
                                 installed::gimbalExecutionConfig());
    static ImuHeadingReference head_heading;
    static InertialGimbalAdapter inertial(installed::manualInertialConfig());
    static WorldYawWindowFollower world_follower;
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
    const int power_error = installed::connections_confirmed ? regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)))
                                                             : -ENODEV;
    diagnostics.event("command", command_error);
    diagnostics.event("head_imu", head_error);
    diagnostics.event("yaw_bus", yaw_error);
    diagnostics.event("pitch_bus", pitch_error);
    diagnostics.event("gimbal", control_error);
    diagnostics.event("motor_rail", power_error);
    const bool setup_ready = command_error == 0 && head_error == 0 && yaw_error == 0 && pitch_error == 0 &&
                             control_error == 0 && power_error == 0;
    samples::control::RcControlAdapter controls;
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
        // Reboot/loss of an established peer withdraws the RC baseline.
        // No independent OperatorControl authorization or clear-estop event.
        static std::uint64_t last_peer_boot = 0;
        static bool was_peer_online = false;
        const auto peer_state = endpoint.snapshot();
        const bool peer_revoked = (was_peer_online && !peer_state.online) ||
                                  (last_peer_boot && peer_state.peer.sender_boot_id &&
                                   last_peer_boot != peer_state.peer.sender_boot_id);
        if (peer_state.peer.sender_boot_id)
            last_peer_boot = peer_state.peer.sender_boot_id;
        was_peer_online = peer_state.online;
        if (peer_revoked) {
            controls.withdraw();
            operator_state.run_allowed = false;
        }
        commands.setOperatorGate(operator_state.run_allowed, 0);
        (void)commands.snapshot(frame);
        const auto head_frame = head.snapshot();
        const auto heading = head_heading.update(head_frame, !operator_state.run_allowed, now_us);
        InertialGimbalInputs adaptation{};
        adaptation.command = frame.decision.command.gimbal;
        if (!operator_state.run_allowed)
            adaptation.command.mode = GimbalMode::Disabled;
        adaptation.source_stamp = sourceStamp(frame, adaptation.command.source);
        adaptation.yaw = yaw.snapshot();
        adaptation.pitch = pitch.snapshot();
        adaptation.head = head_frame;
        adaptation.prerequisites_ready = setup_ready && installed::imu_mounting_confirmed;
        const auto adapted = inertial.update(adaptation, now_us);
        GimbalExecutionInputs input{};
        input.command = adapted.command;
        input.source_stamp = adapted.source_stamp;
        input.yaw_output_valid = adapted.yaw_output_valid;
        input.pitch_output_valid = adapted.pitch_output_valid;
        input.transport_ready = setup_ready;
        auto status = control_error == 0 ? gimbal.update(input, now_us)
                                         : gimbal.suspend(now_us, WaitReason::Configuration, control_error, true);
        const OutputPermission movement_permission{operator_state.fresh,
                                                   operator_state.run_allowed && installed::connections_confirmed,
                                                   operator_state.remote.stamp};
        const bool joint_valid = adaptation.yaw.feedback_fresh && adaptation.yaw.position_reference_valid &&
                                 bool(adaptation.yaw.feedback.valid & motor::FeedbackPosition);
        const bool following = movement_permission.valid && movement_permission.enabled && heading.valid &&
                               joint_valid && adapted.yaw_output_valid && adapted.pitch_output_valid &&
                               status.state == RunState::Active;
        // Reference v4 mode=1 carries world rate, not the joint-relative centering rate.
        // The recent head-world yaw mean lets big Yaw follow small Yaw without
        // fighting a normal manual aiming motion.
        const float joint_offset = installed::small_yaw.direction *
                                   (adaptation.yaw.feedback.position_rad - installed::yaw_center_rad);
        const auto world_follow = world_follower.update(heading.yaw_rad - joint_offset, joint_offset, now_us,
                                                        following);
        if (now_ms >= next_send_ms) {
            next_send_ms = now_ms + 10;
            BigYawRequest request{};
            request.source_sequence = static_cast<std::uint32_t>(adaptation.source_stamp.sequence);
            request.source_age_ms = age(adaptation.source_stamp, now_us);
            request.command_age_ms = age(adaptation.command.stamp, now_ms);
            request.permission = movement_permission;
            // v4 mode=1: operator-desired *world* yaw rate, as in the reference chassis.
            // The receiver closes the heading/rate loop using the 0x0404 sample.
            request.mode = BigYawMode::Disabled;
            request.target_rate_rad_s = 0;
            request.source_sequence = operator_state.remote.stamp.sequence;
            request.source_age_ms = request.command_age_ms = age(operator_state.remote.stamp, now_ms);
            if (operator_state.fresh && operator_state.run_allowed && heading.valid && world_follow.valid) {
                request.mode = BigYawMode::FollowCenter;
                const float wheel = std::clamp(float(operator_state.remote.analog.wheel) / 660.f, -1.f, 1.f);
                const float user_rate = std::fabs(wheel) < .05f ? 0 : wheel;
                request.target_rate_rad_s = std::clamp(world_follow.world_rate_rad_s + user_rate, -1.0f, 1.0f);
            }
            const auto session = endpoint.snapshot();
            if (session.online) {
                GimbalImuYaw sample{};
                sample.valid = heading.valid;
                sample.yaw_rad = heading.yaw_rad;
                sample.yaw_rate_rad_s = heading.rate_rad_s;
                sample.zero_generation = heading.generation;
                sample.sample_sequence = static_cast<std::uint32_t>(heading.stamp.sequence);
                // Ceiling age: transporting sub-ms samples must not make them younger.
                sample.source_age_ms = heading.stamp.valid && now_us >= heading.stamp.time_us
                                           ? static_cast<std::uint32_t>(
                                                 std::min<std::uint64_t>((now_us - heading.stamp.time_us + 999) / 1000,
                                                                         UINT32_MAX))
                                           : UINT32_MAX;
                sample.receiver_boot_id = session.peer.sender_boot_id;
                sample.stamp = {now_ms, request_sequence + 1, true};
                endpoint.submitGimbalImuYaw(sample);
            }
            // Stop remains newly produced even when IMU/motor initialization failed.
            request.stamp = {now_ms, ++request_sequence, true};
            endpoint.submitBigYaw(request);
            auto wheel_command = frame.decision.command.chassis;
            if (!operator_state.run_allowed)
                wheel_command.mode = ChassisMode::Disabled;
            if (wheel_command.mode != ChassisMode::Disabled) {
                // Optional profile assigns the wheel to big Yaw; fixed spin is separate.
                wheel_command.wz_rad_s = operator_state.spin_requested ? .5f : 0;
            }
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
        auto reason = !setup_ready                         ? installed::DiagnosticReason::InitFailed
                      : !operator_state.fresh              ? installed::DiagnosticReason::RemoteStale
                      : !operator_state.run_allowed        ? installed::DiagnosticReason::Disarmed
                      : !installed::imu_mounting_confirmed ? installed::DiagnosticReason::Configuration
                      : !imu_fresh                         ? installed::DiagnosticReason::ImuStale
                      : !adaptation.yaw.feedback_fresh || !adaptation.pitch.feedback_fresh
                          ? installed::DiagnosticReason::FeedbackStale
                      : status.state == RunState::Active ? installed::DiagnosticReason::Ready
                                                         : installed::DiagnosticReason::CommandStale;
        (void)status_led.setStatus(
            installed::statusLedFor(reason, status.state == RunState::Active, false, status.error));
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
