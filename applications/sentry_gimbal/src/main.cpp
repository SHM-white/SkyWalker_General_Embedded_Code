#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <core/clock.hpp>
#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/command/receiver_sources.hpp>
#include <robotics/chassis/imu_heading.hpp>
#include <core/attitude.hpp>
#include <robotics/gimbal/dual_yaw_motion_allocator.hpp>
#include <robotics/gimbal/gimbal_executor.hpp>
#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/led_status.hpp>
#include <drivers/led/status_led.hpp>
#include <robotics/vehicle/manual_profile.hpp>
#include <zephyr/drivers/regulator.h>
#if DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER)
#include <communication/vision/ab_protocol.hpp>
#include <communication/vision/vision_receiver.hpp>
#endif
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
    diagnostics.start("sentry_gimbal", "gimbal",
                      "encoder_manual; yaw<=3rad/s; pitch<=3rad/s; dual-yaw steady<=0.8rad/s; big_world<=1rad/s");
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
    // Vision is only instantiated when the board assigns a UART and the build
    // enables the AB receiver; the arbitration still needs `allow_auto`.
#if DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER)
    static communication::AsyncUart::DmaBuffers vision_dma __nocache;
    static communication::vision::AbProtocol vision_protocol({.command_reference = installed::head_reference});
    static communication::vision::VisionReceiver vision(DEVICE_DT_GET(DT_ALIAS(vision_uart)), vision_dma,
                                                        vision_protocol, {});
    static VisionSource vision_source(vision);
#endif
#if DT_HAS_ALIAS(navigation_uart) && IS_ENABLED(CONFIG_SKYWALKER_NAVIGATION_RECEIVER)
    static communication::AsyncUart::DmaBuffers navigation_dma __nocache;
    static communication::navigation::NavigationReceiver navigation(DEVICE_DT_GET(DT_ALIAS(navigation_uart)),
                                                                    navigation_dma);
    static NavigationSource navigation_source(navigation);
#endif
// Referee polling is owned by CommandManager; its frame callback only enqueues
// copies, leaving USART1 RX/TX exclusively with NavigationReceiver.
#if DT_HAS_ALIAS(referee_uart) && IS_ENABLED(CONFIG_SKYWALKER_REFEREE)
    static communication::AsyncUart::DmaBuffers referee_dma __nocache;
    static communication::RefereeReceiver referee(DEVICE_DT_GET(DT_ALIAS(referee_uart)), referee_dma);
    static RefereePermissionSource referee_source(referee);
#if DT_HAS_ALIAS(navigation_uart) && IS_ENABLED(CONFIG_SKYWALKER_NAVIGATION_RECEIVER)
    referee.setFrameHandler([](const std::uint8_t *bytes, std::size_t size, void *context) {
        auto &receiver = *static_cast<communication::navigation::NavigationReceiver *>(context);
        (void)receiver.forwardRefereeFrame(bytes, size);
    }, &navigation);
#endif
#endif
    static CommandManager commands([] {
        auto config = installed::manualCommandConfig();
        config.allow_navigation = DT_HAS_ALIAS(navigation_uart) && IS_ENABLED(CONFIG_SKYWALKER_NAVIGATION_RECEIVER);
        config.allow_auto = DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER);
        config.mapper.auto_on_up = false;
        config.require_referee_for_chassis = true;
        config.require_referee_for_gimbal = true;
        return config;
    }());
    // Board IMU moves with big Yaw, not with the small-Yaw head. Visual pitch
    // uses its attitude plus encoder geometry; CAN3 head IMU is not instantiated.
    static control::QuaternionEkf onboard_estimator({.initialization_samples = 800, .adapt_gyro_bias = false});
    static imu::Bmi088Imu onboard_source(DEVICE_DT_GET(DT_ALIAS(accel0)), DEVICE_DT_GET(DT_ALIAS(gyro0)),
                                         {.reference = installed::carrier_reference,
                                          .sensor_to_body = installed::carrier_sensor_to_body,
                                          .correct_gyro_bias = false,
                                          .planar_yaw_enabled = true},
                                         &onboard_estimator);
    static imu::ImuReceiver onboard(onboard_source, {.poll_interval_us = 500, .priority = 5});
    // Near-horizontal carrier: yaw is integrated in the acquisition thread.
    // The three-dimensional EKF remains diagnostic-only for this application.
    // The allocator splits one head world-rate target between the big Yaw
    // carrier and the small Yaw joint; it owns no motor and no CAN frame.
    static DualYawMotionAllocator dual_yaw(installed::dualYawAllocatorConfig());
    // Variable-speed top spin: the curve is vehicle::spinRateCurveRadS().
    static SpinSpeedProfile spin(installed::spinProfileConfig());
    static motor::Motor yaw(installed::yawHardware()), pitch(installed::pitchHardware());
    static motor::CanBus yaw_bus(DEVICE_DT_GET(DT_NODELABEL(can1)));
    static motor::CanBus pitch_bus(DEVICE_DT_GET(DT_NODELABEL(can2)));
    static motor::Group group(yaw, pitch);
    static GimbalExecutor gimbal(yaw, pitch, group, installed::yawMotorConfig(), installed::yawAxisConfig(),
                                 installed::pitchMotorConfig(), installed::pitchAxisConfig(),
                                 installed::gimbalExecutionConfig());
    int command_error = commands.registerSource(remote_source);
#if DT_HAS_ALIAS(navigation_uart) && IS_ENABLED(CONFIG_SKYWALKER_NAVIGATION_RECEIVER)
    if (command_error == 0)
        command_error = commands.registerSource(navigation_source);
#endif
#if DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER)
    if (command_error == 0)
        command_error = commands.registerSource(vision_source);
#endif
#if DT_HAS_ALIAS(referee_uart) && IS_ENABLED(CONFIG_SKYWALKER_REFEREE)
    if (command_error == 0)
        command_error = commands.bindPermissions(referee_source);
#endif
    if (command_error == 0)
        command_error = commands.start();
    const int onboard_error = onboard.start();
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
    diagnostics.event("carrier_imu", onboard_error);
    diagnostics.event("yaw_bus", yaw_error);
    diagnostics.event("pitch_bus", pitch_error);
    diagnostics.event("gimbal", control_error);
    diagnostics.event("motor_rail", power_error);
    // Carrier IMU failure blocks big Yaw, but not the local encoder/manual loops.
    const bool setup_ready = command_error == 0 && yaw_error == 0 && pitch_error == 0 && control_error == 0 &&
                             power_error == 0;
    samples::control::RcControlAdapter controls;
    communication::RemoteReceiver::Snapshot remote_frame{};
    CommandSnapshot frame{};
    std::uint64_t next_send_ms = 0;
    std::uint32_t request_sequence = 0;
    std::int64_t deadline = k_uptime_ticks();
    const auto period = k_us_to_ticks_ceil64(installed::gimbal_period_us);
    bool peer_feedback_was_valid = false;
    std::uint64_t last_peer_boot = 0;
    bool was_peer_online = false;
    core::TimeUs previous_cycle_us = 0, last_heading_loss_dt_us = 0;
    bool heading_was_fresh = false, have_heading_loss = false;
    HeadingWaitReason last_heading_loss_reason = HeadingWaitReason::None;
    std::uint32_t heading_loss_count = 0;
    std::uint64_t last_heading_loss_ms = 0;
    core::TimeUs heading_loss_start_us = 0, heading_loss_duration_us = 0;
    imu::EstimatorDiagnostics loss_estimator{};
    CarrierAccelerationEstimator carrier_acceleration;
    std::uint32_t vision_trajectory_sequence = 0, last_heading_generation = 0;
    std::uint64_t last_yaw_sequence = 0;
    bool vision_pitch_was_active = false;
    for (;;) {
        // Snapshot before taking the validation time: a higher-priority IMU
        // update must not make a new gyro stamp appear to be in our future.
        const auto carrier_snapshot = onboard.snapshot();
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        const auto dt_us = previous_cycle_us && now_us > previous_cycle_us ? now_us - previous_cycle_us : 0;
        previous_cycle_us = now_us;
        endpoint.poll(now_ms);
        (void)remote.snapshot(remote_frame);
        auto operator_state = controls.update(remote_frame.remote, now_ms);
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
            operator_state.navigation_active = false;
        }
        const bool navigation_allowed = operator_state.navigation_active && operator_state.fresh && setup_ready;
        const bool gimbal_allowed = operator_state.run_allowed || navigation_allowed;
        commands.setOperatorGate(gimbal_allowed,
                                  static_cast<std::uint32_t>(operator_state.start_event_id));
        // The DR16 dial (spring-centred wheel) is the analog spin control. It
        // only scales and signs the chassis spin rate: translation still comes
        // from the sticks and the gimbal command is untouched.
        const float spin_dial = samples::control::RcControlAdapter::normalize(operator_state.remote.analog.wheel);
        const auto spin_out =
            spin.update(operator_state.spin_requested, operator_state.run_allowed, spin_dial, now_us);
        (void)commands.snapshot(frame);
        const auto heading = planarYawHeading(carrier_snapshot, now_us);
        const bool heading_fresh = heading.valid && core::fresh(heading.stamp, now_us, 20000);
        const auto heading_reason = !heading.valid   ? heading.reason
                                    : !heading_fresh ? HeadingWaitReason::ControlStale
                                                     : HeadingWaitReason::None;
        // Retain transient losses even when heading recovers before the next
        // diagnostic packet. Motor execution errors remain separate in error.
        if (heading_was_fresh && !heading_fresh) {
            have_heading_loss = true;
            last_heading_loss_reason = heading_reason;
            last_heading_loss_dt_us = dt_us;
            ++heading_loss_count;
            last_heading_loss_ms = now_ms;
            heading_loss_start_us = now_us;
            loss_estimator = carrier_snapshot.diagnostics.estimator;
        }
        if (heading_loss_start_us) {
            heading_loss_duration_us = now_us - heading_loss_start_us;
            if (heading_fresh)
                heading_loss_start_us = 0;
        }
        heading_was_fresh = heading_fresh;
        // Accelerations come only from two genuinely new gyro samples; repeated
        // reads keep the filtered value and gaps disable the feedforward.
        const auto &carrier_gyro = carrier_snapshot.sample.gyro_rad_s;
        if (heading.generation != last_heading_generation) {
            // A new software zero invalidates every derivative taken across it.
            last_heading_generation = heading.generation;
            carrier_acceleration.reset();
        }
        carrier_acceleration.update(carrier_gyro.stamp.valid ? carrier_gyro.value.z
                                                             : std::numeric_limits<float>::quiet_NaN(),
                                    carrier_gyro.stamp, 0.02f, 50.0f);
        GimbalExecutionInputs input{};
        input.command = frame.decision.command.gimbal;
        input.source_stamp = sourceStamp(frame, input.command.source);
        const bool gimbal_mode_matches = navigation_allowed
                                             ? frame.decision.operator_mode == OperatorMode::Navigation
                                             : frame.decision.operator_mode == OperatorMode::Manual;
        const float raw_head_rate_rad_s = input.command.yaw_rate_rad_s;
        // Allocation runs on the pre-update snapshot: one measured joint state
        // and one carrier gyro sample produce the plan that this cycle's single
        // gimbal.update() consumes.
        const auto ys = yaw.snapshot();
        const auto ps = pitch.snapshot();
        const bool pitch_valid = ps.feedback_fresh && ps.position_reference_valid &&
                                 bool(ps.feedback.valid & motor::FeedbackPosition) &&
                                 std::isfinite(ps.feedback.position_rad);
        auto carrier_orientation = carrier_snapshot.sample.orientation.value;
        const bool carrier_pose_valid =
            core::fresh(carrier_snapshot.sample.orientation.stamp, now_us, 20000) &&
            core::normalize(carrier_orientation);
        const bool joint_valid = ys.feedback_fresh && ys.position_reference_valid &&
                                 bool(ys.feedback.valid & motor::FeedbackPosition);
        // Last completed Yaw telemetry: the request below is gated on it and the
        // freshly computed plan is applied by the update that follows.
        const bool yaw_running = gimbal.yawTelemetry().output_valid;
        const auto &feedback = peer_state.big_yaw_feedback;
        const bool peer_feedback_valid = peer_state.online && feedback.valid &&
                                         forwardedFresh(feedback.stamp, feedback.production_age_ms, now_ms, 250) &&
                                         std::isfinite(feedback.actual_rate_rad_s) &&
                                         std::fabs(feedback.actual_rate_rad_s) <=
                                             installed::big_yaw.velocity_limit_rad_s;
        if (peer_feedback_valid != peer_feedback_was_valid)
            dual_yaw.reset();
        peer_feedback_was_valid = peer_feedback_valid;
        // Only the arbitrated target is ever consumed. `selected_vision` is kept
        // by the manager exactly while the final gimbal source is Vision.
        const auto &selected_vision = frame.decision.selected_vision;
        const bool vision_selected = selected_vision.stamp.valid && input.command.source == ControlSource::Vision &&
                                     input.command.mode != GimbalMode::Disabled && gimbal_allowed && gimbal_mode_matches;
        // The vision trajectory is expressed in the head world frame, so it is
        // only consumed while the carrier attitude and the joint measurement are
        // fresh; otherwise the axis holds the measured joint instead of chasing
        // a world angle in joint space.
        const bool vision_active = vision_selected && heading_fresh && joint_valid && pitch_valid && carrier_pose_valid;
        if (vision_selected && !vision_active)
            input.command.mode = GimbalMode::Hold;
        float vision_angle_rad = 0, vision_rate_rad_s = 0, vision_acceleration_rad_s2 = 0;
        float vision_pitch_rad = 0;
        std::uint32_t vision_epoch = 0;
        if (vision_selected) {
            const auto &v = selected_vision.value.yaw;
            // AB carries no sample timestamp: the decoder stamps the local byte
            // reception time. Prediction is therefore limited to the measured
            // local age plus the separately verified capture/transport latency;
            // it must not be treated as the vision-side sample age.
            const float local_age_s = now_us >= selected_vision.stamp.time_us
                                          ? float(now_us - selected_vision.stamp.time_us) * 1e-6f
                                          : 0.0f;
            const float age_s = std::min(local_age_s + installed::vision_capture_latency_s, 0.05f);
            vision_angle_rad = v.angle_rad + v.rate_rad_s * age_s + 0.5f * v.acceleration_rad_s2 * age_s * age_s;
            vision_rate_rad_s = v.rate_rad_s;
            vision_acceleration_rad_s2 = v.acceleration_rad_s2;
            const auto &p = selected_vision.value.pitch;
            vision_pitch_rad = p.angle_rad + p.rate_rad_s * age_s + 0.5f * p.acceleration_rad_s2 * age_s * age_s;
            vision_epoch = (static_cast<std::uint32_t>(selected_vision.value.reference.frame_id) << 16) ^
                           static_cast<std::uint32_t>(selected_vision.value.reference.epoch);
        }
        const bool big_channel_ready = setup_ready && operator_state.fresh && gimbal_allowed && gimbal_mode_matches &&
                                       input.command.mode != GimbalMode::Disabled &&
                                       peer_state.online && installed::imu_mounting_confirmed && heading_fresh &&
                                       joint_valid && yaw_running;
        DualYawAllocatorInputs allocation{};
        allocation.head_rate_target_rad_s = raw_head_rate_rad_s;
        allocation.joint_rad = ys.feedback.position_rad;
        const bool joint_rate_valid =
            (ys.feedback.valid & motor::FeedbackVelocity) != 0 && std::isfinite(ys.feedback.velocity_rad_s);
        allocation.joint_rate_rad_s = joint_rate_valid ? ys.feedback.velocity_rad_s : 0.0f;
        allocation.carrier_rate_rad_s = heading_fresh ? heading.rate_rad_s : 0.0f;
        allocation.carrier_stamp = heading.stamp;
        allocation.carrier_valid = heading_fresh;
        allocation.joint_stamp = {ys.feedback.timestamp_ms * 1000, ys.feedback.timestamp_ms, joint_valid};
        allocation.source_stamp = input.source_stamp;
        allocation.big_yaw_available = big_channel_ready && peer_feedback_valid;
        allocation.enabled =
            gimbal_allowed && gimbal_mode_matches &&
            (input.command.mode == GimbalMode::Rate ||
             (vision_active && input.command.mode != GimbalMode::Disabled));
        allocation.permission_valid =
            gimbal_allowed && gimbal_mode_matches && installed::connections_confirmed && installed::imu_mounting_confirmed;
        allocation.vision_active = vision_active;
        allocation.vision_angle_rad = vision_angle_rad;
        allocation.vision_rate_rad_s = vision_rate_rad_s;
        allocation.vision_acceleration_rad_s2 = vision_acceleration_rad_s2;
        allocation.vision_epoch = vision_epoch;
        // Head world angle in the same frame as the vision target. The two
        // heading references must be aligned before Auto is enabled.
        allocation.head_angle_measured_rad =
            vision_active ? heading.yaw_rad +
                                installed::small_yaw.direction *
                                    (ys.feedback.position_rad - installed::yaw_center_rad)
                          : 0.0f;
        allocation.carrier_acceleration_valid = carrier_acceleration.valid();
        allocation.carrier_acceleration_rad_s2 = carrier_acceleration.value_rad_s2();
        const auto alloc = dual_yaw.update(allocation, now_us);
        if (alloc.enabled && alloc.vision_mode) {
            // Yaw uses the allocator; transform the world aiming ray through
            // carrier attitude and the measured Yaw joint before commanding Pitch.
            input.yaw_trajectory.valid = true;
            input.yaw_trajectory.position_rad = alloc.joint_position_rad;
            input.yaw_trajectory.rate_rad_s = alloc.joint_rate_rad_s;
            input.yaw_trajectory.acceleration_rad_s2 = alloc.joint_acceleration_rad_s2;
            input.yaw_trajectory.stamp = {now_us, ++vision_trajectory_sequence, true};
            auto carrier_euler = core::euler(carrier_orientation);
            carrier_euler.yaw = heading.yaw_rad; // Same startup yaw reference as the allocator.
            const auto yaw_pose = core::multiply(
                core::fromEuler(carrier_euler),
                core::fromEuler({0, 0, installed::small_yaw.direction *
                                          (ys.feedback.position_rad - installed::yaw_center_rad)}));
            const float cp = std::cos(vision_pitch_rad);
            const core::Vec3 world_ray{cp * std::cos(vision_angle_rad), cp * std::sin(vision_angle_rad),
                                       -std::sin(vision_pitch_rad)};
            const auto joint_ray = core::rotate(core::conjugate(yaw_pose), world_ray);
            const float target = std::clamp(installed::vision_pitch_horizontal_rad +
                                               installed::pitch.direction * std::atan2(-joint_ray.z, joint_ray.x),
                                           installed::pitch_min_rad, installed::pitch_max_rad);
            const float seed = vision_pitch_was_active ? static_cast<float>(gimbal.pitchTargetRad())
                                                        : ps.feedback.position_rad;
            const float step = installed::manual_max_gimbal_pitch_rate_rad_s *
                               static_cast<float>(std::min<core::TimeUs>(dt_us, 20000)) * 1e-6f;
            input.command.pitch_target_rad = std::clamp(target, seed - step, seed + step);
        }
        else if (vision_selected) {
            // Vision selected but the allocation was rejected this cycle: hold
            // the measured joints instead of driving a world angle through the
            // joint-space rate step.
            input.command.mode = GimbalMode::Hold;
        }
        else if (input.command.mode == GimbalMode::Rate) {
            // Encoder loops own both motors. Coordinated mode consumes the
            // allocated joint rate, which already removed the measured carrier
            // rate; a rejected allocation falls back to the existing local
            // encoder/manual path instead of freezing the axis.
            // The allocator also returns a protected rate on its degraded path,
            // which keeps the outward envelope and the over-envelope rule.
            input.command.yaw_rate_rad_s = alloc.joint_rate_valid
                                               ? alloc.joint_rate_rad_s
                                               : installed::small_yaw.direction * raw_head_rate_rad_s;
            input.command.pitch_rate_rad_s *= installed::pitch.direction;
        }
        vision_pitch_was_active = alloc.enabled && alloc.vision_mode;
        if (!gimbal_allowed || !gimbal_mode_matches)
            input.command.mode = GimbalMode::Disabled;
        input.transport_ready = setup_ready;
        // The travel envelope rejected this cycle's outward command: drop the
        // accumulated Rate lead so the position loop cannot keep pulling the
        // joint toward the soft end while the inward path stays available.
        if (alloc.joint_reseed_requested && control_error == 0)
            (void)gimbal.reseedYawTarget();
        auto status = control_error == 0 ? gimbal.update(input, now_us)
                                         : gimbal.suspend(now_us, WaitReason::Configuration, control_error, true);
        // One commit per physical bus, before the inter-board request is built:
        // this cycle's Yaw execution and its CAN result decide whether a nonzero
        // carrier intent may be published at all.
        int yaw_commit_error = 0;
        if (yaw_started) {
            yaw_commit_error = yaw_bus.commit().error;
            if (yaw_commit_error < 0)
                status.error = yaw_commit_error;
        }
        if (pitch_started) {
            const int error = pitch_bus.commit().error;
            if (error < 0)
                status.error = error;
        }
        const auto yaw_telemetry = gimbal.yawTelemetry();
        const bool yaw_updated = yaw_telemetry.target_sequence != last_yaw_sequence;
        last_yaw_sequence = yaw_telemetry.target_sequence;
        // `RunStatus::Active` may come from Pitch alone, so the carrier request
        // needs its own Yaw evidence: executed this cycle, output valid, CAN
        // committed and the joint measurement still usable.
        const bool yaw_cycle_ok =
            control_error == 0 && yaw_updated && yaw_telemetry.output_valid && yaw_commit_error == 0 && joint_valid;
        if (now_ms >= next_send_ms) {
            next_send_ms = now_ms + 10;
            BigYawRequest request{};
            request.source_sequence = static_cast<std::uint32_t>(input.source_stamp.sequence);
            request.source_age_ms = age(input.source_stamp, now_us);
            request.command_age_ms = age(input.command.stamp, now_ms);
            request.permission = {operator_state.fresh, big_channel_ready, operator_state.remote.stamp};
            // Reference v4 world intent: the peer applies motor sign/gyro control.
            // Zero enable obtains initial real feedback before any nonzero follow.
            request.mode = big_channel_ready && yaw_cycle_ok ? BigYawMode::FollowCenter : BigYawMode::Disabled;
            // Allocated world intent. Zero whenever the carrier channel is not
            // authorized, the allocation was rejected, or this cycle's Yaw did
            // not execute and commit.
            request.target_rate_rad_s = big_channel_ready && yaw_cycle_ok && peer_feedback_valid && alloc.enabled
                                            ? alloc.big.rate_rad_s
                                            : 0;
            if (peer_state.online && peer_state.peer.sender_boot_id) {
                GimbalImuYaw sample{};
                sample.valid = heading_fresh;
                sample.yaw_rad = heading.yaw_rad;
                sample.yaw_rate_rad_s = heading.rate_rad_s;
                sample.zero_generation = heading.generation;
                sample.sample_sequence = static_cast<std::uint32_t>(heading.stamp.sequence);
                sample.source_age_ms = heading.stamp.valid && now_us >= heading.stamp.time_us
                                           ? static_cast<std::uint32_t>(
                                                 std::min<std::uint64_t>((now_us - heading.stamp.time_us + 999) / 1000,
                                                                         UINT32_MAX))
                                           : UINT32_MAX;
                sample.receiver_boot_id = peer_state.peer.sender_boot_id;
                sample.stamp = {now_ms, request_sequence + 1, true};
                endpoint.submitGimbalImuYaw(sample);
            }
            request.stamp = {now_ms, ++request_sequence, true};
            endpoint.submitBigYaw(request);
            auto wheel_command = frame.decision.command.chassis;
            // Reject a cached command from the previous RC mode. In navigation
            // wz is supplied by the navigation frame, never by the spin profile.
            const bool navigation_command = navigation_allowed && wheel_command.source == ControlSource::Autonomous;
            const bool manual_command = operator_state.run_allowed &&
                                        wheel_command.source != ControlSource::Autonomous;
            if (!navigation_command && !manual_command)
                wheel_command.mode = ChassisMode::Disabled;
            if (manual_command && wheel_command.mode != ChassisMode::Disabled &&
                !(frame.decision.chassis_reasons & PermissionDenied))
                wheel_command.wz_rad_s = spin_out.applied_rad_s;
            endpoint.submit(wheel_command);
        }
        endpoint.setStatus(status);
        const auto reason = !setup_ready || onboard_error                ? installed::DiagnosticReason::InitFailed
                            : !operator_state.fresh                      ? installed::DiagnosticReason::RemoteStale
                            : !gimbal_allowed                            ? installed::DiagnosticReason::Disarmed
                            : !installed::imu_mounting_confirmed         ? installed::DiagnosticReason::Configuration
                            : !heading_fresh                             ? installed::DiagnosticReason::ImuStale
                            : !peer_state.online || !peer_feedback_valid ? installed::DiagnosticReason::PeerStale
                            : !joint_valid || !yaw_running               ? installed::DiagnosticReason::FeedbackStale
                            : status.state == RunState::Active           ? installed::DiagnosticReason::Ready
                                                                         : installed::DiagnosticReason::CommandStale;
        (void)status_led.setStatus(
            installed::statusLedFor(reason, status.state == RunState::Active, false, status.error));
        const auto &ed = carrier_snapshot.diagnostics.estimator;
        const float unavailable = std::numeric_limits<float>::quiet_NaN();
        // Fixed channel map documented in docs/dev/大Yaw突发跟转诊断操作指南.md
        // (schema 104, I19..I48).
        const std::array<float, 30> trace{
            104,                                                        // I19 dual-Yaw planar gimbal schema
            float(unsigned(heading_reason)),                            // I20 live reason
            float(unsigned(carrier_snapshot.sample.attitude_quality)),  // I21 quality
            float(heading_fresh),                                        // I22 heading usable
            heading_fresh ? heading.yaw_rad : unavailable,               // I23 world yaw
            carrier_snapshot.sample.gyro_rad_s.stamp.valid
                ? carrier_snapshot.sample.gyro_rad_s.value.z : unavailable, // I24 gyro z
            carrier_snapshot.sample.accel_m_s2.stamp.valid
                ? core::norm(carrier_snapshot.sample.accel_m_s2.value) : unavailable, // I25 accel norm
            float(ed.correction),                                        // I26 correction outcome
            ed.gravity_error_m_s2,                                       // I27 |a|-g absolute error
            ed.innovation_nis,                                           // I28 innovation NIS
            ed.time_us ? float(ed.result) : unavailable,                 // I29 estimator result
            float(carrier_snapshot.diagnostics.io_errors & 0xffff),     // I30 I/O error count
            float(heading_loss_count & 0xffff),                          // I31 loss count
            float(last_heading_loss_ms & 0xffff),                        // I32 event time low
            float((last_heading_loss_ms >> 16) & 0xffff),                // I33 event time high
            have_heading_loss ? float(loss_estimator.correction) : unavailable, // I34 event correction
            loss_estimator.gravity_error_m_s2,                           // I35 event accel error
            loss_estimator.innovation_nis,                               // I36 event NIS
            float((!setup_ready ? 1u : 0u) | (!operator_state.fresh ? 2u : 0u) |
                  (!gimbal_allowed ? 4u : 0u) | (!peer_state.online ? 8u : 0u) |
                  (!installed::imu_mounting_confirmed ? 16u : 0u) | (!heading_fresh ? 32u : 0u) |
                  (!joint_valid ? 64u : 0u) | (!yaw_running ? 128u : 0u)), // I37 disabled prerequisites; zero=ready
            alloc.big.rate_rad_s,                                        // I38 big-Yaw world intent
            peer_feedback_valid ? float(feedback.armed) : unavailable,   // I39 peer armed
            float(age(carrier_snapshot.sample.orientation.stamp, now_us)), // I40 orientation age
            float(age(carrier_snapshot.sample.gyro_rad_s.stamp, now_us)), // I41 gyro age
            spin_out.applied_rad_s,                                      // I42 applied variable spin rate
            std::min(alloc.brake_margin_pos_rad, alloc.brake_margin_neg_rad), // I43 tightest outward travel
            float(heading_loss_duration_us) * .001f,                     // I44 current/last loss duration
            float(dt_us),                                                // I45 current cycle
            raw_head_rate_rad_s,                                          // I46 raw head world-rate target
            alloc.head.rate_rad_s,                                        // I47 feasible head world rate
            float((alloc.enabled ? 1u : 0u) | (alloc.travel_limited ? 2u : 0u) |
                  (alloc.big_saturated ? 4u : 0u) | (alloc.tracking_limited ? 8u : 0u) |
                  (allocation.big_yaw_available ? 16u : 0u) | (alloc.vision_mode ? 512u : 0u) |
                  (alloc.joint_reseed_requested ? 1024u : 0u) |
                  (static_cast<unsigned>(alloc.reason) << 5))}; // I48 allocation flags/reason
        diagnostics.status({.reason = reason,
                            .valid = joint_valid,
                            .enabled = status.state == RunState::Active,
                            .target = static_cast<float>(gimbal.yawTargetRad()),
                            .feedback = ys.feedback.position_rad,
                            .age_ms = age(carrier_snapshot.sample.planar_yaw.yaw_rad.stamp, now_us),
                            .dt_us = float(have_heading_loss ? last_heading_loss_dt_us : dt_us),
                            .effort = gimbal.yawTelemetry().effort_command,
                            .detail = float(unsigned(have_heading_loss ? last_heading_loss_reason : heading_reason)),
                            .error = status.error ? status.error : onboard_error,
                            .extra0 = alloc.center_error_rad,
                            .extra1 = heading_fresh ? heading.rate_rad_s : std::numeric_limits<float>::quiet_NaN(),
                            .extended = true,
                            .extension = trace});
        deadline += period;
        const auto ticks = k_uptime_ticks();
        if (deadline <= ticks)
            deadline += ((ticks - deadline) / period + 1) * period;
        k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
    }
}
