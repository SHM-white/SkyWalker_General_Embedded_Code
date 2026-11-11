#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <core/clock.hpp>
#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <robotics/vehicle/motion_profile.hpp>
#include <robotics/chassis/reference_v4_motion.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/execution/big_yaw_heading_follower.hpp>
#include <robotics/vehicle/big_yaw_profile.hpp>
#include <robotics/vehicle/chassis_profile.hpp>
#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/led_status.hpp>
#include <drivers/led/status_led.hpp>
#include <zephyr/drivers/regulator.h>

namespace {
using namespace skywalker;
using namespace robotics;
namespace installed = robotics::vehicle;
communication::AsyncUart::DmaBuffers link_dma __nocache;
enum class BigYawGate : std::uint8_t {
    None = 0,
    PeerDisabled = 1,
    BodyGyro = 3,
    PeerImu = 4,
    MissingRequest = 5,
};
std::uint32_t age(const MessageStamp &stamp, std::uint64_t now_ms) {
    return stamp.valid && now_ms >= stamp.timestamp_ms
               ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now_ms - stamp.timestamp_ms, UINT32_MAX))
               : UINT32_MAX;
}
} // namespace

int main() {
    static installed::Diagnostics diagnostics;
    diagnostics.start("sentry_chassis", "chassis", "steer=0.8A; drive=0.5A; big-yaw joint limit=installed profile");
    static led::StatusLed status_led;
    const int status_led_error = status_led.start();
    if constexpr (led::StatusLed::available)
        diagnostics.event("status_led", status_led_error);
    (void)status_led.setStatus(led::LedStatus::Initializing);
    static communication::ConfiguredInterBoardTransport transport({.kind = communication::InterBoardTransportKind::Uart,
                                                                   .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))},
                                                                  &link_dma);
    static communication::InterBoardEndpoint endpoint(transport, {BoardRole::ChassisController});
    static motor::Motor s0(installed::steerHardware(0)), s1(installed::steerHardware(1)),
        s2(installed::steerHardware(2)), s3(installed::steerHardware(3));
    static motor::Motor d0(installed::wheelHardware(0)), d1(installed::wheelHardware(1)),
        d2(installed::wheelHardware(2)), d3(installed::wheelHardware(3));
    static motor::Motor big_drive(installed::bigYawHardware());
    static motor::Group wheels(s0, s1, s2, s3, d0, d1, d2, d3);
    static motor::CanBus steer_bus(DEVICE_DT_GET(DT_NODELABEL(can1))), drive_bus(DEVICE_DT_GET(DT_NODELABEL(can2))),
        big_bus(DEVICE_DT_GET(DT_NODELABEL(can3)));
    static SwerveHardware hardware({{&s0, &s1, &s2, &s3, &d0, &d1, &d2, &d3}}, wheels,
                                   installed::swerveHardwareConfig());
    static ChassisExecutor chassis(hardware, installed::swerveConfig(), installed::chassisExecutionConfig());
    static control::QuaternionEkf estimator({});
    static imu::Bmi088Imu body_source(DEVICE_DT_GET(DT_ALIAS(accel0)), DEVICE_DT_GET(DT_ALIAS(gyro0)),
                                      {.reference = installed::chassis_reference,
                                       .sensor_to_body = installed::chassis_sensor_to_body},
                                      &estimator);
    static imu::ImuReceiver body(body_source, {.poll_interval_us = 500, .priority = 5});
    static ImuHeadingReference body_heading;
    static ChassisMotionAdapter motion(installed::chassisMotionConfig());
    // Reference mode=1 requires BMI088 body yaw-rate compensation.
    const bool need_body = true;
    const int body_error = need_body ? body.start() : 0;
    static const auto big_yaw_config = installed::bigYawExecutionConfig();
    // One owner for the world-heading capture/integral. The executor keeps the
    // native velocity backend, sign, ratio, slew and command limits.
    static BigYawWorldHeadingFollower heading_follower(installed::bigYawHeadingFollowerConfig());
    static BigYawExecutor big_axis(big_drive, installed::bigYawMotorConfig(), big_yaw_config);
    int steer_error = installed::connections_confirmed ? steer_bus.attach(s0, s1, s2, s3) : -ENODEV;
    int drive_error = installed::connections_confirmed ? drive_bus.attach(d0, d1, d2, d3) : -ENODEV;
    int big_error = installed::connections_confirmed ? big_bus.attach(big_drive) : -ENODEV;
    if (steer_error == 0)
        steer_error = steer_bus.start();
    if (drive_error == 0)
        drive_error = drive_bus.start();
    if (big_error == 0)
        big_error = big_bus.start();
    const bool steer_started = steer_error == 0, drive_started = drive_error == 0, big_started = big_error == 0;
    const int chassis_error = steer_started && drive_started ? chassis.begin() : -ENODEV;
    if (big_error == 0)
        big_error = big_axis.begin();
    const int power_error = installed::connections_confirmed ? regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)))
                                                             : -ENODEV;
    diagnostics.event("steer_bus", steer_error);
    diagnostics.event("drive_bus", drive_error);
    diagnostics.event("big_yaw", big_error);
    diagnostics.event("chassis", chassis_error);
    diagnostics.event("body_imu", body_error);
    diagnostics.event("motor_rail", power_error);
    std::uint64_t peer_boot = 0;
    int previous_big_commit_error = 0;
    core::TimeUs previous_cycle_us = 0, last_big_stop_dt_us = 0;
    bool big_was_requested = false, big_was_active = false, have_big_stop = false;
    WaitReason last_big_stop_reason = WaitReason::None;
    int last_big_stop_error = 0;
    BigYawGate last_big_stop_gate = BigYawGate::None;
    std::uint32_t big_stop_count = 0;
    std::uint64_t last_big_stop_ms = 0;
    float last_big_stop_drive_status = -1;
    std::int64_t deadline = k_uptime_ticks();
    const auto period = k_us_to_ticks_ceil64(installed::chassis_period_us);
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        const auto dt_us = previous_cycle_us && now_us > previous_cycle_us ? now_us - previous_cycle_us : 0;
        const bool cycle_valid = dt_us && dt_us <= big_yaw_config.max_cycle_us;
        previous_cycle_us = now_us;
        endpoint.poll(now_ms);
        const auto rx = endpoint.snapshot();
        if (rx.peer.stamp.valid && peer_boot != rx.peer.sender_boot_id) {
            peer_boot = rx.peer.sender_boot_id;
            (void)chassis.suspend(now_us, WaitReason::Reference, -ESTALE);
            (void)big_axis.suspend(now_us, WaitReason::Reference, -ESTALE);
        }
        ChassisExecutionInputs input{};
        input.command = rx.control.command;
        if (rx.control.global_action != SafetyAction::Active || !rx.online || !rx.local_boot_id ||
            rx.control.receiver_boot_id != rx.local_boot_id)
            input.command.mode = ChassisMode::Disabled;
        const auto body_snapshot = body.snapshot();
        const auto body_sample = body_heading.update(body_snapshot, input.command.mode == ChassisMode::Disabled,
                                                     now_us);
        // Unlike our old Body-frame v4 implementation, the reference sends
        // gimbal-relative translation. The absolute encoder zero is fixed.
        const auto yaw_feedback = big_drive.snapshot();
        // Use the continuous native protocol coordinate, not the raw +-PMAX
        // value: a J4310 wraps every 25 rad, which is not a multiple of 2*pi.
        // It keeps the calibrated forward zero and survives an RX gap.
        const bool encoder_valid = yaw_feedback.feedback_fresh && yaw_feedback.native_unwrapped_position_valid &&
                                   std::isfinite(yaw_feedback.native_unwrapped_position_rad) &&
                                   isFresh({yaw_feedback.feedback.timestamp_ms, 1, true}, now_ms, 100);
        input.command = referenceV4ChassisToBody(input.command, encoder_valid,
                                                 yaw_feedback.native_unwrapped_position_rad,
                                                 installed::reference_big_yaw_forward_zero_rad,
                                                 installed::reference_big_yaw.direction,
                                                 installed::reference_big_yaw.gear_ratio,
                                                 installed::chassis_spin_forward_scale,
                                                 installed::chassis_spin_lateral_scale);
        input.command = motion.update(input.command, body_sample, rx.gimbal_imu, rx.local_boot_id,
                                      input.command.mode == ChassisMode::Disabled, now_ms, rx.peer);
        input.transport_ready = steer_error == 0 && drive_error == 0 && chassis_error == 0 && power_error == 0 &&
                                rx.online;
        // This fixed manual entry uses the real peer RC authority and bounded
        // current profiles; measured-power/referee execution is a later stage.
        auto status = chassis_error == 0 ? chassis.update(input, now_us)
                                         : chassis.suspend(now_us, WaitReason::Configuration, chassis_error, true);
        BigYawExecutionInputs big_input{};
        big_input.request = rx.big_yaw_request;
        auto big_gate = !rx.big_yaw_request.stamp.valid                  ? BigYawGate::MissingRequest
                        : big_input.request.mode == BigYawMode::Disabled ? BigYawGate::PeerDisabled
                                                                         : BigYawGate::None;
        bool heading_follow_active = false;
        if (big_input.request.mode == BigYawMode::FollowCenter && !installed::big_yaw_heading_lock_enabled) {
            // The wire rate is a *world* rate for the head, while the native
            // velocity backend takes a physical joint rate. The follower owns
            // heading capture and the rate PI; the executor still applies
            // direction/ratio, slew and the joint limit exactly once.
            const bool body_gyro_valid = body_snapshot.state == imu::State::Running &&
                                         core::fresh(body_snapshot.sample.gyro_rad_s.stamp, now_us, 50000) &&
                                         core::finite(body_snapshot.sample.gyro_rad_s.value);
            float joint_rate = 0;
            int follow_error = -ESTALE;
            if (body_gyro_valid) {
                BigYawWorldHeadingInputs follow{};
                follow.head = rx.gimbal_imu;
                follow.now_ms = now_ms;
                follow.body_rate_rad_s = body_snapshot.sample.gyro_rad_s.value.z;
                follow.desired_world_rate_rad_s = big_input.request.target_rate_rad_s;
                // A missed interval stages no motor output this cycle; do not
                // integrate a heading step over it either.
                follow.dt_s = cycle_valid ? static_cast<float>(dt_us) * 1e-6f : 0;
                follow_error = heading_follower.step(follow, joint_rate);
            }
            if (follow_error < 0) {
                big_input.request.mode = BigYawMode::Disabled;
                big_gate = body_gyro_valid ? BigYawGate::PeerImu : BigYawGate::BodyGyro;
            }
            else {
                heading_follow_active = true;
                big_input.request.target_rate_rad_s = std::clamp(joint_rate, -big_yaw_config.rate_abs_max_rad_s,
                                                                 big_yaw_config.rate_abs_max_rad_s);
            }
        }
        if (!heading_follow_active && !installed::big_yaw_heading_lock_enabled)
            heading_follower.reset();
        big_input.head = rx.gimbal_imu;
        big_input.peer_boot_id = rx.peer.sender_boot_id;
        big_input.local_boot_id = rx.local_boot_id;
        big_input.peer_online = rx.online;
        big_input.transport_ready = big_error == 0 && power_error == 0 && rx.online;
        const auto big_status = big_error == 0 ? big_axis.update(big_input, now_us)
                                               : big_axis.suspend(now_us, WaitReason::Configuration, big_error, true);
        // One writer publishes each of the three physical buses once per cycle.
        if (steer_started) {
            const int error = steer_bus.commit().error;
            if (error < 0)
                status.error = error;
        }
        if (drive_started) {
            const int error = drive_bus.commit().error;
            if (error < 0)
                status.error = error;
        }
        if (big_started) {
            const int error = big_bus.commit().error;
            if (error != previous_big_commit_error) {
                diagnostics.event("big_yaw_commit", error);
                previous_big_commit_error = error;
            }
        }
        endpoint.setStatus(status);
        endpoint.setBigYawFeedback(big_axis.feedback());
        const auto feedback = big_axis.feedback();
        const auto big_motor = big_drive.snapshot();
        // Retain short interruptions across the 20 ms diagnostic sampling period.
        // Enabling -> withdrawn also counts, even before the first Active state.
        if ((big_was_requested && !big_status.requested) || (big_was_active && !feedback.armed)) {
            have_big_stop = true;
            last_big_stop_reason = big_status.reason;
            last_big_stop_error = big_status.error ? big_status.error : big_motor.last_fault.error;
            last_big_stop_gate = big_gate;
            last_big_stop_dt_us = dt_us;
            ++big_stop_count;
            last_big_stop_ms = now_ms;
            last_big_stop_drive_status = big_motor.native_drive_status_valid
                                             ? float(big_motor.native_drive_status) : -1;
        }
        big_was_requested = big_status.requested;
        big_was_active = feedback.armed;
        const auto reason = chassis_error || big_error || power_error ? installed::DiagnosticReason::InitFailed
                            : !rx.online                              ? installed::DiagnosticReason::PeerStale
                            : status.reason == WaitReason::Feedback || big_status.reason == WaitReason::Feedback
                                ? installed::DiagnosticReason::FeedbackStale
                            : status.state == RunState::Active || big_status.state == RunState::Active
                                ? installed::DiagnosticReason::Ready
                                : installed::DiagnosticReason::CommandStale;
        const int execution_error = status.error < 0 ? status.error : big_status.error;
        (void)status_led.setStatus(
            installed::statusLedFor(reason, status.state == RunState::Active || big_status.state == RunState::Active,
                                    false, execution_error));
        const auto big_reason = big_error || power_error                   ? installed::DiagnosticReason::InitFailed
                                : !rx.online                               ? installed::DiagnosticReason::PeerStale
                                : big_status.reason == WaitReason::Command ? installed::DiagnosticReason::CommandStale
                                : big_status.reason == WaitReason::Transport
                                    ? installed::DiagnosticReason::TransportError
                                : big_status.reason == WaitReason::Feedback ? installed::DiagnosticReason::FeedbackStale
                                : big_status.reason == WaitReason::Reference ? installed::DiagnosticReason::ImuStale
                                : big_status.reason == WaitReason::Configuration
                                    ? installed::DiagnosticReason::Configuration
                                : big_status.reason == WaitReason::Drive ? installed::DiagnosticReason::TransportError
                                : big_status.reason == WaitReason::Power ? installed::DiagnosticReason::Permission
                                : big_status.reason == WaitReason::Cycle ? installed::DiagnosticReason::CycleInvalid
                                : feedback.armed                         ? installed::DiagnosticReason::Ready
                                                                         : installed::DiagnosticReason::Disarmed;
        const float unavailable = std::numeric_limits<float>::quiet_NaN();
        const auto &hd = heading_follower.diagnostics();
        const std::array<float, 27> trace{
            101,                                                        // I19 schema/board
            float(unsigned(big_gate)),                                  // I20 live gate
            float(unsigned(big_status.reason)),                         // I21 live execution reason
            float(rx.gimbal_imu.valid),                                  // I22 peer heading usable
            rx.gimbal_imu.valid ? rx.gimbal_imu.yaw_rad : unavailable,    // I23 peer world yaw
            rx.gimbal_imu.valid ? rx.gimbal_imu.yaw_rate_rad_s : unavailable, // I24 peer world rate
            body_snapshot.state == imu::State::Running &&
                    core::fresh(body_snapshot.sample.gyro_rad_s.stamp, now_us, 50000)
                ? body_snapshot.sample.gyro_rad_s.value.z : unavailable, // I25 body gyro z
            rx.big_yaw_request.target_rate_rad_s,                        // I26 received world intent
            heading_follower.initialized() ? heading_follower.targetHeadingRad() : unavailable, // I27 target heading
            heading_follower.initialized() ? heading_follower.headingErrorRad() : unavailable, // I28 error
            big_axis.nativeVelocityStaged() ? big_axis.stagedJointRateRadS() : unavailable, // I29 post-slew joint rate
            big_motor.native_position_valid ? big_motor.native_position_rad : unavailable, // I30 raw encoder
            big_motor.native_unwrapped_position_valid ? float(big_motor.native_unwrapped_position_rad) : unavailable, // I31 unwrapped
            big_motor.native_unwrapped_position_valid ? float(big_motor.native_wrap_count) : unavailable, // I32 wrap count
            big_motor.native_drive_status_valid ? float(big_motor.native_drive_status) : unavailable, // I33 DM status
            float(unsigned(big_motor.last_fault.reason)),               // I34 last motor fault reason
            float(big_stop_count & 0xffff),                              // I35 stop count
            float(last_big_stop_ms & 0xffff),                            // I36 stop time low
            float((last_big_stop_ms >> 16) & 0xffff),                    // I37 stop time high
            float(hd.capture_count & 0xffff),                            // I38 heading captures
            float(hd.capture_reason),                                    // I39 last capture cause
            heading_follower.rateIntegralRadS(),                         // I40 integral
            last_big_stop_drive_status,                                 // I41 DM status at stop
            float(hd.capture_ms & 0xffff),                               // I42 capture time low
            float((hd.capture_ms >> 16) & 0xffff),                       // I43 capture time high
            float(hd.clamp_count & 0xffff),                              // I44 90-degree clamp entries
            float(previous_big_commit_error)};                          // I45 CAN commit error
        diagnostics.status({.reason = big_reason,
                            .valid = feedback.valid,
                            .enabled = feedback.armed,
                            .target = big_input.request.target_rate_rad_s,
                            .feedback = feedback.actual_rate_rad_s,
                            .age_ms = age(rx.big_yaw_request.stamp, now_ms),
                            .dt_us = float(have_big_stop ? last_big_stop_dt_us : dt_us),
                            .effort = big_motor.feedback.torque_nm,
                            .detail = float(unsigned(have_big_stop ? last_big_stop_reason : big_status.reason)),
                            .error = big_error          ? big_error
                                     : power_error      ? power_error
                                     : have_big_stop    ? last_big_stop_error
                                     : big_status.error ? big_status.error
                                                        : previous_big_commit_error,
                            .extra0 = float(unsigned(big_motor.state)),
                            .extra1 = float(unsigned(have_big_stop ? last_big_stop_gate : big_gate)),
                            .extended = true,
                            .extension = trace});
        deadline += period;
        const auto ticks = k_uptime_ticks();
        if (deadline <= ticks)
            deadline += ((ticks - deadline) / period + 1) * period;
        k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
    }
}
