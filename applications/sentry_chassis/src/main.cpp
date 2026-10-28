#include <algorithm>
#include <cerrno>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <core/clock.hpp>
#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <robotics/vehicle/motion_profile.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/vehicle/big_yaw_profile.hpp>
#include <robotics/vehicle/chassis_profile.hpp>
#include <robotics/vehicle/diagnostics.hpp>
#include <zephyr/drivers/regulator.h>
#include "../../../samples/robotics/common/operator_controls.hpp"

namespace {
using namespace skywalker;
using namespace robotics;
namespace installed = robotics::vehicle;
communication::AsyncUart::DmaBuffers link_dma __nocache;
std::uint32_t age(const MessageStamp &stamp, std::uint64_t now_ms) {
    return stamp.valid && now_ms >= stamp.timestamp_ms
               ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now_ms - stamp.timestamp_ms, UINT32_MAX))
               : UINT32_MAX;
}
} // namespace

int main() {
    static installed::Diagnostics diagnostics;
    diagnostics.start("sentry_chassis", "chassis", "steer=0.8A; drive=0.5A; big-yaw=1rad/s");
    static communication::ConfiguredInterBoardTransport transport(
        {.kind = communication::InterBoardTransportKind::Uart,
         .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))}, &link_dma);
    static communication::InterBoardEndpoint endpoint(transport, {BoardRole::ChassisController});
    static motor::Motor s0(installed::steerHardware(0)), s1(installed::steerHardware(1)),
                        s2(installed::steerHardware(2)), s3(installed::steerHardware(3));
    static motor::Motor d0(installed::wheelHardware(0)), d1(installed::wheelHardware(1)),
                        d2(installed::wheelHardware(2)), d3(installed::wheelHardware(3));
    static motor::Motor big_drive(installed::bigYawHardware());
    static motor::Group wheels(s0, s1, s2, s3, d0, d1, d2, d3);
    static motor::CanBus steer_bus(DEVICE_DT_GET(DT_NODELABEL(can1))),
                         drive_bus(DEVICE_DT_GET(DT_NODELABEL(can2))),
                         big_bus(DEVICE_DT_GET(DT_NODELABEL(can3)));
    static SwerveHardware hardware({{&s0, &s1, &s2, &s3, &d0, &d1, &d2, &d3}}, wheels,
                                   installed::swerveHardwareConfig());
    static ChassisExecutor chassis(hardware, installed::swerveConfig(), installed::chassisExecutionConfig());
    static control::QuaternionEkf estimator({});
    static imu::Bmi088Imu body_source(DEVICE_DT_GET(DT_ALIAS(accel0)),DEVICE_DT_GET(DT_ALIAS(gyro0)),
        {.reference=installed::chassis_reference,.sensor_to_body=installed::chassis_sensor_to_body},&estimator);
    static imu::ImuReceiver body(body_source,{.poll_interval_us=500,.priority=5});
    static ImuHeadingReference body_heading;
    static ChassisMotionAdapter motion(installed::chassisMotionConfig());
    const auto motion_config=installed::chassisMotionConfig();
    const bool need_body=motion_config.frame!=ChassisMotionAdapter::Frame::Body || motion_config.compensation_enabled;
    const int body_error=need_body ? body.start() : 0;
    static BigYawExecutor big_axis(big_drive, installed::bigYawMotorConfig(), installed::bigYawExecutionConfig());
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
    const int power_error = installed::connections_confirmed
                                ? regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1))) : -ENODEV;
    diagnostics.event("steer_bus", steer_error);
    diagnostics.event("drive_bus", drive_error);
    diagnostics.event("big_yaw", big_error);
    diagnostics.event("chassis", chassis_error);
    diagnostics.event("body_imu",body_error);
    diagnostics.event("motor_rail", power_error);
    samples::control::OperatorControlConsumer operator_controls;
    std::uint64_t peer_boot = 0;
    int previous_big_commit_error = 0;
    std::int64_t deadline = k_uptime_ticks();
    const auto period = k_us_to_ticks_ceil64(installed::chassis_period_us);
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        endpoint.poll(now_ms);
        const auto rx = endpoint.snapshot();
        const auto authority = operator_controls.update(rx, now_ms);
        if (rx.peer.stamp.valid && peer_boot != rx.peer.sender_boot_id) {
            peer_boot = rx.peer.sender_boot_id;
            (void)chassis.suspend(now_us, WaitReason::Reference, -ESTALE);
            (void)big_axis.suspend(now_us, WaitReason::Reference, -ESTALE);
        }
        ChassisExecutionInputs input{};
        input.command = rx.control.command;
        if (rx.control.global_action != SafetyAction::Active || !authority.run_allowed ||
            !rx.local_boot_id || rx.control.receiver_boot_id != rx.local_boot_id)
            input.command.mode = ChassisMode::Disabled;
        const auto body_sample=body_heading.update(need_body ? body.snapshot() : imu::Snapshot{},!authority.run_allowed,now_us);
        input.command=motion.update(input.command,body_sample,rx.gimbal_imu,rx.local_boot_id,!authority.run_allowed,now_ms);
        input.source_stamp = {input.command.stamp.timestamp_ms * 1000, input.command.stamp.sequence,
                               input.command.stamp.valid};
        input.transport_ready = steer_error == 0 && drive_error == 0 && chassis_error == 0 &&
                                 power_error == 0 && rx.online;
        input.emergency_stop = authority.emergency_stop;
        input.clear_estop = authority.clear_estop;
        // This fixed manual entry uses the real peer RC authority and bounded
        // current profiles; measured-power/referee execution is a later stage.
        auto status = chassis_error == 0 ? chassis.update(input, now_us)
                                        : chassis.suspend(now_us, WaitReason::Configuration, chassis_error, true);
        BigYawExecutionInputs big_input{};
        big_input.request = rx.big_yaw_request;
        if (!authority.run_allowed)
            big_input.request.mode = BigYawMode::Disabled;
        big_input.head=rx.gimbal_imu;
        big_input.peer_boot_id=rx.peer.sender_boot_id;
        big_input.local_boot_id = rx.local_boot_id;
        big_input.peer_online = rx.online;
        big_input.transport_ready = big_error == 0 && power_error == 0 && rx.online;
        big_input.emergency_stop = authority.emergency_stop;
        big_input.clear_estop = authority.clear_estop;
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
        const auto reason = chassis_error || big_error || power_error
                                ? installed::DiagnosticReason::InitFailed
                            : !rx.online ? installed::DiagnosticReason::PeerStale
                            : !rx.operator_control_valid ? installed::DiagnosticReason::Permission
                            : !authority.run_allowed ? installed::DiagnosticReason::Disarmed
                            : status.reason == WaitReason::Feedback || big_status.reason == WaitReason::Feedback
                                ? installed::DiagnosticReason::FeedbackStale
                            : status.state == RunState::Active || big_status.state == RunState::Active
                                ? installed::DiagnosticReason::Ready : installed::DiagnosticReason::CommandStale;
        diagnostics.status({.reason = reason,
                            .valid = feedback.valid,
                            .enabled = status.state == RunState::Active || big_status.state == RunState::Active,
                            .target = big_input.request.target_rate_rad_s,
                            .feedback = feedback.actual_rate_rad_s,
                            .age_ms = age(rx.control.command.stamp, now_ms)});
        deadline += period;
        const auto ticks = k_uptime_ticks();
        if (deadline <= ticks)
            deadline += ((ticks - deadline) / period + 1) * period;
        k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
    }
}
