#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/vehicle/bringup_chassis_policy.hpp>
#include <robotics/vehicle/diagnostics.hpp>
#include <zephyr/drivers/regulator.h>
#include "../../../robotics/common/operator_controls.hpp"

namespace {
using namespace skywalker;
using namespace robotics;
namespace installed = robotics::vehicle;
namespace stage = installed::bringup;
communication::AsyncUart::DmaBuffers link_dma __nocache;
K_THREAD_STACK_DEFINE(link_stack, 4096);
k_thread link_thread;
void linkTask(void *context, void *, void *) {
    auto &endpoint = *static_cast<communication::InterBoardEndpoint *>(context);
    for (;;) {
        endpoint.poll(k_uptime_get());
        k_sleep(K_MSEC(1));
    }
}
std::uint32_t age(const MessageStamp &stamp, std::uint64_t now_ms) {
    return stamp.valid && now_ms >= stamp.timestamp_ms
               ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now_ms - stamp.timestamp_ms, UINT32_MAX))
               : UINT32_MAX;
}
} // namespace

int main() {
    static installed::Diagnostics diagnostics;
    diagnostics.start("chassis_drive_link", "chassis", "vx,vy=0.1m/s; wz=0.2rad/s; steer,drive=0.3A");
    diagnostics.event("stage_steer_limit_mA", int(stage::steer_current_a * 1000));
    diagnostics.event("stage_drive_limit_mA", int(stage::drive_current_a * 1000));
    diagnostics.event("stage_velocity_limit_mm_s", int(stage::chassis_speed_m_s * 1000));
    static communication::ConfiguredInterBoardTransport transport(
        {.kind = communication::InterBoardTransportKind::Uart,
         .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))}, &link_dma);
    static communication::InterBoardEndpoint endpoint(transport, {BoardRole::ChassisController});
    k_thread_create(&link_thread, link_stack, K_THREAD_STACK_SIZEOF(link_stack), linkTask,
                    &endpoint, nullptr, nullptr, 6, 0, K_NO_WAIT);
    static motor::Motor s0(installed::steerHardware(0)), s1(installed::steerHardware(1)),
                        s2(installed::steerHardware(2)), s3(installed::steerHardware(3));
    static motor::Motor d0(installed::wheelHardware(0)), d1(installed::wheelHardware(1)),
                        d2(installed::wheelHardware(2)), d3(installed::wheelHardware(3));
    static motor::Group motors(s0, s1, s2, s3, d0, d1, d2, d3);
    static motor::CanBus steer_bus(DEVICE_DT_GET(DT_NODELABEL(can1)));
    static motor::CanBus drive_bus(DEVICE_DT_GET(DT_NODELABEL(can2)));
    static SwerveHardware hardware({{&s0, &s1, &s2, &s3, &d0, &d1, &d2, &d3}}, motors,
                                   installed::swerveHardwareConfig());
    static ChassisExecutor chassis(hardware, stage::chassisConfig(), stage::executionConfig());
    int steer_error = installed::connections_confirmed ? steer_bus.attach(s0, s1, s2, s3) : -ENODEV;
    int drive_error = installed::connections_confirmed ? drive_bus.attach(d0, d1, d2, d3) : -ENODEV;
    if (steer_error == 0)
        steer_error = steer_bus.start();
    if (drive_error == 0)
        drive_error = drive_bus.start();
    const bool steer_started = steer_error == 0, drive_started = drive_error == 0;
    const int controller_error = chassis.begin();
    const int power_error = installed::connections_confirmed
                                ? regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1))) : -ENODEV;
    diagnostics.event("steer_bus", steer_error);
    diagnostics.event("drive_bus", drive_error);
    diagnostics.event("chassis", controller_error);
    diagnostics.event("motor_rail", power_error);
    const int init_error = steer_error ? steer_error : drive_error ? drive_error
                           : controller_error ? controller_error : power_error;
    samples::control::OperatorControlConsumer operator_controls;
    bool emergency_latched = false;
    std::uint64_t peer_boot = 0;
    core::TimeUs previous_us = 0;
    std::int64_t deadline = k_uptime_ticks();
    const auto period = k_us_to_ticks_ceil64(installed::chassis_period_us);
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        const float dt_us = previous_us && now_us > previous_us ? float(now_us - previous_us) : 0;
        previous_us = now_us;
        const auto rx = endpoint.snapshot();
        const auto authority = operator_controls.update(rx, now_ms);
        if (authority.emergency_stop)
            emergency_latched = true;
        if (authority.clear_estop)
            emergency_latched = false;
        if (rx.peer.stamp.valid && peer_boot != rx.peer.sender_boot_id) {
            peer_boot = rx.peer.sender_boot_id;
            (void)chassis.suspend(now_us, WaitReason::Reference, -ESTALE);
        }
        ChassisExecutionInputs input{};
        input.command = stage::boundedCommand(rx.control.command);
        // Fixed complete low-speed body-velocity stage; Spin retains its protocol semantics.
        if (!rx.operator_control_valid || !authority.run_allowed || emergency_latched ||
            rx.control.global_action != SafetyAction::Active || !rx.local_boot_id ||
            rx.control.receiver_boot_id != rx.local_boot_id)
            input.command.mode = ChassisMode::Disabled;
        input.source_stamp = {input.command.stamp.timestamp_ms * 1000, input.command.stamp.sequence,
                               input.command.stamp.valid};
        input.transport_ready = init_error == 0 && rx.online;
        input.emergency_stop = emergency_latched;
        input.clear_estop = authority.clear_estop;
        auto status = init_error == 0 ? chassis.update(input, now_us)
            : chassis.suspend(now_us, WaitReason::Configuration, init_error, true);
        if (steer_started) {
            const int result = steer_bus.commit().error;
            if (result < 0)
                status.error = result;
        }
        if (drive_started) {
            const int result = drive_bus.commit().error;
            if (result < 0)
                status.error = result;
        }
        if (status.error && status.state == RunState::Active) {
            status.ready = false;
            status.state = RunState::Recovering;
            status.reason = WaitReason::Transport;
        }
        endpoint.setStatus(status);
        // Multiplex the four modules in the same stable telemetry layout.
        const std::size_t index = (now_ms / installed::diagnostic_period_ms) % 4;
        const auto &output = chassis.output().module[index];
        const auto steer_view = hardware.motorAt(index).snapshot();
        const auto drive_view = hardware.motorAt(index + 4).snapshot();
        const bool drive_valid = drive_view.feedback_fresh &&
            (drive_view.feedback.valid & motor::FeedbackVelocity) && std::isfinite(drive_view.feedback.velocity_rad_s);
        const bool steer_valid = steer_view.feedback_fresh &&
            (steer_view.feedback.valid & motor::FeedbackAbsolutePosition) &&
            std::isfinite(steer_view.feedback.absolute_position_rad);
        const auto reason = init_error ? installed::DiagnosticReason::InitFailed
            : !rx.online ? installed::DiagnosticReason::PeerStale
            : !rx.operator_control_valid || emergency_latched ? installed::DiagnosticReason::Permission
            : !authority.run_allowed ? installed::DiagnosticReason::Disarmed
            : status.reason == WaitReason::Command ? installed::DiagnosticReason::CommandStale
            : status.state != RunState::Active ? installed::DiagnosticReason::FeedbackStale
                                               : installed::DiagnosticReason::Ready;
        diagnostics.status({.reason = reason,
                            .valid = drive_valid,
                            .enabled = status.state == RunState::Active,
                            .target = output.drive_target_rad_s,
                            .feedback = installed::wheel[index].direction * drive_view.feedback.velocity_rad_s,
                            .age_ms = age(input.command.stamp, now_ms),
                            .dt_us = dt_us,
                            .effort = output.drive_effort * chassis.effortScale(),
                            .detail = float(index),
                            .error = status.error ? status.error : rx.error,
                            .extra0 = output.optimized_angle_rad,
                            .extra1 = steer_valid ? installed::steer[index].direction * steer_view.feedback.absolute_position_rad
                                                  : std::numeric_limits<float>::quiet_NaN()});
        deadline += period;
        const auto ticks = k_uptime_ticks();
        if (deadline <= ticks)
            deadline += ((ticks - deadline) / period + 1) * period;
        k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
    }
}
