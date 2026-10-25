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
    constexpr std::size_t module_index = 2;
    static installed::Diagnostics diagnostics;
    diagnostics.start("chassis_module_rl", "chassis", "wheel=0.1m/s; steer,drive=0.3A");
    diagnostics.event("stage_steer_limit_mA", int(stage::steer_current_a * 1000));
    diagnostics.event("stage_drive_limit_mA", int(stage::drive_current_a * 1000));
    diagnostics.event("stage_velocity_limit_mm_s", int(stage::chassis_speed_m_s * 1000));
    static communication::ConfiguredInterBoardTransport transport(
        {.kind = communication::InterBoardTransportKind::Uart,
         .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))}, &link_dma);
    static communication::InterBoardEndpoint endpoint(transport, {BoardRole::ChassisController});
    k_thread_create(&link_thread, link_stack, K_THREAD_STACK_SIZEOF(link_stack), linkTask,
                    &endpoint, nullptr, nullptr, 6, 0, K_NO_WAIT);
    // Only this fixed module is registered. CAN3 and the other six motors have no owner here.
    static motor::Motor steer(installed::steerHardware(module_index));
    static motor::Motor wheel(installed::wheelHardware(module_index));
    static motor::Group motors(steer, wheel);
    static motor::CanBus steer_bus(DEVICE_DT_GET(DT_NODELABEL(can1)));
    static motor::CanBus drive_bus(DEVICE_DT_GET(DT_NODELABEL(can2)));
    const auto controller_config = stage::chassisConfig();
    SwerveModule controller(controller_config.modules[module_index]);
    SwerveKinematics kinematics(controller_config.kinematics);
    int steer_error = installed::connections_confirmed ? steer_bus.attach(steer) : -ENODEV;
    int drive_error = installed::connections_confirmed ? drive_bus.attach(wheel) : -ENODEV;
    if (steer_error == 0)
        steer_error = steer_bus.start();
    if (drive_error == 0)
        drive_error = drive_bus.start();
    const bool steer_started = steer_error == 0, drive_started = drive_error == 0;
    int controller_error = controller.validate();
    if (controller_error == 0)
        controller_error = kinematics.reset(ModuleTargets{});
    const int power_error = installed::connections_confirmed
                                ? regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1))) : -ENODEV;
    diagnostics.event("steer_bus", steer_error);
    diagnostics.event("drive_bus", drive_error);
    diagnostics.event("controller", controller_error);
    diagnostics.event("motor_rail", power_error);
    const int init_error = steer_error ? steer_error : drive_error ? drive_error
                           : controller_error ? controller_error : power_error;
    samples::control::OperatorControlConsumer operator_controls;
    bool emergency_latched = false;
    std::uint32_t status_sequence = 0;
    core::TimeUs previous_us = 0;
    std::int64_t deadline = k_uptime_ticks();
    const auto period = k_us_to_ticks_ceil64(installed::chassis_period_us);
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        const bool cycle = previous_us && now_us > previous_us && now_us - previous_us <= 20000;
        const float dt_s = cycle ? float(now_us - previous_us) / 1000000.0f : 0;
        previous_us = now_us;
        const auto rx = endpoint.snapshot();
        const auto authority = operator_controls.update(rx, now_ms);
        if (authority.emergency_stop)
            emergency_latched = true;
        if (authority.clear_estop)
            emergency_latched = false;
        auto command = stage::boundedCommand(rx.control.command);
        const bool authorized = rx.online && rx.operator_control_valid && authority.run_allowed &&
                                !emergency_latched && rx.control.global_action == SafetyAction::Active &&
                                rx.local_boot_id && rx.control.receiver_boot_id == rx.local_boot_id &&
                                isFresh(command.stamp, now_ms, stage::command_timeout_ms) &&
                                command.mode != ChassisMode::Disabled;
        const bool run = authorized && init_error == 0;
        RunStatus status{};
        status.state = init_error ? RunState::Blocked : run ? RunState::Recovering : RunState::Disabled;
        status.reason = init_error ? WaitReason::Configuration : run ? WaitReason::Feedback : WaitReason::Command;
        status.error = init_error;
        status.requested = run;
        status.last_command_sequence = command.stamp.sequence;
        status.stamp = {now_ms, ++status_sequence, true};
        if (run)
            status.error = motors.enable();
        else
            motors.disable();
        const auto steer_view = steer.snapshot(), drive_view = wheel.snapshot();
        const auto &sf = steer_view.feedback, &df = drive_view.feedback;
        ModuleFeedback feedback{};
        feedback.steer_absolute_rad = installed::steer[module_index].direction * sf.absolute_position_rad;
        feedback.steer_velocity_rad_s = installed::steer[module_index].direction * sf.velocity_rad_s;
        feedback.drive_velocity_rad_s = installed::wheel[module_index].direction * df.velocity_rad_s;
        const auto steer_fields = motor::FeedbackAbsolutePosition | motor::FeedbackVelocity;
        feedback.steer_valid = steer_view.state == motor::MotorState::Active && steer_view.feedback_fresh &&
                               (sf.valid & steer_fields) == steer_fields &&
                               std::isfinite(feedback.steer_absolute_rad) && std::isfinite(feedback.steer_velocity_rad_s);
        feedback.drive_valid = drive_view.state == motor::MotorState::Active && drive_view.feedback_fresh &&
                               (df.valid & motor::FeedbackVelocity) && std::isfinite(feedback.drive_velocity_rad_s);
        feedback.steer_feedback_stamp_ms = sf.timestamp_ms;
        feedback.steer_enable_generation = steer_view.enable_generation;
        feedback.steer_reference_generation = steer_view.reference_generation;
        feedback.drive_enable_generation = drive_view.enable_generation;
        ModuleOutput output{};
        if (run) {
            ModuleTargets targets{};
            int control_error = kinematics.solve(command, targets);
            if (control_error == 0)
                control_error = controller.step(targets[module_index], feedback, dt_s, output);
            // Loss of steering reference also withdraws this module's wheel effort.
            output.drive_output_valid = output.drive_output_valid && feedback.steer_valid;
            if (control_error < 0) {
                output.steer_output_valid = output.drive_output_valid = false;
                status.error = control_error;
            }
            const int steer_result = output.steer_output_valid
                ? steer.setCurrent(installed::steer[module_index].direction * output.steer_effort,
                                   output.steer_enable_generation) : steer.invalidateComputedEffort();
            const int drive_result = output.drive_output_valid
                ? wheel.setCurrent(installed::wheel[module_index].direction * output.drive_effort,
                                   output.drive_enable_generation) : wheel.invalidateComputedEffort();
            if (!status.error)
                status.error = steer_result ? steer_result : drive_result;
            status.ready = output.steer_output_valid && output.drive_output_valid && status.error == 0;
            status.state = status.ready ? RunState::Active : RunState::Recovering;
            status.reason = !cycle ? WaitReason::Cycle : status.ready ? WaitReason::None : WaitReason::Feedback;
        }
        else {
            (void)controller.reset(feedback);
            ModuleTargets reference{};
            reference[module_index].angle_rad = feedback.steer_valid ? feedback.steer_absolute_rad : 0;
            (void)kinematics.reset(reference);
        }
        const auto members = motors.status();
        status.member_count = 2;
        status.active_count = members.active_count;
        status.waiting_count = 2 - members.active_count;
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
        const auto reason = init_error ? installed::DiagnosticReason::InitFailed
            : !rx.online ? installed::DiagnosticReason::PeerStale
            : !rx.operator_control_valid || emergency_latched ? installed::DiagnosticReason::Permission
            : !authority.run_allowed ? installed::DiagnosticReason::Disarmed
            : !authorized ? installed::DiagnosticReason::CommandStale
            : !status.ready ? installed::DiagnosticReason::FeedbackStale : installed::DiagnosticReason::Ready;
        diagnostics.status({.reason = reason,
                            .valid = feedback.drive_valid,
                            .enabled = status.state == RunState::Active,
                            .target = output.drive_target_rad_s,
                            .feedback = feedback.drive_velocity_rad_s,
                            .age_ms = age(command.stamp, now_ms),
                            .dt_us = dt_s * 1000000.0f,
                            .effort = output.drive_effort,
                            .detail = float(module_index),
                            .error = status.error ? status.error : rx.error,
                            .extra0 = output.optimized_angle_rad,
                            .extra1 = feedback.steer_valid ? feedback.steer_absolute_rad
                                                          : std::numeric_limits<float>::quiet_NaN()});
        deadline += period;
        const auto ticks = k_uptime_ticks();
        if (deadline <= ticks)
            deadline += ((ticks - deadline) / period + 1) * period;
        k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
    }
}
