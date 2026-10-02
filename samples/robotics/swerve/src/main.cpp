#include <cerrno>
#include <cmath>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
#include <robotics/execution/recovery_gate.hpp>
#include <robotics/swerve/swerve_kinematics.hpp>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "board_config.hpp"
LOG_MODULE_REGISTER(swerve_bench, LOG_LEVEL_INF);

int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    static motor::Motor steer(bench::steerHardware()), drive(bench::driveHardware());
    static motor::Group group(steer, drive);
    static motor::CanBus steer_bus(bench::steer_can), drive_bus(bench::drive_can);
    SwerveModule module(bench::moduleConfig());
    RecoveryGate recovery({100, 100000});
    int ret = bench::hardware_confirmed ? module.validate() : -ENODEV;
    if (ret == 0 && bench::steer_can == bench::drive_can) ret = -EINVAL;
    if (ret == 0) ret = steer_bus.attach(steer);
    if (ret == 0) ret = drive_bus.attach(drive);
    if (ret == 0) ret = steer_bus.start();
    if (ret == 0) ret = drive_bus.start();
    const int topology_error = ret;
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    LOG_INF("split-CAN suspended single module init=%d: e enable, space disable, w/s x, a/d y, q/z yaw, 0 zero, p input pause, ! estop, r clear", ret);
    bool requested = false, estop = false, paused = false, seeded = false, ready = false, hard_fault = false;
    std::uint64_t stable_since = 0, first_steer = 0, first_drive = 0, next_log = 0, retry_ms = 0;
    std::uint64_t steer_reference = 0, drive_reference = 0;
    core::TimeUs previous_us = 0, next_input_us = 0;
    core::Stamp source{};
    ChassisCommand command{};
    command.source = ControlSource::Autonomous;
    SwerveKinematics::Config kconfig = samples::chassis::controllerConfig().kinematics;
    SwerveKinematics kinematics(kconfig);
    ModuleTargets targets{};
    ModuleFeedback feedback{};
    ModuleOutput output{};
    const auto suspend = [&](WaitReason reason, int error = 0, bool blocked = false) {
        const auto state = group.status();
        if (state.active || state.enable_pending) group.disable();
        seeded = ready = false;
        output = {};
        recovery.withdraw(reason, error, blocked);
        hard_fault = hard_fault || blocked;
    };
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now = now_us / 1000;
        const bool cycle = previous_us && now_us > previous_us && now_us - previous_us <= 20000;
        const float dt = cycle ? float(now_us - previous_us) / 1000000 : 0;
        previous_us = now_us;
        bool clear = false;
        unsigned char key;
        while (uart_poll_in(console, &key) == 0) {
            if (key == 'e' && !estop) requested = true;
            if (key == ' ' || key == '!') { requested = false; estop = estop || key == '!'; }
            if (key == 'p') paused = !paused;
            if (key == 'r') { clear = true; requested = false; }
            if (key == 'w' || key == 's' || key == 'a' || key == 'd' || key == 'q' || key == 'z' || key == '0') {
                command.vx_m_s = key == 'w' ? bench::test_speed_m_s : key == 's' ? -bench::test_speed_m_s : 0;
                command.vy_m_s = key == 'a' ? bench::test_speed_m_s : key == 'd' ? -bench::test_speed_m_s : 0;
                command.wz_rad_s = key == 'q' ? .2f : key == 'z' ? -.2f : 0;
            }
        }
        if (!paused && now_us >= next_input_us) {
            next_input_us = now_us + 10000;
            command.mode = requested ? ChassisMode::BodyVelocity : ChassisMode::Disabled;
            command.stamp = {now, command.stamp.sequence + 1, true};
            source = {now_us, command.stamp.sequence, true};
        }
        if (clear && topology_error == 0) {
            suspend(WaitReason::Reference);
            ret = group.clearFault();
            if (ret == 0 || ret == -EALREADY) { estop = hard_fault = false; retry_ms = now + 100; }
        }
        const auto sv = steer.snapshot(), dv = drive.snapshot();
        const auto fresh = [](const motor::MotorSnapshot &v, bool absolute) {
            auto required = motor::FeedbackVelocity | motor::FeedbackCurrent;
            if (absolute) required |= motor::FeedbackAbsolutePosition;
            return v.feedback_fresh && (v.feedback.valid & required) == required &&
                std::isfinite(v.feedback.velocity_rad_s) && std::isfinite(v.feedback.current_a) &&
                (!absolute || std::isfinite(v.feedback.absolute_position_rad)) &&
                std::fabs(v.feedback.velocity_rad_s) < 80 &&
                (!(v.feedback.valid & motor::FeedbackTemperature) ||
                 (std::isfinite(v.feedback.temperature_c) && v.feedback.temperature_c < 70));
        };
        const bool healthy = fresh(sv, true) && fresh(dv, false);
        const bool transport = topology_error == 0 && steer_bus.status().state == motor::BusState::Running &&
            drive_bus.status().state == motor::BusState::Running;
        if (topology_error < 0) suspend(WaitReason::Configuration, topology_error, true);
        else if (estop || hard_fault) suspend(WaitReason::Drive, -ECANCELED, true);
        else if (!transport || !cycle || !healthy) suspend(!transport ? WaitReason::Transport :
            !cycle ? WaitReason::Cycle : WaitReason::Feedback, -ESTALE);
        else {
            bool fault = sv.state == motor::MotorState::Fault || dv.state == motor::MotorState::Fault;
            const auto recoverable = [](const motor::MotorSnapshot &v) {
                if (v.state != motor::MotorState::Fault) return true;
                switch (v.last_fault.reason) {
                case motor::FaultReason::FeedbackExpired: case motor::FaultReason::CommandExpired:
                case motor::FaultReason::UnexpectedDisabled: case motor::FaultReason::EnableTimeout:
                case motor::FaultReason::TransportError: case motor::FaultReason::RxOverflow: return true;
                default: return false;
                }
            };
            if (fault) {
                const bool transient = recoverable(sv) && recoverable(dv);
                suspend(WaitReason::Drive, -EIO, !transient);
                if (transient && now >= retry_ms) { retry_ms = now + 100; ret = group.clearFault(); }
            } else {
                auto gs = group.status();
                if (!requested && (gs.active || gs.enable_pending)) { suspend(WaitReason::Command); gs = group.status(); }
                if ((recovery.stage() == RecoveryGate::Stage::Active && !gs.active) ||
                    (recovery.stage() == RecoveryGate::Stage::Enabling && !gs.active && !gs.enable_pending))
                    suspend(WaitReason::Drive, -EAGAIN);
                if (seeded && (sv.reference_generation != steer_reference || dv.reference_generation != drive_reference))
                    suspend(WaitReason::Reference, -ESTALE);
                if (!ready && !group.active() && !group.status().enable_pending && group.ready() && now >= retry_ms) {
                    if (!seeded) {
                        ret = steer.reseedPosition(sv.feedback.absolute_position_rad);
                        if (ret == 0) ret = drive.reseedPosition(0);
                        if (ret == 0) {
                            seeded = true; stable_since = now;
                            first_steer = sv.feedback.timestamp_ms; first_drive = dv.feedback.timestamp_ms;
                            steer_reference = steer.snapshot().reference_generation;
                            drive_reference = drive.snapshot().reference_generation;
                        } else suspend(WaitReason::Reference, ret);
                    } else if (now >= stable_since + 30 && sv.feedback.timestamp_ms > first_steer &&
                        dv.feedback.timestamp_ms > first_drive) {
                        feedback = {bench::steer_direction * sv.feedback.absolute_position_rad,
                            bench::steer_direction * sv.feedback.position_rad, bench::steer_direction * sv.feedback.velocity_rad_s,
                            bench::drive_direction * dv.feedback.velocity_rad_s};
                        ret = module.reset(feedback);
                        for (auto &target : targets) target.angle_rad = feedback.steer_absolute_rad;
                        if (ret == 0) ret = kinematics.reset(targets);
                        if (ret == 0) { ready = true; recovery.prepared(now); }
                        else suspend(WaitReason::Reference, ret);
                    }
                }
                if (ready && requested) {
                    const auto stage = recovery.stage();
                    if (!recovery.accept(command.stamp, source, now)) {
                        ret = -ESTALE;
                        if (stage == RecoveryGate::Stage::Active || stage == RecoveryGate::Stage::Enabling)
                            suspend(WaitReason::Command, ret);
                    } else if (stage == RecoveryGate::Stage::WaitingCommand) {
                        ret = group.enable();
                        if (ret < 0) suspend(WaitReason::Drive, ret);
                    } else if (group.active()) {
                        recovery.enabled();
                        feedback = {bench::steer_direction * sv.feedback.absolute_position_rad,
                            bench::steer_direction * sv.feedback.position_rad, bench::steer_direction * sv.feedback.velocity_rad_s,
                            bench::drive_direction * dv.feedback.velocity_rad_s};
                        ret = kinematics.solve(command, targets);
                        if (ret == 0) ret = module.step(targets[0], feedback, dt, output);
                        if (ret == 0) ret = steer.setCurrent(bench::steer_direction * output.steer_effort * .15f);
                        if (ret == 0) ret = drive.setCurrent(bench::drive_direction * output.drive_effort * .15f);
                        if (ret < 0) suspend(WaitReason::Drive, ret, ret == -ERANGE || ret == -EINVAL);
                    }
                }
            }
        }
        if (topology_error == 0) {
            const int steer_error = steer_bus.commit().error, drive_error = drive_bus.commit().error;
            if (steer_error < 0 || drive_error < 0) suspend(WaitReason::Transport, steer_error < 0 ? steer_error : drive_error);
        }
        if (now >= next_log) {
            next_log = now + 100;
            LOG_INF("active=%d pending=%d ready=%d healthy=%d source=%u age=%llu gen=%u reason=%u err=%d can=%u/%u target=%.3f angle=%.3f wheel=%.3f",
                group.active(), group.status().enable_pending, ready, healthy, command.stamp.sequence,
                static_cast<unsigned long long>(now >= command.stamp.timestamp_ms ? now - command.stamp.timestamp_ms : 0),
                recovery.generation(), unsigned(recovery.reason()), ret, unsigned(steer_bus.status().state),
                unsigned(drive_bus.status().state), double(output.optimized_angle_rad),
                double(feedback.steer_absolute_rad), double(feedback.drive_velocity_rad_s));
        }
        k_sleep(K_MSEC(5));
    }
}
