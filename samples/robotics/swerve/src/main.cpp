#include <cerrno>
#include <cmath>
#include <limits>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
#include <lib/vofa/vofa.h>
#include <robotics/swerve/swerve_kinematics.hpp>
#include "../../common/rc_controls.hpp"
#include "../../common/sample_diagnostics.hpp"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "board_config.hpp"
LOG_MODULE_REGISTER(swerve_bench, LOG_LEVEL_INF);
static_assert(DT_NODE_HAS_COMPAT(DT_ALIAS(telemetry_uart), zephyr_cdc_acm_uart),
              "Swerve telemetry requires a USB CDC ACM telemetry-uart");

int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    namespace input = samples::control;
    const auto steer_config = bench::steerHardware(), drive_config = bench::driveHardware();
    static motor::Motor steer(steer_config), drive(drive_config);
    static motor::Group group(steer, drive);
    static motor::CanBus steer_bus(bench::steer_can), drive_bus(bench::drive_can);
    const auto module_config = bench::moduleConfig();
    SwerveModule module(module_config);
    SwerveKinematics kinematics(bench::kinematicsConfig(module_config));
    int ret = bench::hardware_confirmed && bench::steer_can != bench::drive_can ? module.validate() : -EINVAL;
    if (ret == 0) ret = kinematics.reset(ModuleTargets{});
    if (ret == 0) ret = steer_bus.attach(steer);
    if (ret == 0) ret = drive_bus.attach(drive);
    if (ret == 0) ret = steer_bus.start();
    if (ret == 0) ret = drive_bus.start();
    if (ret < 0) { LOG_ERR("configuration=%d", ret); return ret; }
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma, input::receiverConfig());
    ret = remote.start();
    if (ret < 0) return ret;
    static Vofa vofa{};
    const auto uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
    const int vofa_error = vofa_init(&vofa, uart);
    LOG_INF("GM6020 ID%u %s; M3508 ID%u %s; independent automatic recovery",
        unsigned(steer_config.id), bench::steer_can->name, unsigned(drive_config.id), bench::drive_can->name);
    LOG_INF("RC both Down + centered 500ms then left Middle. Left Down stops. No alignment gate.");
    input::RcControlAdapter adapter;
    input::SampleDiagnostics diagnostics;
    communication::RemoteReceiver::Snapshot remote_snapshot{};
    ChassisCommand command{};
    command.source = ControlSource::Remote;
    ModuleTargets targets{};
    ModuleOutput output{};
    bool zero_captured = !bench::capture_startup_zero;
    float steer_zero_rad = 0;
    auto previous_us = core::monotonicTimeUs();
    std::uint64_t next_log_ms = 0, next_vofa_ms = 0;
    int last_call_error = 0;
    std::uint64_t accepted_targets = 0;
    for (;;) {
        k_sleep(K_USEC(bench::control_period_us));
        const auto now_us = core::monotonicTimeUs(), now = now_us / 1000;
        const float dt = now_us > previous_us ? float(now_us - previous_us) / 1000000 : 0;
        previous_us = now_us;
        (void)remote.snapshot(remote_snapshot);
        const auto &rc = adapter.update(remote_snapshot.remote, now);
        const auto diagnostic = diagnostics.update(now, rc.run_allowed,
            !rc.fresh || rc.remote.left_switch == RcSwitch::Down);
        bool requested = rc.run_allowed;
        if (!diagnostic.input_paused && rc.fresh &&
            (!command.stamp.valid || sequenceAfter(rc.remote.stamp.sequence, command.stamp.sequence))) {
            command.mode = requested ? ChassisMode::BodyVelocity : ChassisMode::Disabled;
            command.vx_m_s = input::RcControlAdapter::normalize(rc.remote.analog.left_y) * bench::test_speed_m_s;
            command.vy_m_s = -input::RcControlAdapter::normalize(rc.remote.analog.left_x) * bench::test_speed_m_s;
            command.wz_rad_s = 0;
            command.stamp = rc.remote.stamp;
        }
        if (requested && !isFresh(command.stamp, now, 100)) { adapter.withdraw(); requested = false; }
        if (!requested) {
            command.mode = ChassisMode::Disabled;
            command.vx_m_s = command.vy_m_s = command.wz_rad_s = 0;
            group.disable();
        } else {
            const int error = group.enable();
            if (error < 0) last_call_error = error;
        }
        if (diagnostic.execution_paused && requested) continue;
        const auto sv = steer.snapshot(), dv = drive.snapshot();
        constexpr auto steer_fields = motor::FeedbackAbsolutePosition | motor::FeedbackVelocity;
        const bool steer_measured = sv.feedback_fresh && (sv.feedback.valid & steer_fields) == steer_fields &&
            std::isfinite(sv.feedback.absolute_position_rad) && std::isfinite(sv.feedback.velocity_rad_s);
        if (!zero_captured && steer_measured) { steer_zero_rad = sv.feedback.absolute_position_rad; zero_captured = true; }
        ModuleFeedback f{};
        f.steer_absolute_rad = bench::steer_direction * (sv.feedback.absolute_position_rad - steer_zero_rad);
        f.steer_velocity_rad_s = bench::steer_direction * sv.feedback.velocity_rad_s;
        f.drive_velocity_rad_s = bench::drive_direction * dv.feedback.velocity_rad_s;
        f.steer_valid = requested && zero_captured && steer_measured && sv.state == motor::MotorState::Active;
        f.drive_valid = requested && dv.state == motor::MotorState::Active && dv.feedback_fresh &&
            (dv.feedback.valid & motor::FeedbackVelocity) && std::isfinite(f.drive_velocity_rad_s);
        f.steer_feedback_stamp_ms = sv.feedback.timestamp_ms;
        f.steer_enable_generation = sv.enable_generation;
        f.steer_reference_generation = sv.reference_generation;
        f.drive_enable_generation = dv.enable_generation;
        const int solve_error = kinematics.solve(command, targets);
        const int step_error = solve_error == 0 ? module.step(targets[0], f, dt, output) : solve_error;
        if (step_error < 0) { last_call_error = step_error; if (solve_error < 0) output = {}; }
        else ++accepted_targets;
        const int steer_error = output.steer_output_valid
            ? steer.setCurrent(bench::steer_direction * output.steer_effort, output.steer_enable_generation)
            : steer.invalidateComputedEffort();
        const int drive_error = output.drive_output_valid
            ? drive.setCurrent(bench::drive_direction * output.drive_effort, output.drive_enable_generation)
            : drive.invalidateComputedEffort();
        if (steer_error < 0) last_call_error = steer_error;
        if (drive_error < 0) last_call_error = drive_error;
        const int sb_error = steer_bus.commit().error, db_error = drive_bus.commit().error;
        if (sb_error < 0) last_call_error = sb_error;
        if (db_error < 0) last_call_error = db_error;
        if (vofa_error == 0 && now >= next_vofa_ms) {
            next_vofa_ms = now + 10;
            const float missing = std::numeric_limits<float>::quiet_NaN();
            const float channels[16] = {
                command.vx_m_s, command.vy_m_s, output.optimized_angle_rad,
                steer_measured && zero_captured ? f.steer_absolute_rad : missing,
                output.steer_output_valid ? output.steer_reference_rad : missing,
                output.steer_output_valid ? output.alignment_error_rad : missing,
                output.drive_target_rad_s, dv.feedback_fresh ? f.drive_velocity_rad_s : missing,
                output.steer_output_valid ? bench::steer_direction * output.steer_effort : 0,
                sv.feedback_fresh && (sv.feedback.valid & motor::FeedbackCurrent) ? sv.feedback.current_a : missing,
                output.drive_output_valid ? bench::drive_direction * output.drive_effort : 0,
                dv.feedback_fresh && (dv.feedback.valid & motor::FeedbackCurrent) ? dv.feedback.current_a : missing,
                float(rc.fresh), float(requested), float(output.steer_output_valid), float(output.drive_output_valid)};
            (void)vofa_send(&vofa, channels, 16);
        }
        if (now >= next_log_ms) {
            next_log_ms = now + 1000;
            LOG_INF("run=%d seq=%llu target=%.3f/%.3f state=%u/%u output=%d/%d current=%.3f/%.3f feedback=%d/%d call=%d can=%u/%u",
                requested, static_cast<unsigned long long>(accepted_targets), double(output.optimized_angle_rad),
                double(output.drive_target_rad_s), unsigned(sv.state), unsigned(dv.state), output.steer_output_valid,
                output.drive_output_valid, double(output.steer_effort), double(output.drive_effort),
                sv.feedback_fresh, dv.feedback_fresh, last_call_error, unsigned(steer_bus.status().state), unsigned(drive_bus.status().state));
        }
    }
}
