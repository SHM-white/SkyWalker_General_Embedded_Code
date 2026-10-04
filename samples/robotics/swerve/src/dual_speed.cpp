// Reciprocating steer / constant-speed drive CAN diagnostic. Original swerve control remains in main.cpp.
#include <cerrno>
#include <cmath>
#include <control/motor_session.hpp>
#include <core/clock.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "../../common/rc_controls.hpp"
#include "board_config.hpp"
LOG_MODULE_REGISTER(dual_speed, LOG_LEVEL_INF);
namespace {
constexpr float kRequestedVelocityAbsMaxRadS = 50.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.8f;
constexpr float kDeadbandRadS = 0.20f;
constexpr float kVelocityFilterTauS = 0.025f;
control_motor_velocity_config makeVelocityLoopConfig() {
    control_motor_velocity_config config{};
    config.regulator.feedback = {
        .kp = 0.4f,
        .ki = 0.1f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = -0.5f,
        .integral_max = 0.5f,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };
    config.regulator.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.005f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };
    config.reference_slew = {
        .rising_rate_per_s = 100.0f,
        .falling_rate_per_s = 100.0f,
    };
    config.measurement_filter_tau_s = kVelocityFilterTauS;
    config.soft_deadband_rad_s = kDeadbandRadS;
    config.requested_velocity_abs_max_rad_s = kRequestedVelocityAbsMaxRadS;
    config.effort_abs_max = kSoftwareCurrentAbsMaxA;
    return config;
}

skywalker::control::VelocityMotor::Config makeMotorConfig() {
    skywalker::control::VelocityMotor::Config config{};
    config.loop = makeVelocityLoopConfig();
    config.effort_unit = skywalker::control::EffortUnit::Ampere;
    config.safety = {1.5f * kRequestedVelocityAbsMaxRadS, 70.0f};
    return config;
}

skywalker::control::PositionMotor::Config makeSteerConfig() {
    skywalker::control::PositionMotor::Config config{};
    // Reuse the swerve's position/velocity cascade and current limits.
    config.loop = bench::moduleConfig().steer;
    config.loop.position.output_min = -bench::dual_steer_speed_limit_rad_s;
    config.loop.position.output_max = bench::dual_steer_speed_limit_rad_s;
    config.loop.velocity.requested_velocity_abs_max_rad_s = bench::dual_steer_speed_limit_rad_s;
    config.effort_unit = skywalker::control::EffortUnit::Ampere;
    config.safety = {75.0f, 70.0f};
    config.reference = skywalker::control::PositionReference::StartupRelative;
    return config;
}

} // namespace

int main() {
    using namespace skywalker;
    namespace input = samples::control;
    const auto steer_config = bench::steerHardware(), drive_config = bench::driveHardware();
    static motor::Motor steer(steer_config), drive(drive_config);
    static motor::CanBus steer_bus(bench::steer_can), drive_bus(bench::drive_can);
    static control::PositionMotor steer_axis(steer, makeSteerConfig());
    static control::VelocityMotor drive_axis(drive, makeMotorConfig());
    static const control::MotorSession::Member members[] = {
        {"steer/6020", &steer, &steer_bus, &steer_axis, control::ReferencePolicy::CaptureOnExplicitStart},
        {"drive/3508", &drive, &drive_bus, &drive_axis, control::ReferencePolicy::NotRequired},
    };
    static control::MotorSession session(members);
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma, input::receiverConfig());
    int ret = bench::hardware_confirmed && bench::steer_can != bench::drive_can ? 0 : -EINVAL;
    if (ret == 0) ret = remote.start();
    if (ret == 0) ret = session.configure();
    if (ret < 0) { LOG_ERR("DUAL_SPEED init failed=%d", ret); return ret; }
    LOG_INF("DUAL_SPEED GM6020 ID%u %s POSITION amplitude=%.3f rad; M3508 ID%u %s speed=%.3f rad/s",
        unsigned(steer_config.id), bench::steer_can->name, double(bench::dual_steer_amplitude_rad),
        unsigned(drive_config.id), bench::drive_can->name, double(bench::dual_drive_rad_s));
    LOG_INF("Arm: both switches Down + axes centered 500 ms, then left Middle. Left Down stops.");
    LOG_INF("Fault acknowledgement: left Down, axes centered, right Up 1000 ms after neutral baseline; then re-arm.");
    input::RcControlAdapter adapter;
    communication::RemoteReceiver::Snapshot remote_snapshot{};
    std::uint64_t next_log = 0, target_sequence = 0;
    auto previous_us = core::monotonicTimeUs();
    for (;;) {
        const auto wake_us = previous_us + bench::control_period_us;
        if (core::monotonicTimeUs() < wake_us) k_sleep(K_TIMEOUT_ABS_US(wake_us));
        else k_sleep(K_TICKS(1));
        const auto now_us = core::monotonicTimeUs();
        previous_us = now_us;
        remote.snapshot(remote_snapshot);
        const auto &rc = adapter.update(remote_snapshot.remote, now_us / 1000);
        const auto before = session.status();
        const auto elapsed_ms = before.state == control::SessionState::Running
            ? (now_us - before.started_us) / 1000 : 0;
        const auto phase_ms = elapsed_ms < 500 ? 0 : (elapsed_ms - 500) % bench::dual_steer_period_ms;
        const double phase = 6.28318530718 * double(phase_ms) / double(bench::dual_steer_period_ms);
        const double targets[] = {
            elapsed_ms < 500 ? 0.0 : bench::dual_steer_amplitude_rad * std::sin(phase),
            elapsed_ms < 500 ? 0.0 : double(bench::dual_drive_rad_s),
        };
        session.step({
            .enabled = rc.run_allowed,
            .start_sequence = rc.start_event_id,
            .acknowledge_sequence = rc.clear_event_id,
            .source = {rc.remote.stamp.timestamp_ms * 1000, rc.remote.stamp.sequence, rc.fresh},
            .target_stamp = {now_us, ++target_sequence, true},
            .targets = targets,
        }, now_us);
        if (now_us >= next_log) {
            next_log = now_us + 1000000;
            const auto status = session.status();
            const auto position = steer_axis.telemetry();
            LOG_INF("DUAL state=%s can_start=%d rc=%d arm_ready=%d blocker=%s last_stop=%s stop_event=%llu stop_progress=%u/%u",
                control::sessionStateName(status.state), status.can_start, rc.fresh, adapter.armReady(),
                control::sessionCauseName(status.current_blocker), control::sessionCauseName(status.last_stop),
                static_cast<unsigned long long>(status.stop_event), unsigned(status.stops[0].progress), unsigned(status.stops[1].progress));
            LOG_INF("6020 target=%.3f actual=%.3f current=%.3f; 3508 speed=%.3f",
                position.requested_position_rad, position.position_rad, double(position.effort_command),
                double(drive.snapshot().feedback.velocity_rad_s));
        }
    }
}
