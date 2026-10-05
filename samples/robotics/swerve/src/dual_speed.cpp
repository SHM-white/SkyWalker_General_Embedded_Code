// Reciprocating steer / constant-speed drive CAN diagnostic. Original swerve control remains in main.cpp.
#include <cerrno>
#include <cmath>
#include <control/velocity_motor.hpp>
#include <control/position_motor.hpp>
#include <drivers/motor/can_bus.hpp>
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

    config.reference = skywalker::control::PositionReference::AbsoluteNearest;
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
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                                input::receiverConfig());
    int ret = bench::hardware_confirmed && bench::steer_can != bench::drive_can ? 0 : -EINVAL;
    if (ret == 0)
        ret = steer_bus.attach(steer);
    if (ret == 0)
        ret = drive_bus.attach(drive);
    if (ret == 0)
        ret = steer_bus.start();
    if (ret == 0)
        ret = drive_bus.start();
    if (ret == 0)
        ret = steer_axis.configure();
    if (ret == 0)
        ret = drive_axis.configure();
    if (ret == 0)
        ret = remote.start();
    if (ret < 0)
        return ret;
    LOG_INF("DUAL_SPEED GM6020 ID%u %s POSITION; M3508 ID%u %s speed=%.3f rad/s", unsigned(steer_config.id),
            bench::steer_can->name, unsigned(drive_config.id), bench::drive_can->name, double(bench::dual_drive_rad_s));
    LOG_INF("RC both Down + centered 500ms then left Middle. Motor recovery needs no re-arm.");
    input::RcControlAdapter adapter;
    communication::RemoteReceiver::Snapshot remote_snapshot{};
    bool previous_requested = false, center_valid = false;
    float center = 0;
    core::TimeUs started_us = 0, next_log_us = 0;
    auto previous_us = core::monotonicTimeUs();
    int last_call_error = 0;
    for (;;) {
        k_sleep(K_USEC(bench::control_period_us));
        const auto now_us = core::monotonicTimeUs();
        const float dt = now_us > previous_us ? float(now_us - previous_us) / 1000000 : 0;
        previous_us = now_us;
        (void)remote.snapshot(remote_snapshot);
        const auto &rc = adapter.update(remote_snapshot.remote, now_us / 1000);
        const bool requested = rc.run_allowed;
        if (requested && !previous_requested) {
            started_us = now_us;
            center_valid = false;
        }
        previous_requested = requested;
        const auto elapsed_ms = requested ? (now_us - started_us) / 1000 : 0;
        const auto phase_ms = elapsed_ms < 500 ? 0 : (elapsed_ms - 500) % bench::dual_steer_period_ms;
        const double phase = 6.28318530718 * double(phase_ms) / double(bench::dual_steer_period_ms);
        const float relative_target = elapsed_ms < 500 ? 0 : bench::dual_steer_amplitude_rad * std::sin(phase);
        const float drive_target = elapsed_ms < 500 ? 0 : bench::dual_drive_rad_s;
        const auto sv = steer.snapshot();
        if (requested && !center_valid && sv.feedback_fresh && (sv.feedback.valid & motor::FeedbackAbsolutePosition) &&
            std::isfinite(sv.feedback.absolute_position_rad)) {
            center = sv.feedback.absolute_position_rad;
            center_valid = true;
        }
        const int se = requested ? steer.enable() : steer.disable();
        const int de = requested ? drive.enable() : drive.disable();
        if (se < 0)
            last_call_error = se;
        if (de < 0)
            last_call_error = de;
        if (requested) {
            if (center_valid) {
                const int error = steer_axis.update(center + relative_target, dt);
                if (error < 0)
                    last_call_error = error;
            }
            else
                (void)steer_axis.reset();
            const int error = drive_axis.update(drive_target, dt);
            if (error < 0)
                last_call_error = error;
        }
        const int sb = steer_bus.commit().error, db = drive_bus.commit().error;
        if (sb < 0)
            last_call_error = sb;
        if (db < 0)
            last_call_error = db;
        if (now_us >= next_log_us) {
            next_log_us = now_us + 1000000;
            const auto position = steer_axis.telemetry();
            const auto velocity = drive_axis.telemetry();
            LOG_INF(
                "run=%d center=%d elapsed=%llu relative=%.3f 6020 target=%.3f actual=%.3f output=%d wait=%u; 3508 target=%.3f actual=%.3f output=%d wait=%u call=%d",
                requested, center_valid, static_cast<unsigned long long>(elapsed_ms), double(relative_target),
                position.requested_position_rad, position.position_rad, position.output_valid && requested,
                unsigned(position.issue), double(velocity.target_rad_s), double(velocity.motor.feedback.velocity_rad_s),
                velocity.output_valid && requested, unsigned(velocity.issue), last_call_error);
        }
    }
}
