// Reciprocating steer / constant-speed drive CAN diagnostic. Original swerve control remains in main.cpp.
#include <cerrno>
#include <cmath>
#include <control/velocity_motor.hpp>
#include <control/position_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
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
        .kp = 0.02f,
        .ki = 0.05f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = -0.1f,
        .integral_max = 0.1f,
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

void logCanState(const char *name, const device *dev, const skywalker::motor::BusStatus &bus) {
    can_state state = CAN_STATE_STOPPED;
    can_bus_err_cnt counts{};
    const int error = can_get_state(dev, &state, &counts);
    LOG_INF("%s bus=%u hw=%u query_err=%d TEC=%u REC=%u last_err=%d tx_err=%d tx_ms=%llu bit0=%u bit1=%u stuff=%u crc=%u form=%u ack=%u",
        name, unsigned(bus.state), unsigned(state), error, unsigned(counts.tx_err_cnt),
        unsigned(counts.rx_err_cnt), bus.last_error, bus.last_tx.error,
        static_cast<unsigned long long>(bus.last_tx.completed_ms),
        can_stats_get_bit0_errors(dev), can_stats_get_bit1_errors(dev),
        can_stats_get_stuff_errors(dev), can_stats_get_crc_errors(dev),
        can_stats_get_form_errors(dev), can_stats_get_ack_errors(dev));
    const auto &fault = bus.last_recovery;
    if (fault.count != 0) {
        LOG_INF("%s pre_restart count=%llu ms=%llu reason=%u err=%d hw=%u query_err=%d TEC=%u REC=%u stats=%d bit0=%u bit1=%u stuff=%u crc=%u form=%u ack=%u",
            name, static_cast<unsigned long long>(fault.count),
            static_cast<unsigned long long>(fault.occurred_ms), unsigned(fault.reason), fault.error,
            unsigned(fault.controller_state), fault.query_error, unsigned(fault.error_counts.tx_err_cnt),
            unsigned(fault.error_counts.rx_err_cnt), fault.stats_valid, fault.bit0, fault.bit1,
            fault.stuff, fault.crc, fault.form, fault.ack);
        LOG_INF("%s pre_restart_tx valid=%d id=0x%x purpose=%u seq=%llu err=%d ms=%llu",
            name, fault.last_tx.valid, unsigned(fault.last_tx.can_id), unsigned(fault.last_tx.purpose),
            static_cast<unsigned long long>(fault.last_tx.sequence), fault.last_tx.error,
            static_cast<unsigned long long>(fault.last_tx.completed_ms));
    }
}

void logMotor(const char *name, const skywalker::motor::MotorSnapshot &s) {
    LOG_INF("%s state=%u fresh=%d speed=%.3f current=%.3f temp=%.1f feedback_ms=%llu fault=%u/%d fault_ms=%llu",
        name, unsigned(s.state), s.feedback_fresh, double(s.feedback.velocity_rad_s),
        double(s.feedback.current_a), double(s.feedback.temperature_c),
        static_cast<unsigned long long>(s.feedback.timestamp_ms), unsigned(s.last_fault.reason),
        s.last_fault.error, static_cast<unsigned long long>(s.last_fault.occurred_ms));
}
} // namespace

int main() {
    using namespace skywalker;
    namespace input = samples::control;
    static motor::Motor steer(bench::steerHardware()), drive(bench::driveHardware());
    static motor::Group group(steer, drive);
    static motor::CanBus steer_bus(bench::steer_can), drive_bus(bench::drive_can);
    static control::PositionMotor steer_axis(steer, makeSteerConfig());
    static control::VelocityMotor drive_axis(drive, makeMotorConfig());
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma, input::receiverConfig());
    int ret = bench::hardware_confirmed && bench::steer_can != bench::drive_can ? 0 : -EINVAL;
    if (ret == 0) ret = steer_bus.attach(steer);
    if (ret == 0) ret = drive_bus.attach(drive);
    if (ret == 0) ret = steer_bus.start();
    if (ret == 0) ret = drive_bus.start();
    if (ret == 0) ret = steer_axis.configure();
    if (ret == 0) ret = drive_axis.configure();
    if (ret == 0) ret = remote.start();
    if (ret < 0) { LOG_ERR("DUAL_SPEED init failed=%d", ret); return ret; }
    LOG_INF("DUAL_SPEED: CAN2 GM6020 ID2 POSITION amplitude=%.3f rad; CAN1 M3508 ID2 constant=%.3f output rad/s",
        double(bench::dual_steer_amplitude_rad), double(bench::dual_drive_rad_s));
    LOG_INF("6020 POSITION reciprocating period_ms=%u; center captured at each enable", unsigned(bench::dual_steer_period_ms));
    LOG_INF("6020 swerve position cascade; 3508 dji_speed_control PI; period=5 ms; temp limit=70 C");
    LOG_INF("Arm: both switches Down + all axes centered 500 ms, then left Middle. Left Down stops. Sticks do not change speed.");
    input::RcControlAdapter adapter;
    communication::RemoteReceiver::Snapshot remote_snapshot{};
    bool engaged = false, running = false;
    std::uint64_t enable_ms = 0, started_ms = 0, next_log = 0;
    std::uint64_t recovery_seen[2]{};
    motor::BusRecoverySnapshot first_recovery[2]{};
    auto previous_us = core::monotonicTimeUs();
    for (;;) {
        k_sleep(K_USEC(bench::control_period_us));
        const auto now_us = core::monotonicTimeUs();
        const auto now = now_us / 1000;
        const auto dt_us = now_us - previous_us;
        previous_us = now_us;
        remote.snapshot(remote_snapshot);
        const auto rc = adapter.update(remote_snapshot.remote, now);
        const auto stop = [&](const char *reason, int error) {
            // Capture fault source before application disable changes motor states.
            const auto gs = group.status();
            const auto ss = steer.snapshot(), ds = drive.snapshot();
            const auto sb = steer_bus.status(), db = drive_bus.status();
            group.disable();
            adapter.withdraw();
            engaged = running = false;
            LOG_ERR("STOP reason=%s err=%d ms=%llu dt_us=%llu group_fault=%u/%d source=%s fault_ms=%llu",
                reason, error, static_cast<unsigned long long>(now), static_cast<unsigned long long>(dt_us),
                unsigned(gs.last_fault.reason), gs.last_fault.error,
                gs.last_fault.source_motor == &steer ? "CAN2/6020" : gs.last_fault.source_motor == &drive ? "CAN1/3508" : "none",
                static_cast<unsigned long long>(gs.last_fault.occurred_ms));
            logMotor("STOP 6020", ss); logMotor("STOP 3508", ds);
            logCanState("STOP CAN2", bench::steer_can, sb);
            logCanState("STOP CAN1", bench::drive_can, db);
        };
        if (rc.clear_fault && !engaged) {
            const int error = group.clearFault();
            LOG_INF("clear_fault=%d; re-arm required", error);
        }
        if (engaged && (!rc.run_allowed || !rc.fresh)) {
            stop("remote", 0);
        } else if (engaged && (steer_bus.status().state != motor::BusState::Running ||
                               drive_bus.status().state != motor::BusState::Running)) {
            stop("transport", -ENETDOWN);
        } else if (engaged && running && !group.active()) {
            stop("group", group.status().last_fault.error);
        } else if (engaged && !running && !group.active() &&
                   (!group.status().enable_pending || now - enable_ms > 1000)) {
            stop("enable", -ETIMEDOUT);
        } else if (!engaged && rc.run_allowed) {
            if (!group.ready()) {
                stop("not_ready", -EAGAIN);
            } else {
                ret = steer_axis.reset();
                if (ret == 0) ret = drive_axis.reset();
                if (ret == 0) ret = group.enable();
                if (ret < 0) stop("enable", ret);
                else { engaged = true; enable_ms = now; }
            }
        }
        if (engaged && group.active()) {
            if (!running) { running = true; started_ms = now; LOG_INF("RUN ms=%llu", static_cast<unsigned long long>(now)); }
            // Position target is relative to the reset anchor, not an integrated speed.
            // Hold the anchor for 500 ms, then command a bounded sinusoidal angle.
            const auto elapsed_ms = now - started_ms;
            const auto phase_ms = elapsed_ms < 500 ? 0 : (elapsed_ms - 500) % bench::dual_steer_period_ms;
            const float phase = 6.28318530718f * float(phase_ms) / float(bench::dual_steer_period_ms);
            const float st = elapsed_ms < 500 ? 0.0f : bench::dual_steer_amplitude_rad * std::sin(phase);
            const float dr = now - started_ms < 500 ? 0.0f : bench::dual_drive_rad_s;
            const float dt = float(dt_us) * 1e-6f;
            const char *operation = "6020_update";
            ret = steer_axis.update(st, dt);
            if (ret == 0) { operation = "3508_update"; ret = drive_axis.update(dr, dt); }
            if (ret == 0) { operation = "CAN2_commit"; ret = steer_bus.commit().error; }
            if (ret == 0) { operation = "CAN1_commit"; ret = drive_bus.commit().error; }
            if (ret < 0) stop(operation, ret);
        }
        const motor::BusStatus buses[] = {steer_bus.status(), drive_bus.status()};
        const device *devices[] = {bench::steer_can, bench::drive_can};
        const char *names[] = {"CAN2", "CAN1"};
        for (unsigned i = 0; i < 2; ++i) {
            const auto &fault = buses[i].last_recovery;
            if (fault.count != recovery_seen[i]) {
                if (first_recovery[i].count == 0) {
                    first_recovery[i] = fault;
                    LOG_ERR("FIRST_RECOVERY %s observed_count=%llu", names[i], static_cast<unsigned long long>(fault.count));
                }
                recovery_seen[i] = fault.count;
                logCanState(names[i], devices[i], buses[i]);
            }
        }
        if (now >= next_log) {
            next_log = now + 1000;
            LOG_INF("DUAL active=%d pending=%d ready=%d rc=%d arm_ready=%d centered=%d dt_us=%llu first_ms=%llu/%llu",
                group.active(), group.status().enable_pending, group.ready(), rc.fresh,
                adapter.armReady(), adapter.controlsCentered(), static_cast<unsigned long long>(dt_us),
                static_cast<unsigned long long>(first_recovery[0].occurred_ms),
                static_cast<unsigned long long>(first_recovery[1].occurred_ms));
            const auto position = steer_axis.telemetry();
            LOG_INF("6020 POSITION valid=%d target_relative=%.3f actual_relative=%.3f current_command=%.3f",
                position.valid && running, position.requested_position_rad, position.position_rad,
                double(position.effort_command));
            logMotor("6020", steer.snapshot()); logMotor("3508", drive.snapshot());
            logCanState("CAN2", bench::steer_can, buses[0]);
            logCanState("CAN1", bench::drive_can, buses[1]);
        }
    }
}
