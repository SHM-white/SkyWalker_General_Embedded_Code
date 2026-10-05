#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/velocity_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(m2006_speed_control, LOG_LEVEL_INF);

#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)
#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {
constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::int64_t kRunDurationMs = 300000000;
constexpr float kRequestedVelocityRadS = 5.0f;
constexpr float kRequestedVelocityAbsMaxRadS = 100.0f;
constexpr float kSoftwareCurrentAbsMaxA = 10.0f;
constexpr std::int64_t kDiagnosticRepeatMs = 500;
constexpr std::int64_t kDiagnosticRecoveryMs = 100;

unsigned long long logU64(std::uint64_t value) {
    return static_cast<unsigned long long>(value);
}

const char *issueName(skywalker::control::ControlIssue issue) {
    using skywalker::control::ControlIssue;
    switch (issue) {
    case ControlIssue::None:
        return "None";
    case ControlIssue::NotConfigured:
        return "NotConfigured";
    case ControlIssue::NotActive:
        return "NotActive";
    case ControlIssue::FeedbackStale:
        return "FeedbackStale";
    case ControlIssue::MissingFeedback:
        return "MissingFeedback";
    case ControlIssue::ReferenceLost:
        return "ReferenceLost";
    case ControlIssue::InvalidMeasurement:
        return "InvalidMeasurement";
    case ControlIssue::InvalidTarget:
        return "InvalidTarget";
    case ControlIssue::InvalidPeriod:
        return "InvalidPeriod";
    case ControlIssue::OutputRejected:
        return "OutputRejected";
    }
    return "Unknown";
}

struct DiagnosticSample {
    std::int64_t now_ms = 0;
    std::int64_t dt_ms = 0;
    std::uint64_t age_ms = 0;
    skywalker::control::VelocityMotor::Telemetry data{};
};

void logSample(const char *label, const DiagnosticSample &sample) {
    const auto &data = sample.data;
    const auto &motor = data.motor;
    const auto &f = motor.feedback;
    LOG_WRN("DIAG %s now=%llu dt=%lld stamp=%llu age=%llu valid=0x%x fresh=%d state=%u permitted=%d", label,
            logU64(sample.now_ms), static_cast<long long>(sample.dt_ms), logU64(f.timestamp_ms), logU64(sample.age_ms),
            unsigned(f.valid), motor.feedback_fresh, unsigned(motor.state), motor.output_permitted);
    LOG_WRN("DIAG %s ch1(ref)=%.4f ch2(speed)=%.4f target=%.4f effort=%.4f output=%d issue=%s error=%d gen=%llu", label,
            double(data.output.velocity_reference_rad_s), double(f.velocity_rad_s), double(data.target_rad_s),
            double(data.output.effort_command), data.output_valid, issueName(data.issue), data.error,
            logU64(motor.enable_generation));
}

control_motor_velocity_config makeVelocityLoopConfig() {
    control_motor_velocity_config loop{};

    loop.regulator.feedback = {
        .kp = 0.32f,
        .ki = 0.3f,
        .kd = 0.008f,
        .derivative_tau_s = 0.0f,
        .integral_min = -3.0f,
        .integral_max = 3.0f,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    loop.regulator.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };

    loop.reference_slew = {
        .rising_rate_per_s = 100.0f,
        .falling_rate_per_s = 100.0f,
    };
    loop.requested_velocity_abs_max_rad_s = kRequestedVelocityAbsMaxRadS;
    loop.effort_abs_max = kSoftwareCurrentAbsMaxA;
    // Preserve raw measurement and zero soft deadband from the old implementation.
    loop.measurement_filter_tau_s = 0.0f;
    loop.soft_deadband_rad_s = 0.0f;
    return loop;
}

skywalker::control::VelocityMotor::Config makeMotorConfig() {
    skywalker::control::VelocityMotor::Config config{};
    config.loop = makeVelocityLoopConfig();
    config.effort_unit = skywalker::control::EffortUnit::Ampere;

    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(can)) {
        LOG_ERR("CAN device not ready: %s", can->name);
        return -ENODEV;
    }
    // static skywalker::motor::Motor drive{skywalker::motor::dji::m3508({
    //     .id = 3,
    //     .current_limit_a = 10.0f,
    //     .gear_ratio = 19.0f,
    //     .timing =
    //         {.feedback_timeout_ms = 20, .command_timeout_ms = 20, .enable_timeout_ms = 100, .retry_interval_ms = 100},
    // })};
    static skywalker::motor::Motor drive{skywalker::motor::dji::m2006({
        .id = 1,
        .current_limit_a = 10.0f,
        .gear_ratio = 36.0f,
        .timing =
            {.feedback_timeout_ms = 20, .command_timeout_ms = 20, .enable_timeout_ms = 100, .retry_interval_ms = 100},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::VelocityMotor axis{drive, makeMotorConfig()};
    static Vofa vofa{};
    const int vofa_error = vofa_init(&vofa, uart);
    if (vofa_error < 0)
        LOG_ERR("VOFA init failed: %d", vofa_error);
    int ret = bus.attach(drive);
    if (ret == 0)
        ret = bus.start();
    if (ret == 0)
        ret = axis.configure();
    if (ret < 0) {
        LOG_ERR("Motor/bus initialization failed: %d", ret);
        return ret;
    }
    const auto feedback_timeout_ms = drive.info().timing.feedback_timeout_ms;
    LOG_INF("DIAG M3508 ID3 %s: feedback timeout=%u ms; VOFA ch1=reference ch2=speed ch11=feedback age ms", can->name,
            unsigned(feedback_timeout_ms));
    LOG_INF("DIAG state: 0=Offline 1=Disabled 2=Enabling 3=Active 4=Fault; details at most every 500ms");
    const auto started_ms = k_uptime_get();
    auto previous_ms = started_ms;
    std::int64_t next_log_ms = 0;
    int last_call_error = 0;
    DiagnosticSample previous_sample{};
    auto previous_bus = bus.status();
    auto last_logged_rx_invalid = previous_bus.rx_invalid_frames;
    bool have_previous = false;
    bool diagnostic_active = false;
    std::int64_t last_bad_ms = 0, next_diagnostic_ms = 0, diagnostic_started_ms = 0;
    std::uint64_t diagnostic_events = 0, bad_samples = 0, vofa_drops = 0;
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const auto dt_ms = now - previous_ms;
        const float dt = float(dt_ms) / 1000;
        previous_ms = now;
        const bool requested = now - started_ms < kRunDurationMs;
        const int request_error = requested ? drive.enable() : drive.disable();
        if (request_error < 0)
            last_call_error = request_error;
        const float target = now - started_ms < 100 ? 0.0f : kRequestedVelocityRadS;
        int update_error = 0;
        if (requested) {
            update_error = axis.update(target, dt);
            if (update_error < 0)
                last_call_error = update_error;
        }
        const auto committed = bus.commit();
        const int commit_error = committed.error;
        if (commit_error < 0)
            last_call_error = commit_error;
        const auto data = axis.telemetry();
        const auto &f = data.motor.feedback;
        const auto &o = data.output;
        const std::uint64_t age_ms = std::uint64_t(now) >= f.timestamp_ms ? std::uint64_t(now) - f.timestamp_ms : 0;
        int vofa_send_error = vofa_error;
        if (vofa_error == 0) {
            const float channels[12] = {target,
                                        o.velocity_reference_rad_s,
                                        f.velocity_rad_s,
                                        o.filtered_velocity_rad_s,
                                        o.velocity_error_rad_s,
                                        o.regulator.feedback.p,
                                        o.regulator.feedback.i,
                                        o.regulator.feedback.d,
                                        o.regulator.feedforward,
                                        requested && data.output_valid ? o.effort_command : 0,
                                        float(data.output_valid && requested),
                                        float(age_ms)};
            vofa_send_error = vofa_send(&vofa, channels, 12);
            if (vofa_send_error < 0)
                ++vofa_drops;
        }

        const DiagnosticSample sample{now, dt_ms, age_ms, data};
        const auto bus_status = bus.status();
        // Take time after the telemetry snapshot: feedback can arrive after loop start.
        const auto observed_ms = std::uint64_t(k_uptime_get());
        const bool bad_stamp = f.timestamp_ms == 0 || f.timestamp_ms > observed_ms ||
                               (have_previous && f.timestamp_ms < previous_sample.data.motor.feedback.timestamp_ms);
        const bool stale = !data.motor.feedback_fresh || age_ms > feedback_timeout_ms;
        const bool bad_period = dt_ms < 1 || dt_ms > 20;
        const bool zero_ref = have_previous && std::fabs(target) > 0.5f &&
                              std::fabs(previous_sample.data.output.velocity_reference_rad_s) > 0.5f &&
                              std::fabs(o.velocity_reference_rad_s) < 0.01f;
        const bool zero_speed = have_previous && std::fabs(target) > 0.5f &&
                                std::fabs(previous_sample.data.motor.feedback.velocity_rad_s) > 0.5f &&
                                std::fabs(f.velocity_rad_s) < 0.01f;
        const bool invalid_speed = !std::isfinite(f.velocity_rad_s) || !(f.valid & skywalker::motor::FeedbackVelocity);
        const bool motor_event = have_previous &&
                                 (data.motor.enable_generation != previous_sample.data.motor.enable_generation ||
                                  data.motor.last_fault.occurred_ms !=
                                      previous_sample.data.motor.last_fault.occurred_ms ||
                                  data.motor.last_fault.reason != previous_sample.data.motor.last_fault.reason);
        const bool can_event = bus_status.state != skywalker::motor::BusState::Running ||
                               bus_status.last_recovery.count != previous_bus.last_recovery.count ||
                               bus_status.rx_overflows != previous_bus.rx_overflows ||
                               (bus_status.last_tx.valid && bus_status.last_tx.error < 0);
        // rx_invalid_frames also includes DJI feedback rejected for equal millisecond
        // timestamps. Report its growth in the heartbeat; alone it is not a control fault.
        const bool bad = requested && (bad_stamp || stale || bad_period || zero_ref || zero_speed || invalid_speed ||
                                       !data.output_valid || motor_event || can_event || request_error < 0 ||
                                       update_error < 0 || commit_error < 0);
        if (bad) {
            if (!diagnostic_active) {
                diagnostic_active = true;
                diagnostic_started_ms = now;
                next_diagnostic_ms = now;
                bad_samples = 0;
                ++diagnostic_events;
            }
            last_bad_ms = now;
            ++bad_samples;
            if (now >= next_diagnostic_ms) {
                next_diagnostic_ms = now + kDiagnosticRepeatMs;
                LOG_WRN(
                    "DIAG trigger #%llu bad_samples=%llu stale=%d stamp=%d period=%d zero_ref=%d zero_speed=%d invalid_speed=%d motor=%d can_fault=%d",
                    logU64(diagnostic_events), logU64(bad_samples), stale, bad_stamp, bad_period, zero_ref, zero_speed,
                    invalid_speed, motor_event, can_event);
                if (have_previous)
                    logSample("prev", previous_sample);
                logSample("curr", sample);
                const auto &m = data.motor;
                LOG_WRN("DIAG motor fault=%u err=%d at=%llu retry=%llu cmd_rev=%llu cmd_stamp=%llu stop=%u stop_err=%d",
                        unsigned(m.last_fault.reason), m.last_fault.error, logU64(m.last_fault.occurred_ms),
                        logU64(m.retry_count), logU64(m.command_revision), logU64(m.command_written_ms),
                        unsigned(m.stop.progress), m.stop.tx_error);
                LOG_WRN(
                    "DIAG raw valid=%d rpm=%d current=%d encoder=%u stamp=%llu; calls enable=%d update=%d commit=%d vofa=%d drops=%llu",
                    m.native_dji_feedback_valid, int(m.native_dji_feedback.speed_rpm),
                    int(m.native_dji_feedback.current_raw), unsigned(m.native_dji_feedback.encoder),
                    logU64(m.native_dji_feedback.timestamp_ms), request_error, update_error, commit_error,
                    vofa_send_error, logU64(vofa_drops));
                const auto &tx = bus_status.last_tx;
                LOG_WRN(
                    "DIAG CAN state=%u err=%d commit_seq=%llu tx_valid=%d tx_seq=%llu id=0x%x purpose=%u err=%d at=%llu rx_overflow=%llu rx_invalid=%llu superseded=%llu",
                    unsigned(bus_status.state), bus_status.last_error, logU64(committed.sequence), tx.valid,
                    logU64(tx.sequence), unsigned(tx.can_id), unsigned(tx.purpose), tx.error, logU64(tx.completed_ms),
                    logU64(bus_status.rx_overflows), logU64(bus_status.rx_invalid_frames),
                    logU64(bus_status.superseded_batches));
                can_state controller_state = CAN_STATE_STOPPED;
                can_bus_err_cnt counts{};
                const int query_error = can_get_state(can, &controller_state, &counts);
                const auto &r = bus_status.last_recovery;
                LOG_WRN(
                    "DIAG CAN live query=%d state=%u TEC=%u REC=%u; recovery count=%llu at=%llu reason=%u err=%d query=%d state=%u TEC=%u REC=%u",
                    query_error, unsigned(controller_state), unsigned(counts.tx_err_cnt), unsigned(counts.rx_err_cnt),
                    logU64(r.count), logU64(r.occurred_ms), unsigned(r.reason), r.error, r.query_error,
                    unsigned(r.controller_state), unsigned(r.error_counts.tx_err_cnt),
                    unsigned(r.error_counts.rx_err_cnt));
                LOG_WRN(
                    "DIAG recovery TX valid=%d seq=%llu id=0x%x purpose=%u err=%d; stats_valid=%d bit0=%u bit1=%u stuff=%u crc=%u form=%u ack=%u",
                    r.last_tx.valid, logU64(r.last_tx.sequence), unsigned(r.last_tx.can_id),
                    unsigned(r.last_tx.purpose), r.last_tx.error, r.stats_valid, unsigned(r.bit0), unsigned(r.bit1),
                    unsigned(r.stuff), unsigned(r.crc), unsigned(r.form), unsigned(r.ack));
            }
        }
        else if (diagnostic_active && now - last_bad_ms >= kDiagnosticRecoveryMs) {
            LOG_INF("DIAG recovered #%llu span=%llu ms bad_samples=%llu speed=%.4f age=%llu dt=%lld issue=%s",
                    logU64(diagnostic_events), logU64(now - diagnostic_started_ms), logU64(bad_samples),
                    double(f.velocity_rad_s), logU64(age_ms), static_cast<long long>(dt_ms), issueName(data.issue));
            diagnostic_active = false;
        }
        previous_sample = sample;
        previous_bus = bus_status;
        have_previous = true;
        if (now >= next_log_ms) {
            next_log_ms = now + 1000;
            LOG_INF(
                "run=%d state=%u target=%.3f seq=%llu output=%d wait=%u call=%d age=%llu dt=%lld vofa=%d drops=%llu rx_invalid=%llu (+%llu)",
                requested, unsigned(data.motor.state), double(target),
                static_cast<unsigned long long>(data.target_sequence), data.output_valid && requested,
                unsigned(data.issue), last_call_error, logU64(age_ms), static_cast<long long>(dt_ms), vofa_send_error,
                logU64(vofa_drops), logU64(bus_status.rx_invalid_frames),
                logU64(bus_status.rx_invalid_frames - last_logged_rx_invalid));
            last_logged_rx_invalid = bus_status.rx_invalid_frames;
        }
    }
}
