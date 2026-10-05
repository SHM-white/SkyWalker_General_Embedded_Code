#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/velocity_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <core/clock.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_speed_control, LOG_LEVEL_INF);

#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)
#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {
constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::uint32_t kTelemetryPeriodCycles = 1U;
constexpr std::int64_t kRunDurationMs = 300000;

constexpr float kRequestedVelocityRadS = 5.0f;
constexpr float kRequestedVelocityAbsMaxRadS = 50.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.8f;
constexpr float kDeadbandRadS = 0.20f;
constexpr float kVelocityFilterTauS = 0.025f;

float requestedVelocityForTime(std::int64_t elapsed_ms) {
    if (elapsed_ms < 500) {
        return 0.0f;
    }
    if (elapsed_ms < kRunDurationMs) {
        return kRequestedVelocityRadS * std::sin(static_cast<float>(elapsed_ms) / 1000.0f);
    }
    return 0.0f;
}

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

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can2));
    if (!device_is_ready(can))
        return -ENODEV;
    static skywalker::motor::Motor drive{skywalker::motor::dji::m3508({
        .id = 3,
        .current_limit_a = 3.0f,
        // .encoder_zero_ticks = 0,
        // .current_mode_confirmed = true,
        .gear_ratio = 19.0f,
        .timing =
            {.feedback_timeout_ms = 20, .command_timeout_ms = 20, .enable_timeout_ms = 100, .retry_interval_ms = 100},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::VelocityMotor axis{drive, makeMotorConfig()};
    static Vofa vofa{};
    const int vofa_error = vofa_init(&vofa, uart);
    int ret = bus.attach(drive);
    if (ret == 0)
        ret = bus.start();
    if (ret == 0)
        ret = axis.configure();
    if (ret < 0)
        return ret;
    const auto started_ms = k_uptime_get();
    auto previous_ms = started_ms;
    std::int64_t next_log_ms = 0;
    int last_call_error = 0;
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const float dt = float(now - previous_ms) / 1000;
        previous_ms = now;
        const bool requested = true;
        const int request_error = requested ? drive.enable() : drive.disable();
        if (request_error < 0)
            last_call_error = request_error;
        const float target = requestedVelocityForTime((now - started_ms) % kRunDurationMs);
        if (requested) {
            const int error = axis.update(target, dt);
            if (error < 0)
                last_call_error = error;
        }
        const int commit_error = bus.commit().error;
        if (commit_error < 0)
            last_call_error = commit_error;
        const auto data = axis.telemetry();
        const auto &f = data.motor.feedback;
        const auto &o = data.output;
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
                                        float(now >= f.timestamp_ms ? now - f.timestamp_ms : 0)};
            (void)vofa_send(&vofa, channels, 12);
        }
        if (now >= next_log_ms) {
            next_log_ms = now + 1000;
            LOG_INF("run=%d state=%u target=%.3f seq=%llu output=%d wait=%u call=%d", requested,
                    unsigned(data.motor.state), double(target), static_cast<unsigned long long>(data.target_sequence),
                    data.output_valid && requested, unsigned(data.issue), last_call_error);
        }
    }
}
