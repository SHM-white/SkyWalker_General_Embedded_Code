#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/velocity_motor.hpp>
#include <control/motor_session.hpp>
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
    config.safety = {1.5f * kRequestedVelocityAbsMaxRadS, 0.0f};
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can2));
    if (!device_is_ready(uart) || !device_is_ready(can))
        return -ENODEV;
    static skywalker::motor::Motor drive{skywalker::motor::dji::m3508({
        .id = 3,
        .current_limit_a = 3.0f,
        // .encoder_zero_ticks = 0,
        // .current_mode_confirmed = true,
        .gear_ratio = 19.0f,
        .timing = {20, 20, 20, 100},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::VelocityMotor axis{drive, makeMotorConfig()};
    static const skywalker::control::MotorSession::Member members[] = {
        {"drive", &drive, &bus, &axis, skywalker::control::ReferencePolicy::NotRequired},
    };
    static skywalker::control::MotorSession session(members);
    static Vofa vofa{};
    vofa_init(&vofa, uart);
    const int ret = session.configure();
    if (ret < 0) {
        LOG_ERR("configuration blocked: %d", ret);
        return ret;
    }
    // Preserve the bench's one-shot automatic start. A fault never retries it.
    std::uint64_t target_sequence = 0;
    auto previous_us = skywalker::core::monotonicTimeUs();
    std::uint32_t telemetry_divider = 0;
    for (;;) {
        const auto wake_us = previous_us + kControlPeriodMs * 1000;
        if (skywalker::core::monotonicTimeUs() < wake_us) k_sleep(K_TIMEOUT_ABS_US(wake_us));
        else k_sleep(K_TICKS(1));
        const auto now_us = skywalker::core::monotonicTimeUs();
        previous_us = now_us;
        const auto before = session.status();
        const auto elapsed_ms = before.state == skywalker::control::SessionState::Running
            ? (now_us - before.started_us) / 1000 : 0;
        const float target = requestedVelocityForTime(elapsed_ms % kRunDurationMs);
        const double targets[] = {target};
        const skywalker::core::Stamp stamp{now_us, ++target_sequence, true};
        session.step({.enabled = true, .start_sequence = 1, .source = stamp,
                      .target_stamp = stamp, .targets = targets}, now_us);
        const auto state = session.status();
        if (state.state == skywalker::control::SessionState::Blocked)
            return state.last_stop.error;
        if (state.state != skywalker::control::SessionState::Running) continue;
        const auto data = axis.telemetry();
        const auto &feedback = data.motor.feedback;
        const auto &output = data.output;
        if (++telemetry_divider >= kTelemetryPeriodCycles) {
            telemetry_divider = 0;
            const float channels[12] = {
                target,
                output.velocity_reference_rad_s,
                feedback.velocity_rad_s,
                output.filtered_velocity_rad_s,
                output.velocity_error_rad_s,
                output.regulator.feedback.p,
                output.regulator.feedback.i,
                output.regulator.feedback.d,
                output.regulator.feedforward,
                output.effort_command,
                output.regulator.feedback.saturated ? 1.0f : 0.0f,
                static_cast<float>(k_uptime_get() - feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 12);
        }
    }
}
