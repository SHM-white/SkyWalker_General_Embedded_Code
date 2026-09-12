#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/velocity_motor.hpp>
#include <control/dji_motor_backend.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_speed_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)
#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {
constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::uint32_t kTelemetryPeriodCycles = 1U;
constexpr std::int64_t kRunDurationMs = 300000;

constexpr float kRequestedVelocityRadS = 20.0f;
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
    config.safety = {1.5f * kRequestedVelocityAbsMaxRadS, 0.0f};
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(uart)) return -ENODEV;
    // Bus and UART callbacks retain these objects after an early return.
    static skywalker::control::DjiMotorBackend backend{
        DEVICE_DT_GET(MOTOR0_NODE)};
    static skywalker::control::VelocityMotor motor{backend, makeMotorConfig()};
    static Vofa vofa{};
    vofa_init(&vofa, uart);
    int ret = motor.begin();
    if (ret < 0) {
        LOG_ERR("begin failed: cause=%d stop=%d", ret, motor.status().stop_error);
        return ret;
    }
    std::uint32_t telemetry_divider = 0;
    while (motor.elapsedMs() < kRunDurationMs) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const float target = requestedVelocityForTime(motor.elapsedMs());
        ret = motor.update(target);
        if (ret < 0) {
            LOG_ERR("update failed: cause=%d stop=%d", ret, motor.status().stop_error);
            return ret;
        }
        const auto &data = motor.telemetry();
        const auto &feedback = data.measurement.feedback;
        const auto &output = data.output;
        if (++telemetry_divider >= kTelemetryPeriodCycles) {
            telemetry_divider = 0;
            const float channels[12] = {
                target, output.velocity_reference_rad_s, feedback.velocity_rad_s,
                output.filtered_velocity_rad_s, output.velocity_error_rad_s,
                output.regulator.feedback.p, output.regulator.feedback.i,
                output.regulator.feedback.d, output.regulator.feedforward,
                output.effort_command, output.regulator.feedback.saturated ? 1.0f : 0.0f,
                static_cast<float>(k_uptime_get() - feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 12);
        }
    }
    ret = motor.stop();
    if (ret < 0) LOG_ERR("stop failed: %d", ret);
    return ret;
}
