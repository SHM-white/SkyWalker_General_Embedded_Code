#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/position_motor.hpp>
#include <control/dji_motor_backend.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_position_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)
#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {
constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::uint32_t kTelemetryPeriodCycles = 5U;
constexpr float kTargetOffsetRad = 3.0f;
constexpr std::int64_t kAbsoluteTargetHoldMs = 5000;

constexpr float kPositionKp = 6.0f;
constexpr float kVelocityKp = 0.03f;
constexpr float kVelocityKi = 0.5f;
constexpr float kVelocityRampRateRadS2 = 40.0f;

constexpr float kVelocityAbsMaxRadS = 30.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.80f;
constexpr float kPositionDeadbandRad = 0.012f;
constexpr float kMeasuredVelocitySafetyMaxRadS = 1.5f * kVelocityAbsMaxRadS;
constexpr float kPi = 3.14159265358979323846f;

enum class PositionTargetMode {
    ContinuousRelative = 0,
    FixedZeroAbsolute,
};

/* Fixed encoder zero; the backend must advertise absolute position. */
constexpr PositionTargetMode kPositionTargetMode = PositionTargetMode::FixedZeroAbsolute;


control_motor_position_config makePositionLoopConfig() {
    control_motor_position_config config{};

    config.position = {
        .kp = kPositionKp,
        .ki = 1.5f,
        .kd = 0.8f,
        .derivative_tau_s = 0.0f,
        .integral_min = -8.0f,
        .integral_max = 8.0f,
        .output_min = -kVelocityAbsMaxRadS,
        .output_max = kVelocityAbsMaxRadS,
        .deadband = kPositionDeadbandRad,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    config.velocity.regulator.feedback = {
        .kp = kVelocityKp,
        .ki = kVelocityKi,
        .kd = 0.00005f,
        .derivative_tau_s = 0.0f,
        .integral_min = -kSoftwareCurrentAbsMaxA,
        .integral_max = kSoftwareCurrentAbsMaxA,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };
    config.velocity.regulator.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };
    config.velocity.reference_slew = {
        .rising_rate_per_s = kVelocityRampRateRadS2,
        .falling_rate_per_s = kVelocityRampRateRadS2,
    };
    /* Zero filtering and zero soft deadband preserve the old sample. */
    config.velocity.measurement_filter_tau_s = 0.0f;
    config.velocity.soft_deadband_rad_s = 0.0f;
    config.velocity.requested_velocity_abs_max_rad_s = kVelocityAbsMaxRadS;
    config.velocity.effort_abs_max = kSoftwareCurrentAbsMaxA;
    return config;
}

float requestedAbsolutePositionRad(std::int64_t elapsed_ms) {
    const std::int64_t phase_ms = elapsed_ms % (4 * kAbsoluteTargetHoldMs);
    if (phase_ms < kAbsoluteTargetHoldMs) {
        return 0.0f;
    }
    if (phase_ms < 2 * kAbsoluteTargetHoldMs) {
        return 90.0f * kPi / 180.0f;
    }
    if (phase_ms < 3 * kAbsoluteTargetHoldMs) {
        return 180.0f * kPi / 180.0f;
    }
    return 270.0f * kPi / 180.0f;
}

skywalker::control::PositionMotor::Config makeMotorConfig() {
    skywalker::control::PositionMotor::Config config{};
    config.loop = makePositionLoopConfig();
    config.effort_unit = skywalker::control::EffortUnit::Ampere;
    config.safety = {kMeasuredVelocitySafetyMaxRadS, 0.0f};
    config.reference = kPositionTargetMode == PositionTargetMode::FixedZeroAbsolute
        ? skywalker::control::PositionReference::AbsoluteNearest
        : skywalker::control::PositionReference::StartupRelative;
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(uart)) return -ENODEV;
    // Bus and UART callbacks retain these objects after an early return.
    static skywalker::control::DjiMotorBackend backend{
        DEVICE_DT_GET(MOTOR0_NODE)};
    static skywalker::control::PositionMotor motor{backend, makeMotorConfig()};
    static Vofa vofa{};
    vofa_init(&vofa, uart);
    int ret = motor.begin();
    if (ret < 0) {
        LOG_ERR("begin failed: cause=%d stop=%d", ret, motor.status().stop_error);
        return ret;
    }
    std::uint32_t telemetry_divider = 0;
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto elapsed_ms = motor.elapsedMs();
        const float target = kPositionTargetMode == PositionTargetMode::FixedZeroAbsolute
            ? requestedAbsolutePositionRad(elapsed_ms)
            : static_cast<float>((elapsed_ms % 10000) / 2500) * kTargetOffsetRad;
        ret = motor.update(target);
        if (ret < 0) {
            LOG_ERR("update failed: cause=%d stop=%d", ret, motor.status().stop_error);
            return ret;
        }
        if (++telemetry_divider >= kTelemetryPeriodCycles) {
            telemetry_divider = 0;
            const auto &data = motor.telemetry();
            const auto &feedback = data.measurement.feedback;
            const auto &output = data.output;
            const float channels[14] = {
                target, static_cast<float>(data.target_position_rad),
                static_cast<float>(data.measurement.position_rad),
                kPositionTargetMode == PositionTargetMode::FixedZeroAbsolute ? feedback.absolute_position_rad : 0.0f,
                output.position.error, output.position.output,
                output.velocity.velocity_reference_rad_s, feedback.velocity_rad_s,
                output.velocity.velocity_error_rad_s, output.velocity.regulator.feedback.p,
                output.velocity.regulator.feedback.i, output.effort_command,
                data.dt_s * 1000.0f, static_cast<float>(k_uptime_get() - feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 14);
        }
    }
}
