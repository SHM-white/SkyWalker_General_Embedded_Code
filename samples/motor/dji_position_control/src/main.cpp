#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/position_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_position_control, LOG_LEVEL_INF);

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

/* Fixed encoder zero; the motor configuration must support absolute position. */
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
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(uart) || !device_is_ready(can))
        return -ENODEV;
    static skywalker::motor::Motor drive{skywalker::motor::dji::gm6020({
        .id = 4, .current_limit_a = 1.5f, .encoder_zero_ticks = 0,
        .current_mode_confirmed = true, .timing = {20, 20, 20, 100},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::PositionMotor axis{drive, makeMotorConfig()};
    static Vofa vofa{};
    vofa_init(&vofa, uart);
    int ret = bus.attach(drive);
    if (ret == 0)
        ret = bus.start();
    if (ret == 0)
        ret = axis.configure();
    if (ret < 0) {
        LOG_ERR("configuration blocked: %d", ret);
        return ret;
    }
    const auto ready_deadline = k_uptime_get() + 3000;
    while (!drive.ready() && k_uptime_get() < ready_deadline)
        k_sleep(K_MSEC(kControlPeriodMs));
    if (!drive.ready())
        return -ETIMEDOUT;
    const auto initial = drive.snapshot();
    if ((initial.feedback.valid & skywalker::motor::FeedbackAbsolutePosition) == 0u)
        return -ENODATA;
    // Preserve the old GM6020 continuous coordinate seeded from encoder zero.
    ret = drive.reseedPosition(initial.feedback.absolute_position_rad);
    if (ret < 0)
        return ret;
    ret = axis.reset(); // Check speed, temperature and reference before enabling.
    if (ret == 0)
        ret = drive.enable();
    if (ret < 0)
        return ret;
    const auto active_deadline = k_uptime_get() + 3000;
    while (!drive.active() && k_uptime_get() < active_deadline)
        k_sleep(K_MSEC(kControlPeriodMs));
    if (!drive.active()) {
        (void)drive.disable();
        return -ETIMEDOUT;
    }
    const auto started_ms = k_uptime_get();
    auto previous_ms = started_ms;
    std::uint32_t telemetry_divider = 0;
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const float dt_s = float(now - previous_ms) / 1000.0f;
        previous_ms = now;
        if (!drive.active())
            return -EHOSTDOWN;
        const auto elapsed_ms = now - started_ms;
        const float target = kPositionTargetMode == PositionTargetMode::FixedZeroAbsolute
                                 ? requestedAbsolutePositionRad(elapsed_ms)
                                 : static_cast<float>((elapsed_ms % 10000) / 2500) * kTargetOffsetRad;
        ret = axis.update(target, dt_s);
        if (ret == 0)
            ret = bus.commit().error;
        if (ret < 0) {
            (void)drive.disable();
            LOG_ERR("cycle stopped: %d", ret);
            return ret;
        }
        if (++telemetry_divider >= kTelemetryPeriodCycles) {
            telemetry_divider = 0;
            const auto data = axis.telemetry();
            const auto &feedback = data.motor.feedback;
            const auto &output = data.output;
            const float channels[14] = {
                target,
                static_cast<float>(data.target_position_rad),
                static_cast<float>(data.position_rad),
                kPositionTargetMode == PositionTargetMode::FixedZeroAbsolute ? feedback.absolute_position_rad : 0.0f,
                output.position.error,
                output.position.output,
                output.velocity.velocity_reference_rad_s,
                feedback.velocity_rad_s,
                output.velocity.velocity_error_rad_s,
                output.velocity.regulator.feedback.p,
                output.velocity.regulator.feedback.i,
                output.effort_command,
                data.dt_s * 1000.0f,
                static_cast<float>(k_uptime_get() - feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 14);
        }
    }
}
