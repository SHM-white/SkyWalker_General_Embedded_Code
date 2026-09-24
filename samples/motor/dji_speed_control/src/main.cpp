#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/velocity_motor.hpp>
#include <drivers/motor/can_bus.hpp>
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
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(uart) || !device_is_ready(can))
        return -ENODEV;
    static skywalker::motor::Motor drive{skywalker::motor::dji::gm6020({
        .id = 4, .current_limit_a = 3.0f, .encoder_zero_ticks = 0,
        .current_mode_confirmed = true, .timing = {20, 20, 20, 100},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::VelocityMotor axis{drive, makeMotorConfig()};
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
        if (!drive.active()) {
            LOG_ERR("motor stopped: reason=%u error=%d", unsigned(drive.snapshot().last_fault.reason),
                    drive.snapshot().last_fault.error);
            return -EHOSTDOWN;
        }
        const float target = requestedVelocityForTime((now - started_ms) % kRunDurationMs);
        ret = axis.update(target, dt_s);
        if (ret == 0)
            ret = bus.commit().error;
        if (ret < 0) {
            (void)drive.disable();
            LOG_ERR("cycle stopped: %d", ret);
            return ret;
        }
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
