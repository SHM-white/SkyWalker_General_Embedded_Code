#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
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
    config.safety = {1.5f * kRequestedVelocityAbsMaxRadS, 0.0f};
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(uart) || !device_is_ready(can))
        return -ENODEV;
    static skywalker::motor::Motor drive{skywalker::motor::dji::m2006({
        .id = 4,
        .current_limit_a = 10.0f,
        .gear_ratio = 36.0f,
        .timing = {20, 20, 20, 100},
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
        LOG_ERR("configuration failed: %d", ret);
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
    while (k_uptime_get() - started_ms < kRunDurationMs) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const float dt_s = float(now - previous_ms) / 1000.0f;
        previous_ms = now;
        if (!drive.active())
            return -EHOSTDOWN;
        const float target = now - started_ms < 100 ? 0.0f : kRequestedVelocityRadS;
        ret = axis.update(target, dt_s);
        if (ret == 0)
            ret = bus.commit().error;
        if (ret < 0) {
            (void)drive.disable();
            LOG_ERR("update failed: %d", ret);
            return ret;
        }
        const auto data = axis.telemetry();
        const auto &feedback = data.motor.feedback;
        const auto &output = data.output;
        const float channels[10] = {
            target,
            output.velocity_reference_rad_s,
            feedback.velocity_rad_s,
            output.regulator.feedback.error,
            output.regulator.feedback.p,
            output.regulator.feedback.i,
            output.regulator.feedforward,
            output.effort_command,
            output.regulator.feedback.saturated ? 1.0f : 0.0f,
            static_cast<float>(k_uptime_get() - feedback.timestamp_ms),
        };
        vofa_send(&vofa, channels, 10);
    }
    ret = drive.disable();
    if (ret < 0)
        LOG_ERR("stop failed: %d", ret);
    return ret;
}
