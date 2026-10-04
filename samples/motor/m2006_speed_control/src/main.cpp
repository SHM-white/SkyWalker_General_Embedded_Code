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
    LOG_INF("M2006 speed control starting; text logs use the board UART console");
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(uart)) {
        LOG_ERR("VOFA device %s is not ready", uart->name);
        return -ENODEV;
    }
    if (!device_is_ready(can)) {
        LOG_ERR("CAN device %s is not ready", can->name);
        return -ENODEV;
    }
    static skywalker::motor::Motor drive{skywalker::motor::dji::m2006({
        .id = 1,
        .current_limit_a = 10.0f,
        .gear_ratio = 36.0f,
        .timing = {20, 20, 20, 100},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::VelocityMotor axis{drive, makeMotorConfig()};
    static Vofa vofa{};
    int ret = vofa_init(&vofa, uart);
    if (ret < 0) {
        LOG_ERR("vofa_init(%s) failed: %d", uart->name, ret);
        return ret;
    }
    ret = bus.attach(drive);
    if (ret < 0) {
        LOG_ERR("bus.attach failed: %d", ret);
        return ret;
    }
    ret = bus.start();
    if (ret < 0) {
        LOG_ERR("bus.start(%s) failed: %d", can->name, ret);
        return ret;
    }
    // Save the original fault before requesting an asynchronous safe stop.
    const auto stop_with_error = [](const char *stage, int error) {
        const auto motor = drive.snapshot();
        const auto transport = bus.status();
        const int stop_error = drive.disable();
        LOG_ERR("%s failed: %d; motor state=%u fresh=%u permitted=%u fault=%u error=%d",
                stage, error, unsigned(motor.state), unsigned(motor.feedback_fresh),
                unsigned(motor.output_permitted), unsigned(motor.last_fault.reason), motor.last_fault.error);
        LOG_ERR("feedback timestamp=%llu ms; bus state=%u error=%d; last TX valid=%u id=0x%x error=%d",
                static_cast<unsigned long long>(motor.feedback.timestamp_ms), unsigned(transport.state),
                transport.last_error, unsigned(transport.last_tx.valid), unsigned(transport.last_tx.can_id),
                transport.last_tx.error);
        if (stop_error < 0)
            LOG_ERR("safe stop request failed: %d", stop_error);
        return error;
    };
    ret = axis.configure();
    if (ret < 0)
        return stop_with_error("axis.configure", ret);
    LOG_INF("Waiting up to 3000 ms for M2006 ID 1 on %s (feedback 0x201); VOFA=%s",
            can->name, uart->name);
    const auto ready_deadline = k_uptime_get() + 3000;
    while (!drive.ready() && k_uptime_get() < ready_deadline)
        k_sleep(K_MSEC(kControlPeriodMs));
    if (!drive.ready())
        return stop_with_error("wait ready", -ETIMEDOUT);
    LOG_INF("Motor ready; resetting controller and enabling drive");
    ret = axis.reset(); // Check speed, temperature and reference before enabling.
    if (ret < 0)
        return stop_with_error("axis.reset", ret);
    ret = drive.enable();
    if (ret < 0)
        return stop_with_error("drive.enable", ret);
    const auto active_deadline = k_uptime_get() + 3000;
    while (!drive.active() && k_uptime_get() < active_deadline)
        k_sleep(K_MSEC(kControlPeriodMs));
    if (!drive.active())
        return stop_with_error("wait active", -ETIMEDOUT);
    LOG_INF("Motor active; starting 5 ms control loop and 10-channel VOFA JustFloat stream");
    const auto started_ms = k_uptime_get();
    auto previous_ms = started_ms;
    std::int64_t next_vofa_warning_ms = 0;
    while (k_uptime_get() - started_ms < kRunDurationMs) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const float dt_s = float(now - previous_ms) / 1000.0f;
        previous_ms = now;
        if (!drive.active())
            return stop_with_error("drive inactive", -EHOSTDOWN);
        const float target = now - started_ms < 100 ? 0.0f : kRequestedVelocityRadS;
        ret = axis.update(target, dt_s);
        if (ret < 0)
            return stop_with_error("axis.update", ret);
        ret = bus.commit().error;
        if (ret < 0)
            return stop_with_error("bus.commit", ret);
#if DT_NODE_HAS_COMPAT(VOFA_UART_NODE, zephyr_cdc_acm_uart)
        // USB enumeration does not mean a host is consuming telemetry. Keep
        // controlling the motor, but do not fill VOFA's queue before port open.
        std::uint32_t dtr = 0;
        const int line_error = uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
        if (line_error < 0 || dtr == 0) {
            if (line_error < 0 && now >= next_vofa_warning_ms) {
                LOG_WRN("VOFA DTR query failed: %d; control loop continues", line_error);
                next_vofa_warning_ms = now + 1000;
            }
            continue;
        }
#endif
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
        const int telemetry_error = vofa_send(&vofa, channels, 10);
        if (telemetry_error < 0 && now >= next_vofa_warning_ms) {
            LOG_WRN("vofa_send failed: %d; control loop continues", telemetry_error);
            next_vofa_warning_ms = now + 1000;
        }
    }
    ret = drive.disable();
    if (ret < 0)
        LOG_ERR("stop failed: %d", ret);
    else
        LOG_INF("Run duration reached; safe stop requested");
    return ret;
}
