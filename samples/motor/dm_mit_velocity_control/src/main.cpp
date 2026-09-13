#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/velocity_motor.hpp>
#include <control/dm_motor_backend.hpp>
#include <dm_sample_support.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_mit_velocity_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)
#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {
constexpr std::int64_t kControlPeriodMs = 5;
constexpr float kSoftwareTorqueAbsMaxNm = 0.5f;
constexpr float kRequestedVelocityAbsMaxRadS = 3.0f;
constexpr float kVelocityCutoffRadS = 10.0f;
constexpr float kTemperatureCutoffC = 60.0f;

static_assert(kRequestedVelocityAbsMaxRadS > 0.0f && kRequestedVelocityAbsMaxRadS < kVelocityCutoffRadS,
              "Velocity command limit must be positive and below the safety cutoff");


float targetVelocityRadS() {
    return 2.0f;
}

control_motor_velocity_config makeVelocityLoopConfig() {
    control_motor_velocity_config config{};
    config.regulator.feedback = {
        .kp = 0.02f,
        .ki = 0.15f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = -0.3f,
        .integral_max = 0.3f,
        .output_min = -kSoftwareTorqueAbsMaxNm,
        .output_max = kSoftwareTorqueAbsMaxNm,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };
    config.regulator.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };
    config.reference_slew = {
        .rising_rate_per_s = 2.0f,
        .falling_rate_per_s = 2.0f,
    };
    config.measurement_filter_tau_s = 0.02f;
    config.soft_deadband_rad_s = 0.02f;
    config.requested_velocity_abs_max_rad_s = kRequestedVelocityAbsMaxRadS;
    config.effort_abs_max = kSoftwareTorqueAbsMaxNm;
    return config;
}

skywalker::control::VelocityMotor::Config makeMotorConfig() {
    skywalker::control::VelocityMotor::Config config{};
    config.loop = makeVelocityLoopConfig();
    config.effort_unit = skywalker::control::EffortUnit::NewtonMeter;
    config.safety = {kVelocityCutoffRadS, kTemperatureCutoffC};
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(uart)) return -ENODEV;
    // Bus and UART callbacks retain these objects after an early return.
    static skywalker::control::DmMotorBackend backend{
        DEVICE_DT_GET(MOTOR0_NODE), skywalker::samples::dm::enableMotorPower};
    static skywalker::control::VelocityMotor motor{backend, makeMotorConfig()};
    static Vofa vofa{};
    vofa_init(&vofa, uart);
    int ret = motor.configure();
    if (ret < 0) {
        LOG_ERR("configuration blocked: cause=%d stop=%d", ret, motor.status().stop_error);
        return ret;
    }
    std::int64_t next_recovery_log_ms = 0;
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        if (motor.state() != skywalker::control::ExecutionState::Active) {
            const auto now = k_uptime_get();
            ret = motor.poll(now);
            if (ret == 0) ret = motor.resume();
            if (now >= next_recovery_log_ms) {
                next_recovery_log_ms = now + 1000;
                LOG_INF("recovery state=%u generation=%u result=%d", unsigned(motor.state()), motor.status().resume_generation, ret);
            }
            continue;
        }

        const float target = targetVelocityRadS();
        ret = motor.update(target);
        if (ret < 0) {
            LOG_WRN("cycle paused: cause=%d stop=%d", ret, motor.status().stop_error);
            continue;
        }
        const auto &data = motor.telemetry();
        const auto &feedback = data.measurement.feedback;
        const auto &output = data.output;
        const float channels[8] = {
            target, output.velocity_reference_rad_s, feedback.velocity_rad_s,
            output.velocity_error_rad_s, output.effort_command, feedback.torque_nm,
            data.measurement.driver_temperature_c, feedback.temperature_c,
        };
        vofa_send(&vofa, channels, 8);
    }
}
