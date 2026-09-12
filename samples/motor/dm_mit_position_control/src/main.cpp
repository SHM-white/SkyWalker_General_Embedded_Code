#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/position_motor.hpp>
#include <control/dm_motor_backend.hpp>
#include <dm_sample_support.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_mit_position_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)
#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {
constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::int64_t kPositionStepPeriodMs = 6000;
constexpr float kSoftwareTorqueAbsMaxNm = 0.5f;
constexpr float kVelocityAbsMaxRadS = 5.0f;
constexpr float kVelocityCutoffRadS = 10.0f;
constexpr float kTemperatureCutoffC = 60.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kPositionStepRad = kPi / 2.0f;
constexpr float kZeroToleranceRad = 0.03f;


float targetPositionRad(std::int64_t elapsed_ms) {
    if (elapsed_ms <= 0) {
        return 0.0f;
    }
    return static_cast<float>((elapsed_ms / kPositionStepPeriodMs) % 4) * kPositionStepRad;
}

float singleTurnRad(float position_rad) {
    float phase = std::fmod(position_rad, kTwoPi);
    if (phase < 0.0f) {
        phase += kTwoPi;
    }
    return phase >= kTwoPi ? 0.0f : phase;
}

control_motor_position_config makePositionLoopConfig() {
    control_motor_position_config config{};
    config.position = {
        .kp = 0.8f,
        .ki = 0.1f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = -0.3f,
        .integral_max = 0.3f,
        .output_min = -kVelocityAbsMaxRadS,
        .output_max = kVelocityAbsMaxRadS,
        .deadband = 0.01f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };
    config.velocity.regulator.feedback = {
        .kp = 0.03f,
        .ki = 0.1f,
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
        .rising_rate_per_s = 2.0f,
        .falling_rate_per_s = 2.0f,
    };
    config.velocity.measurement_filter_tau_s = 0.02f;
    config.velocity.soft_deadband_rad_s = 0.02f;
    config.velocity.requested_velocity_abs_max_rad_s = kVelocityAbsMaxRadS;
    config.velocity.effort_abs_max = kSoftwareTorqueAbsMaxNm;
    return config;
}

skywalker::control::PositionMotor::Config makeMotorConfig() {
    skywalker::control::PositionMotor::Config config{};
    config.loop = makePositionLoopConfig();
    config.effort_unit = skywalker::control::EffortUnit::NewtonMeter;
    config.safety = {kVelocityCutoffRadS, kTemperatureCutoffC};
    config.reference = skywalker::control::PositionReference::DriverContinuous;
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(uart)) return -ENODEV;
    // Bus and UART callbacks retain these objects after an early return.
    static skywalker::control::DmMotorBackend backend{
        DEVICE_DT_GET(MOTOR0_NODE), skywalker::samples::dm::enableMotorPower};
    static skywalker::control::PositionMotor motor{backend, makeMotorConfig()};
    static Vofa vofa{};
    vofa_init(&vofa, uart);
    int ret = motor.begin();
    if (ret < 0) {
        LOG_ERR("begin failed: cause=%d stop=%d", ret, motor.status().stop_error);
        return ret;
    }
    const double initial = motor.telemetry().measurement.position_rad;
    const double phase = singleTurnRad(static_cast<float>(initial));
    const double zero_target = initial - phase +
        (phase <= static_cast<double>(kZeroToleranceRad) ? 0.0 : static_cast<double>(kTwoPi));
    std::int64_t previous_step = 0;
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto elapsed_ms = motor.elapsedMs();
        const auto step = elapsed_ms / kPositionStepPeriodMs;
        const float single_turn_target = targetPositionRad(elapsed_ms);
        const double target = zero_target + static_cast<double>(step / 4) * static_cast<double>(kTwoPi) +
                              static_cast<double>(single_turn_target);
        ret = motor.update(target);
        if (ret < 0) {
            LOG_ERR("update failed: cause=%d stop=%d", ret, motor.status().stop_error);
            return ret;
        }
        if (step != previous_step) {
            LOG_INF("time=%lld ms target=%u deg", elapsed_ms, static_cast<unsigned int>(step % 4) * 90U);
            previous_step = step;
        }
        const auto &data = motor.telemetry();
        const auto &feedback = data.measurement.feedback;
        const auto &output = data.output;
        const float channels[10] = {
            single_turn_target,
            singleTurnRad(static_cast<float>(std::fmod(data.position_rad, static_cast<double>(kTwoPi)))),
            output.position.error, output.velocity.velocity_reference_rad_s,
            feedback.velocity_rad_s, output.velocity.velocity_error_rad_s,
            output.effort_command, feedback.torque_nm,
            data.measurement.driver_temperature_c, feedback.temperature_c,
        };
        vofa_send(&vofa, channels, 10);
    }
}
