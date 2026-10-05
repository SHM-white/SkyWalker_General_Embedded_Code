#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/position_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <dm_sample_support.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_mit_position_control, LOG_LEVEL_INF);

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

    config.reference = skywalker::control::PositionReference::DriverContinuous;
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(can))
        return -ENODEV;
    static skywalker::motor::Motor drive{skywalker::motor::dm::j4310Mit({
        .id = 1,
        .master_id = 0x11,
        .position_max_rad = 12.5f,
        .velocity_max_rad_s = 30.0f,
        .torque_max_nm = 10.0f,
        .torque_limit_nm = 1.0f,
        .timing =
            {.feedback_timeout_ms = 50, .command_timeout_ms = 20, .enable_timeout_ms = 3000, .retry_interval_ms = 100},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::PositionMotor axis{drive, makeMotorConfig()};
    static Vofa vofa{};
    const int vofa_error = vofa_init(&vofa, uart);
    int ret = bus.attach(drive);
    if (ret == 0)
        ret = bus.start();
    if (ret == 0)
        ret = skywalker::samples::dm::enableMotorPower();
    if (ret == 0)
        ret = axis.configure();
    if (ret < 0)
        return ret;
    const auto started_ms = k_uptime_get();
    auto previous_ms = started_ms;
    bool initial_target_valid = false;
    double zero_target = 0;
    std::int64_t next_log_ms = 0;
    int last_call_error = 0;
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const float dt = float(now - previous_ms) / 1000;
        previous_ms = now;
        const auto view = drive.snapshot();
        if (!initial_target_valid && view.feedback_fresh && view.position_reference_valid &&
            (view.feedback.valid & skywalker::motor::FeedbackPosition) && std::isfinite(view.feedback.position_rad)) {
            const double initial = view.feedback.position_rad;
            const double phase = singleTurnRad(static_cast<float>(initial));
            zero_target = initial - phase + (phase <= kZeroToleranceRad ? 0.0 : double(kTwoPi));
            initial_target_valid = true;
        }
        const auto elapsed_ms = now - started_ms;
        const auto step = elapsed_ms / kPositionStepPeriodMs;
        const double target = zero_target + double(step / 4) * kTwoPi + targetPositionRad(elapsed_ms);
        const int enable_error = drive.enable();
        // update accepts the latest goal even when the continuous reference is unavailable.
        const int update_error = axis.update(target, dt);
        const int commit_error = bus.commit().error;
        if (enable_error < 0)
            last_call_error = enable_error;
        if (update_error < 0)
            last_call_error = update_error;
        if (commit_error < 0)
            last_call_error = commit_error;
        const auto data = axis.telemetry();
        if (vofa_error == 0) {
            const float channels[10] = {targetPositionRad(elapsed_ms),
                                        singleTurnRad(float(data.position_rad)),
                                        data.output.position.error,
                                        data.output.velocity.velocity_reference_rad_s,
                                        data.motor.feedback.velocity_rad_s,
                                        data.output.velocity.velocity_error_rad_s,
                                        data.output_valid ? data.effort_command : 0,
                                        data.motor.feedback.torque_nm,
                                        data.motor.native_mos_temperature_c,
                                        data.motor.feedback.temperature_c};
            (void)vofa_send(&vofa, channels, 10);
        }
        if (now >= next_log_ms) {
            next_log_ms = now + 1000;
            LOG_INF("target=%.3f initial=%d seq=%llu output=%d wait=%u call=%d", target, initial_target_valid,
                    static_cast<unsigned long long>(data.target_sequence), data.output_valid, unsigned(data.issue),
                    last_call_error);
        }
    }
}
