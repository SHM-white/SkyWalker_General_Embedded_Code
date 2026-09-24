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
    config.safety = {kVelocityCutoffRadS, kTemperatureCutoffC};
    config.reference = skywalker::control::PositionReference::DriverContinuous;
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(uart) || !device_is_ready(can))
        return -ENODEV;
    static skywalker::motor::Motor drive{skywalker::motor::dm::j4310Mit({
        .id = 1,
        .master_id = 0x11,
        .position_max_rad = 12.5f,
        .velocity_max_rad_s = 30.0f,
        .torque_max_nm = 10.0f,
        .torque_limit_nm = 1.0f,
        .timing = {50, 20, 50, 3000},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::PositionMotor axis{drive, makeMotorConfig()};
    static Vofa vofa{};
    vofa_init(&vofa, uart);
    int ret = bus.attach(drive);
    if (ret == 0)
        ret = bus.start();
    if (ret == 0)
        ret = skywalker::samples::dm::enableMotorPower();
    if (ret == 0)
        k_sleep(K_MSEC(1500));
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
    const auto before_reseed = drive.snapshot();
    if (!before_reseed.native_position_valid)
        return -ENODATA;
    // The old MIT position bench used the drive's native startup angle as its
    // continuous coordinate. Preserve that origin for the 0/90/180/270 path.
    ret = drive.reseedPosition(before_reseed.native_position_rad);
    if (ret < 0)
        return ret;
    const double initial = drive.snapshot().feedback.position_rad;
    const double phase = singleTurnRad(static_cast<float>(initial));
    const double zero_target = initial - phase +
                               (phase <= static_cast<double>(kZeroToleranceRad) ? 0.0 : static_cast<double>(kTwoPi));
    ret = axis.reset(); // Check speed, temperature and reference before enabling.
    if (ret == 0)
        ret = drive.enable();
    if (ret < 0)
        return ret;
    const auto active_deadline = k_uptime_get() + 5000;
    while (!drive.active() && k_uptime_get() < active_deadline)
        k_sleep(K_MSEC(kControlPeriodMs));
    if (!drive.active()) {
        (void)drive.disable();
        return -ETIMEDOUT;
    }
    const auto started_ms = k_uptime_get();
    auto previous_ms = started_ms;
    std::int64_t previous_step = 0;
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const float dt_s = float(now - previous_ms) / 1000.0f;
        previous_ms = now;
        if (!drive.active())
            return -EHOSTDOWN;
        const auto elapsed_ms = now - started_ms;
        const auto step = elapsed_ms / kPositionStepPeriodMs;
        const float single_turn_target = targetPositionRad(elapsed_ms);
        const double target = zero_target + static_cast<double>(step / 4) * static_cast<double>(kTwoPi) +
                              static_cast<double>(single_turn_target);
        ret = axis.update(target, dt_s);
        if (ret == 0)
            ret = bus.commit().error;
        if (ret < 0) {
            (void)drive.disable();
            LOG_ERR("cycle stopped: %d", ret);
            return ret;
        }
        if (step != previous_step) {
            LOG_INF("time=%lld ms target=%u deg", elapsed_ms, static_cast<unsigned int>(step % 4) * 90U);
            previous_step = step;
        }
        const auto data = axis.telemetry();
        const auto &feedback = data.motor.feedback;
        const auto &output = data.output;
        const float channels[10] = {
            single_turn_target,
            singleTurnRad(static_cast<float>(std::fmod(data.position_rad, static_cast<double>(kTwoPi)))),
            output.position.error,
            output.velocity.velocity_reference_rad_s,
            feedback.velocity_rad_s,
            output.velocity.velocity_error_rad_s,
            output.effort_command,
            feedback.torque_nm,
            data.motor.native_mos_temperature_c,
            feedback.temperature_c,
        };
        vofa_send(&vofa, channels, 10);
    }
}
