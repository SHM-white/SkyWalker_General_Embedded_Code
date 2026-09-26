#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/velocity_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <dm_sample_support.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_mit_velocity_control, LOG_LEVEL_INF);

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
    static skywalker::control::VelocityMotor
        axis{drive, []() {
                 skywalker::control::VelocityMotor::Config config{};
                 config.loop = []() {
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
                 }();
                 config.effort_unit = skywalker::control::EffortUnit::NewtonMeter;
                 config.safety = {kVelocityCutoffRadS, kTemperatureCutoffC};
                 return config;
             }()};
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
    auto previous_ms = k_uptime_get();
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const float dt_s = float(now - previous_ms) / 1000.0f;
        previous_ms = now;
        if (!drive.active())
            return -EHOSTDOWN;
        const float target = targetVelocityRadS();
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
        const float channels[8] = {
            target,
            output.velocity_reference_rad_s,
            feedback.velocity_rad_s,
            output.velocity_error_rad_s,
            output.effort_command,
            feedback.torque_nm,
            data.motor.native_mos_temperature_c,
            feedback.temperature_c,
        };
        vofa_send(&vofa, channels, 8);
    }
}
