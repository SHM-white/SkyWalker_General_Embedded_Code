#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/velocity_controller.hpp>
#include <dm_sample_support.hpp>
#include <drivers/motor/motor.hpp>
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

skywalker::control::VelocityController::Config makeVelocityControllerConfig() {
    skywalker::control::VelocityController::Config config{};
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

} // namespace

int main() {
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    const struct device *vofa_uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(vofa_uart)) {
        LOG_ERR("VOFA UART device not ready");
        return -ENODEV;
    }

    // CAN RX callbacks retain session.bus even when main returns after a fault.
    static skywalker::samples::dm::Session session{};
    int ret = skywalker::samples::dm::prepare(session, motor);
    if (ret < 0) {
        return ret;
    }
    if ((skywalker::motor::capabilities(motor) &
         (skywalker::motor::CommandTorque | skywalker::motor::FeedbackVelocity)) !=
        (skywalker::motor::CommandTorque | skywalker::motor::FeedbackVelocity)) {
        LOG_ERR("MIT torque command or velocity feedback capability unavailable");
        return -ENOTSUP;
    }

    const float target_velocity_rad_s = targetVelocityRadS();
    if (!std::isfinite(target_velocity_rad_s) || std::fabs(target_velocity_rad_s) > kRequestedVelocityAbsMaxRadS) {
        LOG_ERR("invalid velocity target: %d mrad/s", static_cast<int>(target_velocity_rad_s * 1000.0f));
        return -ERANGE;
    }

    skywalker::control::VelocityController controller{makeVelocityControllerConfig()};
    ret = controller.validate();
    if (ret < 0) {
        LOG_ERR("controller config invalid: %d", ret);
        return ret;
    }

    if (kSoftwareTorqueAbsMaxNm > session.descriptor.torque_limit_nm) {
        LOG_ERR("controller torque limit exceeds motor software limit: %d > %d mNm",
                static_cast<int>(kSoftwareTorqueAbsMaxNm * 1000.0f),
                static_cast<int>(session.descriptor.torque_limit_nm * 1000.0f));
        return -ERANGE;
    }

    skywalker::motor::Feedback first_feedback{};
    skywalker::motor::dm::RawFeedback first_raw{};
    ret = skywalker::samples::dm::readSafeFeedback(session, kVelocityCutoffRadS, kTemperatureCutoffC, first_feedback,
                                                   first_raw);
    if (ret < 0) {
        return ret;
    }
    ret = controller.reset(first_feedback.velocity_rad_s);
    if (ret < 0) {
        LOG_ERR("controller reset failed: %d", ret);
        return ret;
    }

    // UART callbacks must also remain valid after an early return.
    static Vofa vofa{};
    vofa_init(&vofa, vofa_uart);
    ret = skywalker::samples::dm::arm(session);
    if (ret < 0) {
        return ret;
    }

    LOG_INF("MIT software velocity loop started: target=%d mrad/s torque_limit=%d mNm",
            static_cast<int>(target_velocity_rad_s * 1000.0f), static_cast<int>(kSoftwareTorqueAbsMaxNm * 1000.0f));
    std::int64_t previous_cycle_ms = k_uptime_get();
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const std::int64_t now_ms = k_uptime_get();
        if (now_ms <= previous_cycle_ms) {
            return skywalker::samples::dm::stopAfterFailure(session, -ERANGE);
        }
        const std::int64_t dt_ms = now_ms - previous_cycle_ms;
        const float dt_s = static_cast<float>(dt_ms) / 1000.0f;
        previous_cycle_ms = now_ms;
        if (dt_ms > 20) {
            LOG_ERR("control cycle overrun: dt=%lld ms, limit=20 ms; check debugger pauses", dt_ms);
            return skywalker::samples::dm::stopAfterFailure(session, -ERANGE);
        }

        skywalker::motor::Feedback feedback{};
        skywalker::motor::dm::RawFeedback raw{};
        ret = skywalker::samples::dm::readSafeFeedback(session, kVelocityCutoffRadS, kTemperatureCutoffC, feedback,
                                                       raw);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        skywalker::control::VelocityController::Output output{};
        ret = controller.step(target_velocity_rad_s, feedback.velocity_rad_s, dt_s, output);
        if (ret < 0) {
            LOG_ERR("controller step failed: ret=%d dt=%lld ms", ret, dt_ms);
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }
        ret = skywalker::motor::setTorque(motor, output.effort_command);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }
        ret = skywalker::samples::dm::flush(session);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        const float channels[8] = {
            target_velocity_rad_s,
            output.velocity_reference_rad_s,
            feedback.velocity_rad_s,
            output.velocity_error_rad_s,
            output.effort_command,
            feedback.torque_nm,
            static_cast<float>(raw.mos_temperature_c),
            static_cast<float>(raw.rotor_temperature_c),
        };
        vofa_send(&vofa, channels, 8);
    }
}
