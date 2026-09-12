#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/position_controller.hpp>
#include <dm_sample_support.hpp>
#include <drivers/motor/motor.hpp>
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
constexpr float kArrivalToleranceRad = 0.03f;
constexpr float kArrivalVelocityRadS = 0.05f;

// 360 degrees and 0 degrees name the same encoder phase.
float singleTurnRad(float position_rad) {
    float phase = std::fmod(position_rad, kTwoPi);
    if (phase < 0.0f) {
        phase += kTwoPi;
    }
    return phase >= kTwoPi ? 0.0f : phase;
}

skywalker::control::PositionController::Config makePositionControllerConfig() {
    skywalker::control::PositionController::Config config{};
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
    constexpr std::uint32_t required = skywalker::motor::CommandTorque | skywalker::motor::FeedbackPosition |
                                       skywalker::motor::FeedbackVelocity;
    if ((skywalker::motor::capabilities(motor) & required) != required) {
        LOG_ERR("MIT torque command or position/velocity feedback capability unavailable");
        return -ENOTSUP;
    }

    skywalker::control::PositionController controller{makePositionControllerConfig()};
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
    float previous_feedback_position_rad = first_feedback.position_rad;
    double continuous_position_rad = singleTurnRad(first_feedback.position_rad);
    // Start at the saved encoder zero, approaching it in the positive direction.
    // Treat quantization noise on either side of zero as already at zero.
    double continuous_target_rad = continuous_position_rad <= static_cast<double>(kArrivalToleranceRad)
                                       ? 0.0 : static_cast<double>(kTwoPi);
    ret = controller.reset(static_cast<float>(continuous_position_rad - continuous_target_rad),
                           first_feedback.velocity_rad_s);
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

    unsigned int target_quarter_turn = 0U;
    float target_position_rad = 0.0f;
    bool zero_reached = false;
    std::int64_t next_position_step_ms = 0;
    std::int64_t previous_cycle_ms = k_uptime_get();
    LOG_INF("MIT position: approach saved zero, then +90 deg every %lld ms; command=[0,2pi), torque_limit=%d mNm",
            kPositionStepPeriodMs, static_cast<int>(kSoftwareTorqueAbsMaxNm * 1000.0f));

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

        // MIT feedback is encoded in [-PMAX, PMAX]. Unwrap its range before
        // reducing the displayed angle modulo 2pi: 2*PMAX need not equal 2pi.
        // Requires feedback to wrap at those endpoints, not saturate there.
        const float position_delta_rad = std::remainder(
            feedback.position_rad - previous_feedback_position_rad,
            2.0f * session.descriptor.limits.position_max_rad);
        previous_feedback_position_rad = feedback.position_rad;
        continuous_position_rad += static_cast<double>(position_delta_rad);

        const bool arrived = std::fabs(continuous_target_rad - continuous_position_rad) <=
                                 static_cast<double>(kArrivalToleranceRad) &&
                             std::fabs(feedback.velocity_rad_s) <= kArrivalVelocityRadS;
        if (!zero_reached && arrived) {
            zero_reached = true;
            next_position_step_ms = now_ms + kPositionStepPeriodMs;
            LOG_INF("encoder zero reached; starting positive quarter-turn sequence");
        }
        if (zero_reached && now_ms >= next_position_step_ms) {
            if (arrived) {
                target_quarter_turn = (target_quarter_turn + 1U) % 4U;
                target_position_rad = target_quarter_turn * kPositionStepRad;
                continuous_target_rad += static_cast<double>(kPositionStepRad);
                LOG_INF("new encoder target: %u deg (+90 deg)", target_quarter_turn * 90U);
            } else {
                LOG_WRN("target not settled; holding this step");
            }
            next_position_step_ms = now_ms + kPositionStepPeriodMs;
        }

        skywalker::control::PositionController::Output output{};
        // Use coordinates relative to this target to keep float precision after
        // many turns. The position PID uses P/I only (kd=0, integral frozen).
        ret = controller.step(0.0f, static_cast<float>(continuous_position_rad - continuous_target_rad),
                              feedback.velocity_rad_s, dt_s, output);
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

        const float channels[10] = {
            target_position_rad,
            singleTurnRad(static_cast<float>(std::fmod(continuous_position_rad, static_cast<double>(kTwoPi)))),
            output.position.error,
            output.velocity.velocity_reference_rad_s,
            feedback.velocity_rad_s,
            output.velocity.velocity_error_rad_s,
            output.effort_command,
            feedback.torque_nm,
            static_cast<float>(raw.mos_temperature_c),
            static_cast<float>(raw.rotor_temperature_c),
        };
        vofa_send(&vofa, channels, 10);
    }
}
