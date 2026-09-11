#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/angle.h>
#include <control/position_controller.hpp>
#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_position_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)

#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {

constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::uint32_t kTelemetryPeriodCycles = 5U;
constexpr float kTargetOffsetRad = 3.0f;
constexpr std::int64_t kAbsoluteTargetHoldMs = 5000;

constexpr float kPositionKp = 6.0f;
constexpr float kVelocityKp = 0.03f;
constexpr float kVelocityKi = 0.5f;
constexpr float kVelocityRampRateRadS2 = 40.0f;

constexpr float kVelocityAbsMaxRadS = 30.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.80f;
constexpr float kPositionDeadbandRad = 0.012f;
constexpr float kMeasuredVelocitySafetyMaxRadS = 1.5f * kVelocityAbsMaxRadS;
constexpr float kPi = 3.14159265358979323846f;

enum class PositionTargetMode {
    ContinuousRelative = 0,
    FixedZeroAbsolute,
};

/* Keep relative mode for the first post-refactor hardware comparison. */
constexpr PositionTargetMode kPositionTargetMode = PositionTargetMode::FixedZeroAbsolute;

skywalker::motor::dji::Bus dji_bus;
volatile std::int32_t runtime_diagnostic = 0;

skywalker::control::PositionController::Config makePositionControllerConfig() {
    skywalker::control::PositionController::Config config{};

    config.position = {
        .kp = kPositionKp,
        .ki = 1.5f,
        .kd = 0.8f,
        .derivative_tau_s = 0.0f,
        .integral_min = -8.0f,
        .integral_max = 8.0f,
        .output_min = -kVelocityAbsMaxRadS,
        .output_max = kVelocityAbsMaxRadS,
        .deadband = kPositionDeadbandRad,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    config.velocity.regulator.feedback = {
        .kp = kVelocityKp,
        .ki = kVelocityKi,
        .kd = 0.00005f,
        .derivative_tau_s = 0.0f,
        .integral_min = -kSoftwareCurrentAbsMaxA,
        .integral_max = kSoftwareCurrentAbsMaxA,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
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
        .rising_rate_per_s = kVelocityRampRateRadS2,
        .falling_rate_per_s = kVelocityRampRateRadS2,
    };
    /* Zero filtering and zero soft deadband preserve the old sample. */
    config.velocity.measurement_filter_tau_s = 0.0f;
    config.velocity.soft_deadband_rad_s = 0.0f;
    config.velocity.requested_velocity_abs_max_rad_s = kVelocityAbsMaxRadS;
    config.velocity.current_abs_max_a = kSoftwareCurrentAbsMaxA;
    return config;
}

std::uint32_t requiredMotorCapabilities() {
    std::uint32_t required = skywalker::motor::CommandCurrent | skywalker::motor::FeedbackPosition |
                             skywalker::motor::FeedbackVelocity;
    if (kPositionTargetMode == PositionTargetMode::FixedZeroAbsolute) {
        required |= skywalker::motor::FeedbackAbsolutePosition;
    }
    return required;
}

int requireMotorCapabilities(const struct device *motor) {
    const std::uint32_t required = requiredMotorCapabilities();
    const std::uint32_t available = skywalker::motor::capabilities(motor);
    return (available & required) == required ? 0 : -ENOTSUP;
}

int waitForFreshFeedback(const struct device *motor) {
    const std::int64_t deadline_ms = k_uptime_get() + 2000;
    while (skywalker::motor::getState(motor) != skywalker::motor::State::Ready) {
        if (k_uptime_get() >= deadline_ms) {
            return -ETIMEDOUT;
        }
        k_sleep(K_MSEC(5));
    }
    return 0;
}

int readFreshPositionFeedback(const struct device *motor, std::uint64_t now_ms, skywalker::motor::Feedback &feedback) {
    int ret = skywalker::motor::readFeedback(motor, feedback);
    if (ret < 0) {
        return ret;
    }
    if (skywalker::motor::getState(motor) != skywalker::motor::State::Ready) {
        return -EHOSTDOWN;
    }

    const std::uint32_t required_feedback = requiredMotorCapabilities() & ~skywalker::motor::CommandCurrent;
    if ((feedback.valid & required_feedback) != required_feedback) {
        return -ENODATA;
    }
    if (!std::isfinite(feedback.position_rad) || !std::isfinite(feedback.velocity_rad_s)) {
        return -EINVAL;
    }
    if (kPositionTargetMode == PositionTargetMode::FixedZeroAbsolute &&
        !std::isfinite(feedback.absolute_position_rad)) {
        return -EINVAL;
    }
    if (feedback.timestamp_ms == 0U || now_ms < feedback.timestamp_ms ||
        now_ms - feedback.timestamp_ms > CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS) {
        return -ESTALE;
    }
    return 0;
}

int stopAfterFailure(int original_error) {
    runtime_diagnostic = original_error;
    skywalker::motor::dji::FlushReport report{};
    const int stop_ret = dji_bus.stop(report);
    LOG_ERR("control failed: cause=%d stop=%d zero=%d zero_err=%d", original_error, stop_ret, report.zero_sent ? 1 : 0,
            report.zero_tx_error);
    return stop_ret < 0 ? stop_ret : original_error;
}

float requestedRelativePositionRad(std::int64_t elapsed_ms, float /* initial_position_rad */, float final_target_rad) {
    const std::int64_t phase_ms = elapsed_ms % 10000;
    if (phase_ms < 2500) {
        return final_target_rad * 0.0f;
    }
    if (phase_ms < 5000) {
        return final_target_rad;
    }
    if (phase_ms < 7500) {
        return final_target_rad * 2.0f;
    }
    return final_target_rad * 3.0f;
}

float requestedAbsolutePositionRad(std::int64_t elapsed_ms) {
    const std::int64_t phase_ms = elapsed_ms % (4 * kAbsoluteTargetHoldMs);
    if (phase_ms < kAbsoluteTargetHoldMs) {
        return 0.0f;
    }
    if (phase_ms < 2 * kAbsoluteTargetHoldMs) {
        return 90.0f * kPi / 180.0f;
    }
    if (phase_ms < 3 * kAbsoluteTargetHoldMs) {
        return 180.0f * kPi / 180.0f;
    }
    return 270.0f * kPi / 180.0f;
}

} // namespace

int main() {
    runtime_diagnostic = 1;
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    const struct device *vofa_uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(motor)) {
        LOG_ERR("motor device not ready");
        return -ENODEV;
    }
    if (!device_is_ready(vofa_uart)) {
        LOG_ERR("VOFA UART device not ready");
        return -ENODEV;
    }

    int ret = requireMotorCapabilities(motor);
    if (ret < 0) {
        LOG_ERR("motor capability mismatch: required=0x%08x available=0x%08x", requiredMotorCapabilities(),
                skywalker::motor::capabilities(motor));
        return ret;
    }

    Vofa vofa{};
    vofa_init(&vofa, vofa_uart);

    skywalker::motor::dji::Descriptor descriptor{};
    ret = skywalker::motor::dji::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr || !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }
    runtime_diagnostic = 10;

    skywalker::control::PositionController controller{makePositionControllerConfig()};
    ret = controller.validate();
    if (ret < 0) {
        LOG_ERR("controller config invalid: %d", ret);
        return ret;
    }
    runtime_diagnostic = 11;

    ret = dji_bus.init(descriptor.can);
    if (ret < 0) {
        LOG_ERR("Bus init failed: %d", ret);
        return ret;
    }
    runtime_diagnostic = 12;
    ret = dji_bus.attach(motor);
    if (ret < 0) {
        LOG_ERR("Bus attach failed: %d", ret);
        return ret;
    }
    runtime_diagnostic = 13;
    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("no fresh feedback before arm: %d", ret);
        return ret;
    }
    runtime_diagnostic = 2;

    LOG_INF("feedback ready: arming now; keep the GM6020 output "
            "suspended and hold a power cut");
    if (kPositionTargetMode == PositionTargetMode::FixedZeroAbsolute) {
        LOG_WRN("absolute mode: verify encoder-zero-ticks before arming");
    }

    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("feedback lost before arm: %d", ret);
        return ret;
    }

    skywalker::motor::Feedback first_feedback{};
    const std::uint64_t reset_time_ms = static_cast<std::uint64_t>(k_uptime_get());
    ret = readFreshPositionFeedback(motor, reset_time_ms, first_feedback);
    if (ret < 0) {
        LOG_ERR("initial feedback invalid: %d", ret);
        return ret;
    }
    ret = controller.reset(first_feedback.position_rad, first_feedback.velocity_rad_s);
    if (ret < 0) {
        LOG_ERR("controller reset failed: %d", ret);
        return ret;
    }

    const float initial_position_rad = first_feedback.position_rad;
    const float final_target_rad = initial_position_rad + kTargetOffsetRad;
    skywalker::motor::dji::FlushReport arm_report{};
    ret = dji_bus.arm(arm_report);
    if (ret < 0 || !arm_report.zero_sent) {
        LOG_ERR("arm/zero failed: ret=%d zero=%d zero_err=%d", ret, arm_report.zero_sent ? 1 : 0,
                arm_report.zero_tx_error);
        return ret < 0 ? ret : -EIO;
    }
    runtime_diagnostic = 3;

    const std::int64_t run_start_ms = k_uptime_get();
    std::int64_t previous_cycle_ms = run_start_ms;
    std::uint32_t telemetry_divider = 0;

    for (;;) {
        runtime_diagnostic = 100;
        k_sleep(K_MSEC(kControlPeriodMs));

        const std::int64_t now_signed_ms = k_uptime_get();
        if (now_signed_ms <= previous_cycle_ms) {
            return stopAfterFailure(-ERANGE);
        }
        const float dt_s = static_cast<float>(now_signed_ms - previous_cycle_ms) / 1000.0f;
        previous_cycle_ms = now_signed_ms;
        const std::uint64_t now_ms = static_cast<std::uint64_t>(now_signed_ms);

        skywalker::motor::Feedback feedback{};
        ret = readFreshPositionFeedback(motor, now_ms, feedback);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }
        // if (std::fabs(feedback.velocity_rad_s) >
        //     kMeasuredVelocitySafetyMaxRadS) {
        //     return stopAfterFailure(-ERANGE);
        // }
        (void)kMeasuredVelocitySafetyMaxRadS;

        const std::int64_t elapsed_ms = now_signed_ms - run_start_ms;
        float requested_target_rad = 0.0f;
        float continuous_target_rad = 0.0f;
        if (kPositionTargetMode == PositionTargetMode::ContinuousRelative) {
            requested_target_rad = requestedRelativePositionRad(elapsed_ms, initial_position_rad, final_target_rad);
            continuous_target_rad = requested_target_rad;
        } else {
            requested_target_rad = requestedAbsolutePositionRad(elapsed_ms);
            ret = control_angle_nearest_continuous_target(requested_target_rad, feedback.absolute_position_rad,
                                                          feedback.position_rad, &continuous_target_rad);
            if (ret < 0) {
                return stopAfterFailure(ret);
            }
        }

        skywalker::control::PositionController::Output output{};
        ret = controller.step(continuous_target_rad, feedback.position_rad, feedback.velocity_rad_s, dt_s, output);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        ret = skywalker::motor::setCurrent(motor, output.current_command_a);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        skywalker::motor::dji::FlushReport flush_report{};
        ret = dji_bus.flush(flush_report);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        if (++telemetry_divider >= kTelemetryPeriodCycles) {
            telemetry_divider = 0U;
            /* JustFloat: requested target, continuous target/position,
             * absolute position (zero in relative mode), position error,
             * position output, velocity reference/raw/error/P/I, current,
             * dt ms, age ms.
             */
            const float absolute_position_rad = kPositionTargetMode == PositionTargetMode::FixedZeroAbsolute
                                                    ? feedback.absolute_position_rad
                                                    : 0.0f;
            const float channels[14] = {
                requested_target_rad,
                continuous_target_rad,
                feedback.position_rad,
                absolute_position_rad,
                output.position.error,
                output.position.output,
                output.velocity.velocity_reference_rad_s,
                feedback.velocity_rad_s,
                output.velocity.velocity_error_rad_s,
                output.velocity.regulator.feedback.p,
                output.velocity.regulator.feedback.i,
                output.current_command_a,
                dt_s * 1000.0f,
                static_cast<float>(now_ms - feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 14);
        }
    }
}
