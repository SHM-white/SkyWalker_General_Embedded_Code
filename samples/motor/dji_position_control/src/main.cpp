#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/pid.h>
#include <control/slew_rate_limiter.h>
#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_position_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

namespace {

constexpr std::int64_t kControlPeriodMs = 1;
/* One telemetry frame per five control cycles: 200 Hz. */
constexpr std::uint32_t kTelemetryPeriodCycles = 5U;
constexpr float kTargetOffsetRad = 3.0f;

constexpr float kPositionKp = 3.0f;
constexpr float kPositionKi = 0.0f;
constexpr float kPositionIntegralMaxRadS = 0.0f;
constexpr float kPositionDeadbandRad = 0.012f;

constexpr float kVelocityAbsMaxRadS = 0.30f;
constexpr float kMeasuredVelocitySafetyMaxRadS = 1.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.10f;

constexpr float kInnerKp = 0.02f;
constexpr float kInnerKi = 0.0f;
constexpr float kInnerIntegralMaxA = 0.0f;
constexpr float kRunningFrictionCurrentA = 0.005f;
constexpr float kRunningFrictionBlendVelocityRadS = 0.10f;
constexpr float kBreakawayCurrentA = 0.08f;
constexpr float kBreakawayRequestFullRadS = 0.05f;
constexpr float kBreakawayFadeVelocityRadS = 0.20f;
constexpr float kOverspeedMarginRadS = 0.20f;
constexpr float kOverspeedDampingAperRadS = 0.15f;

constexpr float kVelocityRampRateRisingRadS2 = 4.0f;
constexpr float kVelocityRampRateFallingRadS2 = 4.0f;

struct PositionController {
    control_pid_config position_config{};
    control_pid_state position_state{};
    control_slew_rate_config velocity_reference_config{};
    control_slew_rate_state velocity_reference_state{};
    control_pid_config velocity_config{};
    control_pid_state velocity_state{};
};

struct PositionControlOutput {
    control_pid_result position{};
    float velocity_reference_rad_s = 0.0f;
    float acceleration_reference_rad_s2 = 0.0f;
    control_pid_result velocity{};
    float friction_feedforward_a = 0.0f;
    float overspeed_damping_a = 0.0f;
    float current_command_a = 0.0f;
};

skywalker::motor::dji::Bus dji_bus;
volatile std::int32_t runtime_diagnostic = 0;

float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

float calculateFrictionFeedforward(float velocity_reference_rad_s,
                                   float velocity_rad_s)
{
    const float running_blend = clampFloat(
        velocity_reference_rad_s /
            kRunningFrictionBlendVelocityRadS,
        -1.0f,
        1.0f);
    const float direction = velocity_reference_rad_s > 0.0f ? 1.0f :
                            velocity_reference_rad_s < 0.0f ? -1.0f :
                            0.0f;
    const float request_blend = clampFloat(
        std::fabs(velocity_reference_rad_s) /
            kBreakawayRequestFullRadS,
        0.0f,
        1.0f);
    const float stall_blend = clampFloat(
        1.0f - std::fabs(velocity_rad_s) /
                   kBreakawayFadeVelocityRadS,
        0.0f,
        1.0f);

    return kRunningFrictionCurrentA * running_blend +
           kBreakawayCurrentA * direction *
               request_blend * stall_blend;
}

float calculateOverspeedDamping(float velocity_reference_rad_s,
                                float velocity_rad_s)
{
    const float allowed_magnitude =
        std::fabs(velocity_reference_rad_s) +
        kOverspeedMarginRadS;
    const float excess =
        std::fabs(velocity_rad_s) - allowed_magnitude;
    if (excess <= 0.0f) {
        return 0.0f;
    }
    return -std::copysign(
        kOverspeedDampingAperRadS * excess,
        velocity_rad_s);
}

PositionController makePositionController()
{
    PositionController controller{};

    /* Position error (rad) -> velocity request (rad/s). */
    controller.position_config = {
        .kp = kPositionKp,
        .ki = kPositionKi,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = -kPositionIntegralMaxRadS,
        .integral_max = kPositionIntegralMaxRadS,
        .output_min = -kVelocityAbsMaxRadS,
        .output_max = kVelocityAbsMaxRadS,
        .deadband = kPositionDeadbandRad,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    controller.velocity_reference_config = {
        .rising_rate_per_s = kVelocityRampRateRisingRadS2,
        .falling_rate_per_s = kVelocityRampRateFallingRadS2,
    };

    /* Velocity error (rad/s) -> current correction (A). */
    controller.velocity_config = {
        .kp = kInnerKp,
        .ki = kInnerKi,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = -kInnerIntegralMaxA,
        .integral_max = kInnerIntegralMaxA,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    return controller;
}

int validateController(const PositionController &controller)
{
    int ret = control_pid_validate(&controller.position_config);
    if (ret < 0) {
        return ret;
    }
    ret = control_slew_rate_validate(
        &controller.velocity_reference_config);
    if (ret < 0) {
        return ret;
    }
    return control_pid_validate(&controller.velocity_config);
}

int waitForFreshFeedback(const struct device *motor)
{
    const std::int64_t deadline_ms = k_uptime_get() + 2000;

    while (skywalker::motor::getState(motor) !=
           skywalker::motor::State::Ready) {
        if (k_uptime_get() >= deadline_ms) {
            return -ETIMEDOUT;
        }
        k_sleep(K_MSEC(5));
    }
    return 0;
}

int readFreshPositionFeedback(
    const struct device *motor,
    std::uint64_t now_ms,
    skywalker::motor::Feedback &feedback)
{
    int ret = skywalker::motor::readFeedback(motor, feedback);
    if (ret < 0) {
        return ret;
    }
    if (skywalker::motor::getState(motor) !=
        skywalker::motor::State::Ready) {
        return -EHOSTDOWN;
    }

    constexpr std::uint32_t required =
        skywalker::motor::FeedbackPosition |
        skywalker::motor::FeedbackVelocity;
    if ((feedback.valid & required) != required) {
        return -ENODATA;
    }
    if (!std::isfinite(feedback.position_rad) ||
        !std::isfinite(feedback.velocity_rad_s)) {
        return -EINVAL;
    }
    if (feedback.timestamp_ms == 0U ||
        now_ms < feedback.timestamp_ms ||
        now_ms - feedback.timestamp_ms >
            CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS) {
        return -ESTALE;
    }
    return 0;
}

int resetController(PositionController &controller,
                    const skywalker::motor::Feedback &feedback)
{
    int ret = control_pid_reset(&controller.position_state,
                                feedback.position_rad);
    if (ret < 0) {
        return ret;
    }
    ret = control_slew_rate_reset(
        &controller.velocity_reference_state,
        0.0f);
    if (ret < 0) {
        return ret;
    }
    return control_pid_reset(&controller.velocity_state,
                             feedback.velocity_rad_s);
}

int calculatePositionCurrent(
    PositionController &controller,
    const skywalker::motor::Feedback &feedback,
    float position_target_rad,
    float dt_s,
    PositionControlOutput &output)
{
    if (!std::isfinite(position_target_rad) ||
        !std::isfinite(dt_s)) {
        return -EINVAL;
    }

    control_pid_state next_position_state =
        controller.position_state;
    control_slew_rate_state next_reference_state =
        controller.velocity_reference_state;
    control_pid_state next_velocity_state =
        controller.velocity_state;
    PositionControlOutput next_output{};

    const float position_error =
        position_target_rad - feedback.position_rad;
    const bool inside_deadband =
        std::fabs(position_error) <= kPositionDeadbandRad;

    const control_pid_input position_input = {
        .setpoint = position_target_rad,
        .measurement = feedback.position_rad,
        .dt_s = dt_s,
        .freeze_integrator = inside_deadband,
    };
    int ret = control_pid_step(&next_position_state,
                               &controller.position_config,
                               &position_input,
                               &next_output.position);
    if (ret < 0) {
        return ret;
    }
    if (inside_deadband) {
        /* Do not carry breakaway-fighting integral into the settled band. */
        next_position_state.integral_output = 0.0f;
    }

    ret = control_slew_rate_step(
        &next_reference_state,
        &controller.velocity_reference_config,
        next_output.position.output,
        dt_s,
        &next_output.velocity_reference_rad_s,
        &next_output.acceleration_reference_rad_s2);
    if (ret < 0) {
        return ret;
    }

    const control_pid_input velocity_input = {
        .setpoint = next_output.velocity_reference_rad_s,
        .measurement = feedback.velocity_rad_s,
        .dt_s = dt_s,
        .freeze_integrator = false,
    };
    ret = control_pid_step(&next_velocity_state,
                           &controller.velocity_config,
                           &velocity_input,
                           &next_output.velocity);
    if (ret < 0) {
        return ret;
    }

    next_output.friction_feedforward_a =
        calculateFrictionFeedforward(
            next_output.velocity_reference_rad_s,
            feedback.velocity_rad_s);
    next_output.overspeed_damping_a =
        calculateOverspeedDamping(
            next_output.velocity_reference_rad_s,
            feedback.velocity_rad_s);
    next_output.current_command_a = clampFloat(
        next_output.velocity.output +
            next_output.friction_feedforward_a +
            next_output.overspeed_damping_a,
        -kSoftwareCurrentAbsMaxA,
        kSoftwareCurrentAbsMaxA);
    if (!std::isfinite(next_output.current_command_a)) {
        return -ERANGE;
    }

    controller.position_state = next_position_state;
    controller.velocity_reference_state = next_reference_state;
    controller.velocity_state = next_velocity_state;
    output = next_output;
    return 0;
}

int stopAfterFailure(int original_error)
{
    runtime_diagnostic = original_error;
    skywalker::motor::dji::FlushReport report{};
    const int stop_ret = dji_bus.stop(report);
    LOG_ERR("control failed: cause=%d stop=%d zero=%d zero_err=%d",
            original_error,
            stop_ret,
            report.zero_sent ? 1 : 0,
            report.zero_tx_error);
    return stop_ret < 0 ? stop_ret : original_error;
}

/*
 * Position trajectory (rad). Returns where the output shaft should be at
 * elapsed_ms. Edit this function to change the motion profile without
 * touching the control loop: 0.5 s settle at the start position, then a
 * step to the final target held forever.
 */
float requestedPositionRad(std::int64_t elapsed_ms,
                           float initial_position_rad,
                           float final_target_rad)
{
    if (elapsed_ms < 5000) {
        return initial_position_rad;
    }
    return final_target_rad;
}

} // namespace

int main()
{
    runtime_diagnostic = 1;
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    const struct device *vofa_uart = DEVICE_DT_GET(DT_NODELABEL(usart6));
    if (!device_is_ready(motor)) {
        LOG_ERR("motor device not ready");
        return -ENODEV;
    }
    if (!device_is_ready(vofa_uart)) {
        LOG_ERR("VOFA UART device not ready");
        return -ENODEV;
    }

    Vofa vofa{};
    vofa_init(&vofa, vofa_uart);

    skywalker::motor::dji::Descriptor descriptor{};
    int ret = skywalker::motor::dji::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr ||
        !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }
    runtime_diagnostic = 10;

    PositionController controller = makePositionController();
    ret = validateController(controller);
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

    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("feedback lost before arm: %d", ret);
        return ret;
    }

    skywalker::motor::Feedback first_feedback{};
    const std::uint64_t reset_time_ms =
        static_cast<std::uint64_t>(k_uptime_get());
    ret = readFreshPositionFeedback(motor,
                                    reset_time_ms,
                                    first_feedback);
    if (ret < 0) {
        LOG_ERR("initial feedback invalid: %d", ret);
        return ret;
    }
    ret = resetController(controller, first_feedback);
    if (ret < 0) {
        LOG_ERR("controller reset failed: %d", ret);
        return ret;
    }

    /* Feedback position is continuous; this sample commands a relative move. */
    const float initial_position_rad = first_feedback.position_rad;
    const float final_target_rad =
        initial_position_rad + kTargetOffsetRad;

    skywalker::motor::dji::FlushReport arm_report{};
    ret = dji_bus.arm(arm_report);
    if (ret < 0 || !arm_report.zero_sent) {
        LOG_ERR("arm/zero failed: ret=%d zero=%d zero_err=%d",
                ret,
                arm_report.zero_sent ? 1 : 0,
                arm_report.zero_tx_error);
        return ret < 0 ? ret : -EIO;
    }
    runtime_diagnostic = 3;

    const std::int64_t run_start_ms = k_uptime_get();
    std::int64_t previous_cycle_ms = run_start_ms;
    std::uint32_t telemetry_divider = 0;

    /* Run forever; pull power or reset to stop. */
    for (;;) {
        runtime_diagnostic = 100;
        k_sleep(K_MSEC(kControlPeriodMs));

        const std::int64_t now_signed_ms = k_uptime_get();
        if (now_signed_ms <= previous_cycle_ms) {
            return stopAfterFailure(-ERANGE);
        }
        const float dt_s = static_cast<float>(
            now_signed_ms - previous_cycle_ms) / 1000.0f;
        previous_cycle_ms = now_signed_ms;
        const std::uint64_t now_ms =
            static_cast<std::uint64_t>(now_signed_ms);

        skywalker::motor::Feedback feedback{};
        ret = readFreshPositionFeedback(motor, now_ms, feedback);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }
        if (std::fabs(feedback.velocity_rad_s) >
            kMeasuredVelocitySafetyMaxRadS) {
            return stopAfterFailure(-ERANGE);
        }

        const float target_rad = requestedPositionRad(
            now_signed_ms - run_start_ms,
            initial_position_rad,
            final_target_rad);

        PositionControlOutput output{};
        ret = calculatePositionCurrent(controller,
                                       feedback,
                                       target_rad,
                                       dt_s,
                                       output);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        ret = skywalker::motor::setCurrent(
            motor,
            output.current_command_a);
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
            /* JustFloat channels, grouped for tuning:
             * position target/measurement/error,
             * velocity reference/measurement/P/I,
             * friction/overspeed compensation, current and feedback age.
             */
            const float channels[11] = {
                target_rad,
                feedback.position_rad,
                output.position.error,
                output.velocity_reference_rad_s,
                feedback.velocity_rad_s,
                output.velocity.p,
                output.velocity.i,
                output.friction_feedforward_a,
                output.overspeed_damping_a,
                output.current_command_a,
                static_cast<float>(now_ms - feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 11);
        }
    }

    /* Unreachable: the loop above never exits. */
    return 0;
}
