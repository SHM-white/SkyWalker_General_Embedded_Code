#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/console/console.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/feedforward_pid.h>
#include <control/pid.h>
#include <control/slew_rate_limiter.h>
#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>

LOG_MODULE_REGISTER(dji_position_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

namespace {

constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::int64_t kRunDurationMs = 8000;
constexpr float kTargetOffsetRad = 0.25f;
constexpr float kVelocityAbsMaxRadS = 1.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.30f;

struct PositionController {
    control_pid_config position_config{};
    control_pid_state position_state{};
    control_slew_rate_config velocity_reference_config{};
    control_slew_rate_state velocity_reference_state{};
    control_feedforward_pid_config velocity_config{};
    control_feedforward_pid_state velocity_state{};
};

struct PositionControlOutput {
    control_pid_result position{};
    float velocity_reference_rad_s = 0.0f;
    float acceleration_reference_rad_s2 = 0.0f;
    control_feedforward_pid_result velocity{};
    float current_command_a = 0.0f;
};

skywalker::motor::dji::Bus dji_bus;

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

PositionController makePositionController()
{
    PositionController controller{};

    /* Position error (rad) -> velocity request (rad/s). */
    controller.position_config = {
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = 0.0f,
        .integral_max = 0.0f,
        .output_min = -kVelocityAbsMaxRadS,
        .output_max = kVelocityAbsMaxRadS,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    controller.velocity_reference_config = {
        .rising_rate_per_s = 0.5f,
        .falling_rate_per_s = 0.5f,
    };

    /* Velocity error (rad/s) -> current command (A). */
    controller.velocity_config.feedback = {
        .kp = 0.05f,
        .ki = 0.0f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = 0.0f,
        .integral_max = 0.0f,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };
    controller.velocity_config.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
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
    return control_feedforward_pid_validate(
        &controller.velocity_config);
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

int waitForManualArmToken()
{
    int ret = console_init();
    if (ret < 0) {
        return ret;
    }

    printk("Suspend the motor and prepare a physical power cut.\n");
    printk("Press 'a' to move +250 mrad from the current output position.\n");
    for (;;) {
        const int ch = console_getchar();
        if (ch < 0) {
            return ch;
        }
        if (ch == 'a' || ch == 'A') {
            return 0;
        }
    }
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
    return control_feedforward_pid_reset(
        &controller.velocity_state,
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
    control_feedforward_pid_state next_velocity_state =
        controller.velocity_state;
    PositionControlOutput next_output{};

    const control_pid_input position_input = {
        .setpoint = position_target_rad,
        .measurement = feedback.position_rad,
        .dt_s = dt_s,
        .freeze_integrator = false,
    };
    int ret = control_pid_step(&next_position_state,
                               &controller.position_config,
                               &position_input,
                               &next_output.position);
    if (ret < 0) {
        return ret;
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

    const control_feedforward_pid_input velocity_input = {
        .feedback = {
            .setpoint = next_output.velocity_reference_rad_s,
            .measurement = feedback.velocity_rad_s,
            .dt_s = dt_s,
            .freeze_integrator = false,
        },
        .reference = {
            .position_ref_rad = position_target_rad,
            .velocity_ref = next_output.velocity_reference_rad_s,
            .acceleration_ref =
                next_output.acceleration_reference_rad_s2,
        },
    };
    ret = control_feedforward_pid_step(
        &next_velocity_state,
        &controller.velocity_config,
        &velocity_input,
        &next_output.velocity);
    if (ret < 0) {
        return ret;
    }

    next_output.current_command_a = clampFloat(
        next_output.velocity.output,
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
    skywalker::motor::dji::FlushReport report{};
    const int stop_ret = dji_bus.stop(report);
    LOG_ERR("control failed: cause=%d stop=%d zero=%d zero_err=%d",
            original_error,
            stop_ret,
            report.zero_sent ? 1 : 0,
            report.zero_tx_error);
    return stop_ret < 0 ? stop_ret : original_error;
}

} // namespace

int main()
{
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    if (!device_is_ready(motor)) {
        LOG_ERR("motor device not ready");
        return -ENODEV;
    }

    skywalker::motor::dji::Descriptor descriptor{};
    int ret = skywalker::motor::dji::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr ||
        !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }

    PositionController controller = makePositionController();
    ret = validateController(controller);
    if (ret < 0) {
        LOG_ERR("controller config invalid: %d", ret);
        return ret;
    }

    ret = dji_bus.init(descriptor.can);
    if (ret < 0) {
        LOG_ERR("Bus init failed: %d", ret);
        return ret;
    }
    ret = dji_bus.attach(motor);
    if (ret < 0) {
        LOG_ERR("Bus attach failed: %d", ret);
        return ret;
    }
    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("no fresh feedback before arm: %d", ret);
        return ret;
    }
    ret = waitForManualArmToken();
    if (ret < 0) {
        LOG_ERR("manual arm input failed: %d", ret);
        return ret;
    }
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

    const std::int64_t run_start_ms = k_uptime_get();
    std::int64_t previous_cycle_ms = run_start_ms;
    std::uint32_t telemetry_divider = 0;

    while (k_uptime_get() - run_start_ms < kRunDurationMs) {
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

        const float target_rad =
            now_signed_ms - run_start_ms < 500
                ? initial_position_rad
                : final_target_rad;

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

        if (++telemetry_divider >= 40U) {
            telemetry_divider = 0U;
            printk("target=%d pos=%d pos_err=%d vel_ref=%d "
                   "vel=%d current=%d mA sat=%d age=%llu ms\n",
                   static_cast<int>(target_rad * 1000.0f),
                   static_cast<int>(feedback.position_rad * 1000.0f),
                   static_cast<int>(output.position.error * 1000.0f),
                   static_cast<int>(
                       output.velocity_reference_rad_s * 1000.0f),
                   static_cast<int>(feedback.velocity_rad_s * 1000.0f),
                   static_cast<int>(output.current_command_a * 1000.0f),
                   output.velocity.feedback.saturated ? 1 : 0,
                   static_cast<unsigned long long>(
                       now_ms - feedback.timestamp_ms));
        }
    }

    ret = skywalker::motor::setCurrent(motor, 0.0f);
    if (ret < 0) {
        return stopAfterFailure(ret);
    }
    skywalker::motor::dji::FlushReport final_flush_report{};
    ret = dji_bus.flush(final_flush_report);
    if (ret < 0) {
        return stopAfterFailure(ret);
    }

    skywalker::motor::dji::FlushReport stop_report{};
    ret = dji_bus.stop(stop_report);
    if (ret < 0 || !stop_report.zero_sent) {
        LOG_ERR("normal stop failed: ret=%d zero=%d zero_err=%d",
                ret,
                stop_report.zero_sent ? 1 : 0,
                stop_report.zero_tx_error);
        return ret < 0 ? ret : -EIO;
    }

    LOG_INF("timed position-control test completed and motor stopped");
    return 0;
}
