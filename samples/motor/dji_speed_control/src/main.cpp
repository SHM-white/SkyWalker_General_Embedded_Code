#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/console/console.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/feedforward_pid.h>
#include <control/slew_rate_limiter.h>
#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>

LOG_MODULE_REGISTER(dji_speed_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

namespace {

constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::int64_t kRunDurationMs = 6000;
constexpr float kRequestedVelocityRadS = 1.0f;
constexpr float kRequestedVelocityAbsMaxRadS = 2.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.30f;

struct VelocityController {
    control_feedforward_pid_config config{};
    control_slew_rate_config reference_config{};
    control_feedforward_pid_state controller_state{};
    control_slew_rate_state reference_state{};
};

struct VelocityControlOutput {
    float velocity_reference_rad_s = 0.0f;
    float acceleration_reference_rad_s2 = 0.0f;
    float current_command_a = 0.0f;
    control_feedforward_pid_result controller{};
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

VelocityController makeVelocityController()
{
    VelocityController loop{};

    loop.config.feedback = {
        /* Small P-only value for a suspended, low-current direction check. */
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

    /* Zero feedforward makes this first pass a classic feedback controller. */
    loop.config.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };

    loop.reference_config = {
        .rising_rate_per_s = 0.5f,
        .falling_rate_per_s = 0.5f,
    };
    return loop;
}

int validateController(const VelocityController &loop)
{
    int ret = control_feedforward_pid_validate(&loop.config);
    if (ret < 0) {
        return ret;
    }
    return control_slew_rate_validate(&loop.reference_config);
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

int readFreshVelocityFeedback(
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
    if ((feedback.valid & skywalker::motor::FeedbackVelocity) == 0U) {
        return -ENODATA;
    }
    if (!std::isfinite(feedback.velocity_rad_s)) {
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
    printk("Press 'a' to arm the six-second speed-control test.\n");
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

int resetController(VelocityController &loop,
                    float first_velocity_rad_s)
{
    int ret = control_slew_rate_reset(&loop.reference_state, 0.0f);
    if (ret < 0) {
        return ret;
    }
    return control_feedforward_pid_reset(
        &loop.controller_state,
        first_velocity_rad_s);
}

int calculateVelocityCurrent(
    VelocityController &loop,
    const skywalker::motor::Feedback &feedback,
    float requested_velocity_rad_s,
    float dt_s,
    VelocityControlOutput &output)
{
    if (!std::isfinite(requested_velocity_rad_s) ||
        !std::isfinite(dt_s)) {
        return -EINVAL;
    }
    if (std::fabs(requested_velocity_rad_s) >
        kRequestedVelocityAbsMaxRadS) {
        return -ERANGE;
    }

    control_slew_rate_state next_reference_state =
        loop.reference_state;
    control_feedforward_pid_state next_controller_state =
        loop.controller_state;
    VelocityControlOutput next_output{};

    int ret = control_slew_rate_step(
        &next_reference_state,
        &loop.reference_config,
        requested_velocity_rad_s,
        dt_s,
        &next_output.velocity_reference_rad_s,
        &next_output.acceleration_reference_rad_s2);
    if (ret < 0) {
        return ret;
    }

    const control_feedforward_pid_input input = {
        .feedback = {
            .setpoint = next_output.velocity_reference_rad_s,
            .measurement = feedback.velocity_rad_s,
            .dt_s = dt_s,
            .freeze_integrator = false,
        },
        .reference = {
            .position_ref_rad = 0.0f,
            .velocity_ref = next_output.velocity_reference_rad_s,
            .acceleration_ref =
                next_output.acceleration_reference_rad_s2,
        },
    };

    ret = control_feedforward_pid_step(
        &next_controller_state,
        &loop.config,
        &input,
        &next_output.controller);
    if (ret < 0) {
        return ret;
    }

    next_output.current_command_a = clampFloat(
        next_output.controller.output,
        -kSoftwareCurrentAbsMaxA,
        kSoftwareCurrentAbsMaxA);
    if (!std::isfinite(next_output.current_command_a)) {
        return -ERANGE;
    }

    loop.reference_state = next_reference_state;
    loop.controller_state = next_controller_state;
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

float requestedVelocityForTime(std::int64_t elapsed_ms)
{
    if (elapsed_ms < 500) {
        return 0.0f;
    }
    if (elapsed_ms < 3000) {
        return kRequestedVelocityRadS;
    }
    return 0.0f;
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

    VelocityController loop = makeVelocityController();
    ret = validateController(loop);
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
    ret = readFreshVelocityFeedback(motor,
                                    reset_time_ms,
                                    first_feedback);
    if (ret < 0) {
        LOG_ERR("initial feedback invalid: %d", ret);
        return ret;
    }
    ret = resetController(loop, first_feedback.velocity_rad_s);
    if (ret < 0) {
        LOG_ERR("controller reset failed: %d", ret);
        return ret;
    }

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
        ret = readFreshVelocityFeedback(motor, now_ms, feedback);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        const float request =
            requestedVelocityForTime(now_signed_ms - run_start_ms);
        VelocityControlOutput output{};
        ret = calculateVelocityCurrent(loop,
                                       feedback,
                                       request,
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
            printk("req=%d ref=%d vel=%d err=%d p=%d i=%d "
                   "ff=%d out=%d sat=%d age=%llu ms\n",
                   static_cast<int>(request * 1000.0f),
                   static_cast<int>(
                       output.velocity_reference_rad_s * 1000.0f),
                   static_cast<int>(feedback.velocity_rad_s * 1000.0f),
                   static_cast<int>(
                       output.controller.feedback.error * 1000.0f),
                   static_cast<int>(
                       output.controller.feedback.p * 1000.0f),
                   static_cast<int>(
                       output.controller.feedback.i * 1000.0f),
                   static_cast<int>(output.controller.feedforward * 1000.0f),
                   static_cast<int>(output.current_command_a * 1000.0f),
                   output.controller.feedback.saturated ? 1 : 0,
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

    LOG_INF("timed speed-control test completed and motor stopped");
    return 0;
}
