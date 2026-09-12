#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/velocity_controller.hpp>
#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_speed_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)

#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {

constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::uint32_t kTelemetryPeriodCycles = 1U;
constexpr std::int64_t kRunDurationMs = 300000;

constexpr float kRequestedVelocityRadS = 20.0f;
constexpr float kRequestedVelocityAbsMaxRadS = 50.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.8f;
constexpr float kDeadbandRadS = 0.20f;
constexpr float kVelocityFilterTauS = 0.025f;

skywalker::motor::dji::Bus dji_bus;

skywalker::control::VelocityController::Config makeVelocityControllerConfig() {
    skywalker::control::VelocityController::Config config{};
    config.regulator.feedback = {
        .kp = 0.02f,
        .ki = 0.05f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = -0.1f,
        .integral_max = 0.1f,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };
    config.regulator.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.005f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };
    config.reference_slew = {
        .rising_rate_per_s = 100.0f,
        .falling_rate_per_s = 100.0f,
    };
    config.measurement_filter_tau_s = kVelocityFilterTauS;
    config.soft_deadband_rad_s = kDeadbandRadS;
    config.requested_velocity_abs_max_rad_s = kRequestedVelocityAbsMaxRadS;
    config.effort_abs_max = kSoftwareCurrentAbsMaxA;
    return config;
}

int requireMotorCapabilities(const struct device *motor) {
    constexpr std::uint32_t required = skywalker::motor::CommandCurrent | skywalker::motor::FeedbackVelocity;
    const std::uint32_t available = skywalker::motor::capabilities(motor);
    return (available & required) == required ? 0 : -ENOTSUP;
}

int waitForFreshFeedback(const struct device *motor) {
    const std::int64_t deadline_ms = k_uptime_get() + 2000;
    std::int64_t next_log_ms = k_uptime_get();

    while (skywalker::motor::getState(motor) != skywalker::motor::State::Ready) {
        const std::int64_t now_ms = k_uptime_get();
        if (now_ms >= next_log_ms) {
            LOG_WRN("still waiting for fresh motor feedback "
                    "(check power, CAN wiring and motor ID)");
            next_log_ms = now_ms + 1000;
        }
        if (now_ms >= deadline_ms) {
            return -ETIMEDOUT;
        }
        k_sleep(K_MSEC(5));
    }
    return 0;
}

int readFreshVelocityFeedback(const struct device *motor, std::uint64_t now_ms, skywalker::motor::Feedback &feedback) {
    int ret = skywalker::motor::readFeedback(motor, feedback);
    if (ret < 0) {
        return ret;
    }
    if (skywalker::motor::getState(motor) != skywalker::motor::State::Ready) {
        return -EHOSTDOWN;
    }
    if ((feedback.valid & skywalker::motor::FeedbackVelocity) == 0U) {
        return -ENODATA;
    }
    if (!std::isfinite(feedback.velocity_rad_s)) {
        return -EINVAL;
    }
    if (feedback.timestamp_ms == 0U || now_ms < feedback.timestamp_ms || now_ms - feedback.timestamp_ms > CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS) {
        return -ESTALE;
    }
    return 0;
}

int stopAfterFailure(int original_error) {
    skywalker::motor::dji::FlushReport report{};
    const int stop_ret = dji_bus.stop(report);
    LOG_ERR("control failed: cause=%d stop=%d zero=%d zero_err=%d", original_error, stop_ret, report.zero_sent ? 1 : 0, report.zero_tx_error);
    return stop_ret < 0 ? stop_ret : original_error;
}

float requestedVelocityForTime(std::int64_t elapsed_ms) {
    if (elapsed_ms < 500) {
        return 0.0f;
    }
    if (elapsed_ms < kRunDurationMs) {
        return kRequestedVelocityRadS * std::sinf(static_cast<float>(elapsed_ms) / 1000.0f);
    }
    return 0.0f;
}

} // namespace

int main() {
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
        LOG_ERR("motor lacks current-command or velocity-feedback capability");
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
    LOG_INF("GM6020 ID=%u feedback=0x%03x command=0x%03x slot=%u "
            "gear=%.3f limit=%.0f mA",
            descriptor.motor_id, descriptor.feedback_id, descriptor.command_id, descriptor.command_slot,
            static_cast<double>(descriptor.gear_ratio),
            static_cast<double>(descriptor.configured_current_limit_a * 1000.0f));

    skywalker::control::VelocityController controller{
        makeVelocityControllerConfig()};
    ret = controller.validate();
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

    LOG_INF("feedback ready: arming now; keep the GM6020 output "
            "suspended and hold a power cut");

    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("feedback lost before arm: %d", ret);
        return ret;
    }

    skywalker::motor::Feedback first_feedback{};
    const std::uint64_t reset_time_ms = static_cast<std::uint64_t>(k_uptime_get());
    ret = readFreshVelocityFeedback(motor, reset_time_ms, first_feedback);
    if (ret < 0) {
        LOG_ERR("initial feedback invalid: %d", ret);
        return ret;
    }
    ret = controller.reset(first_feedback.velocity_rad_s);
    if (ret < 0) {
        LOG_ERR("controller reset failed: %d", ret);
        return ret;
    }

    skywalker::motor::dji::FlushReport arm_report{};
    ret = dji_bus.arm(arm_report);
    if (ret < 0 || !arm_report.zero_sent) {
        LOG_ERR("arm/zero failed: ret=%d zero=%d zero_err=%d", ret, arm_report.zero_sent ? 1 : 0, arm_report.zero_tx_error);
        return ret < 0 ? ret : -EIO;
    }

    LOG_INF("speed-control test running: target=%.0f mrad/s, "
            "software current clamp=%.0f mA, duration=%lld ms",
            static_cast<double>(kRequestedVelocityRadS * 1000.0f),
            static_cast<double>(kSoftwareCurrentAbsMaxA * 1000.0f),
            static_cast<long long>(kRunDurationMs));

    const std::int64_t run_start_ms = k_uptime_get();
    std::int64_t previous_cycle_ms = run_start_ms;
    std::uint32_t telemetry_divider = 0;

    while (k_uptime_get() - run_start_ms < kRunDurationMs) {
        k_sleep(K_MSEC(kControlPeriodMs));

        const std::int64_t now_signed_ms = k_uptime_get();
        if (now_signed_ms <= previous_cycle_ms) {
            return stopAfterFailure(-ERANGE);
        }
        const float dt_s = static_cast<float>(now_signed_ms - previous_cycle_ms) / 1000.0f;
        previous_cycle_ms = now_signed_ms;
        const std::uint64_t now_ms = static_cast<std::uint64_t>(now_signed_ms);

        skywalker::motor::Feedback feedback{};
        ret = readFreshVelocityFeedback(motor, now_ms, feedback);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        const float request = requestedVelocityForTime(now_signed_ms - run_start_ms);
        skywalker::control::VelocityController::Output output{};
        ret = controller.step(
            request, feedback.velocity_rad_s, dt_s, output);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        ret = skywalker::motor::setCurrent(motor, output.effort_command);
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
            /* JustFloat: request, reference, raw/filtered velocity, error,
             * P, I, D, feedforward, current, saturated, feedback age ms.
             */
            const float channels[12] = {
                request,
                output.velocity_reference_rad_s,
                feedback.velocity_rad_s,
                output.filtered_velocity_rad_s,
                output.velocity_error_rad_s,
                output.regulator.feedback.p,
                output.regulator.feedback.i,
                output.regulator.feedback.d,
                output.regulator.feedforward,
                output.effort_command,
                output.regulator.feedback.saturated ? 1.0f : 0.0f,
                static_cast<float>(now_ms - feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 12);
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
        LOG_ERR("normal stop failed: ret=%d zero=%d zero_err=%d", ret, stop_report.zero_sent ? 1 : 0, stop_report.zero_tx_error);
        return ret < 0 ? ret : -EIO;
    }

    LOG_INF("timed speed-control test completed and motor stopped");
    return 0;
}
