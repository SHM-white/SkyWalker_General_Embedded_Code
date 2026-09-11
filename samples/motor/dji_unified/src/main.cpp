#include <errno.h>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_unified, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

static skywalker::motor::dji::Bus dji_bus;

namespace {

constexpr float kCurrentCommandA = -0.05f;
constexpr std::int16_t kSpeedCutoffRpm = 330;
constexpr float kTemperatureCutoffC = 70.0f;
constexpr std::int64_t kControlPeriodMs = 5;
/* One telemetry frame per control cycle: 200 Hz. */
constexpr std::uint32_t kTelemetryPeriodCycles = 1U;

static int waitForFreshFeedback(const struct device *motor) {
    std::int64_t next_log_ms = 0;

    while (skywalker::motor::getState(motor) != skywalker::motor::State::Ready) {
        const std::int64_t now_ms = k_uptime_get();
        if (now_ms >= next_log_ms) {
            LOG_WRN("waiting for GM6020 ID 7 feedback on CAN1 (0x20B)");
            next_log_ms = now_ms + 1000;
        }
        k_sleep(K_MSEC(5));
    }
    return 0;
}

static int stopAfterFailure(int original_error) {
    skywalker::motor::dji::FlushReport stop_report{};
    const int stop_ret = dji_bus.stop(stop_report);
    LOG_ERR("stopped: cause=%d stop=%d zero=%d zero_err=%d", original_error, stop_ret, stop_report.zero_sent ? 1 : 0, stop_report.zero_tx_error);
    return stop_ret < 0 ? stop_ret : original_error;
}

} // namespace

int main() {
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    const struct device *vofa_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));
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
    if (ret < 0 || descriptor.can == nullptr || !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }
    LOG_INF("GM6020 ID=%u feedback=0x%03x command=0x%03x slot=%u", descriptor.motor_id, descriptor.feedback_id, descriptor.command_id, descriptor.command_slot);

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
        LOG_ERR("No fresh feedback: %d", ret);
        return ret;
    }

    skywalker::motor::dji::FlushReport arm_report{};
    ret = dji_bus.arm(arm_report);
    if (ret < 0 || !arm_report.zero_sent) {
        LOG_ERR("Arm/zero failed: ret=%d zero=%d", ret, arm_report.zero_sent ? 1 : 0);
        return ret < 0 ? ret : -EIO;
    }

    LOG_INF("continuous open-loop current test started: %d mA", static_cast<int>(kCurrentCommandA * 1000.0f));

    std::uint32_t print_divider = 0;
    for (;;) {
        skywalker::motor::Feedback feedback{};
        skywalker::motor::dji::RawFeedback raw{};
        ret = skywalker::motor::readFeedback(motor, feedback);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }
        ret = skywalker::motor::dji::readRawFeedback(motor, raw);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }
        if (skywalker::motor::getState(motor) != skywalker::motor::State::Ready) {
            return stopAfterFailure(-EHOSTDOWN);
        }
        if ((feedback.valid & skywalker::motor::FeedbackTemperature) != 0u && feedback.temperature_c >= kTemperatureCutoffC) {
            LOG_ERR("temperature cutoff: %d C", static_cast<int>(feedback.temperature_c));
            return stopAfterFailure(-EOVERFLOW);
        }

        const std::int32_t speed_abs_rpm = raw.speed_rpm < 0 ? -static_cast<std::int32_t>(raw.speed_rpm) : static_cast<std::int32_t>(raw.speed_rpm);
        if (speed_abs_rpm > kSpeedCutoffRpm) {
            LOG_ERR("speed cutoff: %d rpm", raw.speed_rpm);
            return stopAfterFailure(-ERANGE);
        }

        ret = skywalker::motor::setCurrent(motor, kCurrentCommandA);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        skywalker::motor::dji::FlushReport report{};
        ret = dji_bus.flush(report);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        if (++print_divider >= kTelemetryPeriodCycles) {
            print_divider = 0;
            /* JustFloat channels: state, command_a, encoder, speed_rpm,
             * current_raw, temperature_c, timestamp_ms.
             */
            const float channels[7] = {
                static_cast<float>(skywalker::motor::getState(motor)),
                kCurrentCommandA,
                static_cast<float>(raw.encoder),
                static_cast<float>(raw.speed_rpm),
                static_cast<float>(raw.current_raw),
                feedback.temperature_c,
                static_cast<float>(feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 7);
        }

        k_sleep(K_MSEC(kControlPeriodMs));
    }
}
