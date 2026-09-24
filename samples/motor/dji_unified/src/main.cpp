#include <errno.h>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_unified, LOG_LEVEL_INF);

namespace {

constexpr float kCurrentCommandA = -0.05f;
constexpr std::int16_t kSpeedCutoffRpm = 330;
constexpr float kTemperatureCutoffC = 70.0f;
constexpr std::int64_t kControlPeriodMs = 5;
/* One telemetry frame per control cycle: 200 Hz. */
constexpr std::uint32_t kTelemetryPeriodCycles = 1U;

int waitForReady(skywalker::motor::Motor &motor) {
    std::int64_t next_log_ms = 0;
    while (!motor.ready()) {
        const auto view = motor.snapshot();
        if (view.state == skywalker::motor::MotorState::Fault)
            return view.last_fault.error < 0 ? view.last_fault.error : -EHOSTDOWN;
        const std::int64_t now_ms = k_uptime_get();
        if (now_ms >= next_log_ms) {
            LOG_WRN("waiting for GM6020 ID 4 feedback on CAN1 (0x208); state=%u stop=%u",
                    unsigned(view.state), unsigned(view.stop.progress));
            next_log_ms = now_ms + 1000;
        }
        k_sleep(K_MSEC(5));
    }
    return 0;
}

int waitForActive(skywalker::motor::Motor &motor) {
    const std::int64_t started_ms = k_uptime_get();
    while (k_uptime_get() - started_ms < 1000) {
        if (motor.active())
            return 0;
        const auto view = motor.snapshot();
        if (view.state == skywalker::motor::MotorState::Fault)
            return view.last_fault.error < 0 ? view.last_fault.error : -EHOSTDOWN;
        k_sleep(K_MSEC(5));
    }
    (void)motor.disable();
    return -ETIMEDOUT;
}

int stopAfterFailure(skywalker::motor::Motor &motor, int original_error) {
    const int stop_ret = motor.disable();
    const auto report = motor.snapshot().stop;
    LOG_ERR("stopped: cause=%d stop_request=%d progress=%u stop_tx_error=%d",
            original_error, stop_ret, unsigned(report.progress), report.tx_error);
    return stop_ret < 0 ? stop_ret : original_error;
}

int checkSafeFeedback(const skywalker::motor::MotorSnapshot &view) {
    const auto &feedback = view.feedback;
    if (!view.feedback_fresh || !view.native_dji_feedback_valid ||
        (feedback.valid & skywalker::motor::FeedbackTemperature) == 0u)
        return -ENODATA;
    if (feedback.temperature_c >= kTemperatureCutoffC) {
        LOG_ERR("temperature cutoff: %d C", int(feedback.temperature_c));
        return -EOVERFLOW;
    }
    const auto rpm = view.native_dji_feedback.speed_rpm;
    const std::int32_t speed_abs_rpm = rpm < 0 ? -std::int32_t(rpm) : std::int32_t(rpm);
    if (speed_abs_rpm > kSpeedCutoffRpm) {
        LOG_ERR("speed cutoff: %d rpm", rpm);
        return -ERANGE;
    }
    return 0;
}

} // namespace

int main() {
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    const device *vofa_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));
    if (!device_is_ready(can) || !device_is_ready(vofa_uart)) {
        LOG_ERR("CAN or VOFA UART not ready");
        return -ENODEV;
    }

    const auto config = skywalker::motor::dji::gm6020({
        .id = 4,
        .current_limit_a = 0.1f,
        .encoder_zero_ticks = 0,
        .current_mode_confirmed = true,
        .timing = {20, 20, 20, 100},
    });
    // CAN callbacks and the I/O thread retain these objects for the firmware lifetime.
    static skywalker::motor::Motor motor{config};
    static skywalker::motor::CanBus bus{can};
    static Vofa vofa{};
    vofa_init(&vofa, vofa_uart);

    skywalker::motor::dji::Descriptor descriptor{};
    int ret = skywalker::motor::dji::describe(config, descriptor);
    if (ret < 0)
        return ret;
    LOG_INF("GM6020 ID=%u feedback=0x%03x command=0x%03x slot=%u", descriptor.motor_id,
            descriptor.feedback_id, descriptor.command_id, descriptor.command_slot);
    ret = bus.attach(motor);
    if (ret == 0)
        ret = bus.start();
    if (ret < 0) {
        LOG_ERR("CAN configuration failed: %d", ret);
        return ret;
    }
    ret = waitForReady(motor);
    if (ret < 0)
        return ret;
    ret = checkSafeFeedback(motor.snapshot());
    if (ret < 0)
        return ret;
    ret = motor.enable();
    if (ret == 0)
        ret = waitForActive(motor);
    if (ret < 0)
        return stopAfterFailure(motor, ret);

    LOG_INF("continuous open-loop current test started: %d mA", int(kCurrentCommandA * 1000.0f));

    std::uint32_t print_divider = 0;
    for (;;) {
        const auto view = motor.snapshot();
        const auto &feedback = view.feedback;
        const auto &raw = view.native_dji_feedback;
        if (view.state != skywalker::motor::MotorState::Active || !view.feedback_fresh ||
            !view.output_permitted || !view.native_dji_feedback_valid)
            return stopAfterFailure(motor, -EHOSTDOWN);
        ret = checkSafeFeedback(view);
        if (ret < 0)
            return stopAfterFailure(motor, ret);

        ret = motor.setCurrent(kCurrentCommandA);
        if (ret < 0)
            return stopAfterFailure(motor, ret);
        const auto committed = bus.commit();
        if (committed.error < 0)
            return stopAfterFailure(motor, committed.error);

        if (++print_divider >= kTelemetryPeriodCycles) {
            print_divider = 0;
            /* JustFloat channels: ready, command_a, encoder, speed_rpm,
             * current_raw, temperature_c, timestamp_ms.
             */
            const float channels[7] = {
                1.0f,
                kCurrentCommandA,
                float(raw.encoder),
                float(raw.speed_rpm),
                float(raw.current_raw),
                feedback.temperature_c,
                float(feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 7);
        }
        k_sleep(K_MSEC(kControlPeriodMs));
    }
}
