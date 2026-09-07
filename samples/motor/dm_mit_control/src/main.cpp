#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/dm_bus.hpp>
#include <drivers/motor/dm_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_mit_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

namespace {

skywalker::motor::dm::Bus dm_bus;

constexpr std::int64_t kControlPeriodMs = 5;
/* One telemetry frame per control cycle: 200 Hz. */
constexpr std::uint32_t kTelemetryPeriodCycles = 1U;
constexpr float kCommandTorqueNm = 0.0f;
constexpr float kSpeedCutoffRadS = 1.0f;
constexpr float kTemperatureCutoffC = 60.0f;

int stopAfterFailure(int original_error)
{
    skywalker::motor::dm::TxReport report{};
    const int stop_ret = dm_bus.stop(report);
    LOG_ERR("motor disabled: cause=%d stop=%d tx=%d failed_id=%u",
            original_error,
            stop_ret,
            report.tx_error,
            report.failed_motor_id);
    return stop_ret < 0 ? stop_ret : original_error;
}

int waitForDisabled(const struct device *motor)
{
    std::int64_t next_log_ms = 0;
    for (;;) {
        skywalker::motor::dm::DriveStatus status{};
        const int status_ret =
            skywalker::motor::dm::getDriveStatus(motor, status);
        if (status_ret == 0 &&
            status == skywalker::motor::dm::DriveStatus::Disabled &&
            skywalker::motor::getState(motor) ==
                skywalker::motor::State::Ready) {
            return 0;
        }

        const std::int64_t now_ms = k_uptime_get();
        if (now_ms >= next_log_ms) {
            LOG_WRN("waiting for disabled DM-J4310 feedback");
            next_log_ms = now_ms + 1000;
        }
        k_sleep(K_MSEC(5));
    }
}

} // namespace

int main()
{
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

    skywalker::motor::dm::Descriptor descriptor{};
    int ret = skywalker::motor::dm::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr ||
        !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }

    LOG_INF("DM-J4310 id=%u master=0x%03x command=0x%03x "
            "P/V/T=%d/%d/%d",
            descriptor.motor_id,
            descriptor.master_id,
            descriptor.control_id,
            static_cast<int>(descriptor.limits.position_max_rad * 1000.0f),
            static_cast<int>(descriptor.limits.velocity_max_rad_s * 1000.0f),
            static_cast<int>(descriptor.limits.torque_max_nm * 1000.0f));

    ret = dm_bus.init(descriptor.can);
    if (ret < 0) {
        LOG_ERR("Bus init failed: %d", ret);
        return ret;
    }
    ret = dm_bus.attach(motor);
    if (ret < 0) {
        LOG_ERR("Bus attach failed: %d", ret);
        return ret;
    }

    skywalker::motor::dm::TxReport stop_report{};
    ret = dm_bus.stop(stop_report);
    if (ret < 0) {
        LOG_ERR("Initial disable failed: %d", ret);
        return ret;
    }
    ret = waitForDisabled(motor);
    if (ret < 0) {
        return ret;
    }

    skywalker::motor::dm::TxReport arm_report{};
    ret = dm_bus.arm(arm_report);
    if (ret < 0) {
        LOG_ERR("Arm failed: %d", ret);
        return ret;
    }

    LOG_INF("MIT zero-torque loop started");
    std::uint32_t print_divider = 0u;
    for (;;) {
        skywalker::motor::Feedback feedback{};
        skywalker::motor::dm::RawFeedback raw{};
        ret = skywalker::motor::readFeedback(motor, feedback);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }
        ret = skywalker::motor::dm::readRawFeedback(motor, raw);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }
        if (skywalker::motor::getState(motor) !=
            skywalker::motor::State::Ready) {
            return stopAfterFailure(-EHOSTDOWN);
        }
        if (std::fabs(feedback.velocity_rad_s) > kSpeedCutoffRadS ||
            feedback.temperature_c >= kTemperatureCutoffC ||
            static_cast<float>(raw.mos_temperature_c) >=
                kTemperatureCutoffC) {
            return stopAfterFailure(-ERANGE);
        }

        ret = skywalker::motor::setTorque(motor, kCommandTorqueNm);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }
        skywalker::motor::dm::TxReport flush_report{};
        ret = dm_bus.flush(flush_report);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        if (++print_divider >= kTelemetryPeriodCycles) {
            print_divider = 0u;
            /* JustFloat channels: status, position_rad, velocity_rad_s,
             * torque_nm, mos_temperature_c, rotor_temperature_c.
             */
            const float channels[6] = {
                static_cast<float>(raw.status),
                feedback.position_rad,
                feedback.velocity_rad_s,
                feedback.torque_nm,
                static_cast<float>(raw.mos_temperature_c),
                static_cast<float>(raw.rotor_temperature_c),
            };
            vofa_send(&vofa, channels, 6);
        }
        k_sleep(K_MSEC(kControlPeriodMs));
    }
}
