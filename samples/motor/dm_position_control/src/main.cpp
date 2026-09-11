#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#if defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/dm_bus.hpp>
#include <drivers/motor/dm_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_position_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)

#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

#if defined(CONFIG_BOARD_DM_MC02)
#define XT30_1_NODE DT_NODELABEL(power1)
static const struct device *const xt30_1 = DEVICE_DT_GET(XT30_1_NODE);

static int xt30_enable(const struct device *dev) {
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }

    return regulator_enable(dev);
}
#endif

namespace {

skywalker::motor::dm::Bus dm_bus;

#if defined(CONFIG_BOARD_DM_MC02)
constexpr std::int64_t kMotorPowerOnDelayMs = 1500;
#endif
constexpr std::int64_t kDisableRetryPeriodMs = 100;
constexpr std::int64_t kDisabledHandshakeTimeoutMs = 3000;
constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::int64_t kPositionStepPeriodMs = 6000;
constexpr std::uint32_t kTelemetryPeriodCycles = 1U;
constexpr float kSpeedSafetyMarginRadS = 10.0f;
constexpr float kTemperatureCutoffC = 60.0f;
constexpr float kPi = 3.14159265358979323846f;

float targetPositionFromSavedZeroRad(bool at_ninety_degrees) {
    /* Alternate between the persistent zero and +90 degrees. */
    constexpr float step_degrees = 90.0f;
    return at_ninety_degrees ? step_degrees * kPi / 180.0f : 0.0f;
}

float positionVelocityLimitRadS() {
    return 0.5f;
}

int stopAfterFailure(int original_error) {
    skywalker::motor::dm::TxReport report{};
    const int stop_ret = dm_bus.stop(report);
    LOG_ERR("motor disabled: cause=%d stop=%d tx=%d failed_id=%u", original_error, stop_ret, report.tx_error,
            report.failed_motor_id);
    return stop_ret < 0 ? stop_ret : original_error;
}

int waitForDisabled(const struct device *motor) {
    const std::int64_t started_ms = k_uptime_get();
    std::int64_t next_disable_ms = started_ms;
    std::int64_t next_log_ms = 0;
    for (;;) {
        skywalker::motor::dm::DriveStatus status{};
        const int status_ret = skywalker::motor::dm::getDriveStatus(motor, status);
        const skywalker::motor::State state = skywalker::motor::getState(motor);
        if (status_ret == 0 && status == skywalker::motor::dm::DriveStatus::Disabled &&
            state == skywalker::motor::State::Ready) {
            return 0;
        }

        const std::int64_t now_ms = k_uptime_get();
        if (now_ms - started_ms >= kDisabledHandshakeTimeoutMs) {
            LOG_ERR("disabled feedback timeout: status_ret=%d status=%u state=%u", status_ret,
                    static_cast<unsigned int>(status), static_cast<unsigned int>(state));
            return -ETIMEDOUT;
        }

        if (now_ms >= next_disable_ms) {
            skywalker::motor::dm::TxReport report{};
            const int stop_ret = dm_bus.stop(report);
            if (stop_ret < 0) {
                LOG_ERR("disable handshake failed: ret=%d prep=%d tx=%d failed_id=%u", stop_ret,
                        report.preparation_error, report.tx_error, report.failed_motor_id);
                return stop_ret;
            }
            next_disable_ms = now_ms + kDisableRetryPeriodMs;
        }

        if (now_ms >= next_log_ms) {
            LOG_WRN("waiting for disabled DM-J4310 feedback: status_ret=%d status=%u state=%u", status_ret,
                    static_cast<unsigned int>(status), static_cast<unsigned int>(state));
            next_log_ms = now_ms + 1000;
        }
        k_sleep(K_MSEC(5));
    }
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

    int ret = 0;
#if defined(CONFIG_BOARD_DM_MC02)
    ret = xt30_enable(xt30_1);
    if (ret < 0) {
        LOG_ERR("XT30_1 enable failed: %d", ret);
        return ret;
    }
    LOG_INF("XT30_1 enabled; waiting %lld ms for motor startup", kMotorPowerOnDelayMs);
    k_sleep(K_MSEC(kMotorPowerOnDelayMs));
#endif

    Vofa vofa{};
    vofa_init(&vofa, vofa_uart);

    skywalker::motor::dm::Descriptor descriptor{};
    ret = skywalker::motor::dm::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr || !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }

    const float zero_position_rad = targetPositionFromSavedZeroRad(false);
    const float ninety_degree_position_rad = targetPositionFromSavedZeroRad(true);
    const float velocity_limit_rad_s = positionVelocityLimitRadS();
    if (!std::isfinite(zero_position_rad) || !std::isfinite(ninety_degree_position_rad) ||
        std::fabs(zero_position_rad) > descriptor.limits.position_max_rad ||
        std::fabs(ninety_degree_position_rad) > descriptor.limits.position_max_rad) {
        LOG_ERR("Invalid position step: zero=%d mrad ninety=%d mrad", static_cast<int>(zero_position_rad * 1000.0f),
                static_cast<int>(ninety_degree_position_rad * 1000.0f));
        return -ERANGE;
    }
    if (!std::isfinite(velocity_limit_rad_s) || velocity_limit_rad_s < 0.0f ||
        velocity_limit_rad_s > descriptor.limits.velocity_max_rad_s) {
        LOG_ERR("Invalid velocity limit: %d mrad/s", static_cast<int>(velocity_limit_rad_s * 1000.0f));
        return -ERANGE;
    }
    const float speed_cutoff_rad_s = velocity_limit_rad_s + kSpeedSafetyMarginRadS;

    LOG_INF("DM-J4310 id=%u master=0x%03x command=0x%03x step_mrad=%d period_ms=%lld velocity_limit_mrad_s=%d",
            descriptor.motor_id, descriptor.master_id, descriptor.control_id,
            static_cast<int>(ninety_degree_position_rad * 1000.0f), kPositionStepPeriodMs,
            static_cast<int>(velocity_limit_rad_s * 1000.0f));

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

    bool at_ninety_degrees = false;
    float target_position_rad = zero_position_rad;
    std::int64_t next_position_step_ms = k_uptime_get() + kPositionStepPeriodMs;
    LOG_INF("position stepping started at saved zero; first +90 degree step in %lld ms", kPositionStepPeriodMs);
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
        if (skywalker::motor::getState(motor) != skywalker::motor::State::Ready) {
            return stopAfterFailure(-EHOSTDOWN);
        }
        if (std::fabs(feedback.velocity_rad_s) > speed_cutoff_rad_s || feedback.temperature_c >= kTemperatureCutoffC ||
            static_cast<float>(raw.mos_temperature_c) >= kTemperatureCutoffC) {
            return stopAfterFailure(-ERANGE);
        }

        const std::int64_t now_ms = k_uptime_get();
        if (now_ms >= next_position_step_ms) {
            at_ninety_degrees = !at_ninety_degrees;
            target_position_rad = targetPositionFromSavedZeroRad(at_ninety_degrees);
            next_position_step_ms = now_ms + kPositionStepPeriodMs;
            LOG_INF("new saved-zero-relative target: %d mdeg", static_cast<int>(target_position_rad * 180000.0f / kPi));
        }

        ret = skywalker::motor::dm::setPositionVelocity(motor, target_position_rad, velocity_limit_rad_s);
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
