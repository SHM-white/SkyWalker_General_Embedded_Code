#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#if defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <dm_sample_support.hpp>

LOG_MODULE_REGISTER(dm_sample_support, LOG_LEVEL_INF);

namespace skywalker::samples::dm {
namespace {

#if defined(CONFIG_BOARD_DM_MC02)
#define XT30_1_NODE DT_NODELABEL(power1)
const struct device *const xt30_1 = DEVICE_DT_GET(XT30_1_NODE);
constexpr std::int64_t kMotorPowerOnDelayMs = 1500;
#endif

constexpr std::int64_t kDisableRetryPeriodMs = 100;
constexpr std::int64_t kInitialPassiveFeedbackWaitMs = 100;
constexpr std::int64_t kStatusHandshakeTimeoutMs = 3000;

int enableMotorPowerImpl() {
#if defined(CONFIG_BOARD_DM_MC02)
    if (!device_is_ready(xt30_1)) {
        return -ENODEV;
    }
    const int ret = regulator_enable(xt30_1);
    if (ret < 0) {
        return ret;
    }
    LOG_INF("XT30_1 enabled; waiting %lld ms for motor startup", kMotorPowerOnDelayMs);
    k_sleep(K_MSEC(kMotorPowerOnDelayMs));
#endif
    return 0;
}

int waitForOperationalStatus(Session &session) {
    const std::int64_t started_ms = k_uptime_get();
    std::int64_t next_disable_ms = started_ms + kInitialPassiveFeedbackWaitMs;
    std::int64_t next_log_ms = started_ms;
    bool disable_probe_logged = false;

    for (;;) {
        skywalker::motor::dm::DriveStatus status{};
        const int status_ret = skywalker::motor::dm::getDriveStatus(session.motor, status);
        const skywalker::motor::State state = skywalker::motor::getState(session.motor);
        if (status_ret == 0) {
            if (state == skywalker::motor::State::Ready &&
                (status == skywalker::motor::dm::DriveStatus::Disabled ||
                 status == skywalker::motor::dm::DriveStatus::Enabled)) {
                LOG_INF("DM-J4310 startup status=%u; arm will %s", static_cast<unsigned int>(status),
                        status == skywalker::motor::dm::DriveStatus::Disabled ? "send Enable" : "keep enabled");
                return 0;
            }
            if (skywalker::motor::dm::isFaultStatus(status)) {
                LOG_ERR("DM-J4310 fault during startup: status=%u state=%u", static_cast<unsigned int>(status),
                        static_cast<unsigned int>(state));
                return -EHOSTDOWN;
            }
        }

        const std::int64_t now_ms = k_uptime_get();
        if (now_ms - started_ms >= kStatusHandshakeTimeoutMs) {
            LOG_ERR("operational feedback timeout: status_ret=%d status=%u state=%u", status_ret,
                    static_cast<unsigned int>(status), static_cast<unsigned int>(state));
            LOG_ERR("check motor power, CAN wiring/bitrate, motor ID=%u and feedback Master ID=0x%03x",
                    session.descriptor.motor_id, session.descriptor.master_id);
            return -ETIMEDOUT;
        }

        if (now_ms >= next_disable_ms) {
            if (!disable_probe_logged) {
                LOG_WRN("no fresh startup feedback; using Disable as a safe status probe");
                disable_probe_logged = true;
            }
            skywalker::motor::dm::TxReport report{};
            const int stop_ret = session.bus.stop(report);
            if (stop_ret < 0) {
                LOG_ERR("disable handshake failed: ret=%d prep=%d tx=%d failed_id=%u", stop_ret,
                        report.preparation_error, report.tx_error, report.failed_motor_id);
                return stop_ret;
            }
            next_disable_ms = now_ms + kDisableRetryPeriodMs;
        }

        if (now_ms >= next_log_ms) {
            LOG_WRN("waiting for operational DM-J4310 feedback: status_ret=%d status=%u state=%u", status_ret,
                    static_cast<unsigned int>(status), static_cast<unsigned int>(state));
            next_log_ms = now_ms + 1000;
        }
        k_sleep(K_MSEC(5));
    }
}

} // namespace

int enableMotorPower() {
    return enableMotorPowerImpl();
}

int prepare(Session &session, const struct device *motor) {
    if (motor == nullptr || !device_is_ready(motor)) {
        return -ENODEV;
    }

    skywalker::motor::dm::Descriptor descriptor{};
    int ret = skywalker::motor::dm::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr || !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }

    session.motor = motor;
    session.descriptor = descriptor;
    LOG_INF("DM-J4310 id=%u master=0x%03x command=0x%03x mode=%u P/V/T=%d/%d/%d torque_limit=%d mNm",
            descriptor.motor_id, descriptor.master_id, descriptor.control_id,
            static_cast<unsigned int>(descriptor.mode), static_cast<int>(descriptor.limits.position_max_rad * 1000.0f),
            static_cast<int>(descriptor.limits.velocity_max_rad_s * 1000.0f),
            static_cast<int>(descriptor.limits.torque_max_nm * 1000.0f),
            static_cast<int>(descriptor.torque_limit_nm * 1000.0f));

    ret = session.bus.init(descriptor.can);
    if (ret < 0) {
        LOG_ERR("Bus init failed: %d", ret);
        return ret;
    }
    ret = session.bus.attach(motor);
    if (ret < 0) {
        LOG_ERR("Bus attach failed: %d", ret);
        return ret;
    }

    ret = enableMotorPower();
    if (ret < 0) {
        LOG_ERR("motor power enable failed: %d", ret);
        return ret;
    }
    return waitForOperationalStatus(session);
}

int arm(Session &session) {
    skywalker::motor::dm::TxReport report{};
    const int ret = session.bus.arm(report);
    if (ret < 0) {
        LOG_ERR("arm failed: ret=%d prep=%d tx=%d failed_id=%u", ret, report.preparation_error, report.tx_error,
                report.failed_motor_id);
    } else {
        LOG_INF("motor armed: Enable acknowledged (or already enabled), neutral command sent");
    }
    return ret;
}

int readSafeFeedback(const Session &session, float velocity_abs_max_rad_s, float temperature_max_c,
                     skywalker::motor::Feedback &feedback, skywalker::motor::dm::RawFeedback &raw) {
    if (session.motor == nullptr || !std::isfinite(velocity_abs_max_rad_s) || velocity_abs_max_rad_s <= 0.0f ||
        !std::isfinite(temperature_max_c)) {
        return -EINVAL;
    }

    int ret = skywalker::motor::readFeedback(session.motor, feedback);
    if (ret < 0) {
        return ret;
    }
    ret = skywalker::motor::dm::readRawFeedback(session.motor, raw);
    if (ret < 0) {
        return ret;
    }
    const auto state = skywalker::motor::getState(session.motor);
    if (state != skywalker::motor::State::Ready) {
        LOG_ERR("feedback unavailable: state=%u drive_status=0x%x age=%lld ms (timeout=%d ms)",
                static_cast<unsigned int>(state), static_cast<unsigned int>(raw.status),
                k_uptime_get() - static_cast<std::int64_t>(raw.timestamp_ms),
                CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS);
        return -EHOSTDOWN;
    }
    // Controllers seed their state from feedback before arm(). Disabled is
    // valid only in the Safe bus state, never during a running control loop.
    const bool disabled_before_arm = session.bus.state() == skywalker::motor::dm::BusState::Safe &&
                                     raw.status == skywalker::motor::dm::DriveStatus::Disabled;
    if (raw.status != skywalker::motor::dm::DriveStatus::Enabled && !disabled_before_arm) {
        LOG_ERR("drive is not enabled: status=0x%x", static_cast<unsigned int>(raw.status));
        return -EHOSTDOWN;
    }

    constexpr std::uint32_t required = skywalker::motor::FeedbackPosition | skywalker::motor::FeedbackVelocity |
                                       skywalker::motor::FeedbackTorque | skywalker::motor::FeedbackTemperature;
    if ((feedback.valid & required) != required) {
        return -ENODATA;
    }
    if (!std::isfinite(feedback.position_rad) || !std::isfinite(feedback.velocity_rad_s) ||
        !std::isfinite(feedback.torque_nm) || !std::isfinite(feedback.temperature_c)) {
        return -EINVAL;
    }
    if (std::fabs(feedback.velocity_rad_s) > velocity_abs_max_rad_s || feedback.temperature_c >= temperature_max_c ||
        static_cast<float>(raw.mos_temperature_c) >= temperature_max_c) {
        LOG_ERR("safety cutoff: speed=%d limit=%d mrad/s MOS=%u rotor=%u limit=%d C",
                static_cast<int>(feedback.velocity_rad_s * 1000.0f),
                static_cast<int>(velocity_abs_max_rad_s * 1000.0f),
                raw.mos_temperature_c, raw.rotor_temperature_c, static_cast<int>(temperature_max_c));
        return -ERANGE;
    }
    return 0;
}

int flush(Session &session) {
    skywalker::motor::dm::TxReport report{};
    const int ret = session.bus.flush(report);
    if (ret < 0) {
        LOG_ERR("flush failed: ret=%d prep=%d tx=%d failed_id=%u", ret, report.preparation_error, report.tx_error,
                report.failed_motor_id);
    }
    return ret;
}

int stop(Session &session) {
    skywalker::motor::dm::TxReport report{};
    const int ret = session.bus.stop(report);
    if (ret < 0) {
        LOG_ERR("stop failed: ret=%d prep=%d tx=%d failed_id=%u", ret, report.preparation_error, report.tx_error,
                report.failed_motor_id);
    }
    return ret;
}

int stopAfterFailure(Session &session, int original_error) {
    const int stop_ret = stop(session);
    LOG_ERR("motor disabled after failure: cause=%d stop=%d", original_error, stop_ret);
    return stop_ret < 0 ? stop_ret : original_error;
}

} // namespace skywalker::samples::dm
