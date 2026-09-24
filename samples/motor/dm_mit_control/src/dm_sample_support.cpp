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
const device *const xt30_1 = DEVICE_DT_GET(DT_NODELABEL(power1));
constexpr std::int64_t kMotorPowerOnDelayMs = 1500;
#endif

constexpr std::int64_t kStatusHandshakeTimeoutMs = 3000;
constexpr float kPreEnableVelocityCutoffRadS = 30.0f;
constexpr float kPreEnableTemperatureCutoffC = 60.0f;

int enableMotorPower() {
#if defined(CONFIG_BOARD_DM_MC02)
    if (!device_is_ready(xt30_1))
        return -ENODEV;
    const int ret = regulator_enable(xt30_1);
    if (ret < 0)
        return ret;
    LOG_INF("XT30_1 enabled; waiting %lld ms for motor startup", kMotorPowerOnDelayMs);
    k_sleep(K_MSEC(kMotorPowerOnDelayMs));
#endif
    return 0;
}

int waitUntilReady(Session &session) {
    const std::int64_t started_ms = k_uptime_get();
    std::int64_t next_log_ms = started_ms;
    while (k_uptime_get() - started_ms < kStatusHandshakeTimeoutMs) {
        if (session.motor.ready())
            return 0;
        const auto view = session.motor.snapshot();
        if (view.state == motor::MotorState::Fault) {
            LOG_ERR("startup fault: reason=%u error=%d status=0x%x", unsigned(view.last_fault.reason),
                    view.last_fault.error, unsigned(view.native_drive_status));
            return view.last_fault.error < 0 ? view.last_fault.error : -EHOSTDOWN;
        }
        const auto now_ms = k_uptime_get();
        if (now_ms >= next_log_ms) {
            LOG_WRN("waiting for safe DM feedback: state=%u status=0x%x stop=%u",
                    unsigned(view.state), unsigned(view.native_drive_status), unsigned(view.stop.progress));
            next_log_ms = now_ms + 1000;
        }
        k_sleep(K_MSEC(5));
    }
    LOG_ERR("operational feedback timeout: check power, CAN, ID=%u and Master ID=0x%03x",
            unsigned(session.descriptor.motor_id), unsigned(session.descriptor.master_id));
    return -ETIMEDOUT;
}

int checkSafeFeedback(const Session &session, float velocity_abs_max_rad_s, float temperature_max_c,
                      bool before_enable, motor::MotorSnapshot &snapshot) {
    if (!std::isfinite(velocity_abs_max_rad_s) || velocity_abs_max_rad_s <= 0.0f ||
        !std::isfinite(temperature_max_c) || temperature_max_c <= 0.0f)
        return -EINVAL;
    const auto view = session.motor.snapshot();
    const auto expected_state = before_enable ? motor::MotorState::Disabled : motor::MotorState::Active;
    const auto expected_status = before_enable ? motor::dm::DriveStatus::Disabled : motor::dm::DriveStatus::Enabled;
    if (view.state != expected_state || !view.feedback_fresh ||
        (!before_enable && !view.output_permitted) || !view.native_drive_status_valid ||
        view.native_drive_status != unsigned(expected_status)) {
        LOG_ERR("feedback unavailable: state=%u status=0x%x fresh=%d", unsigned(view.state),
                unsigned(view.native_drive_status), view.feedback_fresh);
        return -EHOSTDOWN;
    }
    constexpr std::uint32_t required = motor::FeedbackPosition | motor::FeedbackVelocity |
                                       motor::FeedbackTorque | motor::FeedbackTemperature;
    const auto &feedback = view.feedback;
    if ((feedback.valid & required) != required || !view.native_temperatures_valid || !view.native_position_valid)
        return -ENODATA;
    if (!std::isfinite(feedback.position_rad) || !std::isfinite(feedback.velocity_rad_s) ||
        !std::isfinite(feedback.torque_nm) || !std::isfinite(feedback.temperature_c) ||
        !std::isfinite(view.native_position_rad) ||
        !std::isfinite(view.native_mos_temperature_c) || !std::isfinite(view.native_rotor_temperature_c))
        return -EINVAL;
    if (std::fabs(feedback.velocity_rad_s) > velocity_abs_max_rad_s ||
        view.native_mos_temperature_c >= temperature_max_c ||
        view.native_rotor_temperature_c >= temperature_max_c) {
        LOG_ERR("safety cutoff: speed=%d limit=%d mrad/s MOS=%d rotor=%d limit=%d C",
                int(feedback.velocity_rad_s * 1000.0f), int(velocity_abs_max_rad_s * 1000.0f),
                int(view.native_mos_temperature_c), int(view.native_rotor_temperature_c), int(temperature_max_c));
        return -ERANGE;
    }
    snapshot = view;
    return 0;
}

} // namespace

int prepare(Session &session) {
    if (session.can == nullptr || !device_is_ready(session.can))
        return -ENODEV;
    int ret = motor::dm::describe(session.config, session.descriptor);
    if (ret < 0)
        return ret;
    LOG_INF("DM-J4310 id=%u master=0x%03x command=0x%03x mode=%u P/V/T=%d/%d/%d torque_limit=%d mNm",
            unsigned(session.descriptor.motor_id), unsigned(session.descriptor.master_id),
            unsigned(session.descriptor.control_id), unsigned(session.descriptor.mode),
            int(session.descriptor.limits.position_max_rad * 1000.0f),
            int(session.descriptor.limits.velocity_max_rad_s * 1000.0f),
            int(session.descriptor.limits.torque_max_nm * 1000.0f),
            int(session.descriptor.torque_limit_nm * 1000.0f));
    ret = session.bus.attach(session.motor);
    if (ret == 0)
        ret = session.bus.start();
    if (ret < 0) {
        LOG_ERR("CAN configuration failed: %d", ret);
        return ret;
    }
    ret = enableMotorPower();
    if (ret < 0) {
        LOG_ERR("motor power enable failed: %d", ret);
        return ret;
    }
    return waitUntilReady(session);
}

int arm(Session &session) {
    motor::MotorSnapshot pre_enable{};
    int ret = checkSafeFeedback(session, kPreEnableVelocityCutoffRadS, kPreEnableTemperatureCutoffC,
                                true, pre_enable);
    if (ret < 0) {
        LOG_ERR("pre-enable safety check failed: %d", ret);
        return ret;
    }
    ret = session.motor.enable();
    if (ret < 0) {
        LOG_ERR("enable request failed: %d", ret);
        return ret;
    }
    const std::int64_t started_ms = k_uptime_get();
    while (k_uptime_get() - started_ms < kStatusHandshakeTimeoutMs) {
        if (session.motor.active()) {
            LOG_INF("motor active: Enable acknowledged and neutral command sent");
            return 0;
        }
        const auto view = session.motor.snapshot();
        if (view.state == motor::MotorState::Fault) {
            LOG_ERR("enable fault: reason=%u error=%d", unsigned(view.last_fault.reason), view.last_fault.error);
            return view.last_fault.error < 0 ? view.last_fault.error : -EHOSTDOWN;
        }
        k_sleep(K_MSEC(5));
    }
    (void)session.motor.disable();
    LOG_ERR("enable handshake timeout");
    return -ETIMEDOUT;
}

int readSafeFeedback(const Session &session, float velocity_abs_max_rad_s, float temperature_max_c,
                     motor::MotorSnapshot &snapshot) {
    return checkSafeFeedback(session, velocity_abs_max_rad_s, temperature_max_c, false, snapshot);
}

int flush(Session &session) {
    const auto result = session.bus.commit();
    if (result.error < 0)
        LOG_ERR("commit failed: %d", result.error);
    return result.error;
}

int stop(Session &session) {
    const int ret = session.motor.disable();
    if (ret < 0)
        LOG_ERR("disable request failed: %d", ret);
    return ret;
}

int stopAfterFailure(Session &session, int original_error) {
    const int stop_ret = stop(session);
    LOG_ERR("motor disabled after failure: cause=%d stop=%d", original_error, stop_ret);
    return stop_ret < 0 ? stop_ret : original_error;
}

} // namespace skywalker::samples::dm
