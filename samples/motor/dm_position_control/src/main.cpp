#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <dm_sample_support.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_position_control, LOG_LEVEL_INF);

#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)

#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {

constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::int64_t kPositionStepPeriodMs = 6000;
constexpr float kSpeedSafetyMarginRadS = 0.5f;
constexpr float kTemperatureCutoffC = 60.0f;
constexpr float kPi = 3.14159265358979323846f;

float targetPositionFromSavedZeroRad(bool at_ninety_degrees) {
    constexpr float step_degrees = 90.0f;
    return at_ninety_degrees ? step_degrees * kPi / 180.0f : 0.0f;
}

float positionVelocityLimitRadS() {
    return 0.5f;
}

} // namespace

int main() {
    const struct device *vofa_uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(vofa_uart)) {
        LOG_ERR("VOFA UART device not ready");
        return -ENODEV;
    }

    static skywalker::samples::dm::Session session{
        DEVICE_DT_GET(DT_NODELABEL(can1)),
        skywalker::motor::dm::j4310PositionVelocity({.id = 1,
                                                     .master_id = 0x11,
                                                     .position_max_rad = 12.5f,
                                                     .velocity_max_rad_s = 45.0f,
                                                     .torque_max_nm = 18.0f,
                                                     .torque_limit_nm = 0.05f,
                                                     .timing = {50, 20, 50, 3000}})};
    int ret = skywalker::samples::dm::prepare(session);
    if (ret < 0) {
        return ret;
    }

    const float zero_position_rad = targetPositionFromSavedZeroRad(false);
    const float ninety_degree_position_rad = targetPositionFromSavedZeroRad(true);
    const float velocity_limit_rad_s = positionVelocityLimitRadS();
    if (!std::isfinite(ninety_degree_position_rad) ||
        std::fabs(ninety_degree_position_rad) > session.descriptor.limits.position_max_rad ||
        !std::isfinite(velocity_limit_rad_s) || velocity_limit_rad_s <= 0.0f ||
        velocity_limit_rad_s > session.descriptor.limits.velocity_max_rad_s) {
        LOG_ERR("invalid position command: step=%d mrad velocity=%d mrad/s",
                static_cast<int>(ninety_degree_position_rad * 1000.0f),
                static_cast<int>(velocity_limit_rad_s * 1000.0f));
        return -ERANGE;
    }
    const float velocity_cutoff_rad_s = velocity_limit_rad_s + kSpeedSafetyMarginRadS;

    static Vofa vofa{};
    vofa_init(&vofa, vofa_uart);
    ret = skywalker::samples::dm::arm(session);
    if (ret < 0) {
        return ret;
    }

    bool at_ninety_degrees = false;
    float target_position_rad = zero_position_rad;
    std::int64_t next_position_step_ms = k_uptime_get() + kPositionStepPeriodMs;
    LOG_INF("native position control started: saved zero <-> +90 deg every %lld ms", kPositionStepPeriodMs);

    for (;;) {
        skywalker::motor::MotorSnapshot view{};
        ret = skywalker::samples::dm::readSafeFeedback(session, velocity_cutoff_rad_s, kTemperatureCutoffC, view);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        const std::int64_t now_ms = k_uptime_get();
        if (now_ms >= next_position_step_ms) {
            at_ninety_degrees = !at_ninety_degrees;
            target_position_rad = targetPositionFromSavedZeroRad(at_ninety_degrees);
            next_position_step_ms = now_ms + kPositionStepPeriodMs;
            LOG_INF("new saved-zero-relative target: %d mdeg", static_cast<int>(target_position_rad * 180000.0f / kPi));
        }

        ret = session.motor.setPositionVelocity(target_position_rad, velocity_limit_rad_s);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }
        ret = skywalker::samples::dm::flush(session);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        const auto &feedback = view.feedback;
        const float channels[6] = {
            target_position_rad,
            view.native_position_rad,
            feedback.velocity_rad_s,
            feedback.torque_nm,
            view.native_mos_temperature_c,
            view.native_rotor_temperature_c,
        };
        vofa_send(&vofa, channels, 6);
        k_sleep(K_MSEC(kControlPeriodMs));
    }
}
