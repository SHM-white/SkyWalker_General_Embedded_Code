#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <dm_sample_support.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_velocity_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)

#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {

constexpr std::int64_t kControlPeriodMs = 5;
constexpr float kSpeedSafetyMarginRadS = 0.5f;
constexpr float kTemperatureCutoffC = 60.0f;

float targetVelocityRadS() {
    return 0.5f;
}

} // namespace

int main() {
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    const struct device *vofa_uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(vofa_uart)) {
        LOG_ERR("VOFA UART device not ready");
        return -ENODEV;
    }

    skywalker::samples::dm::Session session{};
    int ret = skywalker::samples::dm::prepare(session, motor);
    if (ret < 0) {
        return ret;
    }

    const float target_velocity_rad_s = targetVelocityRadS();
    if (!std::isfinite(target_velocity_rad_s) ||
        std::fabs(target_velocity_rad_s) > session.descriptor.limits.velocity_max_rad_s) {
        LOG_ERR("invalid velocity target: %d mrad/s", static_cast<int>(target_velocity_rad_s * 1000.0f));
        return -ERANGE;
    }
    const float velocity_cutoff_rad_s = std::fabs(target_velocity_rad_s) + kSpeedSafetyMarginRadS;

    Vofa vofa{};
    vofa_init(&vofa, vofa_uart);
    ret = skywalker::samples::dm::arm(session);
    if (ret < 0) {
        return ret;
    }

    LOG_INF("native velocity control started: target=%d mrad/s", static_cast<int>(target_velocity_rad_s * 1000.0f));
    for (;;) {
        skywalker::motor::Feedback feedback{};
        skywalker::motor::dm::RawFeedback raw{};
        ret = skywalker::samples::dm::readSafeFeedback(session, velocity_cutoff_rad_s, kTemperatureCutoffC, feedback,
                                                       raw);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        ret = skywalker::motor::dm::setVelocity(motor, target_velocity_rad_s);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }
        ret = skywalker::samples::dm::flush(session);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        const float channels[6] = {
            target_velocity_rad_s,
            feedback.position_rad,
            feedback.velocity_rad_s,
            feedback.torque_nm,
            static_cast<float>(raw.mos_temperature_c),
            static_cast<float>(raw.rotor_temperature_c),
        };
        vofa_send(&vofa, channels, 6);
        k_sleep(K_MSEC(kControlPeriodMs));
    }
}
