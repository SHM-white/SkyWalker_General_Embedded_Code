#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <dm_sample_support.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_velocity_control, LOG_LEVEL_INF);

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
    const struct device *vofa_uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(vofa_uart)) {
        LOG_ERR("VOFA UART device not ready");
        return -ENODEV;
    }

    static skywalker::samples::dm::Session session{
        DEVICE_DT_GET(DT_NODELABEL(can1)),
        skywalker::motor::dm::j4310Velocity({.id = 1,
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

    const float target_velocity_rad_s = targetVelocityRadS();
    if (!std::isfinite(target_velocity_rad_s) ||
        std::fabs(target_velocity_rad_s) > session.descriptor.limits.velocity_max_rad_s) {
        LOG_ERR("invalid velocity target: %d mrad/s", static_cast<int>(target_velocity_rad_s * 1000.0f));
        return -ERANGE;
    }
    const float velocity_cutoff_rad_s = std::fabs(target_velocity_rad_s) + kSpeedSafetyMarginRadS;

    static Vofa vofa{};
    vofa_init(&vofa, vofa_uart);
    ret = skywalker::samples::dm::arm(session);
    if (ret < 0) {
        return ret;
    }

    LOG_INF("native velocity control started: target=%d mrad/s", static_cast<int>(target_velocity_rad_s * 1000.0f));
    for (;;) {
        skywalker::motor::MotorSnapshot view{};
        ret = skywalker::samples::dm::readSafeFeedback(session, velocity_cutoff_rad_s, kTemperatureCutoffC, view);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        ret = session.motor.setVelocity(target_velocity_rad_s);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }
        ret = skywalker::samples::dm::flush(session);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        const auto &feedback = view.feedback;
        const float channels[6] = {
            target_velocity_rad_s,
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
