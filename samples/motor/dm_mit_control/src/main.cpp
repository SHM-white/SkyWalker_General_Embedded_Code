#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <dm_sample_support.hpp>
#include <drivers/motor/motor.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dm_mit_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)

#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {

constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::int64_t kRunDurationMs = 200000;
constexpr float kVelocityCutoffRadS = 30.0f;
constexpr float kTemperatureCutoffC = 60.0f;

float targetTorqueNm() {
    return 0.2f;
}

} // namespace

int main() {
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    const struct device *vofa_uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(vofa_uart)) {
        LOG_ERR("VOFA UART device not ready");
        return -ENODEV;
    }

    // The CAN RX filter retains a pointer to session.bus even after main exits.
    static skywalker::samples::dm::Session session{};
    int ret = skywalker::samples::dm::prepare(session, motor);
    if (ret < 0) {
        return ret;
    }
    if ((skywalker::motor::capabilities(motor) & skywalker::motor::CommandTorque) == 0U) {
        LOG_ERR("MIT torque capability unavailable; check motor control mode");
        return -ENOTSUP;
    }

    const float target_torque_nm = targetTorqueNm();
    if (!std::isfinite(target_torque_nm) || std::fabs(target_torque_nm) > session.descriptor.torque_limit_nm) {
        LOG_ERR("invalid torque target: %d mNm, limit=%d mNm", static_cast<int>(target_torque_nm * 1000.0f),
                static_cast<int>(session.descriptor.torque_limit_nm * 1000.0f));
        return -ERANGE;
    }

    // The asynchronous UART callback also outlives main on a failure/stop path.
    static Vofa vofa{};
    vofa_init(&vofa, vofa_uart);
    ret = skywalker::samples::dm::arm(session);
    if (ret < 0) {
        return ret;
    }

    LOG_INF("MIT torque test started: target=%d mNm duration=%lld ms", static_cast<int>(target_torque_nm * 1000.0f),
            kRunDurationMs);
    const std::int64_t started_ms = k_uptime_get();
    while (k_uptime_get() - started_ms < kRunDurationMs) {
        skywalker::motor::Feedback feedback{};
        skywalker::motor::dm::RawFeedback raw{};
        ret = skywalker::samples::dm::readSafeFeedback(session, kVelocityCutoffRadS, kTemperatureCutoffC, feedback,
                                                       raw);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        ret = skywalker::motor::setTorque(motor, target_torque_nm);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }
        ret = skywalker::samples::dm::flush(session);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        const float channels[6] = {
            target_torque_nm,
            feedback.position_rad,
            feedback.velocity_rad_s,
            feedback.torque_nm,
            static_cast<float>(raw.mos_temperature_c),
            static_cast<float>(raw.rotor_temperature_c),
        };
        vofa_send(&vofa, channels, 6);
        k_sleep(K_MSEC(kControlPeriodMs));
    }

    ret = skywalker::motor::setTorque(motor, 0.0f);
    if (ret < 0) {
        return skywalker::samples::dm::stopAfterFailure(session, ret);
    }
    ret = skywalker::samples::dm::flush(session);
    if (ret < 0) {
        return skywalker::samples::dm::stopAfterFailure(session, ret);
    }
    ret = skywalker::samples::dm::stop(session);
    if (ret < 0) {
        return ret;
    }
    LOG_INF("MIT torque test completed and motor disabled");
    return 0;
}
