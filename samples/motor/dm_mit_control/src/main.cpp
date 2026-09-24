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
    const struct device *vofa_uart = DEVICE_DT_GET(VOFA_UART_NODE);
    if (!device_is_ready(vofa_uart)) {
        LOG_ERR("VOFA UART device not ready");
        return -ENODEV;
    }

    // CAN callbacks and the I/O thread retain this session for the firmware lifetime.
    static skywalker::samples::dm::Session session{DEVICE_DT_GET(DT_NODELABEL(can1)),
                                                   skywalker::motor::dm::j4310Mit({.id = 1,
                                                                                   .master_id = 0x11,
                                                                                   .position_max_rad = 12.5f,
                                                                                   .velocity_max_rad_s = 30.0f,
                                                                                   .torque_max_nm = 10.0f,
                                                                                   .torque_limit_nm = 10.0f,
                                                                                   .timing = {50, 20, 50, 3000}})};
    int ret = skywalker::samples::dm::prepare(session);
    if (ret < 0) {
        return ret;
    }
    if ((session.motor.info().capabilities & skywalker::motor::CommandTorque) == 0U) {
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
        skywalker::motor::MotorSnapshot view{};
        ret = skywalker::samples::dm::readSafeFeedback(session, kVelocityCutoffRadS, kTemperatureCutoffC, view);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        ret = session.motor.setTorque(target_torque_nm);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }
        ret = skywalker::samples::dm::flush(session);
        if (ret < 0) {
            return skywalker::samples::dm::stopAfterFailure(session, ret);
        }

        const auto &feedback = view.feedback;
        const float channels[6] = {
            target_torque_nm,   view.native_position_rad,      feedback.velocity_rad_s,
            feedback.torque_nm, view.native_mos_temperature_c, view.native_rotor_temperature_c,
        };
        vofa_send(&vofa, channels, 6);
        k_sleep(K_MSEC(kControlPeriodMs));
    }

    ret = session.motor.setTorque(0.0f);
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
