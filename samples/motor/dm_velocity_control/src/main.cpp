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
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    static skywalker::samples::dm::Session session{DEVICE_DT_GET(DT_NODELABEL(can1)),
                                                   skywalker::motor::dm::j4310Velocity(
                                                       {.id = 1,
                                                        .master_id = 0x11,
                                                        .position_max_rad = 12.5f,
                                                        .velocity_max_rad_s = 45.0f,
                                                        .torque_max_nm = 18.0f,
                                                        .torque_limit_nm = 0.05f,
                                                        .timing = {.feedback_timeout_ms = 50,
                                                                   .command_timeout_ms = 20,
                                                                   .enable_timeout_ms = 3000,
                                                                   .retry_interval_ms = 100}})};
    int ret = skywalker::samples::dm::prepare(session);
    if (ret < 0)
        return ret;
    static Vofa vofa{};
    const int vofa_error = vofa_init(&vofa, uart);
    const auto started_ms = k_uptime_get();
    std::int64_t next_log_ms = 0;
    int last_call_error = 0;
    for (;;) {
        const auto now = k_uptime_get();
        const bool requested = true;
        const float target = targetVelocityRadS();
        const int request_error = requested ? session.motor.enable() : session.motor.disable();
        if (request_error < 0)
            last_call_error = request_error;
        if (requested) {
            const int error = session.motor.setVelocity(target);
            if (error < 0)
                last_call_error = error;
        }
        const int commit_error = session.bus.commit().error;
        if (commit_error < 0)
            last_call_error = commit_error;
        const auto view = session.motor.snapshot();
        if (vofa_error == 0) {
            const float channels[6] = {target,
                                       view.native_position_rad,
                                       view.feedback.velocity_rad_s,
                                       view.feedback.torque_nm,
                                       view.native_mos_temperature_c,
                                       view.native_rotor_temperature_c};
            (void)vofa_send(&vofa, channels, 6);
        }
        if (now >= next_log_ms) {
            next_log_ms = now + 1000;
            LOG_INF("run=%d requested=%d state=%u target=%.3f fresh=%d call=%d", requested, view.enabled_requested,
                    unsigned(view.state), double(target), view.feedback_fresh, last_call_error);
        }
        k_sleep(K_MSEC(kControlPeriodMs));
    }
}
