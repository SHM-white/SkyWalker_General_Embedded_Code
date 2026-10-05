#include <cerrno>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <lib/vofa/vofa.h>
LOG_MODULE_REGISTER(dji_unified, LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(can)) return -ENODEV;
    static motor::Motor drive{motor::dji::gm6020({.id = 4, .current_limit_a = 0.1f,
        .encoder_zero_ticks = 0, .current_mode_confirmed = true,
        .timing = {.feedback_timeout_ms = 20, .command_timeout_ms = 20, .enable_timeout_ms = 100, .retry_interval_ms = 100}})};
    static motor::CanBus bus{can};
    static Vofa vofa{};
    const int vofa_error = vofa_init(&vofa, DEVICE_DT_GET(DT_NODELABEL(usart1)));
    int ret = bus.attach(drive);
    if (ret == 0) ret = bus.start();
    if (ret < 0) return ret;
    constexpr float current = -0.05f;
    std::int64_t next_log_ms = 0;
    int last_call_error = 0;
    for (;;) {
        const int enable_error = drive.enable();
        const int command_error = drive.setCurrent(current);
        const int commit_error = bus.commit().error;
        if (enable_error < 0) last_call_error = enable_error;
        if (command_error < 0) last_call_error = command_error;
        if (commit_error < 0) last_call_error = commit_error;
        const auto view = drive.snapshot();
        if (vofa_error == 0) {
            const auto &raw = view.native_dji_feedback;
            const float channels[7] = {float(view.feedback_fresh), current, float(raw.encoder), float(raw.speed_rpm),
                float(raw.current_raw), view.feedback.temperature_c, float(view.feedback.timestamp_ms)};
            (void)vofa_send(&vofa, channels, 7);
        }
        const auto now = k_uptime_get();
        if (now >= next_log_ms) {
            next_log_ms = now + 1000;
            LOG_INF("requested=%d state=%u fresh=%d current=%.3f call=%d", view.enabled_requested,
                unsigned(view.state), view.feedback_fresh, double(current), last_call_error);
        }
        k_sleep(K_MSEC(5));
    }
}
