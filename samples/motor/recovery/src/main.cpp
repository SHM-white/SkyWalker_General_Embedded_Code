#include <cerrno>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(SKYWALKER_RECOVERY_DM) && defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
#include <drivers/motor/can_bus.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(recovery_bench, LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(can) || !device_is_ready(console))
        return -ENODEV;
#ifdef SKYWALKER_RECOVERY_DM
    static motor::Motor drive{motor::dm::j4310Mit({.id = 1,
                                                   .master_id = 0x11,
                                                   .position_max_rad = 12.5f,
                                                   .velocity_max_rad_s = 30,
                                                   .torque_max_nm = 10,
                                                   .torque_limit_nm = 0.5f,
                                                   .timing = {.feedback_timeout_ms = 50,
                                                              .command_timeout_ms = 20,
                                                              .enable_timeout_ms = 3000,
                                                              .retry_interval_ms = 100}})};
#else
    static motor::Motor drive{motor::dji::gm6020({.id = 3,
                                                  .current_limit_a = 0.5f,
                                                  .encoder_zero_ticks = 0,
                                                  .current_mode_confirmed = true,
                                                  //   .gear_ratio = 36,
                                                  .timing = {.feedback_timeout_ms = 20,
                                                             .command_timeout_ms = 20,
                                                             .enable_timeout_ms = 100,
                                                             .retry_interval_ms = 100}})};
#endif
    static motor::CanBus bus{can};
    static control::VelocityMotor axis{drive, bench::motorConfig()};
    int ret = bus.attach(drive);
    if (ret == 0)
        ret = bus.start();
#if defined(SKYWALKER_RECOVERY_DM) && defined(CONFIG_BOARD_DM_MC02)
    if (ret == 0) {
        const device *power = DEVICE_DT_GET(DT_NODELABEL(power1));
        ret = device_is_ready(power) ? regulator_enable(power) : -ENODEV;
    }
#endif
    if (ret == 0)
        ret = axis.configure();
    if (ret < 0)
        return ret;
    LOG_INF("e run; space stop; ! estop; r release estop. Power recovery resumes automatically.");
    bool run_requested = false, estop = false;
    auto previous_ms = k_uptime_get();
    std::uint64_t next_log = 0;
    int last_call_error = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt = float(now - previous_ms) / 1000;
        previous_ms = now;
        unsigned char key;
        while (uart_poll_in(console, &key) == 0) {
            if (key == 'e' && !estop)
                run_requested = true;
            if (key == ' ' || key == '!') {
                run_requested = false;
                estop |= key == '!';
            }
            if (key == 'r') {
                estop = false;
                run_requested = false;
            }
        }
        const int request_error = run_requested && !estop ? drive.enable() : drive.disable();
        if (request_error < 0)
            last_call_error = request_error;
        if (run_requested && !estop) {
            const int update_error = axis.update(bench::target_velocity_rad_s, dt);
            if (update_error < 0)
                last_call_error = update_error;
        }
        const int commit_error = bus.commit().error;
        if (commit_error < 0)
            last_call_error = commit_error;
        if (now >= next_log) {
            next_log = now + 250;
            const auto view = drive.snapshot();
            const auto data = axis.telemetry();
            LOG_INF(
                "run=%d estop=%d requested=%d state=%u target=%.3f seq=%llu output=%d wait=%u age=%llu velocity=%.3f effort=%.3f call=%d fault=%u/%d",
                run_requested, estop, view.enabled_requested, unsigned(view.state), double(data.target_rad_s),
                static_cast<unsigned long long>(data.target_sequence), data.output_valid && run_requested,
                unsigned(data.issue),
                static_cast<unsigned long long>(now >= view.feedback.timestamp_ms ? now - view.feedback.timestamp_ms
                                                                                  : 0),
                double(view.feedback.velocity_rad_s), double(data.effort_command), last_call_error,
                unsigned(view.last_fault.reason), view.last_fault.error);
        }
        k_sleep(K_MSEC(5));
    }
}
