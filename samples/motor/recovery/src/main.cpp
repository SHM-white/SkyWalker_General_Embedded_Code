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
    static motor::Motor drive{motor::dm::j4310Mit({
        .id = 1,
        .master_id = 0x11,
        .position_max_rad = 12.5f,
        .velocity_max_rad_s = 30.0f,
        .torque_max_nm = 10.0f,
        .torque_limit_nm = 0.5f,
        .timing = {50, 20, 50, 3000},
    })};
#else
    static motor::Motor drive{motor::dji::gm6020({
        .id = 1,
        .current_limit_a = 0.5f,
        .encoder_zero_ticks = 0,
        .current_mode_confirmed = true,
        .timing = {20, 20, 20, 100},
    })};
#endif
    static motor::CanBus bus{can};
    static control::VelocityMotor axis{drive, bench::motorConfig()};
    int configured = bus.attach(drive);
    if (configured == 0)
        configured = bus.start();
#if defined(SKYWALKER_RECOVERY_DM) && defined(CONFIG_BOARD_DM_MC02)
    if (configured == 0) {
        const device *power = DEVICE_DT_GET(DT_NODELABEL(power1));
        configured = device_is_ready(power) ? regulator_enable(power) : -ENODEV;
        if (configured == 0)
            k_sleep(K_MSEC(1500));
    }
#endif
    if (configured == 0)
        configured = axis.configure();
    LOG_INF("configure=%d family=%s; e enable, space disable, ! estop, r clear fault; power cycling requires a new e",
            configured, bench::dm ? "DM MIT" : "DJI");

    bool run = false, estop = false;
    auto previous_ms = k_uptime_get();
    std::uint64_t next_log = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt_s = float(now - previous_ms) / 1000.0f;
        previous_ms = now;
        unsigned char key;
        if (uart_poll_in(console, &key) == 0) {
            if (key == 'e' && !estop && configured == 0 && drive.ready()) {
                int ret = axis.reset();
                if (ret == 0)
                    ret = drive.enable();
                run = ret == 0;
                LOG_INF("enable=%d", ret);
            }
            if (key == ' ' || key == '!') {
                run = false;
                estop |= key == '!';
                (void)drive.disable();
            }
            if (key == 'r') {
                estop = false;
                run = false;
                const auto snapshot = drive.snapshot();
                if (snapshot.state == motor::MotorState::Fault)
                    LOG_INF("clearFault=%d", drive.clearFault());
            }
        }
        const auto snapshot = drive.snapshot();
        if (snapshot.state == motor::MotorState::Fault || snapshot.state == motor::MotorState::Offline)
            run = false;
        if (configured == 0 && run && drive.active()) {
            int ret = axis.update(bench::target_velocity_rad_s, dt_s);
            if (ret == 0)
                ret = bus.commit().error;
            if (ret < 0) {
                run = false;
                (void)drive.disable();
                LOG_ERR("cycle stopped: %d", ret);
            }
        }
        if (now >= next_log) {
            next_log = now + 250;
            const auto view = drive.snapshot();
            const auto telemetry = axis.telemetry();
            LOG_INF("uptime=%llu state=%u ready=%d run=%d gen=%llu fault=%u error=%d stop=%u velocity=%.3f effort=%.3f",
                    now, unsigned(view.state), drive.ready(), run, view.enable_generation,
                    unsigned(view.last_fault.reason), view.last_fault.error, unsigned(view.stop.progress),
                    double(view.feedback.velocity_rad_s), double(telemetry.effort_command));
        }
        k_sleep(K_MSEC(5));
    }
}
