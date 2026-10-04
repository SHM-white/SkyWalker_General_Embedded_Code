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
    if (!device_is_ready(can) || !device_is_ready(console)) {
        LOG_ERR("CAN or console device not ready");
        return -ENODEV;
    }
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
    static motor::Motor drive{motor::dji::m2006({
        .id = 1,
        .current_limit_a = 0.5f,
        .gear_ratio = 36.0f,
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
            configured, bench::dm ? "DM MIT" : "M2006/C610 CAN1 ID1");
    if (configured < 0)
        return configured;
    LOG_INF("Console=%s; state: 0 offline, 1 disabled, 2 enabling, 3 active, 4 fault",
            console->name);

    bool run = false, estop = false;
    auto previous_ms = k_uptime_get();
    std::uint64_t next_log = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt_s = float(now - previous_ms) / 1000.0f;
        previous_ms = now;
        unsigned char key;
        if (uart_poll_in(console, &key) == 0) {
            if (key == 'e') {
                if (estop || !drive.ready()) {
                    LOG_WRN("enable blocked: estop=%d ready=%d; r clears estop/fault, then wait for ready=1",
                            estop, drive.ready());
                } else {
                    int ret = axis.reset();
                    if (ret == 0)
                        ret = drive.enable();
                    run = ret == 0;
                    LOG_INF("enable=%d", ret);
                }
            }
            if (key == ' ' || key == '!') {
                run = false;
                estop |= key == '!';
                const int ret = drive.disable();
                LOG_INF("disable=%d estop=%d", ret, estop);
            }
            if (key == 'r') {
                estop = false;
                run = false;
                // Clearing the local run flag alone leaves the old command
                // permitted until its deadline. Request safe output now.
                const int ret = drive.disable();
                LOG_INF("reset: disable=%d; press e again after ready=1", ret);
                const auto snapshot = drive.snapshot();
                if (snapshot.state == motor::MotorState::Fault)
                    LOG_INF("clearFault=%d", drive.clearFault());
            }
        }
        const auto snapshot = drive.snapshot();
        if (snapshot.state != motor::MotorState::Active && snapshot.state != motor::MotorState::Enabling)
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
