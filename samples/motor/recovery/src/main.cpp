#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <control/dji_motor_backend.hpp>
#include <control/dm_motor_backend.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(recovery_bench, LOG_LEVEL_INF);
int main() {
    using namespace skywalker::control;
    static DjiMotorBackend dji(bench::motor);
    static DmMotorBackend dm(bench::motor);
    MotorBackend &backend = bench::dm ? static_cast<MotorBackend &>(dm) : static_cast<MotorBackend &>(dji);
    static VelocityMotor motor(backend, bench::motorConfig());
    const int configured = motor.configure();
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    LOG_INF(
        "configure=%d family=%s. e run, space pause, ! estop, r reset. Cut ONLY motor power to exercise auto-recovery; MCU stays powered.",
        configured, bench::dm ? "DM MIT" : "DJI");
    bool run = false, estop = false;
    std::uint64_t next_log = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        unsigned char key;
        if (uart_poll_in(console, &key) == 0) {
            if (key == 'e' && !estop)
                run = true;
            if (key == ' ' || key == '!') {
                run = false;
                estop |= key == '!';
                motor.suspend(estop ? PauseReason::EmergencyStop : PauseReason::OperatorDisabled);
            }
            if (key == 'r') {
                estop = false;
                motor.clearEmergencyStop(true);
            } // Reset alone never starts motion.
        }
        if (configured == 0 && !estop) {
            if (motor.state() != ExecutionState::Active) {
                if (motor.poll(now) == 0 && run)
                    motor.resume();
            }
            else if (run)
                motor.update(bench::target_velocity_rad_s);
            else
                motor.suspend(PauseReason::OperatorDisabled);
        }
        if (now >= next_log) {
            next_log = now + 250;
            const auto &s = motor.status();
            const auto &t = motor.telemetry();
            LOG_INF(
                "uptime=%llu state=%u run=%d gen=%u attempts=%u error=%d recovery=%d zero_error=%d velocity=%.3f effort=%.3f",
                now, unsigned(motor.state()), run, s.resume_generation, s.recovery_attempts, s.error,
                s.last_recovery_error, s.stop_error, double(t.measurement.feedback.velocity_rad_s),
                double(t.output.effort_command));
        }
        k_sleep(K_MSEC(5));
    }
}
