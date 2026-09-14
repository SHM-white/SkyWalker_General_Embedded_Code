#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <control/dji_motor_backend.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(yaw_bench, LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    static control::DjiMotorBackend backend(bench::motor);
    static control::PositionMotor motor(backend, bench::motorConfig());
    YawGimbal yaw(motor, bench::yaw);
    const int configured = yaw.begin();
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    LOG_INF(
        "yaw configure=%d. e enable Hold, a/d +/- rate, h Hold, 0/1 absolute 0/0.5rad, space Disable, ! estop, r reset",
        configured);
    bool enabled = false, estop = false;
    GimbalCommand command{};
    command.mode = GimbalMode::Hold;
    std::uint64_t next_log = 0;
    auto previous = k_uptime_get();
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt = (now - previous) / 1000.0f;
        previous = now;
        unsigned char key;
        if (uart_poll_in(console, &key) == 0) {
            if (key == 'e' && !estop) {
                enabled = true;
                command.mode = GimbalMode::Hold;
            }
            if (key == ' ' || key == '!') {
                enabled = false;
                estop |= key == '!';
                yaw.suspend(estop ? PauseReason::EmergencyStop : PauseReason::OperatorDisabled);
            }
            if (key == 'r') {
                estop = false;
                yaw.clearEmergencyStop(true);
            }
            if (key == 'a' || key == 'd') {
                command.mode = GimbalMode::Rate;
                command.yaw_rate_rad_s = key == 'a' ? .3f : -.3f;
            }
            if (key == 'h')
                command.mode = GimbalMode::Hold;
            if (key == '0' || key == '1') {
                command.mode = GimbalMode::AbsoluteAngle;
                command.yaw_target_rad = key == '0' ? 0 : .5f;
            }
        }
        if (configured == 0 && !estop) {
            if (motor.state() != ExecutionState::Active)
                yaw.poll(now);
            if (enabled)
                yaw.update(command, SafetyAction::Active, dt);
        }
        if (now >= next_log) {
            next_log = now + 100;
            const auto &t = motor.telemetry();
            LOG_INF("state=%u mode=%u gen=%u target=%.3f absolute=%.3f velocity=%.3f effort=%.3f error=%d",
                    unsigned(motor.state()), unsigned(command.mode), motor.status().resume_generation,
                    yaw.targetAngleRad(), double(t.measurement.feedback.absolute_position_rad),
                    double(t.measurement.feedback.velocity_rad_s), double(t.output.effort_command),
                    motor.status().last_recovery_error);
        }
        k_sleep(K_MSEC(5));
    }
}
