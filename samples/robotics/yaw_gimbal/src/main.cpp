#include <cstdint>

#include <control/position_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/gimbal/yaw_gimbal.hpp>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_config.hpp"

LOG_MODULE_REGISTER(yaw_bench, LOG_LEVEL_INF);

int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;

    static motor::Motor drive(bench::motorHardware());
    static motor::CanBus bus(bench::can);
    static control::PositionMotor axis(drive, bench::motorConfig());
    YawGimbal yaw(drive, axis, bench::yaw);

    int ret = bus.attach(drive);
    if (ret == 0)
        ret = bus.start();
    if (ret == 0)
        ret = yaw.begin();
    if (ret < 0) {
        LOG_ERR("configuration blocked: %d", ret);
        return ret;
    }

    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    LOG_INF("e enable Hold, a/d +/- rate, h Hold, 0/1 absolute 0/0.5rad, space Disable, ! estop, r reset");
    bool requested = false, estop = false, enable_issued = false;
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
                requested = true;
                command.mode = GimbalMode::Hold;
            }
            if (key == ' ' || key == '!') {
                requested = false;
                estop |= key == '!';
            }
            if (key == 'r') {
                estop = false;
                if (drive.snapshot().state == motor::MotorState::Fault) {
                    const int clear = drive.clearFault();
                    if (clear < 0)
                        LOG_ERR("clear fault: %d", clear);
                }
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

        const auto view = drive.snapshot();
        if (!requested || estop) {
            if (view.state == motor::MotorState::Active ||
                view.state == motor::MotorState::Enabling) {
                ret = drive.disable();
                if (ret < 0)
                    LOG_ERR("disable: %d", ret);
            }
            enable_issued = false;
        }
        else if (enable_issued && view.state != motor::MotorState::Active &&
                 view.state != motor::MotorState::Enabling) {
            requested = false; // A fault or lost feedback needs another e.
            enable_issued = false;
        }

        if (requested && !estop && !enable_issued && drive.ready()) {
            ret = yaw.reset(); // Seed Hold from a fresh, disabled measurement.
            if (ret == 0)
                ret = drive.enable();
            if (ret == 0)
                enable_issued = true;
            else {
                requested = false;
                LOG_ERR("enable: %d", ret);
            }
        }
        if (requested && drive.active()) {
            ret = yaw.update(command, SafetyAction::Active, dt);
            if (ret == 0)
                ret = bus.commit().error;
            if (ret < 0) {
                (void)drive.disable();
                requested = false;
                enable_issued = false;
                LOG_ERR("control update: %d", ret);
            }
        }

        if (now >= next_log) {
            next_log = now + 100;
            const auto snapshot = drive.snapshot();
            const auto telemetry = axis.telemetry();
            LOG_INF("state=%u mode=%u gen=%llu target=%.3f absolute=%.3f velocity=%.3f effort=%.3f fault=%u err=%d",
                    unsigned(snapshot.state), unsigned(command.mode),
                    static_cast<unsigned long long>(snapshot.enable_generation),
                    double(yaw.targetAngleRad()), double(snapshot.feedback.absolute_position_rad),
                    double(snapshot.feedback.velocity_rad_s), double(telemetry.effort_command),
                    unsigned(snapshot.last_fault.reason), bus.status().last_error);
        }
        k_sleep(K_MSEC(5));
    }
}
