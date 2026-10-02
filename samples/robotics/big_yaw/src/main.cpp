#include "../../common/chassis_can.hpp"
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/vehicle/big_yaw_profile.hpp>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
LOG_MODULE_REGISTER(big_yaw_bench, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;
int main() {
    // TODO(wiring): CAN2 is the reserved big-Yaw physical bus on the chassis.
    static motor::Motor drive(vehicle::bigYawHardware());
    static motor::CanBus bus(skywalker::samples::chassis::big_yaw_can);
    static BigYawExecutor axis(drive, vehicle::bigYawMotorConfig(), vehicle::bigYawExecutionConfig());
    bool started = false;
    int ret = 0;
    if (vehicle::connections_confirmed) {
        ret = bus.attach(drive);
        if (ret == 0) ret = bus.start();
#if defined(CONFIG_BOARD_DM_MC02)
        if (ret == 0) ret = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
#endif
        if (ret == 0) ret = axis.begin();
        started = ret == 0;
    }
    LOG_INF("a: arm, d: disable, +/-: direction, p: pause producer, s: pause execution, x: estop, r: clear; confirmed=%d setup=%d",
            vehicle::connections_confirmed, ret);
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    bool armed = false, producing = true, executing = true, stop = false;
    float target = 0.1f;
    std::uint32_t sequence = 0;
    std::uint64_t next_input = 0, next_log = 0;
    BigYawRequest request{};
    for (;;) {
        const auto now = core::monotonicTimeUs(), ms = now / 1000;
        bool clear = false;
        unsigned char key = 0;
        if (uart_poll_in(console, &key) == 0) {
            if (key == 'a') armed = true;
            if (key == 'd') armed = false;
            if (key == '+') target = 0.1f;
            if (key == '-') target = -0.1f;
            if (key == 'p') producing = !producing;
            if (key == 's') executing = !executing;
            if (key == 'x') { stop = true; armed = false; }
            if (key == 'r') { stop = false; clear = true; }
        }
        if (producing && ms >= next_input) {
            next_input = ms + 10;
            request.mode = armed ? BigYawMode::FollowCenter : BigYawMode::Disabled;
            request.target_rate_rad_s = target;
            request.stamp = {ms, ++sequence, true}; request.source_sequence = sequence;
            request.receiver_boot_id = 1; request.resume_generation = axis.status().generation;
            // Unloaded bench permission, produced in the same owner as the input.
            request.permission = {true, vehicle::connections_confirmed && armed, request.stamp};
        }
        if (executing) {
            if (started) {
                BigYawExecutionInputs in{};
                in.request = request; in.local_boot_id = 1; in.contract_compatible = true;
                in.transport_ready = bus.status().state == motor::BusState::Running;
                in.emergency_stop = stop; in.clear_fault = clear;
                axis.update(in, now);
                const int error = bus.commit().error;
                if (error < 0) axis.suspend(now, WaitReason::Transport, error);
            } else axis.suspend(now, WaitReason::Configuration, ret ? ret : -ENODEV, true);
        }
        if (ms >= next_log) {
            next_log = ms + 200;
            const auto f = axis.feedback(); const auto s = axis.status();
            LOG_INF("run=%u wait=%u gen=%u ready=%d active=%d target=%f actual=%f status_seq=%u error=%d",
                    unsigned(s.state), unsigned(s.reason), s.generation, s.ready, f.armed,
                    double(target), double(f.actual_rate_rad_s), s.stamp.sequence, s.error);
        }
        k_sleep(K_MSEC(5));
    }
}
