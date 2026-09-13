#include <drivers/motor/motor.hpp>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <control/can_recovery.hpp>
#include <drivers/motor/dji_bus.hpp>
#include <robotics/swerve/swerve_kinematics.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(swerve_bench, LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    static motor::dji::Bus bus;
    motor::dji::FlushReport report{};
    motor::dji::Descriptor steer{}, drive{};
    SwerveModule module(bench::moduleConfig());
    int ret = module.validate();
    if (ret == 0)
        ret = motor::dji::describe(bench::steer, steer);
    if (ret == 0)
        ret = motor::dji::describe(bench::drive, drive);
    if (ret == 0 && (steer.can != drive.can || !(motor::capabilities(bench::steer) & motor::FeedbackAbsolutePosition)))
        ret = -ENOTSUP;
    if (ret == 0)
        ret = bus.init(steer.can);
    if (ret == 0)
        ret = bus.attach(bench::steer);
    if (ret == 0)
        ret = bus.attach(bench::drive);
    if (ret < 0) {
        LOG_ERR("configuration blocked: %d", ret);
        return ret;
    }
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    LOG_INF(
        "Single physical module: e enable, space disable, w forward, s reverse, a left, d right, q rotation, ! estop, r reset. Suspend wheels before e.");
    bool requested = false, estop = false, ready = false;
    std::uint64_t next_probe = 0, stable_since = 0, next_log = 0, first_stamp = 0;
    ChassisCommand command{};
    command.mode = ChassisMode::BodyVelocity;
    SwerveKinematics kinematics({{{{.2f, .2f}, {.2f, -.2f}, {-.2f, .2f}, {-.2f, -.2f}}}, .3f, .01f});
    ModuleTargets targets{};
    ModuleFeedback feedback{};
    ModuleOutput output{};
    auto previous = k_uptime_get();
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt = (now - previous) / 1000.0f;
        previous = now;
        unsigned char key;
        if (uart_poll_in(console, &key) == 0) {
            if (key == 'e' && !estop)
                requested = true;
            if (key == ' ' || key == '!') {
                requested = false;
                estop |= key == '!';
            }
            if (key == 'r')
                estop = false;
            if (key == 'w' || key == 's' || key == 'a' || key == 'd' || key == 'q') {
                command.vx_m_s = key == 'w' ? bench::test_speed_m_s : key == 's' ? -bench::test_speed_m_s : 0;
                command.vy_m_s = key == 'a' ? bench::test_speed_m_s : key == 'd' ? -bench::test_speed_m_s : 0;
                command.wz_rad_s = key == 'q' ? .5f : 0;
            }
        }
        motor::Feedback sf{}, df{};
        motor::readFeedback(bench::steer, sf);
        motor::readFeedback(bench::drive, df);
        auto fresh = [&](const motor::Feedback &f) {
            return f.timestamp_ms && now >= f.timestamp_ms &&
                   now - f.timestamp_ms <= CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS && std::isfinite(f.position_rad) &&
                   std::isfinite(f.velocity_rad_s) && std::fabs(f.velocity_rad_s) < 40 &&
                   (!(f.valid & motor::FeedbackTemperature) ||
                    (std::isfinite(f.temperature_c) && f.temperature_c < 70));
        };
        bool healthy = fresh(sf) && fresh(df) && std::isfinite(sf.absolute_position_rad) &&
                       (sf.valid & motor::FeedbackAbsolutePosition);
        if ((!requested || !healthy) && bus.state() == motor::dji::BusState::Armed) {
            bus.stop(report);
            ready = false;
            stable_since = 0;
        }
        if (bus.state() != motor::dji::BusState::Armed) {
            if (now >= next_probe) {
                next_probe = now + 100;
                ret = control::pollCanRecovery(steer.can);
                if (ret == 0 && bus.state() == motor::dji::BusState::Fault)
                    ret = bus.recover(report);
                if (ret < 0 || !healthy) {
                    stable_since = 0;
                    ready = false;
                }
            }
            if (healthy && !ready && bus.state() == motor::dji::BusState::Safe) {
                if (!stable_since) {
                    stable_since = now;
                    first_stamp = sf.timestamp_ms;
                }
                else if (now - stable_since >= 30 && sf.timestamp_ms != first_stamp) {
                    ret = motor::dji::resetMeasurementReference(bench::steer);
                    if (ret == 0)
                        ret = motor::dji::resetMeasurementReference(bench::drive);
                    if (ret == 0) {
                        motor::readFeedback(bench::steer, sf);
                        motor::readFeedback(bench::drive, df);
                        feedback = {bench::steer_direction * sf.absolute_position_rad,
                                    bench::steer_direction * sf.position_rad,
                                    bench::steer_direction * sf.velocity_rad_s,
                                    bench::drive_direction * df.velocity_rad_s};
                        for (auto &t : targets)
                            t.angle_rad = feedback.steer_absolute_rad;
                        kinematics.reset(targets);
                        ret = module.reset(feedback);
                        ready = ret == 0;
                    }
                }
            }
            if (requested && !estop && ready && healthy) {
                ret = bus.arm(report);
                if (ret < 0)
                    ready = false;
                k_sleep(K_MSEC(5));
                continue;
            }
        }
        else {
            feedback = {bench::steer_direction * sf.absolute_position_rad, bench::steer_direction * sf.position_rad,
                        bench::steer_direction * sf.velocity_rad_s, bench::drive_direction * df.velocity_rad_s};
            ret = kinematics.solve(command, targets);
            if (ret == 0)
                ret = module.step(targets[0], feedback, dt, output);
            if (ret == 0)
                ret = motor::setCurrent(bench::steer, bench::steer_direction * output.steer_effort);
            if (ret == 0)
                ret = motor::setCurrent(bench::drive, bench::drive_direction * output.drive_effort);
            if (ret == 0)
                ret = bus.flush(report);
            if (ret < 0) {
                bus.stop(report);
                ready = false;
                stable_since = 0;
            }
        }
        if (now >= next_log) {
            next_log = now + 100;
            LOG_INF(
                "armed=%d ready=%d feedback=%d err=%d target_angle=%.3f angle=%.3f wheel_m_s=%.3f drive_rad_s=%.3f currents=%.3f/%.3f",
                bus.state() == motor::dji::BusState::Armed, ready, healthy, ret, double(output.optimized_angle_rad),
                double(feedback.steer_absolute_rad), double(output.optimized_wheel_velocity_m_s),
                double(feedback.drive_velocity_rad_s), double(output.steer_effort), double(output.drive_effort));
        }
        k_sleep(K_MSEC(5));
    }
}
