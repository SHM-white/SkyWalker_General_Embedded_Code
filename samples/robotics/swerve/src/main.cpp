#include <cerrno>
#include <cmath>
#include <cstdint>

#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
#include <robotics/swerve/swerve_kinematics.hpp>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_config.hpp"

LOG_MODULE_REGISTER(swerve_bench, LOG_LEVEL_INF);

int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;

    static motor::Motor steer(bench::steerHardware());
    static motor::Motor drive(bench::driveHardware());
    static motor::Group module_group(steer, drive);
    static motor::CanBus bus(bench::can);
    SwerveModule module(bench::moduleConfig());

    int ret = module.validate();
    if (ret == 0 && !(steer.info().capabilities & motor::FeedbackAbsolutePosition))
        ret = -ENOTSUP;
    if (ret == 0)
        ret = bus.attach(steer, drive);
    if (ret == 0)
        ret = bus.start();
    if (ret < 0) {
        LOG_ERR("configuration blocked: %d", ret);
        return ret;
    }

    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    LOG_INF("Single suspended module: e enable, space disable, w/s forward/reverse, a/d lateral, q rotate, ! estop, r reset");
    bool requested = false, estop = false, ready = false, enable_issued = false;
    std::uint64_t stable_since = 0, first_stamp = 0, next_log = 0;
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
            if (key == 'r') {
                module_group.disable();
                ret = module_group.clearFault();
                if (ret < 0)
                    LOG_ERR("group clear fault failed: %d", ret);
                estop = false;
                requested = ready = enable_issued = false;
                stable_since = 0; // A fresh feedback baseline and e are required.
            }
            if (key == 'w' || key == 's' || key == 'a' || key == 'd' || key == 'q') {
                command.vx_m_s = key == 'w' ? bench::test_speed_m_s : key == 's' ? -bench::test_speed_m_s : 0;
                command.vy_m_s = key == 'a' ? bench::test_speed_m_s : key == 'd' ? -bench::test_speed_m_s : 0;
                command.wz_rad_s = key == 'q' ? .5f : 0;
            }
        }

        const auto steer_view = steer.snapshot();
        const auto drive_view = drive.snapshot();
        const auto fresh = [&](const motor::MotorSnapshot &view) {
            const auto &f = view.feedback;
            return view.feedback_fresh && std::isfinite(f.position_rad) &&
                   std::isfinite(f.velocity_rad_s) && std::fabs(f.velocity_rad_s) < 40 &&
                   (!(f.valid & motor::FeedbackTemperature) ||
                    (std::isfinite(f.temperature_c) && f.temperature_c < 70));
        };
        const bool healthy = fresh(steer_view) && fresh(drive_view) &&
                             (steer_view.feedback.valid & motor::FeedbackAbsolutePosition) &&
                             std::isfinite(steer_view.feedback.absolute_position_rad);

        const auto group_status = module_group.status();
        if (estop || !healthy || !requested) {
            if (group_status.active || group_status.enable_pending) {
                module_group.disable();
                ready = false;
                stable_since = 0;
                enable_issued = false;
            }
            if (!healthy || estop) {
                requested = false; // Recovery never re-enables without another e.
                ready = false;
                stable_since = 0;
                enable_issued = false;
            }
        }
        else if (enable_issued && !group_status.active && !group_status.enable_pending) {
            requested = false; // Internal fault or failed handshake.
            ready = false;
            stable_since = 0;
            enable_issued = false;
        }

        if (healthy && !ready && !group_status.active && !group_status.enable_pending &&
            module_group.ready()) {
            if (stable_since == 0) {
                stable_since = now;
                first_stamp = steer_view.feedback.timestamp_ms;
            }
            else if (now - stable_since >= 30 &&
                     steer_view.feedback.timestamp_ms != first_stamp) {
                ret = steer.reseedPosition(steer_view.feedback.absolute_position_rad);
                if (ret == 0)
                    ret = drive.reseedPosition(0.0);
                if (ret == 0) {
                    const auto sf = steer.snapshot().feedback;
                    const auto df = drive.snapshot().feedback;
                    feedback = {bench::steer_direction * sf.absolute_position_rad,
                                bench::steer_direction * sf.position_rad,
                                bench::steer_direction * sf.velocity_rad_s,
                                bench::drive_direction * df.velocity_rad_s};
                    for (auto &target : targets)
                        target.angle_rad = feedback.steer_absolute_rad;
                    kinematics.reset(targets);
                    ret = module.reset(feedback);
                    ready = ret == 0;
                }
            }
        }

        if (requested && !estop && ready && healthy && !enable_issued &&
            module_group.ready()) {
            ret = module_group.enable();
            enable_issued = ret == 0;
            if (ret < 0)
                requested = false;
        }

        if (requested && module_group.active()) {
            const auto sf = steer.snapshot().feedback;
            const auto df = drive.snapshot().feedback;
            feedback = {bench::steer_direction * sf.absolute_position_rad,
                        bench::steer_direction * sf.position_rad,
                        bench::steer_direction * sf.velocity_rad_s,
                        bench::drive_direction * df.velocity_rad_s};
            ret = kinematics.solve(command, targets);
            if (ret == 0)
                ret = module.step(targets[0], feedback, dt, output);
            if (ret == 0)
                ret = steer.setCurrent(bench::steer_direction * output.steer_effort);
            if (ret == 0)
                ret = drive.setCurrent(bench::drive_direction * output.drive_effort);
            if (ret == 0)
                ret = bus.commit().error;
            if (ret < 0) {
                module_group.disable();
                requested = false;
                enable_issued = false;
                ready = false;
                stable_since = 0;
            }
        }

        if (now >= next_log) {
            next_log = now + 100;
            const auto status = module_group.status();
            LOG_INF("active=%d pending=%d ready=%d feedback=%d err=%d bus=%u target=%.3f angle=%.3f wheel=%.3f drive=%.3f current=%.3f/%.3f",
                    status.active, status.enable_pending, ready, healthy, ret, unsigned(bus.status().state),
                    double(output.optimized_angle_rad), double(feedback.steer_absolute_rad),
                    double(output.optimized_wheel_velocity_m_s), double(feedback.drive_velocity_rad_s),
                    double(output.steer_effort), double(output.drive_effort));
        }
        k_sleep(K_MSEC(5));
    }
}
