#pragma once

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/chassis/chassis_executor.hpp>
#include <robotics/vehicle/swerve_profile.hpp>
#include "chassis_can.hpp"
#include "chassis_period.hpp"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_UART_TRANSPORT)
#include <communication/referee/referee_receiver.hpp>
#endif

namespace skywalker::samples::chassis {
LOG_MODULE_REGISTER(chassis_bench, LOG_LEVEL_INF);
namespace calibration = robotics::vehicle;

inline motor::dji::Config steerConfig(std::size_t index) {
    const auto &c = calibration::steer[index];
    return motor::dji::gm6020({.id = c.id, .current_limit_a = c.effort_limit,
        .encoder_zero_ticks = c.encoder_zero_ticks, .current_mode_confirmed = calibration::connections_confirmed,
        .timing = {20, 20, 30, 100}});
}
inline motor::dji::Config driveConfig(std::size_t index) {
    const auto &c = calibration::wheel[index];
    return motor::dji::m3508({.id = c.id, .current_limit_a = c.effort_limit,
        .gear_ratio = c.gear_ratio, .timing = {20, 20, 30, 100}});
}
inline robotics::SwerveHardware::Config hardwareConfig() {
    robotics::SwerveHardware::Config c{};
    c.hardware_confirmed = calibration::connections_confirmed;
    // TODO(calibration): tune overspeed limits on the real suspended chassis.
    c.velocity_safety_rad_s = 80;
    for (std::size_t i = 0; i < 4; ++i) {
        c.directions[i] = calibration::steer[i].direction;
        c.directions[i + 4] = calibration::wheel[i].direction;
    }
    return c;
}
inline robotics::SwerveChassis::Config controllerConfig() {
    return calibration::swerveConfig();
}
inline robotics::ChassisExecutor::Config executorConfig(bool power_budget) {
    robotics::ChassisExecutor::Config c{};
    c.require_power_budget = power_budget;
    c.power_model_calibrated = calibration::power_model_calibrated;
    // TODO(power): complete measured-current/voltage model identification and
    // mechanical tracking acceptance before opening full power control.
    c.power_control_calibrated = false;
    c.allow_estimated_power = false;
    c.estimate_idle_power_w = calibration::power_idle_w;
    c.estimate_w_per_abs_amp = calibration::power_per_abs_amp_w;
    c.bench_effort_scale = calibration::chassis_drive_scale;
    c.bench_steer_effort_scale = calibration::chassis_steer_scale;
    c.bench_drive_current_limit_a = calibration::drive_bench_current_a;
    c.bench_steer_current_limit_a = calibration::steer_bench_current_a;
    return c;
}

// Physical resources are owned by the application, outside the executor.
class Hardware {
public:
    motor::Motor s0{steerConfig(0)}, s1{steerConfig(1)}, s2{steerConfig(2)}, s3{steerConfig(3)};
    motor::Motor d0{driveConfig(0)}, d1{driveConfig(1)}, d2{driveConfig(2)}, d3{driveConfig(3)};
    motor::Group group{s0, s1, s2, s3, d0, d1, d2, d3};
    motor::CanBus steer_bus, drive_bus;
    robotics::SwerveHardware adapter;
    Hardware(const device *steer_can, const device *drive_can)
        : steer_bus(steer_can), drive_bus(drive_can),
          adapter({{&s0, &s1, &s2, &s3, &d0, &d1, &d2, &d3}}, group, hardwareConfig()),
          steer_can_(steer_can), drive_can_(drive_can) {}
    int start() {
        if (!calibration::connections_confirmed) return -ENODEV;
        if (!steer_can_ || !drive_can_ || steer_can_ == drive_can_ ||
            !device_is_ready(steer_can_) || !device_is_ready(drive_can_)) return -ENODEV;
        if (!attached_) {
            int ret = steer_bus.attach(s0, s1, s2, s3);
            if (ret < 0) return ret;
            ret = drive_bus.attach(d0, d1, d2, d3);
            if (ret < 0) return ret;
            attached_ = true;
        }
        if (!steer_started_) {
            const int ret = steer_bus.start();
            if (ret < 0) return ret;
            steer_started_ = true;
        }
        if (!drive_started_) {
            const int ret = drive_bus.start();
            if (ret < 0) return ret;
            drive_started_ = true;
        }
        return 0;
    }
    bool running() const {
        return steer_started_ && drive_started_ && steer_bus.status().state == motor::BusState::Running &&
               drive_bus.status().state == motor::BusState::Running;
    }
    int commit() {
        // Always submit both buses in the same application cycle. Group
        // withdrawal provides joint stop even if just one bus commit fails.
        const int steer_error = steer_started_ ? steer_bus.commit().error : -EACCES;
        const int drive_error = drive_started_ ? drive_bus.commit().error : -EACCES;
        return steer_error < 0 ? steer_error : drive_error;
    }
private:
    const device *steer_can_, *drive_can_;
    bool attached_ = false, steer_started_ = false, drive_started_ = false;
};

inline int run(const device *steer_can, const device *drive_can, bool with_power,
               const device *referee_uart = nullptr, robotics::IPowerMeasurementSource *power_source = nullptr) {
    using namespace robotics;
    static Hardware hardware(steer_can, drive_can);
    ChassisExecutor executor(hardware.adapter, controllerConfig(), executorConfig(with_power));
    const int topology_error = hardware.start();
    const int config_error = executor.begin();
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    bool requested = false, source_paused = false, execution_paused = false, measurement_paused = false, estop = false;
    std::uint32_t source_sequence = 0;
    core::TimeUs next_source = 0, next_log = 0, lease_until_us = 0;
    PeriodicDeadline deadline;
    ChassisExecutionInputs inputs{};
    RunStatus status{};
    PowerMeasurement measurement{};
    RefereeState referee{};
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_UART_TRANSPORT)
    static communication::AsyncUart::DmaBuffers referee_dma __nocache;
    static communication::RefereeReceiver receiver(referee_uart, referee_dma, communication::RefereeVersion::Rm2026V1_3);
#endif
    printk("8 motors FL/FR/RL/RR: steer CAN1, drive CAN3; topology=%d config=%d\n", topology_error, config_error);
    printk("e enable, space disable, w/s x, a/d y, q/z yaw, u/j x+yaw, 0 coast, ! estop, r clear; p source pause, x execution pause, m measurement pause\n");
    for (;;) {
        const auto now_us = core::monotonicTimeUs();
        const auto now_ms = now_us / 1000;
        bool clear = false;
        unsigned char key = 0;
        while (uart_poll_in(console, &key) == 0) {
            if (key == 'e' && !estop) {
                requested = true;
                inputs.command.vx_m_s = inputs.command.vy_m_s = inputs.command.wz_rad_s = 0;
                lease_until_us = now_us + calibration::bench_command_lease_us;
            }
            if (key == ' ' || key == '!') { requested = false; estop = estop || key == '!'; }
            if (key == 'r') { clear = true; estop = false; requested = false; }
            if (key == 'p') source_paused = !source_paused;
            if (key == 'x') execution_paused = !execution_paused;
            if (key == 'm') measurement_paused = !measurement_paused;
            if (key == 'w' || key == 's' || key == 'a' || key == 'd' || key == 'q' || key == 'z' || key == 'u' || key == 'j' || key == '0') {
                inputs.command.vx_m_s = (key == 'w' || key == 'u' || key == 'j') ? .1f : key == 's' ? -.1f : 0;
                inputs.command.vy_m_s = key == 'a' ? .1f : key == 'd' ? -.1f : 0;
                inputs.command.wz_rad_s = (key == 'q' || key == 'u') ? .2f : (key == 'z' || key == 'j') ? -.2f : 0;
                lease_until_us = now_us + calibration::bench_command_lease_us;
            }
        }
        // Synthetic source production is not evidence of operator presence.
        // Expiry withdraws immediately, even while source production is paused.
        if (requested && now_us >= lease_until_us) requested = false;
        if (!requested) {
            inputs.command.mode = ChassisMode::Disabled;
            inputs.command.vx_m_s = inputs.command.vy_m_s = inputs.command.wz_rad_s = 0;
        }
        if (!source_paused && now_us >= next_source) {
            next_source = now_us + 10000;
            inputs.command.mode = requested ? ChassisMode::BodyVelocity : ChassisMode::Disabled;
            inputs.command.source = ControlSource::Autonomous;
            inputs.command.stamp = {now_ms, ++source_sequence, true};
            inputs.source_stamp = {now_us, source_sequence, true};
        }
        inputs.emergency_stop = estop;
        inputs.clear_fault = clear;
        inputs.transport_ready = topology_error == 0 && hardware.running();
        inputs.require_permission = with_power;
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_UART_TRANSPORT)
        if (with_power && referee_uart && device_is_ready(referee_uart)) referee = receiver.poll(now_ms);
#endif
        inputs.permission = referee.robot.chassis_output;
        inputs.power_budget = referee.power;
        if (!measurement_paused) {
            if (power_source) {
                PowerMeasurement next{};
                const int ret = power_source->sample(now_us, next);
                if (ret == 0) measurement = next;
                else if (ret != -EAGAIN) measurement = {};
            } else {
                // Referee power is a real measured value. Its own production
                // stamp is retained; polling never renews power or budget age.
                measurement = {referee.power.chassis_power_w, 0, 0, referee.power.stamp.valid, false,
                    PowerMeasurementSource::Referee,
                    {referee.power.stamp.timestamp_ms * 1000, referee.power.stamp.sequence, referee.power.stamp.valid}};
                // TODO(power-sensor): supply IPowerMeasurementSource backed by
                // installed ADC/power monitor for independent current+voltage.
            }
        }
        inputs.measured_power = measurement;
        if (!execution_paused || clear || estop || !requested) {
            status = executor.update(inputs, now_us);
            if (topology_error == 0) {
                const int commit = hardware.commit();
                if (commit < 0) status = executor.suspend(now_us, WaitReason::Transport, commit);
            }
        }
        if (now_us >= next_log) {
            next_log = now_us + 500000;
            const bool status_fresh = isFresh(status.stamp, now_ms, 100);
            LOG_INF("src=%u age_ms=%llu exec=%u fresh=%d ready=%d reason=%u err=%d gen=%u power_valid=%d scale_milli=%d power_mW=%d budget_mW=%d bus=%u/%u",
                source_sequence, static_cast<unsigned long long>(now_ms >= inputs.command.stamp.timestamp_ms ?
                now_ms - inputs.command.stamp.timestamp_ms : 0), unsigned(status.state), status_fresh,
                status_fresh && status.ready, unsigned(status.reason), status.error, status.generation,
                measurement.valid && core::fresh(measurement.stamp, now_us, 300000),
                int(executor.effortScale() * 1000), int(measurement.power_w * 1000),
                int(referee.power.chassis_power_limit_w * 1000), unsigned(hardware.steer_bus.status().state),
                unsigned(hardware.drive_bus.status().state));
            const auto &feedback = executor.feedback();
            const auto &output = executor.output();
            for (std::size_t i = 0; i < 4; ++i)
                LOG_INF("wheel=%u steer_mrad=%d final_mrad=%d ramp_mrad=%d error_mrad=%d ready=%d drive_on=%d flip=%d coast=%d drive_mrad_s=%d target_mrad_s=%d current_mA=%d/%d scale_milli=%d/%d", unsigned(i),
                    int(feedback.module[i].steer_absolute_rad * 1000), int(output.module[i].optimized_angle_rad * 1000),
                    int(output.module[i].steer_reference_rad * 1000), int(output.module[i].alignment_error_rad * 1000),
                    output.module[i].drive_ready, output.module[i].drive_enabled, output.module[i].flipped,
                    output.module[i].coasting, int(feedback.module[i].drive_velocity_rad_s * 1000),
                    int(output.module[i].drive_target_rad_s * 1000),
                    int(output.module[i].steer_effort * executor.steerEffortScale() * 1000),
                    int(output.module[i].drive_effort * executor.effortScale() * 1000),
                    int(executor.steerEffortScale() * 1000), int(executor.effortScale() * 1000));
        }
        deadline.wait();
    }
}
} // namespace skywalker::samples::chassis
