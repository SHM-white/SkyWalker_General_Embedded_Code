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
#include "rc_controls.hpp"
#include "sample_diagnostics.hpp"
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
    return motor::dji::gm6020({.id = c.id,
                               .current_limit_a = c.effort_limit,
                               .encoder_zero_ticks = c.encoder_zero_ticks,
                               .current_mode_confirmed = calibration::connections_confirmed,
                               .timing = {.feedback_timeout_ms = 20,
                                          .command_timeout_ms = 20,
                                          .enable_timeout_ms = 100,
                                          .retry_interval_ms = 100}});
}
inline motor::dji::Config driveConfig(std::size_t index) {
    const auto &c = calibration::wheel[index];
    return motor::dji::m3508({.id = c.id,
                              .current_limit_a = c.effort_limit,
                              .gear_ratio = c.gear_ratio,
                              .timing = {.feedback_timeout_ms = 20,
                                         .command_timeout_ms = 20,
                                         .enable_timeout_ms = 100,
                                         .retry_interval_ms = 100}});
}
inline robotics::SwerveHardware::Config hardwareConfig() {
    robotics::SwerveHardware::Config c{};
    c.hardware_confirmed = calibration::connections_confirmed;
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
          adapter({{&s0, &s1, &s2, &s3, &d0, &d1, &d2, &d3}}, group, hardwareConfig()), steer_can_(steer_can),
          drive_can_(drive_can) {
    }
    int start() {
        if (!calibration::connections_confirmed)
            return -ENODEV;
        if (!steer_can_ || !drive_can_ || steer_can_ == drive_can_ || !device_is_ready(steer_can_) ||
            !device_is_ready(drive_can_))
            return -ENODEV;
        if (!attached_) {
            int ret = steer_bus.attach(s0, s1, s2, s3);
            if (ret < 0)
                return ret;
            ret = drive_bus.attach(d0, d1, d2, d3);
            if (ret < 0)
                return ret;
            attached_ = true;
        }
        if (!steer_started_) {
            const int ret = steer_bus.start();
            if (ret < 0)
                return ret;
            steer_started_ = true;
        }
        if (!drive_started_) {
            const int ret = drive_bus.start();
            if (ret < 0)
                return ret;
            drive_started_ = true;
        }
        return 0;
    }
    bool running() const {
        return steer_started_ && drive_started_;
    }
    int commit() {
        // Publish each physical bus independently, including while recovering.
        const int steer_error = steer_started_ ? steer_bus.commit().error : -EACCES;
        const int drive_error = drive_started_ ? drive_bus.commit().error : -EACCES;
        return steer_error < 0 ? steer_error : drive_error;
    }

private:
    const device *steer_can_, *drive_can_;
    bool attached_ = false, steer_started_ = false, drive_started_ = false;
};

inline int run(const device *steer_can, const device *drive_can, bool with_power, const device *referee_uart = nullptr,
               robotics::IPowerMeasurementSource *power_source = nullptr) {
    using namespace robotics;
    static Hardware hardware(steer_can, drive_can);
    ChassisExecutor executor(hardware.adapter, controllerConfig(), executorConfig(with_power));
    const int topology_error = hardware.start();
    const int config_error = executor.begin();
    namespace input = skywalker::samples::control;
    if (input::diagnostic_scenario != input::DiagnosticScenario::None &&
        input::diagnostic_scenario != input::DiagnosticScenario::InputPause &&
        input::diagnostic_scenario != input::DiagnosticScenario::ExecutionPause &&
        input::diagnostic_scenario != input::DiagnosticScenario::StatusPause &&
        input::diagnostic_scenario != input::DiagnosticScenario::PermissionPause &&
        input::diagnostic_scenario != input::DiagnosticScenario::MeasurementPause)
        return -ENOTSUP;
    if (!with_power && (input::diagnostic_scenario == input::DiagnosticScenario::PermissionPause ||
                        input::diagnostic_scenario == input::DiagnosticScenario::MeasurementPause))
        return -EINVAL;
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                                input::receiverConfig());
    const int remote_error = remote.start();
    if (remote_error < 0)
        return remote_error;
    communication::RemoteReceiver::Snapshot remote_snapshot{};
    input::RcControlAdapter rc_adapter;
    input::SampleDiagnostics diagnostics;
    core::TimeUs next_log = 0;
    PeriodicDeadline deadline;
    ChassisExecutionInputs inputs{};
    RunStatus status{};
    PowerMeasurement measurement{};
    RefereeState referee{};
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_UART_TRANSPORT)
    static communication::AsyncUart::DmaBuffers referee_dma __nocache;
    static communication::RefereeReceiver receiver(referee_uart, referee_dma,
                                                   communication::RefereeVersion::Rm2026V1_3);
#endif
    printk("8 motors FL/FR/RL/RR: steer CAN1, drive CAN3; topology=%d config=%d\n", topology_error, config_error);
    printk("RC safe+center 0.5s then left Middle; left stick xy, wheel yaw; console is telemetry only\n");
    for (;;) {
        const auto now_us = core::monotonicTimeUs();
        const auto now_ms = now_us / 1000;
        (void)remote.snapshot(remote_snapshot);
        const auto &rc = rc_adapter.update(remote_snapshot.remote, now_ms);
        const auto diagnostic = diagnostics.update(now_ms, rc.run_allowed,
                                                   !rc.fresh || rc.remote.left_switch == RcSwitch::Down);
        if (!diagnostic.input_paused && rc.fresh &&
            (!inputs.command.stamp.valid || sequenceAfter(rc.remote.stamp.sequence, inputs.command.stamp.sequence))) {
            inputs.command.mode = rc.run_allowed ? ChassisMode::BodyVelocity : ChassisMode::Disabled;
            inputs.command.source = ControlSource::Remote;
            inputs.command.vx_m_s = input::RcControlAdapter::normalize(rc.remote.analog.left_y) * .1f;
            inputs.command.vy_m_s = -input::RcControlAdapter::normalize(rc.remote.analog.left_x) * .1f;
            inputs.command.wz_rad_s = input::RcControlAdapter::normalize(rc.remote.analog.wheel) * .2f;
            inputs.command.stamp = rc.remote.stamp;
            inputs.source_stamp = {rc.remote.stamp.timestamp_ms * 1000, rc.remote.stamp.sequence, true};
        }
        if (!rc.run_allowed) {
            inputs.command.mode = ChassisMode::Disabled;
            inputs.command.vx_m_s = inputs.command.vy_m_s = inputs.command.wz_rad_s = 0;
        }
        inputs.clear_estop = rc.clear_estop;
        inputs.transport_ready = topology_error == 0;
        inputs.require_permission = with_power;
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_UART_TRANSPORT)
        if (with_power && referee_uart && device_is_ready(referee_uart))
            referee = receiver.poll(now_ms);
#endif
        if (!diagnostic.permission_paused)
            inputs.permission = referee.robot.chassis_output;
        inputs.power_budget = referee.power;
        if (!diagnostic.measurement_paused) {
            if (power_source) {
                PowerMeasurement next{};
                const int ret = power_source->sample(now_us, next);
                if (ret == 0)
                    measurement = next;
                else if (ret != -EAGAIN)
                    measurement = {};
            }
            else {
                // Referee power is a real measured value. Its own production
                // stamp is retained; polling never renews power or budget age.
                measurement = {referee.power.chassis_power_w,
                               0,
                               0,
                               referee.power.stamp.valid,
                               false,
                               PowerMeasurementSource::Referee,
                               {referee.power.stamp.timestamp_ms * 1000, referee.power.stamp.sequence,
                                referee.power.stamp.valid}};
                // TODO(power-sensor): supply IPowerMeasurementSource backed by
                // installed ADC/power monitor for independent current+voltage.
            }
        }
        inputs.measured_power = measurement;
        if (!diagnostic.execution_paused || rc.clear_estop || !rc.run_allowed) {
            auto execution_status = executor.update(inputs, now_us);
            if (topology_error == 0) {
                const int commit = hardware.commit();
                if (commit < 0)
                    execution_status.error = commit;
            }
            if (!diagnostic.status_paused)
                status = execution_status;
        }
        if (now_us >= next_log) {
            next_log = now_us + 500000;
            const bool status_fresh = isFresh(status.stamp, now_ms, 100);
            LOG_INF(
                "src=%u age_ms=%llu exec=%u fresh=%d ready=%d reason=%u err=%d active=%u power_valid=%d scale_milli=%d power_mW=%d budget_mW=%d bus=%u/%u",
                inputs.command.stamp.sequence,
                static_cast<unsigned long long>(
                    now_ms >= inputs.command.stamp.timestamp_ms ? now_ms - inputs.command.stamp.timestamp_ms : 0),
                unsigned(status.state), status_fresh, status_fresh && status.ready, unsigned(status.reason),
                status.error, unsigned(status.active_count),
                measurement.valid && core::fresh(measurement.stamp, now_us, 300000), int(executor.effortScale() * 1000),
                int(measurement.power_w * 1000), int(referee.power.chassis_power_limit_w * 1000),
                unsigned(hardware.steer_bus.status().state), unsigned(hardware.drive_bus.status().state));
            const auto &feedback = executor.feedback();
            const auto &output = executor.output();
            for (std::size_t i = 0; i < 4; ++i)
                LOG_INF(
                    "wheel=%u steer_mrad=%d final_mrad=%d ramp_mrad=%d error_mrad=%d steer_out=%d drive_out=%d flip=%d coast=%d drive_mrad_s=%d target_mrad_s=%d current_mA=%d/%d scale_milli=%d/%d",
                    unsigned(i), int(feedback.module[i].steer_absolute_rad * 1000),
                    int(output.module[i].optimized_angle_rad * 1000), int(output.module[i].steer_reference_rad * 1000),
                    int(output.module[i].alignment_error_rad * 1000), output.module[i].steer_output_valid,
                    output.module[i].drive_output_valid, output.module[i].flipped, output.module[i].coasting,
                    int(feedback.module[i].drive_velocity_rad_s * 1000),
                    int(output.module[i].drive_target_rad_s * 1000),
                    int(output.module[i].steer_effort * executor.steerEffortScale() * 1000),
                    int(output.module[i].drive_effort * executor.effortScale() * 1000),
                    int(executor.steerEffortScale() * 1000), int(executor.effortScale() * 1000));
        }
        deadline.wait();
    }
}
} // namespace skywalker::samples::chassis
