#pragma once
#include <core/clock.hpp>
#ifdef CONFIG_SKYWALKER_LIB_VOFA
#include <cmath>
#include <lib/vofa/vofa.h>
#endif
#include <drivers/motor/can_bus.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/command/receiver_sources.hpp>
#include <robotics/vehicle/shooter_profile.hpp>
#include <zephyr/sys/printk.h>
#include "../command_gimbal/src/board_config.hpp"
#include "rc_controls.hpp"
#include "sample_diagnostics.hpp"
#include "shooter_rc.hpp"

namespace skywalker::samples::shooter {
using namespace robotics;
// TODO(referee): provide an IShooterHeatSource using the actual referee profile
// or installed heat model. Missing data remains invalid, not synthetic permission.
class PendingHeatSource final : public IShooterHeatSource {
public:
    int sample(ShooterHeatState &) override {
        return -ENODATA;
    }
};
class Hardware {
public:
    motor::Motor yaw{board_config::yawHardware()}, pitch{board_config::pitchHardware()};
    motor::Motor left{vehicle::frictionHardware(0)}, right{vehicle::frictionHardware(1)}, dial{vehicle::dialHardware()};
    motor::Group gimbal_group{yaw, pitch}, friction_group{left, right}, dial_group{dial};
    motor::CanBus dji_bus, pitch_bus;
    GimbalExecutor gimbal;
    ShooterExecutor shooter;
    Hardware(const device *dji_can, const device *pitch_can, bool unloaded_relative = false)
        : dji_bus(dji_can), pitch_bus(pitch_can),
          gimbal(yaw, pitch, gimbal_group, board_config::yawMotorConfig(), board_config::yaw,
                 board_config::pitchMotorConfig(), board_config::pitch, board_config::execution_policy),
          shooter(left, right, dial, friction_group, dial_group, vehicle::frictionMotorConfig(),
                  vehicle::dialMotorConfig(), vehicle::shooterExecutionConfig(unloaded_relative)) {
    }
    int start(bool with_gimbal) {
        if (!vehicle::connections_confirmed)
            return -ENODEV;
        if (!attached_) {
            int ret = with_gimbal ? dji_bus.attach(yaw, left, right, dial) : dji_bus.attach(left, right, dial);
            if (ret < 0)
                return ret;
            if (with_gimbal) {
                ret = pitch_bus.attach(pitch);
                if (ret < 0)
                    return ret;
            }
            attached_ = true;
            with_gimbal_ = with_gimbal;
        }
        if (!dji_started_) {
            const int ret = dji_bus.start();
            if (ret < 0)
                return ret;
            dji_started_ = true;
        }
        if (with_gimbal_ && !pitch_started_) {
            const int ret = pitch_bus.start();
            if (ret < 0)
                return ret;
            pitch_started_ = true;
        }
        if (!controllers_started_) {
            int ret = shooter.begin();
            if (ret == 0 && with_gimbal_)
                ret = gimbal.begin();
            if (ret < 0)
                return ret;
            controllers_started_ = true;
        }
        return 0;
    }
    bool running() const {
        return controllers_started_;
    }
    int commit() {
        int ret = dji_started_ ? dji_bus.commit().error : -EACCES;
        const int pitch_ret = pitch_started_ ? pitch_bus.commit().error : 0;
        return ret < 0 ? ret : pitch_ret;
    }

private:
    bool attached_ = false, dji_started_ = false, pitch_started_ = false;
    bool controllers_started_ = false, with_gimbal_ = false;
};

inline int run(bool with_gimbal, bool unloaded_feed) {
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver receiver(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                                  control::receiverConfig());
    static RemoteSource source(receiver);
    static const CommandManager::Config policy = [] {
        auto c = board_config::command_policy;
        c.allow_auto = false;
        c.require_referee_for_motion = false;
        return c;
    }();
    static CommandManager manager(policy);
    static Hardware hardware(DEVICE_DT_GET(DT_NODELABEL(can1)), DEVICE_DT_GET(DT_NODELABEL(can2)), unloaded_feed);
    static PendingHeatSource heat_source;
    int ret = manager.registerSource(source);
    if (ret == 0)
        ret = manager.start();
    if (ret < 0)
        return ret;
    const int setup = hardware.start(with_gimbal);
    printk("shared DJI CAN yaw/friction/dial; setup=%d; unloaded_feed=%d\n", setup, unloaded_feed);
    printk(
        "RC left Down=stop, neutral Down/Down 500ms then left Middle=arm; right Middle=spin, Middle->Up=single/hold continuous\n");
    CommandSnapshot frame{};
    ShooterCommand local_request{};
    ShooterStatus shooter{};
    RunStatus gimbal{};
    control::RcControlAdapter controls;
    control::SampleDiagnostics diagnostics;
    control::RcShooterRequest firing;
    communication::RemoteReceiver::Snapshot operator_cache{};
    std::uint64_t next_log = 0;
#ifdef CONFIG_SKYWALKER_LIB_VOFA
    static_assert(DT_NODE_HAS_COMPAT(DT_ALIAS(telemetry_uart), zephyr_cdc_acm_uart),
                  "Shooter bench VOFA requires USB CDC ACM");
    static Vofa vofa{};
    const auto *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
    const int vofa_ret = vofa_init(&vofa, telemetry_uart);
    printk("VOFA device=%s init=%d; JustFloat 19 channels, 50 Hz\n", telemetry_uart->name, vofa_ret);
    std::uint64_t next_telemetry = 0;
    std::uint32_t telemetry_sequence = 0;
    int last_send_error = 0;
#endif
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now = now_us / 1000;
        (void)receiver.snapshot(operator_cache);
        const auto &operator_state = controls.update(operator_cache.remote, now);
        const auto exercise = diagnostics.update(now, shooter.friction.state == RunState::Active,
                                                 !operator_state.fresh ||
                                                     operator_state.remote.left_switch == RcSwitch::Down);
        const bool clear = operator_state.clear_estop;
        const bool estop = board_config::emergencyStopRequested();
        if (!exercise.input_paused)
            (void)manager.snapshot(frame);
        local_request = firing.update(operator_state, frame, shooter, now);
        if (!exercise.execution_paused || clear || estop || !operator_state.run_allowed) {
            if (with_gimbal) {
                GimbalExecutionInputs gi{};
                gi.command = frame.decision.command.gimbal;
                gi.source_stamp = sourceStamp(frame, gi.command.source);
                if (!operator_state.run_allowed)
                    gi.command.mode = GimbalMode::Disabled;
                gi.transport_ready = setup == 0;
                gi.emergency_stop = estop;
                gi.clear_estop = clear;
                gimbal = hardware.gimbal.update(gi, now_us);
            }
            ShooterExecutionInputs si{};
            si.command = local_request;
            si.source_stamp = sourceStamp(frame, si.command.source);
            si.transport_ready = setup == 0;
            si.emergency_stop = estop;
            si.clear_estop = clear;
            si.gimbal = gimbal;
            si.require_permission = !unloaded_feed;
            si.require_heat = !unloaded_feed;
            si.require_gimbal = with_gimbal && !unloaded_feed;
            si.allow_feed = unloaded_feed ? vehicle::connections_confirmed : vehicle::shooter_constraints_confirmed;
            (void)heat_source.sample(si.heat);
            // TODO(referee): wire real shooter_output permission for loaded runs.
            shooter = hardware.shooter.update(si, now_us);
            if (setup == 0) {
                const int commit = hardware.commit();
                if (commit < 0) {
                    shooter.friction.error = shooter.feed.error = commit;
                    if (with_gimbal)
                        gimbal.error = commit;
                }
            }
        }
#ifdef CONFIG_SKYWALKER_LIB_VOFA
        if (!exercise.status_paused && now >= next_telemetry) {
            next_telemetry = now + 20;
            const auto left = hardware.left.snapshot(), right = hardware.right.snapshot();
            const auto dial = hardware.dial.snapshot();
            const auto dt = hardware.shooter.dialTelemetry();
            const auto valid = [](const auto &view, auto field, float value) {
                return view.feedback_fresh && (view.feedback.valid & field) && std::isfinite(value);
            };
            const bool lv = valid(left, motor::FeedbackVelocity, left.feedback.velocity_rad_s);
            const bool rv = valid(right, motor::FeedbackVelocity, right.feedback.velocity_rad_s);
            const bool dp = dial.position_reference_valid &&
                            valid(dial, motor::FeedbackPosition, dial.feedback.position_rad);
            const bool dv = valid(dial, motor::FeedbackVelocity, dial.feedback.velocity_rad_s);
            const bool dc = valid(dial, motor::FeedbackCurrent, dial.feedback.current_a);
            const bool output = !exercise.execution_paused && shooter.feed.requested && dt.output_valid &&
                                dial.feedback_fresh && dial.output_permitted &&
                                dial.state == motor::MotorState::Active &&
                                dt.motor.enable_generation == dial.enable_generation &&
                                dt.motor.reference_generation == dial.reference_generation;
            const bool request_valid = !exercise.execution_paused && shooter.stamp.valid &&
                                       shooter.stamp.timestamp_ms == now &&
                                       std::isfinite(shooter.dial_requested_velocity_rad_s);
            const unsigned validity = unsigned(lv) | (unsigned(rv) << 1) | (unsigned(dp) << 2) |
                                      (unsigned(dv) << 3) | (unsigned(dc) << 4) | (unsigned(output) << 5) |
                                      (unsigned(request_valid) << 6);
            telemetry_sequence = (telemetry_sequence + 1) % 1000000;
            const float channels[] = {
                float(telemetry_sequence),
                shooter.friction.requested ? vehicle::friction_speed_rad_s * vehicle::friction[0].direction : 0.0f,
                lv ? left.feedback.velocity_rad_s : 0.0f,
                shooter.friction.requested ? vehicle::friction_speed_rad_s * vehicle::friction[1].direction : 0.0f,
                rv ? right.feedback.velocity_rad_s : 0.0f,
                float(shooter.dial_target_rad),
                dp ? dial.feedback.position_rad : 0.0f,
                dv ? dial.feedback.velocity_rad_s : 0.0f,
                output ? dt.effort_command : 0.0f,
                dc ? dial.feedback.current_a : 0.0f,
                shooter.friction_ready ? 1.0f : 0.0f,
                float(unsigned(shooter.feed.state)),
                float(unsigned(shooter.feed.reason)),
                shooter.dial_busy ? 1.0f : 0.0f,
                shooter.jammed ? 1.0f : 0.0f,
                float(validity),
                request_valid ? shooter.dial_requested_velocity_rad_s : 0.0f,
                dv ? dial.feedback.velocity_rad_s : 0.0f,
                dv ? dial.feedback.velocity_rad_s * vehicle::dial.gear_ratio * 60.0f / vehicle::two_pi : 0.0f,
            };
            constexpr auto count = sizeof(channels) / sizeof(channels[0]);
            static_assert(count <= VOFA_MAX_FLOATS);
            if (vofa_ret == 0) {
                const int error = vofa_send(&vofa, channels, static_cast<std::uint8_t>(count));
                if (error < 0 && error != last_send_error)
                    printk("VOFA send=%d seq=%u\n", error, telemetry_sequence);
                last_send_error = error;
            }
        }
#endif
        if (!exercise.status_paused && now >= next_log) {
            next_log = now + 200;
            printk(
                "src=%u cmd=%u gimbal=%u friction=%u ready=%d feed=%u wait=%u active=%u/%u event=%u shots=%u busy=%d jam=%d bus=%u\n",
                frame.observed.remote.stamp.sequence, local_request.stamp.sequence, unsigned(gimbal.state),
                unsigned(shooter.friction.state), shooter.friction_ready, unsigned(shooter.feed.state),
                unsigned(shooter.feed.reason), unsigned(shooter.friction.active_count),
                unsigned(shooter.feed.active_count), shooter.last_event_id, shooter.shots, shooter.dial_busy,
                shooter.jammed, unsigned(hardware.dji_bus.status().state));
        }
        k_sleep(K_MSEC(5));
    }
}
} // namespace skywalker::samples::shooter
