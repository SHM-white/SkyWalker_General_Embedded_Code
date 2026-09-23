#include <algorithm>
#include <cerrno>
#include <cmath>
#include <type_traits>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <communication/async_uart.hpp>
#include <communication/remote/remote_service.hpp>
#include <control/dji_motor_backend.hpp>
#include <control/dm_motor_backend.hpp>
#include "board_config.hpp"
#include <latest.hpp>

LOG_MODULE_REGISTER(gimbal_rc, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;

namespace {
Latest<RemoteState> remote_state;

void remoteTask(void *, void *, void *) {
    static communication::AsyncUart::DmaBuffers dma_buffers __nocache;
    static communication::AsyncUart uart(board_config::remote_uart, dma_buffers);
    communication::RemoteService service({}, {});
    int ret = uart.init();
    LOG_INF("remote UART init: %d", ret);
    if (ret < 0)
        return; // No valid snapshot is published; both axes stay disabled.
    for (;;) {
        const auto now = k_uptime_get();
        uart.service(now);
        communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget = 0; budget < 8; ++budget) {
            ret = uart.read(chunk);
            if (ret == -EOVERFLOW) {
                service.discardPartial();
                continue;
            }
            if (ret < 0)
                break;
            service.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
        }
        RemoteState state{};
        service.snapshot(now, state);
        remote_state.put(state);
        k_sleep(K_MSEC(1));
    }
}

float normalizeStick(std::int16_t raw) {
    const float x = std::clamp(float(raw) / 660.0f, -1.0f, 1.0f);
    constexpr float deadband = 0.03f;
    return std::fabs(x) <= deadband ? 0.0f : std::copysign((std::fabs(x) - deadband) / (1.0f - deadband), x);
}

// Each axis owns its motor lifecycle. Only gimbalTask accesses these objects.
struct Axis {
    control::PositionMotor motor;
    YawGimbal controller;
    const YawGimbalConfig config;
    const std::uint32_t feedback_timeout_ms;
    std::uint32_t generation = 0;
    std::uint64_t ready_ms = 0;

    Axis(control::MotorBackend &backend, const control::PositionMotor::Config &motor_config,
         const YawGimbalConfig &axis_config, std::uint32_t timeout)
        : motor(backend, motor_config), controller(motor, axis_config), config(axis_config),
          feedback_timeout_ms(timeout) {
    }

    bool ready(std::uint64_t now) {
        if (motor.state() != ExecutionState::Active) {
            if (controller.poll(now) < 0)
                return false;
            if (generation != motor.status().resume_generation) {
                generation = motor.status().resume_generation;
                ready_ms = now;
            }
        }
        const auto &measurement = motor.telemetry().measurement;
        const auto stamp = measurement.feedback.timestamp_ms;
        // Check bounds before the first arm too; the single-axis controller
        // checks them during subsequent active updates.
        return stamp != 0 && now >= stamp && now - stamp <= feedback_timeout_ms &&
               std::isfinite(measurement.position_rad) &&
               (config.topology != YawTopology::Limited ||
                (measurement.position_rad >= config.min_angle_rad && measurement.position_rad <= config.max_angle_rad));
    }

    void suspend(PauseReason reason, bool force = false) {
        if (force || motor.state() == ExecutionState::Active) {
            const int ret = controller.suspend(reason);
            if (ret < 0)
                LOG_ERR("axis stop failed: %d", ret);
        }
    }

    int update(float rate, const MessageStamp &stamp, float dt) {
        GimbalCommand command{};
        command.mode = GimbalMode::Rate;
        command.source = ControlSource::Remote;
        command.stamp = stamp;
        // YawGimbal is reused as a single-axis controller: even the pitch
        // instance consumes yaw_rate_rad_s, not pitch_rate_rad_s.
        command.yaw_rate_rad_s = rate;
        return controller.update(command, SafetyAction::Active, dt);
    }
};

void gimbalTask(void *, void *, void *) {
    using YawBackend = std::conditional_t<board_config::yaw_is_dm, control::DmMotorBackend, control::DjiMotorBackend>;
    using PitchBackend = std::conditional_t<board_config::pitch_is_dm, control::DmMotorBackend,
                                            control::DjiMotorBackend>;
    static YawBackend yaw_backend(board_config::yaw_motor);
    static PitchBackend pitch_backend(board_config::pitch_motor);
    static Axis yaw(yaw_backend, board_config::yawMotorConfig(), board_config::yaw,
                    board_config::yaw_is_dm ? CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS
                                            : CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS);
    static Axis pitch(pitch_backend, board_config::pitchMotorConfig(), board_config::pitch,
                      board_config::pitch_is_dm ? CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS
                                                : CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS);
    const int yaw_configured = board_config::connections_configured ? yaw.controller.begin() : -ENODEV;
    const int pitch_configured = board_config::connections_configured ? pitch.controller.begin() : -ENODEV;
    LOG_INF("configure: yaw=%d pitch=%d", yaw_configured, pitch_configured);
    if (yaw_configured < 0 || pitch_configured < 0)
        return; // Neither axis has been armed.

    RemoteState remote{};
    bool estop_latched = false, rearm_allowed = false;
    auto previous_ms = k_uptime_get();
    std::uint64_t next_log = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt = (now - previous_ms) / 1000.0f;
        previous_ms = now;
        // On lock contention retain the previous snapshot and its original age.
        remote_state.get(remote);
        const bool fresh = remote.online && isFresh(remote.stamp, now, board_config::command_timeout_ms);
        const bool estop = board_config::emergencyStopRequested();
        if (estop && !estop_latched) {
            yaw.suspend(PauseReason::EmergencyStop, true);
            pitch.suspend(PauseReason::EmergencyStop, true);
            estop_latched = true;
        }
        if (board_config::takeEmergencyResetRequest() && !estop && estop_latched) {
            const int yr = yaw.controller.clearEmergencyStop(true);
            const int pr = pitch.controller.clearEmergencyStop(true);
            if (yr == 0 && pr == 0)
                estop_latched = false;
        }
        // Evaluate separately: both axes must make recovery progress.
        const bool yaw_ready = !estop_latched && yaw.ready(now);
        const bool pitch_ready = !estop_latched && pitch.ready(now);
        const bool timing_ok = dt > 0.0f && dt <= 0.02f;
        const bool safe_switch = remote.left_switch == RcSwitch::Up || remote.left_switch == RcSwitch::Down;
        const bool new_command = remote.stamp.timestamp_ms > std::max(yaw.ready_ms, pitch.ready_ms);
        if (!fresh || !yaw_ready || !pitch_ready || !timing_ok || estop_latched ||
            remote.left_switch == RcSwitch::Unknown)
            rearm_allowed = false;
        else if (safe_switch && new_command)
            rearm_allowed = true;

        const bool enabled = fresh && yaw_ready && pitch_ready && timing_ok && !estop_latched && rearm_allowed &&
                             new_command && remote.left_switch == RcSwitch::Middle;
        if (!enabled) {
            const auto reason = estop_latched                  ? PauseReason::EmergencyStop
                                : !fresh                       ? PauseReason::CommandTimeout
                                : !timing_ok                   ? PauseReason::InvalidCycle
                                : (!yaw_ready || !pitch_ready) ? PauseReason::FeedbackTimeout
                                                               : PauseReason::OperatorDisabled;
            yaw.suspend(reason);
            pitch.suspend(reason);
        }
        else {
            const float yaw_rate = board_config::yaw_direction * normalizeStick(remote.analog.right_x) *
                                   board_config::yaw.max_rate_rad_s;
            const float pitch_rate = board_config::pitch_direction * normalizeStick(remote.analog.right_y) *
                                     board_config::pitch.max_rate_rad_s;
            const int yr = yaw.update(yaw_rate, remote.stamp, dt);
            const int pr = yr == 0 ? pitch.update(pitch_rate, remote.stamp, dt) : 0;
            if (yr < 0 || pr < 0) {
                yaw.suspend(PauseReason::InvalidCycle, true);
                pitch.suspend(PauseReason::InvalidCycle, true);
                rearm_allowed = false;
                LOG_ERR("axis update failed: yaw=%d pitch=%d", yr, pr);
            }
        }
        if (now >= next_log) {
            next_log = now + 1000;
            LOG_INF("rc=%d switch=%u rearm=%d estop=%d yaw=%u pitch=%u errors=%d/%d", fresh,
                    unsigned(remote.left_switch), rearm_allowed, estop_latched, unsigned(yaw.motor.state()),
                    unsigned(pitch.motor.state()), yaw.motor.status().last_recovery_error,
                    pitch.motor.status().last_recovery_error);
        }
        k_sleep(K_MSEC(5));
    }
}
} // namespace

K_THREAD_DEFINE(remote_thread, 3072, remoteTask, nullptr, nullptr, nullptr, 6, 0, 0);
K_THREAD_DEFINE(gimbal_thread, 6144, gimbalTask, nullptr, nullptr, nullptr, 4, 0, 0);
int main() {
    LOG_INF("RC small yaw + pitch: configured=%d; check app.overlay and src/board_config.hpp",
            board_config::connections_configured);
    return 0;
}
