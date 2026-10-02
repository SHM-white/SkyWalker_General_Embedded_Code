#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <lib/vofa/vofa.h>
#include <robotics/command/receiver_sources.hpp>
#include <robotics/execution/snapshot_cache.hpp>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "board_config.hpp"

LOG_MODULE_REGISTER(command_gimbal, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;

namespace {
atomic_t input_paused = 0, execution_paused = 0, status_paused = 0;
atomic_t emergency_stop = 0, clear_requested = 0;
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(board_config::remote_uart, remote_dma, {});
RemoteSource remote_source(remote);
// Pausing publication freezes the manager's cached original source stamp; the
// receiver and its DMA worker continue independently.
class PausableRemoteSource final : public ICommandSource {
public:
    SourceRole role() const override { return remote_source.role(); }
    int start() override { return remote_source.start(); }
    int sample(SourceSample &out) override {
        return atomic_get(&input_paused) ? -EAGAIN : remote_source.sample(out);
    }
} pausable_source;
CommandManager commands(board_config::command_policy);
motor::Motor yaw_drive(board_config::yawHardware()), pitch_drive(board_config::pitchHardware());
motor::Group gimbal_group(yaw_drive, pitch_drive);
motor::CanBus yaw_bus(board_config::yaw_can), pitch_bus(board_config::pitch_can);
GimbalExecutor executor(yaw_drive, pitch_drive, gimbal_group, board_config::yawMotorConfig(), board_config::yaw,
                         board_config::pitchMotorConfig(), board_config::pitch, board_config::execution_policy);
struct Observation {
    RunStatus run{};
    double yaw_target = 0, pitch_target = 0;
    core::TimeUs duration_us = 0;
    std::uint32_t cycle_overruns = 0;
};
SnapshotCache<Observation> execution_observation;
K_SEM_DEFINE(telemetry_ready, 0, 1);

void onTelemetryCommand(const char *key, float value) {
    if (!std::isfinite(value))
        return;
    const bool set = value != 0.0f;
    if (std::strcmp(key, "input_pause") == 0)
        atomic_set(&input_paused, set);
    else if (std::strcmp(key, "execution_pause") == 0)
        atomic_set(&execution_paused, set);
    else if (std::strcmp(key, "status_pause") == 0)
        atomic_set(&status_paused, set);
    else if (std::strcmp(key, "estop") == 0)
        atomic_set(&emergency_stop, set);
    else if (std::strcmp(key, "clear") == 0 && set)
        atomic_set(&clear_requested, 1);
}

float age(const MessageStamp &stamp, std::uint64_t now_ms) {
    return stamp.valid && now_ms >= stamp.timestamp_ms ? float(now_ms - stamp.timestamp_ms) : -1.0f;
}

bool permanentSetupError(int error) {
    return error == -EINVAL || error == -ENOTSUP || error == -ENODEV || error == -ENOSPC || error == -EBUSY ||
           error == -EADDRINUSE || error == -ERANGE || error == -EACCES;
}

void telemetryTask(void *, void *, void *) {
    k_sem_take(&telemetry_ready, K_FOREVER);
    static Vofa vofa{};
    static std::uint8_t rx_buffer[80];
    const int vofa_ret = vofa_init(&vofa, DEVICE_DT_GET(DT_ALIAS(telemetry_uart)));
    const int rx_ret = vofa_ret == 0 ? vofa_set_handler(&vofa, rx_buffer, sizeof(rx_buffer), onTelemetryCommand) : vofa_ret;
    LOG_INF("VOFA start=%d controls=%d", vofa_ret, rx_ret);
    CommandSnapshot frame{};
    Observation observed{};
    std::uint64_t next_log_ms = 0;
    for (;;) {
        (void)commands.snapshot(frame); // Independent, non-consuming reader.
        (void)execution_observation.snapshot(observed);
        const auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
        const auto yaw = yaw_drive.snapshot(), pitch = pitch_drive.snapshot();
        const auto ybus = yaw_bus.status();
        const auto pbus = board_config::pitch_can != board_config::yaw_can ? pitch_bus.status() : ybus;
        const auto &command = frame.decision.command.gimbal;
        const auto feedback = wireFeedback(observed.run, now_ms, board_config::command_timeout_ms);
        const float channels[] = {
            float(frame.observed.remote.stamp.sequence), age(frame.observed.remote.stamp, now_ms),
            float(command.stamp.sequence), age(command.stamp, now_ms), float(unsigned(observed.run.state)),
            float(unsigned(observed.run.reason)), float(observed.run.generation), age(observed.run.stamp, now_ms),
            float(observed.run.last_command_sequence), yaw.feedback.position_rad, pitch.feedback.position_rad,
            float(observed.yaw_target), float(observed.pitch_target), float(ybus.last_error), float(pbus.last_error),
            float(observed.cycle_overruns),
        };
        static_assert(sizeof(channels) / sizeof(channels[0]) <= VOFA_MAX_FLOATS);
        if (vofa_ret == 0)
            (void)vofa_send(&vofa, channels, static_cast<std::uint8_t>(sizeof(channels) / sizeof(channels[0])));
        if (now_ms >= next_log_ms) {
            next_log_ms = now_ms + 1000;
            LOG_INF("source=%u command=%u run=%u wait=%u gen=%u fresh=%d ready=%d error=%d cycle_us=%u overruns=%u",
                    frame.observed.remote.stamp.sequence, command.stamp.sequence, unsigned(observed.run.state),
                    unsigned(observed.run.reason), observed.run.generation,
                    isFresh(observed.run.stamp, now_ms, board_config::command_timeout_ms), feedback.ready,
                    observed.run.error, unsigned(observed.duration_us), observed.cycle_overruns);
        }
        k_sleep(K_MSEC(50));
    }
}
} // namespace

K_THREAD_DEFINE(telemetry_thread, 4096, telemetryTask, nullptr, nullptr, nullptr, 7, 0, 0);

int main() {
    int ret = commands.registerSource(pausable_source);
    if (ret == 0)
        ret = commands.start();
    k_sem_give(&telemetry_ready);
    if (ret < 0) {
        LOG_ERR("command service start=%d", ret);
        return ret;
    }
    const bool split_buses = board_config::pitch_can != board_config::yaw_can;
    bool topology_attached = false, yaw_started = false, pitch_started = !split_buses, executor_started = false;
    bool setup_blocked = !board_config::connections_configured;
    int setup_error = board_config::connections_configured ? 0 : -ENODEV;
    std::uint64_t setup_retry_ms = 0;
    if (board_config::connections_configured) {
        ret = split_buses ? yaw_bus.attach(yaw_drive) : yaw_bus.attach(yaw_drive, pitch_drive);
        if (ret == 0 && split_buses)
            ret = pitch_bus.attach(pitch_drive);
        topology_attached = ret == 0;
        setup_error = ret;
        setup_blocked = ret < 0;
        if (setup_blocked)
            LOG_ERR("hardware attach=%d", ret);
    }
    else {
        LOG_WRN("connections_configured=false: calibrate both axes in board_config.hpp before enabling");
    }
    CommandSnapshot frame{};
    Observation observation{};
    core::TimeUs previous_cycle_us = 0;
    for (;;) {
        if (atomic_get(&execution_paused)) {
            // Deliberately freeze the execution producer while command and
            // telemetry workers remain alive. Motor I/O owns expiry/stop.
            k_sleep(K_MSEC(5));
            continue;
        }
        const auto now_us = core::monotonicTimeUs();
        const auto now_ms = now_us / 1000;
        if (topology_attached && !setup_blocked && !executor_started && now_ms >= setup_retry_ms) {
            setup_retry_ms = now_ms + board_config::execution_policy.fault_retry_ms;
            int start_error = 0;
            if (!yaw_started) {
                start_error = yaw_bus.start();
                yaw_started = start_error == 0;
            }
            if (start_error == 0 && !pitch_started) {
                start_error = pitch_bus.start();
                pitch_started = start_error == 0;
            }
            if (start_error == 0) {
                start_error = executor.begin();
                executor_started = start_error == 0;
                setup_blocked = start_error < 0; // Axis configuration is a one-shot operation.
            }
            else {
                setup_blocked = permanentSetupError(start_error);
            }
            setup_error = start_error;
            LOG_INF("hardware start/configure=%d blocked=%d", setup_error, setup_blocked);
        }
        const bool buses_started = yaw_started && pitch_started;
        if (previous_cycle_us && now_us - previous_cycle_us > board_config::execution_policy.max_cycle_us)
            ++observation.cycle_overruns;
        previous_cycle_us = now_us;
        (void)commands.snapshot(frame);
        GimbalExecutionInputs inputs{};
        inputs.command = frame.decision.command.gimbal;
        inputs.command.yaw_rate_rad_s *= board_config::yaw_command_sign;
        inputs.command.pitch_rate_rad_s *= board_config::pitch_command_sign;
        inputs.source_stamp = sourceStamp(frame, inputs.command.source);
        inputs.transport_ready = buses_started && yaw_bus.status().state == motor::BusState::Running &&
                                 (!split_buses || pitch_bus.status().state == motor::BusState::Running);
        inputs.emergency_stop = atomic_get(&emergency_stop) || board_config::emergencyStopRequested();
        inputs.clear_fault = atomic_set(&clear_requested, 0) || board_config::takeEmergencyResetRequest();
        observation.run = executor_started ? executor.update(inputs, now_us)
            : executor.suspend(now_us, setup_blocked ? WaitReason::Configuration : WaitReason::Transport,
                                setup_error, setup_blocked);
        // Future shared-CAN mechanisms stage here, in this same thread.
        if (buses_started) {
            const int yaw_error = yaw_bus.commit().error;
            const int pitch_error = split_buses ? pitch_bus.commit().error : 0;
            const int error = yaw_error < 0 ? yaw_error : pitch_error;
            if (error < 0)
                observation.run = executor.suspend(core::monotonicTimeUs(), WaitReason::Transport, error);
        }
        observation.yaw_target = executor.yawTargetRad();
        observation.pitch_target = executor.pitchTargetRad();
        observation.duration_us = core::monotonicTimeUs() - now_us;
        if (!atomic_get(&status_paused))
            execution_observation.publish(observation);
        k_sleep(K_MSEC(5));
    }
}
