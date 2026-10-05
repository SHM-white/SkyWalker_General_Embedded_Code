#include "board_config.hpp"
#include "../../common/rc_controls.hpp"
#include "../../common/sample_diagnostics.hpp"

#include <cerrno>
#include <cmath>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <lib/vofa/vofa.h>
#include <robotics/command/receiver_sources.hpp>
#include <robotics/execution/snapshot_cache.hpp>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(inertial_gimbal_bench, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;

namespace {
communication::AsyncUart::DmaBuffers remote_dma __nocache, head_dma __nocache;
communication::RemoteReceiver remote(board_config::remote_uart, remote_dma, samples::control::receiverConfig());
RemoteSource remote_source(remote);
atomic_t input_paused = 0, execution_paused = 0, status_paused = 0, head_paused = 0;
atomic_t estop = 0, clear_requested = 0;
class PausableRemote final : public ICommandSource {
public:
    SourceRole role() const override { return remote_source.role(); }
    int start() override { return remote_source.start(); }
    int sample(SourceSample &out) override {
        return atomic_get(&input_paused) ? -EAGAIN : remote_source.sample(out);
    }
} input;
CommandManager commands(board_config::command_policy);
imu::DmImuRs485Source head_source(inertial_bench::head_uart, head_dma, inertial_bench::head_sensor());
imu::ImuReceiver head(head_source, inertial_bench::head_receiver);
motor::Motor yaw(board_config::yawHardware()), pitch(board_config::pitchHardware());
motor::Group group(yaw, pitch);
motor::CanBus yaw_bus(board_config::yaw_can), pitch_bus(board_config::pitch_can);
GimbalExecutor executor(yaw, pitch, group, board_config::yawMotorConfig(), board_config::yaw,
                         board_config::pitchMotorConfig(), board_config::pitch, board_config::execution_policy);
InertialGimbalAdapter adapter(inertial_bench::controller());
struct Observation {
    InertialGimbalOutput inertial{};
    RunStatus execution{};
    core::Stamp head_stamp{};
    core::TimeUs duration_us = 0;
    std::uint32_t overruns = 0;
    int yaw_commit_error = 0, pitch_commit_error = 0;
};
SnapshotCache<Observation> observation;
K_SEM_DEFINE(telemetry_ready, 0, 1);

void telemetryTask(void *, void *, void *) {
    k_sem_take(&telemetry_ready, K_FOREVER);
    Vofa vofa{};
    const int ret = vofa_init(&vofa, DEVICE_DT_GET(DT_ALIAS(telemetry_uart)));
    std::uint64_t next_log_ms = 0;
    for (;;) {
        Observation seen{};
        (void)observation.snapshot(seen);
        const auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
        const auto &a = seen.inertial;
        const float age_ms = seen.head_stamp.valid && now_ms * 1000 >= seen.head_stamp.time_us
            ? float(now_ms * 1000 - seen.head_stamp.time_us) * 0.001f : -1;
        const float values[] = {a.head_yaw_rad, a.head_pitch_rad, a.yaw_error_rad, a.pitch_error_rad,
            a.command.yaw_rate_rad_s, a.command.pitch_rate_rad_s, float(a.yaw_output_valid),
            float(seen.execution.active_count), float(unsigned(seen.execution.state)),
            float(unsigned(seen.execution.reason)), float(a.stabilization_valid), age_ms,
            float(seen.duration_us), float(seen.overruns), float(yaw_bus.status().last_error),
            float(pitch_bus.status().last_error)};
        if (ret == 0) (void)vofa_send(&vofa, values, 16);
        if (now_ms >= next_log_ms) {
            next_log_ms = now_ms + 1000;
            LOG_INF("head_seq=%llu head_age_ms=%d requested=%d run=%u wait=%u active=%zu waiting=%zu stable=%d cycle_us=%u overruns=%u commit=%d/%d",
                seen.head_stamp.sequence, int(age_ms), seen.execution.requested, unsigned(seen.execution.state),
                unsigned(seen.execution.reason), seen.execution.active_count, seen.execution.waiting_count,
                a.stabilization_valid, unsigned(seen.duration_us), seen.overruns,
                seen.yaw_commit_error, seen.pitch_commit_error);
        }
        k_sleep(K_MSEC(50));
    }
}
} // namespace
K_THREAD_DEFINE(telemetry_thread, 4096, telemetryTask, nullptr, nullptr, nullptr, 7, K_FP_REGS, 0);

int main() {
    int ret = head.start();
    if (ret == 0) ret = commands.registerSource(input);
    if (ret == 0) ret = commands.start();
    k_sem_give(&telemetry_ready);
    if (ret < 0) { LOG_ERR("source startup=%d", ret); return ret; }
    LOG_INF("inertial head bench: motor_confirmation=%d mounting_confirmation=%d pitch_lock=%d",
        board_config::connections_configured, inertial_bench::mounting_configured,
        IS_ENABLED(CONFIG_INERTIAL_GIMBAL_LOCK_PITCH));
    const bool split = board_config::yaw_can != board_config::pitch_can;
    bool attached = false, yaw_started = false, pitch_started = !split, configured = false;
    bool blocked = !board_config::connections_configured;
    int setup_error = blocked ? -EACCES : 0;
    if (!blocked) {
        ret = split ? yaw_bus.attach(yaw) : yaw_bus.attach(yaw, pitch);
        if (ret == 0 && split) ret = pitch_bus.attach(pitch);
        attached = ret == 0;
        blocked = ret < 0;
        setup_error = ret;
    }
    core::TimeUs retry_us = 0, previous_us = 0;
    Observation current{};
    imu::Snapshot head_cache{};
    samples::control::RcControlAdapter controls;
    samples::control::SampleDiagnostics diagnostics;
    communication::RemoteReceiver::Snapshot operator_cache{};
    for (;;) {
        (void)remote.snapshot(operator_cache);
        const auto &operator_state = controls.update(operator_cache.remote, k_uptime_get());
        const auto exercise = diagnostics.update(k_uptime_get(), current.execution.state == RunState::Active,
            !operator_state.fresh || operator_state.remote.left_switch == RcSwitch::Down);
        atomic_set(&input_paused, exercise.input_paused);
        atomic_set(&execution_paused, exercise.execution_paused);
        atomic_set(&status_paused, exercise.status_paused);
        atomic_set(&head_paused, exercise.head_paused);
        if (operator_state.clear_estop) atomic_set(&clear_requested, 1);
        if (atomic_get(&execution_paused) && operator_state.run_allowed && !operator_state.clear_estop &&
            !board_config::emergencyStopRequested()) {
            k_sleep(K_MSEC(5)); continue;
        }
        const auto now = core::monotonicTimeUs();
        if (previous_us && now - previous_us > board_config::execution_policy.max_cycle_us) ++current.overruns;
        previous_us = now;
        if (attached && !blocked && !configured && now >= retry_us) {
            retry_us = now + 100000;
            ret = 0;
            if (!yaw_started) { ret = yaw_bus.start(); yaw_started = ret == 0; }
            if (ret == 0 && !pitch_started) { ret = pitch_bus.start(); pitch_started = ret == 0; }
            if (ret == 0) {
                ret = executor.begin();
                configured = ret == 0;
                blocked = ret < 0;
            }
            else if (ret == -EINVAL || ret == -ENODEV || ret == -ENOTSUP || ret == -ENOSPC || ret == -EBUSY)
                blocked = true;
            setup_error = ret;
        }
        CommandSnapshot frame{};
        (void)commands.snapshot(frame);
        if (!atomic_get(&head_paused)) head_cache = head.snapshot();
        InertialGimbalInputs inertial{};
        inertial.head = head_cache;
        inertial.yaw = yaw.snapshot();
        inertial.pitch = pitch.snapshot();
        inertial.command = frame.decision.command.gimbal;
        if (!operator_state.run_allowed) inertial.command.mode = GimbalMode::Disabled;
        inertial.source_stamp = sourceStamp(frame, inertial.command.source);
        inertial.prerequisites_ready = configured && inertial_bench::mounting_configured;
        current.inertial = adapter.update(inertial, now);
        GimbalExecutionInputs request{};
        request.command = current.inertial.command;
        request.source_stamp = current.inertial.source_stamp;
        request.transport_ready = configured;
        request.yaw_output_valid = current.inertial.yaw_output_valid;
        request.pitch_output_valid = current.inertial.pitch_output_valid;
        request.emergency_stop = atomic_get(&estop) || board_config::emergencyStopRequested();
        request.clear_estop = atomic_set(&clear_requested, 0) || board_config::takeEmergencyResetRequest();
        current.execution = configured ? executor.update(request, now)
            : executor.suspend(now, blocked ? WaitReason::Configuration : WaitReason::Transport, setup_error, blocked);
        current.yaw_commit_error = yaw_started ? yaw_bus.commit().error : 0;
        current.pitch_commit_error = split && pitch_started ? pitch_bus.commit().error : 0;
        current.head_stamp = head_cache.sample.orientation.stamp;
        current.duration_us = core::monotonicTimeUs() - now;
        if (!atomic_get(&status_paused)) observation.publish(current);
        k_sleep(K_MSEC(5));
    }
}
