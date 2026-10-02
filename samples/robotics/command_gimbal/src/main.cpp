#include <algorithm>
#include <cerrno>
#include <cmath>

#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <lib/vofa/vofa.h>
#include <robotics/command/receiver_sources.hpp>
#include <robotics/execution/snapshot_cache.hpp>
#if defined(CONFIG_COMMAND_GIMBAL_VISION_OBSERVE) || defined(CONFIG_COMMAND_GIMBAL_VISION_EXECUTE)
#include <communication/vision/ab_protocol.hpp>
#endif
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "board_config.hpp"
#include "../../common/rc_controls.hpp"
#include "../../common/sample_diagnostics.hpp"

LOG_MODULE_REGISTER(command_gimbal, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;

namespace {
atomic_t input_paused = 0, execution_paused = 0, status_paused = 0;
atomic_t emergency_stop = 0, clear_requested = 0;
atomic_t vision_paused = 0, permission_paused = 0, head_paused = 0;
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(board_config::remote_uart, remote_dma, samples::control::receiverConfig());
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
#ifdef CONFIG_COMMAND_GIMBAL_REFEREE
communication::AsyncUart::DmaBuffers referee_dma __nocache;
const device *const referee_uart = DEVICE_DT_GET(DT_ALIAS(referee_uart));
communication::RefereeReceiver referee(referee_uart, referee_dma, communication::RefereeVersion::Rm2026V1_3, 500);
RefereePermissionSource referee_source(referee);
class PausablePermissions final : public IPermissionSource {
public:
    int start() override { return referee_source.start(); }
    int sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics) override {
        return atomic_get(&permission_paused) ? -EAGAIN : referee_source.sample(now_ms, out, diagnostics);
    }
} permission_source;
#endif
#if defined(CONFIG_COMMAND_GIMBAL_VISION_OBSERVE) || defined(CONFIG_COMMAND_GIMBAL_VISION_EXECUTE)
communication::AsyncUart::DmaBuffers vision_dma __nocache;
communication::vision::AbProtocol vision_protocol({.command_reference = vehicle::head_reference});
communication::vision::VisionReceiver::Config visionConfig() {
    communication::vision::VisionReceiver::Config c{};
    // AB requires genuine projectile speed/count as well as head measurements.
    // TODO(vision-feedback): connect those measurements before enabling TX;
    // this gimbal-only bench already publishes real head data to setFeedback.
    c.feedback_period_us = 0;
    return c;
}
communication::vision::VisionReceiver vision(DEVICE_DT_GET(DT_ALIAS(vision_uart)), vision_dma,
                                             vision_protocol, visionConfig());
VisionSource vision_source(vision);
class PausableVision final : public ICommandSource {
public:
    SourceRole role() const override { return vision_source.role(); }
    int start() override { return vision_source.start(); }
    int sample(SourceSample &out) override {
        return atomic_get(&vision_paused) ? -EAGAIN : vision_source.sample(out);
    }
} pausable_vision;
#endif
#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
communication::AsyncUart::DmaBuffers head_dma __nocache;
imu::DmImuRs485Source head_source(board_config::head_uart, head_dma, board_config::headSensor());
imu::ImuReceiver head(head_source, board_config::head_receiver);
InertialGimbalAdapter inertial_adapter(board_config::inertialPolicy());
#endif
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
#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
    InertialGimbalOutput inertial{};
    core::Stamp head_stamp{};
#endif
};
SnapshotCache<Observation> execution_observation;
K_SEM_DEFINE(telemetry_ready, 0, 1);

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
    const device *const telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
    bool telemetry_available = true;
#ifdef CONFIG_COMMAND_GIMBAL_REFEREE
    telemetry_available = telemetry_uart != referee_uart;
#endif
    // USART1 has exactly one owner. With the referee profile, console logs and
    // debugger atomic flags remain available while VOFA RX/TX stays unstarted.
    const int vofa_ret = telemetry_available ? vofa_init(&vofa, telemetry_uart) : -EBUSY;
    LOG_INF("VOFA telemetry start=%d; controls=physical RC", vofa_ret);
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
#if defined(CONFIG_COMMAND_GIMBAL_VISION_OBSERVE) || defined(CONFIG_COMMAND_GIMBAL_VISION_EXECUTE)
            const auto &aim = frame.observed.vision;
            LOG_INF("vision_seq=%llu age_us=%llu control=%d source=%u ref=%u/%u reasons=%x auto=%d",
                aim.stamp.sequence, aim.stamp.valid && now_ms * 1000 >= aim.stamp.time_us
                    ? now_ms * 1000 - aim.stamp.time_us : UINT64_MAX,
                aim.value.control_requested, unsigned(command.source), aim.value.reference.frame_id,
                aim.value.reference.epoch, frame.decision.gimbal_reasons, board_config::command_policy.allow_auto);
#endif
#ifdef CONFIG_COMMAND_GIMBAL_REFEREE
            LOG_INF("referee_online=%d gimbal_allowed=%d permission_age_ms=%d",
                frame.observed.referee.online, frame.observed.referee.robot.gimbal_output.enabled,
                int(age(frame.observed.referee.robot.gimbal_output.stamp, now_ms)));
#endif
#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
            LOG_INF("head_seq=%llu inertial_session=%u stable=%d head_yaw_mrad=%d head_pitch_mrad=%d",
                observed.head_stamp.sequence, observed.inertial.status.generation,
                observed.inertial.stabilization_valid && observed.run.state == RunState::Active,
                int(observed.inertial.head_yaw_rad * 1000), int(observed.inertial.head_pitch_rad * 1000));
#endif
        }
        k_sleep(K_MSEC(50));
    }
}
} // namespace

K_THREAD_DEFINE(telemetry_thread, 4096, telemetryTask, nullptr, nullptr, nullptr, 7, 0, 0);

int main() {
    int ret = commands.registerSource(pausable_source);
#if defined(CONFIG_COMMAND_GIMBAL_VISION_OBSERVE) || defined(CONFIG_COMMAND_GIMBAL_VISION_EXECUTE)
    if (ret == 0) ret = commands.registerSource(pausable_vision);
#endif
#ifdef CONFIG_COMMAND_GIMBAL_REFEREE
    if (ret == 0) ret = commands.bindPermissions(permission_source);
#endif
#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
    if (ret == 0) ret = head.start();
#endif
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
    samples::control::RcControlAdapter controls({board_config::command_policy.allow_auto});
    samples::control::SampleDiagnostics diagnostics;
    communication::RemoteReceiver::Snapshot operator_cache{};
#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
    imu::Snapshot head_cache{};
#endif
    for (;;) {
        (void)remote.snapshot(operator_cache);
        const auto &operator_state = controls.update(operator_cache.remote, k_uptime_get());
        const auto exercise = diagnostics.update(k_uptime_get(), observation.run.state == RunState::Active,
            !operator_state.fresh || operator_state.remote.left_switch == RcSwitch::Down);
        atomic_set(&input_paused, exercise.input_paused);
        atomic_set(&execution_paused, exercise.execution_paused);
        atomic_set(&status_paused, exercise.status_paused);
        atomic_set(&head_paused, exercise.head_paused);
        atomic_set(&vision_paused, exercise.vision_paused);
        atomic_set(&permission_paused, exercise.permission_paused);
        if (operator_state.clear_fault) atomic_set(&clear_requested, 1);
        if (atomic_get(&execution_paused) && operator_state.run_allowed && !operator_state.clear_fault) {
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
        if (!operator_state.run_allowed) inputs.command.mode = GimbalMode::Disabled;
#ifndef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
        inputs.command.yaw_rate_rad_s *= board_config::yaw_command_sign;
        inputs.command.pitch_rate_rad_s *= board_config::pitch_command_sign;
#endif
        inputs.source_stamp = sourceStamp(frame, inputs.command.source);
        inputs.transport_ready = buses_started && yaw_bus.status().state == motor::BusState::Running &&
                                 (!split_buses || pitch_bus.status().state == motor::BusState::Running);
        inputs.emergency_stop = atomic_get(&emergency_stop) || board_config::emergencyStopRequested();
        inputs.clear_fault = atomic_set(&clear_requested, 0) || board_config::takeEmergencyResetRequest();
#ifdef CONFIG_COMMAND_GIMBAL_REFEREE
        inputs.require_permission = true;
        inputs.permission = frame.observed.referee.robot.gimbal_output;
#endif
#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
        if (!atomic_get(&head_paused)) head_cache = head.snapshot();
        InertialGimbalInputs inertial{};
        inertial.head = head_cache;
        inertial.yaw = yaw_drive.snapshot();
        inertial.pitch = pitch_drive.snapshot();
        inertial.command = inputs.command;
        inertial.source_stamp = inputs.source_stamp;
        inertial.prerequisites_ready = inputs.transport_ready && vehicle::imu_mounting_confirmed &&
            !inputs.emergency_stop && !inputs.clear_fault;
        const bool vision_reference = inputs.command.source != ControlSource::Vision ||
            frame.decision.selected_vision.value.reference == head_cache.sample.reference;
        observation.inertial = vision_reference ? inertial_adapter.update(inertial, now_us)
            : inertial_adapter.suspend(now_us, WaitReason::Reference, -ESTALE);
        inputs.command = observation.inertial.command;
        inputs.source_stamp = observation.inertial.source_stamp;
        observation.head_stamp = head_cache.sample.orientation.stamp;
        communication::vision::Feedback feedback{};
        feedback.mode = frame.decision.operator_mode == OperatorMode::Auto
            ? communication::vision::Mode::AutoAim : communication::vision::Mode::Idle;
        feedback.reference = head_cache.sample.reference;
        feedback.orientation = head_cache.sample.orientation;
        feedback.gyro_rad_s = head_cache.sample.gyro_rad_s;
        // No shooting sensor exists on this bench; unset measurements remain invalid.
        (void)vision.setFeedback(feedback);
 #endif
        const auto previous_run = observation.run;
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
#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
        if (observation.run.generation != previous_run.generation ||
            (previous_run.state == RunState::Active && observation.run.state != RunState::Active))
            observation.inertial = inertial_adapter.suspend(now_us, WaitReason::Reference, -ESTALE);
        observation.inertial.stabilization_valid = observation.inertial.stabilization_valid &&
            observation.run.state == RunState::Active;
#endif
        if (operator_state.run_allowed && previous_run.state == RunState::Active &&
            (observation.run.state != RunState::Active || observation.run.generation != previous_run.generation))
            controls.withdraw();
        observation.yaw_target = executor.yawTargetRad();
        observation.pitch_target = executor.pitchTargetRad();
        observation.duration_us = core::monotonicTimeUs() - now_us;
        if (!atomic_get(&status_paused))
            execution_observation.publish(observation);
        k_sleep(K_MSEC(5));
    }
}
