#include "../../common/chassis_can.hpp"
#include "../../common/rc_controls.hpp"
#include "../../common/sample_diagnostics.hpp"
#include "../../common/operator_controls.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <communication/interboard/interboard_endpoint.hpp>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/vehicle/big_yaw_profile.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#if defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
#if !defined(CONFIG_DUAL_YAW_CHASSIS_ROLE)
#include "../../inertial_gimbal/src/board_config.hpp"
#include <robotics/command/receiver_sources.hpp>
#include <robotics/gimbal/yaw_centering.hpp>
#endif
LOG_MODULE_REGISTER(dual_yaw_bench, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
#if defined(CONFIG_DUAL_YAW_CHASSIS_ROLE)
constexpr auto role = BoardRole::ChassisController;
#else
constexpr auto role = BoardRole::GimbalController;
#endif
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::ConfiguredInterBoardTransport::Config linkConfig() {
    communication::ConfiguredInterBoardTransport::Config c{};
    c.uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart));
    return c;
}
communication::ConfiguredInterBoardTransport transport(linkConfig(), &link_dma);
communication::InterBoardEndpoint::Config endpointConfig() {
    return communication::InterBoardEndpoint::Config{role};
}
communication::InterBoardEndpoint endpoint(transport, endpointConfig());
atomic_t execution_paused = 0, input_paused = 0, status_paused = 0, estop = 0, clear_requested = 0;
void commTask(void *, void *, void *) {
    for (;;) {
        endpoint.poll(k_uptime_get());
        k_sleep(K_MSEC(1));
    }
}
#if !defined(CONFIG_DUAL_YAW_CHASSIS_ROLE)
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(board_config::remote_uart, remote_dma, samples::control::receiverConfig());
RemoteSource original_source(remote);
class PausableSource final : public ICommandSource {
public:
    SourceRole role() const override {
        return original_source.role();
    }
    int start() override {
        return original_source.start();
    }
    int sample(SourceSample &out) override {
        return atomic_get(&input_paused) ? -EAGAIN : original_source.sample(out);
    }
} source;
CommandManager commands(board_config::command_policy);
#if !defined(CONFIG_DUAL_YAW_ENCODER_ONLY)
communication::AsyncUart::DmaBuffers head_dma __nocache;
imu::DmImuRs485Source head_source(inertial_bench::head_uart, head_dma, inertial_bench::head_sensor());
imu::ImuReceiver head(head_source, inertial_bench::head_receiver);
InertialGimbalAdapter adapter(inertial_bench::controller());
#endif
motor::Motor yaw(board_config::yawHardware()), pitch(board_config::pitchHardware());
motor::Group group(yaw, pitch);
motor::CanBus yaw_bus(board_config::yaw_can), pitch_bus(board_config::pitch_can);
GimbalExecutor gimbal(yaw, pitch, group, board_config::yawMotorConfig(), board_config::yaw,
                      board_config::pitchMotorConfig(), board_config::pitch, board_config::execution_policy);
YawCenteringController::Config centeringConfig() {
    YawCenteringController::Config c{};
    c.center_rad = vehicle::yaw_center_rad;
    // TODO(calibration): confirm large-yaw motor direction against joint-error decrease.
    c.follow_direction = vehicle::small_yaw.direction;
    return c;
}
YawCenteringController centering(centeringConfig());
#endif
}
K_THREAD_DEFINE(interboard_worker, 4096, commTask, nullptr, nullptr, nullptr, 6, K_FP_REGS, 0);
int main() {
    LOG_INF("dualYaw role=%u controls=physical RC; confirmed=%d", unsigned(role), vehicle::connections_confirmed);
#if defined(CONFIG_DUAL_YAW_CHASSIS_ROLE)
    static motor::Motor drive(vehicle::bigYawHardware());
    static motor::CanBus bus(skywalker::samples::chassis::big_yaw_can);
    static BigYawExecutor axis(drive, vehicle::bigYawMotorConfig(), vehicle::bigYawExecutionConfig());
    bool started = false;
    int ret = 0;
    if (vehicle::connections_confirmed) {
        ret = bus.attach(drive);
        if (ret == 0)
            ret = bus.start();
#if defined(CONFIG_BOARD_DM_MC02)
        if (ret == 0)
            ret = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
#endif
        if (ret == 0)
            ret = axis.begin();
        started = ret == 0;
    }
    std::uint64_t next_log = 0;
    samples::control::OperatorControlConsumer operator_controls;
    samples::control::SampleDiagnostics diagnostics;
    BigYawRequest cached_request{};
    int commit_error = 0;
    for (;;) {
        const auto now = core::monotonicTimeUs();
        const auto rx = endpoint.snapshot();
        const auto operator_state = operator_controls.update(rx, now / 1000);
        const auto exercise = diagnostics.update(now / 1000, axis.status().state == RunState::Active,
                                                 !operator_state.run_allowed);
        if (!exercise.input_paused)
            cached_request = rx.big_yaw_request;
        if (!exercise.execution_paused || !operator_state.run_allowed || operator_state.clear_estop ||
            operator_state.emergency_stop) {
            BigYawExecutionInputs in{};
            in.request = cached_request;
            in.local_boot_id = rx.local_boot_id;
            if (!operator_state.run_allowed)
                in.request.mode = BigYawMode::Disabled;
            in.peer_online = rx.online;
            in.transport_ready = started;
            in.emergency_stop = operator_state.emergency_stop;
            in.clear_estop = operator_state.clear_estop;
            if (started) {
                axis.update(in, now);
                commit_error = bus.commit().error;
            }
            else
                axis.suspend(now, WaitReason::Configuration, ret ? ret : -ENODEV, true);
            if (!exercise.status_paused)
                endpoint.setBigYawFeedback(axis.feedback());
        }
        if (now / 1000 >= next_log) {
            next_log = now / 1000 + 200;
            const auto s = axis.status();
            const auto f = axis.feedback();
            LOG_INF("operator=%d online=%d source=%u requested=%d run=%u reason=%u rate=%f error=%d commit=%d",
                    rx.operator_control_valid, rx.online, rx.big_yaw_request.source_sequence, s.requested,
                    unsigned(s.state), unsigned(s.reason), double(f.actual_rate_rad_s), s.error, commit_error);
        }
        k_sleep(K_MSEC(5));
    }
#else
    LOG_INF("dualYaw reference=%s", IS_ENABLED(CONFIG_DUAL_YAW_ENCODER_ONLY) ? "encoders" : "head IMU");
    int ret = 0;
#if !defined(CONFIG_DUAL_YAW_ENCODER_ONLY)
    ret = head.start();
#endif
    if (ret == 0)
        ret = commands.registerSource(source);
    if (ret == 0)
        ret = commands.start();
    if (ret < 0)
        return ret;
    const bool split = board_config::yaw_can != board_config::pitch_can;
    bool started = false;
    if (vehicle::connections_confirmed && board_config::connections_configured) {
        ret = split ? yaw_bus.attach(yaw) : yaw_bus.attach(yaw, pitch);
        if (ret == 0 && split)
            ret = pitch_bus.attach(pitch);
        if (ret == 0)
            ret = yaw_bus.start();
        if (ret == 0 && split)
            ret = pitch_bus.start();
#if defined(CONFIG_BOARD_DM_MC02)
        if (ret == 0)
            ret = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
#endif
        if (ret == 0)
            ret = gimbal.begin();
        started = ret == 0;
    }
    std::uint64_t next_log = 0;
    RunStatus run{};
    samples::control::RcControlAdapter controls;
    samples::control::SampleDiagnostics diagnostics;
    samples::control::OperatorControlPublisher operator_controls;
    communication::RemoteReceiver::Snapshot operator_cache{};
#if !defined(CONFIG_DUAL_YAW_ENCODER_ONLY)
    imu::Snapshot head_cache{};
#else
    bool peer_rate_was_valid = false;
#endif
    int yaw_commit_error = 0, pitch_commit_error = 0;
    for (;;) {
        const auto now = core::monotonicTimeUs(), ms = now / 1000;
        (void)remote.snapshot(operator_cache);
        const auto &operator_state = controls.update(operator_cache.remote, ms);
        const auto exercise = diagnostics.update(ms, run.state == RunState::Active,
                                                 !operator_state.fresh ||
                                                     operator_state.remote.left_switch == RcSwitch::Down);
        atomic_set(&input_paused, exercise.input_paused);
        if (operator_controls.publish(endpoint, operator_state.run_allowed, board_config::emergencyStopRequested(),
                                      operator_state.remote.stamp, operator_state.clear_event_id,
                                      operator_state.clear_stamp, ms))
            controls.withdraw();
        if (exercise.execution_paused && operator_state.run_allowed && !operator_state.clear_estop &&
            !board_config::emergencyStopRequested()) {
            k_sleep(K_MSEC(5));
            continue;
        }
        CommandSnapshot frame{};
        (void)commands.snapshot(frame);
        InertialGimbalInputs input{};
#if !defined(CONFIG_DUAL_YAW_ENCODER_ONLY)
        if (!exercise.head_paused)
            head_cache = head.snapshot();
        input.head = head_cache;
#endif
        input.yaw = yaw.snapshot();
        input.pitch = pitch.snapshot();
        input.command = frame.decision.command.gimbal;
        input.source_stamp = sourceStamp(frame, input.command.source);
        if (!operator_state.run_allowed)
            input.command.mode = GimbalMode::Disabled;
        const auto rx = endpoint.snapshot();
        GimbalExecutionInputs gimbal_input{};
#if defined(CONFIG_DUAL_YAW_ENCODER_ONLY)
        const auto &peer = rx.big_yaw_feedback;
        const bool peer_rate_valid = rx.online && peer.valid &&
                                     forwardedFresh(peer.stamp, peer.production_age_ms, ms, 50) &&
                                     std::isfinite(peer.actual_rate_rad_s) &&
                                     std::fabs(peer.actual_rate_rad_s) <= vehicle::big_yaw.velocity_limit_rad_s;
        gimbal_input.command = input.command;
        gimbal_input.source_stamp = input.source_stamp;
        // Manual rates are in body coordinates; executor targets are joint coordinates.
        // Subtract measured carrier motion, never the requested big-Yaw speed.
        if (input.command.mode == GimbalMode::Rate) {
            const float carrier_rate = peer_rate_valid ? peer.actual_rate_rad_s : 0;
            gimbal_input.command.yaw_rate_rad_s = vehicle::small_yaw.direction *
                                                 (input.command.yaw_rate_rad_s - carrier_rate);
            gimbal_input.command.pitch_rate_rad_s = IS_ENABLED(CONFIG_INERTIAL_GIMBAL_LOCK_PITCH)
                                                       ? 0
                                                       : vehicle::pitch.direction * input.command.pitch_rate_rad_s;
        }
        else
            gimbal_input.command.mode = GimbalMode::Disabled;
#else
        input.prerequisites_ready = started && inertial_bench::mounting_configured;
        auto inertial = adapter.update(input, now);
        gimbal_input.command = inertial.command;
        gimbal_input.source_stamp = inertial.source_stamp;
        gimbal_input.yaw_output_valid = inertial.yaw_output_valid;
        gimbal_input.pitch_output_valid = inertial.pitch_output_valid;
#endif
        gimbal_input.transport_ready = started;
        gimbal_input.emergency_stop = board_config::emergencyStopRequested();
        gimbal_input.clear_estop = operator_state.clear_estop || board_config::takeEmergencyResetRequest();
        run = started ? gimbal.update(gimbal_input, now)
                      : gimbal.suspend(now, WaitReason::Configuration, ret ? ret : -ENODEV, true);
        if (started) {
            yaw_commit_error = yaw_bus.commit().error;
            pitch_commit_error = split ? pitch_bus.commit().error : 0;
        }
        YawCenteringInputs follow{};
        follow.joint_yaw_rad = input.yaw.feedback.position_rad;
        follow.joint_stamp = {input.yaw.feedback.timestamp_ms * 1000, input.yaw.feedback.timestamp_ms,
                              input.yaw.feedback_fresh};
        follow.source_stamp = input.source_stamp;
        follow.enabled = input.command.mode != GimbalMode::Disabled && operator_state.run_allowed;
#if defined(CONFIG_DUAL_YAW_ENCODER_ONLY)
        follow.enabled = follow.enabled && input.command.mode == GimbalMode::Rate;
        // In this explicit bench mode the prerequisite is a working joint loop,
        // not a claim that the head has an inertially stable orientation.
        follow.head_stable = gimbal.yawTelemetry().output_valid;
#else
        follow.head_stable = inertial.stabilization_valid;
#endif
        // Explicit unloaded bench permission. Vehicle integration uses referee authority.
        follow.permission_valid = vehicle::connections_confirmed && operator_state.run_allowed &&
                                  !gimbal_input.emergency_stop && !gimbal_input.clear_estop;
#if defined(CONFIG_DUAL_YAW_ENCODER_ONLY)
        // Reset only on feedback transitions. Resetting on every missing sample
        // erases the cycle clock forever and prevents the zero-rate enable request.
        // Reset again on recovery so no rate accumulated while waiting is applied.
        if (peer_rate_valid != peer_rate_was_valid)
            centering.reset();
        peer_rate_was_valid = peer_rate_valid;
#endif
        auto centered = centering.update(follow, now);
#if defined(CONFIG_DUAL_YAW_ENCODER_ONLY)
        if (!peer_rate_valid) {
            // A zero-rate FollowCenter request lets a silent/disabled DM drive
            // enable and produce feedback. Do not require ready/armed to start.
            // Missing or stale measured speed must never sustain a turn.
            centered.velocity_rad_s = 0;
        }
#endif
        BigYawRequest request{};
        request.mode = centered.enabled ? BigYawMode::FollowCenter : BigYawMode::Disabled;
        request.target_rate_rad_s = centered.velocity_rad_s;
        request.source_sequence = static_cast<std::uint32_t>(input.source_stamp.sequence);
        request.source_age_ms = input.source_stamp.valid && now >= input.source_stamp.time_us
                                    ? static_cast<std::uint32_t>(
                                          std::min<std::uint64_t>((now - input.source_stamp.time_us) / 1000,
                                                                  UINT32_MAX))
                                    : UINT32_MAX;
        request.command_age_ms = input.command.stamp.valid && ms >= input.command.stamp.timestamp_ms
                                     ? static_cast<std::uint32_t>(
                                           std::min<std::uint64_t>(ms - input.command.stamp.timestamp_ms, UINT32_MAX))
                                     : UINT32_MAX;
        request.stamp = centered.stamp;
        request.permission = {operator_state.fresh, follow.permission_valid, operator_state.remote.stamp};
        endpoint.submitBigYaw(request);
        if (!exercise.status_paused)
            endpoint.setStatus(run);
        if (ms >= next_log) {
            next_log = ms + 200;
            LOG_INF("rcFresh=%d centered=%d armReady=%d allowed=%d mode=%u started=%d init=%d reason=%u error=%d",
                    operator_state.fresh, controls.controlsCentered(), controls.armReady(),
                    operator_state.run_allowed, unsigned(gimbal_input.command.mode), started, ret,
                    unsigned(run.reason), run.error);
            LOG_INF("followReady=%d centerError=%f request=%f peerRate=%f online=%d ready=%d localRun=%u commit=%d/%d",
                    follow.head_stable, double(centered.center_error_rad), double(centered.velocity_rad_s),
                    double(rx.big_yaw_feedback.actual_rate_rad_s), rx.online, rx.big_yaw_feedback.ready,
                    unsigned(run.state), yaw_commit_error, pitch_commit_error);
        }
        k_sleep(K_MSEC(5));
    }
#endif
}
