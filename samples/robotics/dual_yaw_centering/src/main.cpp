#include <algorithm>
#include <cerrno>
#include <communication/interboard/interboard_endpoint.hpp>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/vehicle/big_yaw_profile.hpp>
#include <zephyr/drivers/uart.h>
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
    c.uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart)); return c;
}
communication::ConfiguredInterBoardTransport transport(linkConfig(), &link_dma);
communication::InterBoardEndpoint::Config endpointConfig() {
    communication::InterBoardEndpoint::Config c{role}; c.enable_big_yaw = true; return c;
}
communication::InterBoardEndpoint endpoint(transport, endpointConfig());
atomic_t execution_paused = 0, input_paused = 0, status_paused = 0, estop = 0, clear_requested = 0;
void commTask(void *, void *, void *) {
    for (;;) { endpoint.poll(k_uptime_get()); k_sleep(K_MSEC(1)); }
}
void consoleControls() {
    unsigned char key = 0;
    if (uart_poll_in(DEVICE_DT_GET(DT_CHOSEN(zephyr_console)), &key) != 0) return;
    if (key == 'p') atomic_set(&input_paused, !atomic_get(&input_paused));
    if (key == 's') atomic_set(&status_paused, !atomic_get(&status_paused));
    if (key == 'e') atomic_set(&execution_paused, !atomic_get(&execution_paused));
    if (key == 'x') atomic_set(&estop, 1);
    if (key == 'r') { atomic_set(&estop, 0); atomic_set(&clear_requested, 1); }
}
#if !defined(CONFIG_DUAL_YAW_CHASSIS_ROLE)
communication::AsyncUart::DmaBuffers remote_dma __nocache, head_dma __nocache;
communication::RemoteReceiver remote(board_config::remote_uart, remote_dma, {});
RemoteSource original_source(remote);
class PausableSource final : public ICommandSource {
public:
    SourceRole role() const override { return original_source.role(); }
    int start() override { return original_source.start(); }
    int sample(SourceSample &out) override {
        return atomic_get(&input_paused) ? -EAGAIN : original_source.sample(out);
    }
} source;
CommandManager commands(board_config::command_policy);
imu::DmImuRs485Source head_source(inertial_bench::head_uart, head_dma, inertial_bench::head_sensor());
imu::ImuReceiver head(head_source, inertial_bench::head_receiver);
motor::Motor yaw(board_config::yawHardware()), pitch(board_config::pitchHardware());
motor::Group group(yaw, pitch);
motor::CanBus yaw_bus(board_config::yaw_can), pitch_bus(board_config::pitch_can);
GimbalExecutor gimbal(yaw, pitch, group, board_config::yawMotorConfig(), board_config::yaw,
    board_config::pitchMotorConfig(), board_config::pitch, board_config::execution_policy);
InertialGimbalAdapter adapter(inertial_bench::controller());
YawCenteringController::Config centeringConfig() {
    YawCenteringController::Config c{};
    c.center_rad = vehicle::yaw_center_rad;
    // TODO(calibration): confirm large-yaw motor direction against joint-error decrease.
    c.follow_direction = vehicle::small_yaw.direction; return c;
}
YawCenteringController centering(centeringConfig());
#endif
}
K_THREAD_DEFINE(interboard_worker, 4096, commTask, nullptr, nullptr, nullptr, 6, K_FP_REGS, 0);
int main() {
    LOG_INF("dualYaw role=%u p: pause input, s: pause status, e: pause executor, x: estop, r: clear; confirmed=%d",
        unsigned(role), vehicle::connections_confirmed);
#if defined(CONFIG_DUAL_YAW_CHASSIS_ROLE)
    static motor::Motor drive(vehicle::bigYawHardware());
    static motor::CanBus bus(DEVICE_DT_GET(DT_NODELABEL(can3)));
    static BigYawExecutor axis(drive, vehicle::bigYawMotorConfig(), vehicle::bigYawExecutionConfig());
    bool started = false;
    int ret = 0;
    if (vehicle::connections_confirmed) {
        ret = bus.attach(drive); if (ret == 0) ret = bus.start();
#if defined(CONFIG_BOARD_DM_MC02)
        if (ret == 0) ret = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
#endif
        if (ret == 0) ret = axis.begin();
        started = ret == 0;
    }
    std::uint64_t next_log = 0;
    for (;;) {
        consoleControls();
        const auto now = core::monotonicTimeUs(); const auto rx = endpoint.snapshot();
        if (!atomic_get(&execution_paused)) {
            BigYawExecutionInputs in{};
            in.request = rx.big_yaw_request; in.local_boot_id = rx.local_boot_id;
            in.contract_compatible = rx.big_yaw_compatible;
            in.transport_ready = rx.online && started && bus.status().state == motor::BusState::Running;
            in.emergency_stop = atomic_get(&estop); in.clear_fault = atomic_set(&clear_requested, 0);
            if (started) {
                axis.update(in, now);
                const int error = bus.commit().error;
                if (error < 0) axis.suspend(now, WaitReason::Transport, error);
            } else axis.suspend(now, WaitReason::Configuration, ret ? ret : -ENODEV, true);
            if (!atomic_get(&status_paused)) endpoint.setBigYawFeedback(axis.feedback());
        }
        if (now / 1000 >= next_log) {
            next_log = now / 1000 + 200;
            const auto s = axis.status(); const auto f = axis.feedback();
            LOG_INF("compatible=%d online=%d source=%u gen=%u run=%u reason=%u rate=%f error=%d",
                rx.big_yaw_compatible, rx.online, rx.big_yaw_request.source_sequence, s.generation,
                unsigned(s.state), unsigned(s.reason), double(f.actual_rate_rad_s), s.error);
        }
        k_sleep(K_MSEC(5));
    }
#else
    int ret = head.start();
    if (ret == 0) ret = commands.registerSource(source);
    if (ret == 0) ret = commands.start();
    if (ret < 0) return ret;
    const bool split = board_config::yaw_can != board_config::pitch_can;
    bool started = false;
    if (vehicle::connections_confirmed && board_config::connections_configured) {
        ret = split ? yaw_bus.attach(yaw) : yaw_bus.attach(yaw, pitch);
        if (ret == 0 && split) ret = pitch_bus.attach(pitch);
        if (ret == 0) ret = yaw_bus.start();
        if (ret == 0 && split) ret = pitch_bus.start();
#if defined(CONFIG_BOARD_DM_MC02)
        if (ret == 0) ret = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
#endif
        if (ret == 0) ret = gimbal.begin();
        started = ret == 0;
    }
    std::uint64_t next_log = 0;
    RunStatus run{};
    for (;;) {
        consoleControls();
        const auto now = core::monotonicTimeUs(), ms = now / 1000;
        if (atomic_get(&execution_paused)) { k_sleep(K_MSEC(5)); continue; }
        CommandSnapshot frame{}; (void)commands.snapshot(frame);
        InertialGimbalInputs input{};
        input.head = head.snapshot(); input.yaw = yaw.snapshot(); input.pitch = pitch.snapshot();
        input.command = frame.decision.command.gimbal; input.source_stamp = sourceStamp(frame, input.command.source);
        input.prerequisites_ready = started && inertial_bench::mounting_configured &&
            yaw_bus.status().state == motor::BusState::Running && (!split || pitch_bus.status().state == motor::BusState::Running);
        auto inertial = adapter.update(input, now);
        GimbalExecutionInputs gimbal_input{};
        gimbal_input.command = inertial.command; gimbal_input.source_stamp = inertial.source_stamp;
        gimbal_input.transport_ready = input.prerequisites_ready;
        gimbal_input.emergency_stop = atomic_get(&estop);
        gimbal_input.clear_fault = atomic_set(&clear_requested, 0);
        const auto previous = run;
        run = started ? gimbal.update(gimbal_input, now) : gimbal.suspend(now, WaitReason::Configuration, ret ? ret : -ENODEV, true);
        if (started) {
            const int y = yaw_bus.commit().error, p = split ? pitch_bus.commit().error : 0;
            if (y < 0 || p < 0) run = gimbal.suspend(now, WaitReason::Transport, y < 0 ? y : p);
        }
        if (run.generation != previous.generation || (previous.state == RunState::Active && run.state != RunState::Active))
            inertial = adapter.suspend(now, WaitReason::Reference, -ESTALE);
        const auto rx = endpoint.snapshot();
        YawCenteringInputs follow{};
        follow.joint_yaw_rad = input.yaw.feedback.position_rad;
        follow.joint_stamp = {input.yaw.feedback.timestamp_ms * 1000, input.yaw.feedback.timestamp_ms, input.yaw.feedback_fresh};
        follow.source_stamp = input.source_stamp;
        follow.enabled = input.command.mode != GimbalMode::Disabled && rx.big_yaw_compatible &&
            rx.big_yaw_feedback.valid && rx.big_yaw_feedback.ready;
        follow.head_stable = inertial.stabilization_valid && run.state == RunState::Active;
        // Explicit unloaded bench permission. Vehicle integration uses referee authority.
        follow.permission_valid = vehicle::connections_confirmed && !atomic_get(&estop);
        const auto centered = centering.update(follow, now);
        BigYawRequest request{};
        request.mode = centered.enabled ? BigYawMode::FollowCenter : BigYawMode::Disabled;
        request.target_rate_rad_s = centered.velocity_rad_s;
        request.source_sequence = static_cast<std::uint32_t>(input.source_stamp.sequence);
        request.source_age_ms = input.source_stamp.valid && now >= input.source_stamp.time_us
            ? static_cast<std::uint32_t>(std::min<std::uint64_t>((now - input.source_stamp.time_us) / 1000, UINT32_MAX)) : UINT32_MAX;
        request.command_age_ms = input.command.stamp.valid && ms >= input.command.stamp.timestamp_ms
            ? static_cast<std::uint32_t>(std::min<std::uint64_t>(ms - input.command.stamp.timestamp_ms, UINT32_MAX)) : UINT32_MAX;
        request.stamp = centered.stamp;
        request.permission = {true, follow.permission_valid, {ms, centered.stamp.sequence, true}};
        endpoint.submitBigYaw(request);
        if (!atomic_get(&status_paused)) endpoint.setStatus(run);
        if (ms >= next_log) {
            next_log = ms + 200;
            LOG_INF("headStable=%d centerError=%f request=%f peerRate=%f peerGen=%u compatible=%d ready=%d localRun=%u",
                follow.head_stable, double(centered.center_error_rad), double(centered.velocity_rad_s),
                double(rx.big_yaw_feedback.actual_rate_rad_s), rx.big_yaw_feedback.resume_generation,
                rx.big_yaw_compatible, rx.big_yaw_feedback.ready, unsigned(run.state));
        }
        k_sleep(K_MSEC(5));
    }
#endif
}
