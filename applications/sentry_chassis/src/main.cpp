#include <cmath>
#include <cerrno>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <communication/async_uart.hpp>
#include <communication/interboard/interboard_link.hpp>
#include <robotics/safety/chassis_local_safety.hpp>
#include <robotics/chassis/chassis_power_limiter.hpp>
#include "board_config.hpp"
#include "chassis_hardware.hpp"
#include "latest.hpp"
LOG_MODULE_REGISTER(sentry_chassis, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
// Control, permission and receive timestamp stay in one coherent command value.
struct Received {
    RemoteChassisControl command{};
    ChassisConstraint constraint{};
    BoardHeartbeat peer{};
};
struct Published {
    ChassisFeedbackSummary feedback{};
    std::uint32_t generation = 0;
};
Latest<Received> received;
Latest<Published> published;
std::uint64_t local_boot_id = 0; // Set once in main before explicitly starting tasks.
void linkTask(void *, void *, void *) {
    static communication::AsyncUart uart(board_config::interboard_uart);
    communication::InterBoardLink link(BoardRole::ChassisController);
    int ret = uart.init();
    LOG_INF("interboard UART init: %d", ret);
    std::uint32_t heartbeat_sequence = 0, feedback_sequence = 0;
    std::uint64_t next_tx = 0;
    Published status{};
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        uart.service(now);
        communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget = 0; budget < 8; ++budget) {
            ret = uart.read(chunk);
            if (ret == -EOVERFLOW) {
                link.discardPartial();
                continue;
            }
            if (ret < 0)
                break;
            link.processRxBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
        }
        link.processRxBytes(nullptr, 0, now);
        Received snapshot{};
        link.latestHeartbeat(snapshot.peer);
        link.latestChassisControl(snapshot.command);
        link.latestChassisConstraint(snapshot.constraint);
        received.put(snapshot);
        if (now >= next_tx && !uart.txBusy()) {
            next_tx = now + 20;
            published.get(status);
            BoardHeartbeat h{};
            h.role = BoardRole::ChassisController;
            h.sender_boot_id = local_boot_id;
            h.sender_uptime_ms = now;
            h.resume_generation = status.generation;
            h.ready = status.feedback.ready;
            h.safety_state = status.feedback.safety_state;
            h.active_reasons = status.feedback.active_reasons;
            h.sync_requested = !link.peerOnline(now);
            std::uint8_t bytes[128]{};
            int n = communication::InterBoardCodec::encodeHeartbeat(h, ++heartbeat_sequence, bytes, sizeof(bytes));
            if (n > 0) {
                int f = communication::InterBoardCodec::encodeChassisFeedback(status.feedback, ++feedback_sequence,
                                                                              bytes + n, sizeof(bytes) - n);
                if (f > 0)
                    uart.send(bytes, n + f);
            }
        }
        k_sleep(K_MSEC(1));
    }
}
void chassisTask(void *, void *, void *) {
    static DjiChassisHardware hardware(board_config::motors);
    SwerveChassis chassis(board_config::chassisConfig());
    ChassisLocalSafety safety({board_config::command_timeout_ms, 3});
    ChassisPowerLimiter limiter;
    int configured = chassis.validate();
    if (configured == 0)
        configured = board_config::connections_configured ? hardware.init() : -ENODEV;
    if (!std::isfinite(board_config::bench_effort_scale) || board_config::bench_effort_scale < 0 ||
        board_config::bench_effort_scale > 1 || !std::isfinite(board_config::idle_power_w) ||
        board_config::idle_power_w < 0 || !std::isfinite(board_config::power_per_abs_amp_w) ||
        (board_config::power_model_calibrated && board_config::power_per_abs_amp_w <= 0))
        configured = -EINVAL;
    std::uint64_t peer_boot = 0, last_log = 0;
    std::uint32_t generation = 0, last_accepted = 0;
    bool algorithm_ready = false;
    auto previous_ms = k_uptime_get();
    Received rx{};
    int recovery_error = 0;
    bool estop_latched = false;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        float dt = (now - previous_ms) / 1000.0f;
        previous_ms = now;
        received.get(rx);
        if (rx.peer.stamp.valid && peer_boot != rx.peer.sender_boot_id) {
            peer_boot = rx.peer.sender_boot_id;
            hardware.suspend();
            algorithm_ready = false;
            limiter.reset();
        }
        const bool estop_input = board_config::emergencyStopRequested() ||
                                 (isFresh(rx.command.stamp, now, board_config::command_timeout_ms) &&
                                  (rx.command.active_reasons & EmergencyStop));
        estop_latched |= estop_input;
        if (board_config::takeEmergencyResetRequest() && !estop_input) {
            safety.clearEmergencyStop(true);
            estop_latched = false;
            hardware.suspend();
            algorithm_ready = false;
        }
        const auto &constraint = rx.constraint;
        bool powered = !board_config::require_referee_for_motion;
        if (constraint.output.valid)
            powered = constraint.output.enabled && forwardedFresh(constraint.stamp, constraint.output_age_ms, now,
                                                                  board_config::permission_timeout_ms);
        const bool budget_fresh = constraint.power_valid && forwardedFresh(constraint.stamp, constraint.power_age_ms,
                                                                           now, board_config::permission_timeout_ms);
        const bool budget_allowed = !board_config::require_referee_for_motion ||
                                    (budget_fresh && board_config::power_model_calibrated);
        ChassisFeedback feedback{};
        int feedback_ret = -EAGAIN;
        if (configured == 0) {
            if (estop_latched || !powered || !budget_allowed) {
                if (hardware.armed() || hardware.ready())
                    hardware.suspend();
                algorithm_ready = false;
            }
            else if (!hardware.armed()) {
                recovery_error = hardware.pollRecovery(now);
                if (recovery_error == 0 && !algorithm_ready) {
                    feedback_ret = hardware.read(feedback);
                    if (feedback_ret == 0 && chassis.reset(feedback) == 0) {
                        ++generation;
                        algorithm_ready = true;
                        limiter.reset();
                    }
                }
                if (!hardware.ready())
                    algorithm_ready = false;
            }
            if (hardware.ready() || hardware.armed())
                feedback_ret = hardware.read(feedback);
            if (feedback_ret < 0 && hardware.armed()) {
                hardware.suspend();
                algorithm_ready = false;
            }
        }
        LocalSafetyInputs input{};
        input.now_ms = now;
        input.receiver_boot_id = local_boot_id;
        input.resume_generation = generation;
        input.command_boot_id = rx.command.receiver_boot_id;
        input.command_generation = rx.command.resume_generation;
        input.command_stamp = rx.command.command.stamp;
        input.global_action = rx.command.command.mode == ChassisMode::Disabled ? SafetyAction::Disable
                                                                               : rx.command.global_action;
        input.power_allowed = powered && budget_allowed;
        input.feedback_fresh = feedback_ret == 0;
        input.hardware_ready = hardware.ready() && algorithm_ready;
        input.armed = hardware.armed();
        input.config_valid = configured == 0;
        input.emergency_stop_requested = estop_latched;
        LocalSafetyDecision decision{};
        safety.evaluate(input, decision);
        if (!budget_allowed)
            decision.active_reasons |= PowerBudgetStale;
        if (decision.action != SafetyAction::Active) {
            if (hardware.armed()) {
                hardware.suspend();
                algorithm_ready = false;
            }
        }
        else if (!hardware.armed()) {
            recovery_error = hardware.arm();
            previous_ms = now;
            if (recovery_error < 0)
                algorithm_ready = false;
        }
        else {
            ChassisOutput output{};
            int ret = chassis.step(rx.command.command, feedback, dt, output);
            float scale = board_config::bench_effort_scale;
            if (ret == 0 && board_config::power_model_calibrated && budget_fresh) {
                ChassisPowerDecision power{};
                ret = limiter.step({hardware.estimatedPowerW(), constraint.power_limit_w, constraint.buffer_energy_j},
                                   dt, power);
                scale = power.effort_scale;
            }
            if (ret == 0)
                ret = hardware.apply(output, scale);
            if (ret < 0) {
                recovery_error = ret;
                hardware.suspend();
                algorithm_ready = false;
            }
            else
                last_accepted = rx.command.command.stamp.sequence;
        }
        Published status{};
        status.generation = generation;
        auto &f = status.feedback;
        f.execution_state = configured < 0     ? ExecutionState::ConfigBlocked
                            : estop_latched    ? ExecutionState::EStopLatched
                            : hardware.armed() ? ExecutionState::Active
                            : hardware.ready() ? ExecutionState::Ready
                                               : decision.state;
        f.safety_state = f.execution_state == ExecutionState::ConfigBlocked ? SafetyState::ConfigBlocked
                         : estop_latched                                    ? SafetyState::EmergencyStop
                         : hardware.armed()                                 ? SafetyState::Active
                         : hardware.ready()                                 ? SafetyState::Ready
                                                                            : SafetyState::Waiting;
        f.ready = hardware.ready();
        f.armed = hardware.armed();
        f.active_reasons = decision.active_reasons;
        f.last_command_sequence = last_accepted;
        // No odometry measurement is available yet; bit0 remains clear. bit1 is measured power, not our estimator.
        f.valid_fields = 0;
        f.stamp = {now, 0, true};
        published.put(status);
        if (now >= last_log + 1000) {
            last_log = now;
            LOG_INF("uptime=%llu chassis=%d reasons=%x gen=%u command=%u error=%d", now, int(f.execution_state),
                    f.active_reasons, generation, last_accepted, configured < 0 ? configured : recovery_error);
        }
        k_sleep(K_MSEC(5));
    }
}
}
K_THREAD_DEFINE(link_thread, 6144, linkTask, nullptr, nullptr, nullptr, 5, 0, SYS_FOREVER_MS);
K_THREAD_DEFINE(chassis_thread, 8192, chassisTask, nullptr, nullptr, nullptr, 4, 0, SYS_FOREVER_MS);
int main() {
    local_boot_id = sys_rand64_get() | 1ULL;
    LOG_INF("MC02 sentry chassis: configured=%d referee_required=%d; edit app.overlay and src/board_config.hpp",
            board_config::connections_configured, board_config::require_referee_for_motion);
    k_thread_start(link_thread);
    k_thread_start(chassis_thread);
    return 0;
}
