#include "chassis_executor.hpp"
#include <cerrno>
#include <cmath>
#include <limits>
using namespace skywalker;
using namespace skywalker::robotics;
RunStatus ChassisExecutor::begin() {
    if (checked_) return status_;
    return update(core::monotonicTimeUs());
}
void ChassisExecutor::suspend() {
    (void)hardware_.suspend();
    prepared_ = false;
    limiter_.reset();
}
RunStatus ChassisExecutor::update(core::TimeUs now_us) {
    const auto now = now_us / 1000;
    const bool valid_cycle = have_time_ && now_us > previous_us_ && now_us - previous_us_ <= 20000;
    const float dt = valid_cycle ? float(now_us - previous_us_) / 1000000.0f : 0;
    previous_us_ = now_us;
    have_time_ = true;
    const auto rx = link_.snapshot();
    const auto finish = [&](RunState state, WaitReason reason, int error = 0) {
        status_.state = state;
        status_.reason = reason;
        status_.error = error;
        status_.ready = prepared_ && (hardware_.ready() || hardware_.armed() || hardware_.enabling());
        status_.generation = generation_;
        status_.stamp = {now, status_.stamp.sequence + 1, true};
        return status_;
    };
    if (!checked_) {
        checked_ = true;
        config_error_ = board_config::connections_configured ? chassis_.validate() : -ENODEV;
        if (!std::isfinite(board_config::bench_effort_scale) || board_config::bench_effort_scale < 0 ||
            board_config::bench_effort_scale > 1 || !std::isfinite(board_config::idle_power_w) ||
            board_config::idle_power_w < 0 || !std::isfinite(board_config::power_per_abs_amp_w) ||
            board_config::power_per_abs_amp_w < 0 ||
            (board_config::power_model_calibrated && board_config::power_per_abs_amp_w <= 0))
            config_error_ = -EINVAL;
    }
    if (config_error_ < 0) return finish(RunState::Blocked, WaitReason::Configuration, config_error_);
    if (!initialized_) {
        if (now < retry_ms_) return finish(RunState::Recovering, WaitReason::Transport, -EAGAIN);
        const int ret = hardware_.init();
        if (ret < 0) {
            retry_ms_ = now + board_config::recovery_retry_ms;
            if (ret == -EINVAL || ret == -ENOTSUP || ret == -ENODEV || ret == -ERANGE || ret == -EBUSY || ret == -ENOSPC)
                config_error_ = ret;
            return finish(config_error_ < 0 ? RunState::Blocked : RunState::Recovering, WaitReason::Configuration, ret);
        }
        initialized_ = true;
    }
    if (rx.peer.stamp.valid && peer_boot_ != rx.peer.sender_boot_id) {
        peer_boot_ = rx.peer.sender_boot_id;
        suspend();
    }
    const auto &command = rx.control.command;
    const bool online = rx.online && isFresh(rx.peer.stamp, now, board_config::heartbeat_timeout_ms);
    const bool fresh = isFresh(command.stamp, now, board_config::command_timeout_ms);
    const bool requested = online && fresh && command.mode != ChassisMode::Disabled &&
                           rx.control.global_action == SafetyAction::Active;
    const auto &constraint = rx.constraint;
    const bool budget_fresh = constraint.power_valid && forwardedFresh(constraint.stamp, constraint.power_age_ms,
                                                                        now, board_config::permission_timeout_ms);
    const bool budget_allowed = !board_config::require_power_budget ||
                                (budget_fresh && board_config::power_model_calibrated);
    const bool context = rx.local_boot_id && rx.control.receiver_boot_id == rx.local_boot_id &&
                         rx.control.resume_generation == generation_;
    // Withdrawal never waits for feedback, retry deadlines or preparation.
    if ((hardware_.armed() || hardware_.enabling()) && (!requested || !budget_allowed || !context))
        suspend();
    if (hardware_.hasBlockingFault()) {
        suspend();
        return finish(RunState::Blocked, WaitReason::Drive, -EIO);
    }
    ChassisFeedback feedback{};
    if (!hardware_.armed() && !hardware_.enabling()) {
        const int ret = hardware_.pollRecovery(now);
        if (ret < 0) {
            prepared_ = false;
            return finish(RunState::Recovering, WaitReason::Feedback, ret);
        }
        if (!prepared_) {
            int reset = hardware_.read(feedback);
            if (reset == 0) reset = chassis_.reset(feedback);
            if (reset < 0) { suspend(); return finish(RunState::Recovering, WaitReason::Reference, reset); }
            if (generation_ == std::numeric_limits<std::uint32_t>::max()) {
                config_error_ = -EOVERFLOW;
                suspend();
                return finish(RunState::Blocked, WaitReason::Configuration, config_error_);
            }
            ++generation_;
            ready_ms_ = now;
            prepared_ = true;
            limiter_.reset();
        }
    }
    if (!requested)
        return finish(online && fresh && command.mode == ChassisMode::Disabled ? RunState::Disabled : RunState::Recovering,
                      online ? WaitReason::Command : WaitReason::Transport);
    if (!budget_allowed) return finish(RunState::Recovering, WaitReason::Power, -EAGAIN);
    // Re-evaluate after preparing: preparation may have advanced generation_.
    if (!rx.local_boot_id || rx.control.receiver_boot_id != rx.local_boot_id ||
        rx.control.resume_generation != generation_ || command.stamp.timestamp_ms <= ready_ms_)
        return finish(RunState::Recovering, WaitReason::Command, -EAGAIN);
    if (hardware_.enabling()) return finish(RunState::Recovering, WaitReason::Drive);
    if (!hardware_.armed()) {
        const int ret = hardware_.arm();
        if (ret < 0) prepared_ = false;
        return finish(RunState::Recovering, WaitReason::Drive, ret);
    }
    if (!valid_cycle) { suspend(); return finish(RunState::Recovering, WaitReason::Cycle, -ESTALE); }
    int ret = hardware_.read(feedback);
    ChassisOutput output{};
    if (ret == 0) ret = chassis_.step(command, feedback, dt, output);
    float scale = board_config::bench_effort_scale;
    if (ret == 0 && board_config::power_model_calibrated && budget_fresh) {
        ChassisPowerDecision power{};
        ret = limiter_.step({hardware_.estimatedPowerW(), constraint.power_limit_w, constraint.buffer_energy_j}, dt, power);
        scale = power.effort_scale;
    }
    if (ret == 0) ret = hardware_.apply(output, scale);
    if (ret < 0) { suspend(); return finish(RunState::Recovering, WaitReason::Drive, ret); }
    status_.last_command_sequence = command.stamp.sequence;
    return finish(RunState::Active, WaitReason::None);
}
