#include <robotics/execution/big_yaw_executor.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
namespace skywalker::robotics {
namespace {
bool transient(motor::FaultReason reason) {
    return reason == motor::FaultReason::FeedbackExpired || reason == motor::FaultReason::CommandExpired ||
           reason == motor::FaultReason::UnexpectedDisabled || reason == motor::FaultReason::EnableTimeout ||
           reason == motor::FaultReason::TransportError || reason == motor::FaultReason::RxOverflow;
}
}
BigYawExecutor::BigYawExecutor(motor::Motor &drive, const control::VelocityMotor::Config &motor,
                             const Config &config)
    : drive_(drive), axis_(drive, motor), config_(config), recovery_(config.recovery) {}
int BigYawExecutor::begin() {
    if (begin_attempted_) return -EALREADY;
    begin_attempted_ = true;
    config_error_ = !config_.permission_timeout_ms || !config_.fault_retry_ms || !config_.max_cycle_us ||
        config_.max_cycle_us > 20000 || !std::isfinite(config_.rate_abs_max_rad_s) || config_.rate_abs_max_rad_s <= 0 ||
        !std::isfinite(config_.motor_to_joint_ratio) || config_.motor_to_joint_ratio <= 0 ||
        (config_.direction != 1 && config_.direction != -1) || !config_.recovery.command_timeout_ms ||
        !config_.recovery.source_timeout_us ? -EINVAL : axis_.configure();
    configured_ = config_error_ == 0;
    return config_error_;
}
void BigYawExecutor::withdraw(WaitReason reason, int error, bool blocked) {
    const auto state = drive_.snapshot().state;
    if (state == motor::MotorState::Active || state == motor::MotorState::Enabling) (void)drive_.disable();
    recovery_.withdraw(reason, error, blocked);
    status_.ready = false;
    if (blocked) latched_error_ = error == 0 ? -EIO : error;
}
RunStatus BigYawExecutor::publish(core::TimeUs now, RunState state, WaitReason reason, int error) {
    status_.state = state; status_.reason = reason; status_.error = error;
    status_.generation = recovery_.generation(); status_.stamp = {now / 1000, ++production_sequence_, true};
    return status_;
}
RunStatus BigYawExecutor::suspend(core::TimeUs now, WaitReason reason, int error, bool blocked) {
    withdraw(reason, error, blocked);
    return publish(now, blocked ? RunState::Blocked : RunState::Recovering, reason, error);
}
RunStatus BigYawExecutor::update(const BigYawExecutionInputs &in, core::TimeUs now) {
    const auto now_ms = now / 1000;
    const bool cycle = previous_us_ && now > previous_us_ && now - previous_us_ <= config_.max_cycle_us;
    const float dt = cycle ? float(now - previous_us_) / 1000000 : 0;
    previous_us_ = now; status_.ready = false;
    if (in.emergency_stop) emergency_latched_ = true;
    if (in.clear_fault && !in.emergency_stop && configured_) {
        withdraw(WaitReason::Reference, 0, false);
        const int ret = drive_.clearFault();
        if (ret == 0) { emergency_latched_ = false; latched_error_ = 0; }
        else if (ret != -EAGAIN && ret != -EBUSY) latched_error_ = ret;
    }
    if (!configured_) return suspend(now, WaitReason::Configuration, config_error_ ? config_error_ : -EACCES, true);
    if (emergency_latched_) return suspend(now, WaitReason::Drive, -ECANCELED, true);
    if (latched_error_) return suspend(now, WaitReason::Drive, latched_error_, true);
    if (!in.transport_ready || !in.contract_compatible || !in.local_boot_id)
        return suspend(now, WaitReason::Transport, -EAGAIN);
    if (!cycle) return suspend(now, WaitReason::Cycle, -ESTALE);
    auto view = drive_.snapshot();
    if (view.state == motor::MotorState::Fault) {
        const bool recoverable = transient(view.last_fault.reason);
        withdraw(WaitReason::Drive, view.last_fault.error, !recoverable);
        if (recoverable && view.feedback_fresh && now_ms >= retry_ms_) {
            retry_ms_ = now_ms + config_.fault_retry_ms;
            (void)drive_.clearFault();
        }
        return publish(now, recoverable ? RunState::Recovering : RunState::Blocked, WaitReason::Drive, view.last_fault.error);
    }
    if (!view.feedback_fresh || !(view.feedback.valid & motor::FeedbackVelocity))
        return suspend(now, WaitReason::Feedback, -ESTALE);
    if ((recovery_.stage() == RecoveryGate::Stage::Active && view.state != motor::MotorState::Active) ||
        (recovery_.stage() == RecoveryGate::Stage::Enabling && view.state != motor::MotorState::Enabling &&
         view.state != motor::MotorState::Active)) return suspend(now, WaitReason::Drive, -EAGAIN);
    if (recovery_.stage() == RecoveryGate::Stage::WaitingPrerequisites) {
        if (!drive_.ready() || now_ms < retry_ms_) return publish(now, RunState::Recovering, WaitReason::Feedback, -EAGAIN);
        const int ret = axis_.reset();
        if (ret < 0) return suspend(now, WaitReason::Feedback, ret, ret == -ERANGE || ret == -EINVAL);
        recovery_.prepared(now_ms);
    }
    status_.ready = true;
    const auto &r = in.request;
    if (r.mode == BigYawMode::Disabled) {
        if (drive_.active() || drive_.snapshot().state == motor::MotorState::Enabling) {
            withdraw(WaitReason::Command, 0, false);
        }
        return publish(now, RunState::Disabled, WaitReason::Command);
    }
    if (r.mode != BigYawMode::FollowCenter || !std::isfinite(r.target_rate_rad_s) ||
        std::fabs(r.target_rate_rad_s) > config_.rate_abs_max_rad_s) return suspend(now, WaitReason::Command, -ERANGE, true);
    if (r.receiver_boot_id != in.local_boot_id || r.resume_generation != recovery_.generation()) {
        if (recovery_.stage() == RecoveryGate::Stage::Active || recovery_.stage() == RecoveryGate::Stage::Enabling)
            return suspend(now, WaitReason::Reference, -ESTALE);
        return publish(now, RunState::Recovering, WaitReason::Reference, -ESTALE);
    }
    if (!r.permission.valid || !r.permission.enabled || !forwardedFresh(r.permission.stamp, r.permission_age_ms,
        now_ms, config_.permission_timeout_ms)) return suspend(now, WaitReason::Power, -EACCES);
    auto command = r.stamp;
    command.valid = command.valid && command.timestamp_ms >= r.command_age_ms;
    if (command.valid) command.timestamp_ms -= r.command_age_ms;
    core::Stamp source{};
    source.valid = r.stamp.valid && r.stamp.timestamp_ms >= r.source_age_ms;
    source.sequence = r.source_sequence;
    if (source.valid) source.time_us = (r.stamp.timestamp_ms - r.source_age_ms) * 1000;
    const auto previous = recovery_.stage();
    if (!recovery_.accept(command, source, now_ms)) {
        if (previous == RecoveryGate::Stage::Active || previous == RecoveryGate::Stage::Enabling) {
            (void)drive_.disable(); status_.ready = false;
        }
        return publish(now, RunState::Recovering, WaitReason::Command, -ESTALE);
    }
    if (previous == RecoveryGate::Stage::WaitingCommand) {
        const int ret = drive_.enable();
        if (ret < 0) return suspend(now, WaitReason::Drive, ret);
        return publish(now, RunState::Recovering, WaitReason::Drive);
    }
    if (!drive_.active()) return publish(now, RunState::Recovering, WaitReason::Drive);
    recovery_.enabled();
    const int ret = axis_.update(r.target_rate_rad_s * config_.motor_to_joint_ratio * config_.direction, dt);
    if (ret < 0) return suspend(now, WaitReason::Drive, ret, ret == -EINVAL || ret == -ERANGE);
    status_.last_command_sequence = r.stamp.sequence;
    return publish(now, RunState::Active, WaitReason::None);
}
BigYawFeedback BigYawExecutor::feedback() const {
    const auto mapped = wireFeedback(status_);
    BigYawFeedback out{};
    out.execution_state = mapped.execution_state; out.ready = status_.ready; out.armed = mapped.armed;
    out.active_reasons = mapped.active_reasons; out.resume_generation = status_.generation;
    out.last_command_sequence = status_.last_command_sequence; out.stamp = status_.stamp;
    const auto view = drive_.snapshot();
    out.valid = configured_ && status_.stamp.valid && view.feedback_fresh &&
        (view.feedback.valid & motor::FeedbackVelocity) && std::isfinite(view.feedback.velocity_rad_s);
    if (out.valid) out.actual_rate_rad_s = view.feedback.velocity_rad_s * config_.direction / config_.motor_to_joint_ratio;
    return out;
}
} // namespace skywalker::robotics
