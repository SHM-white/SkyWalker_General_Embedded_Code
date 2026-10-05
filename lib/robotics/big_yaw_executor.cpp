#include <robotics/execution/big_yaw_executor.hpp>
#include <cerrno>
#include <cmath>
namespace skywalker::robotics {
BigYawExecutor::BigYawExecutor(motor::Motor &drive, const control::VelocityMotor::Config &motor,
                             const Config &config)
    : drive_(drive), axis_(drive, motor), config_(config) {}
int BigYawExecutor::begin() {
    if (begin_attempted_) return -EALREADY;
    begin_attempted_ = true;
    config_error_ = !config_.permission_timeout_ms || !config_.max_cycle_us ||
        !std::isfinite(config_.rate_abs_max_rad_s) || config_.rate_abs_max_rad_s <= 0 ||
        !std::isfinite(config_.motor_to_joint_ratio) || config_.motor_to_joint_ratio <= 0 ||
        (config_.direction != 1 && config_.direction != -1) || !config_.command_timeout_ms ||
        !config_.source_timeout_us ? -EINVAL : axis_.configure();
    configured_ = !config_error_; return config_error_;
}
void BigYawExecutor::withdraw(WaitReason, int, bool) {
    (void)drive_.disable(); status_.ready = status_.requested = false;
}
RunStatus BigYawExecutor::publish(core::TimeUs now, RunState state, WaitReason reason, int error) {
    status_.state = state; status_.reason = reason; status_.error = error;
    status_.member_count = 1; status_.active_count = drive_.snapshot().state == motor::MotorState::Active;
    status_.waiting_count = 1 - status_.active_count;
    status_.stamp = {now / 1000, ++production_sequence_, true}; return status_;
}
RunStatus BigYawExecutor::suspend(core::TimeUs now, WaitReason reason, int error, bool blocked) {
    withdraw(reason, error, blocked);
    return publish(now, blocked ? RunState::Blocked : RunState::Disabled, reason, error);
}
RunStatus BigYawExecutor::update(const BigYawExecutionInputs &in, core::TimeUs now) {
    const auto now_ms = now / 1000;
    const bool cycle = previous_us_ && now > previous_us_ && now - previous_us_ <= config_.max_cycle_us;
    const float dt = cycle ? float(now - previous_us_) / 1e6f : 0;
    previous_us_ = now;
    if (!configured_) return suspend(now, WaitReason::Configuration, config_error_ ? config_error_ : -EACCES, true);
    if (in.emergency_stop || in.clear_estop) return suspend(now, WaitReason::Command, in.emergency_stop ? -ECANCELED : 0);
    const auto &r = in.request;
    if (r.mode == BigYawMode::Disabled) return suspend(now, WaitReason::Command);
    if (r.mode != BigYawMode::FollowCenter || !std::isfinite(r.target_rate_rad_s) ||
        std::fabs(r.target_rate_rad_s) > config_.rate_abs_max_rad_s) return suspend(now, WaitReason::Command, -ERANGE);
    if (!in.peer_online || !in.transport_ready || !in.local_boot_id || r.receiver_boot_id != in.local_boot_id)
        return suspend(now, WaitReason::Transport, -ESTALE);
    if (!forwardedFresh(r.stamp, r.command_age_ms, now_ms, config_.command_timeout_ms) ||
        !forwardedFresh(r.stamp, r.source_age_ms, now_ms, config_.source_timeout_us / 1000))
        return suspend(now, WaitReason::Command, -ESTALE);
    if (!r.permission.valid || !r.permission.enabled ||
        !forwardedFresh(r.permission.stamp, r.permission_age_ms, now_ms, config_.permission_timeout_ms))
        return suspend(now, WaitReason::Power, -EACCES);
    status_.requested = true; status_.last_command_sequence = r.stamp.sequence;
    int error = drive_.enable();
    const int ret = axis_.update(r.target_rate_rad_s * config_.motor_to_joint_ratio * config_.direction, dt);
    if (!error) error = ret;
    status_.ready = axis_.telemetry().output_valid;
    return publish(now, status_.ready ? RunState::Active : RunState::Recovering,
        !cycle ? WaitReason::Cycle : status_.ready ? WaitReason::None : WaitReason::Feedback, error);
}
BigYawFeedback BigYawExecutor::feedback() const {
    const auto mapped = wireFeedback(status_);
    BigYawFeedback out{};
    out.execution_state = mapped.execution_state; out.ready = status_.ready; out.armed = mapped.armed;
    out.active_reasons = mapped.active_reasons;
    out.last_command_sequence = status_.last_command_sequence; out.stamp = status_.stamp;
    const auto view = drive_.snapshot();
    out.valid = configured_ && status_.stamp.valid && view.feedback_fresh &&
        (view.feedback.valid & motor::FeedbackVelocity) && std::isfinite(view.feedback.velocity_rad_s);
    if (out.valid) out.actual_rate_rad_s = view.feedback.velocity_rad_s * config_.direction / config_.motor_to_joint_ratio;
    return out;
}
} // namespace skywalker::robotics
