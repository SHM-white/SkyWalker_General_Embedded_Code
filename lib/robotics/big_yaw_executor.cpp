#include <robotics/execution/big_yaw_executor.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
namespace skywalker::robotics {
BigYawExecutor::BigYawExecutor(motor::Motor &drive, const control::VelocityMotor::Config &motor, const Config &config)
    : drive_(drive), axis_(drive, motor), config_(config), heading_(config.heading) {
}
int BigYawExecutor::begin() {
    if (begin_attempted_)
        return -EALREADY;
    begin_attempted_ = true;
    config_error_ = !config_.max_cycle_us || !std::isfinite(config_.rate_abs_max_rad_s) ||
                            config_.rate_abs_max_rad_s <= 0 || !std::isfinite(config_.motor_to_joint_ratio) ||
                            config_.motor_to_joint_ratio <= 0 || (config_.direction != 1 && config_.direction != -1) ||
                            !config_.command_timeout_ms || !config_.source_timeout_us
                        ? -EINVAL
                    : config_.native_velocity_mode_confirmed
                        ? 0
                        : (config_.heading_lock_enabled ? heading_.validate() : axis_.configure());
    if (!config_error_ && config_.native_velocity_mode_confirmed &&
        (config_.heading_lock_enabled || !(drive_.info().capabilities & motor::CommandVelocity) ||
         !std::isfinite(config_.native_velocity_acceleration_rad_s2) ||
         config_.native_velocity_acceleration_rad_s2 <= 0))
        config_error_ = -EINVAL;
    if (!config_error_ && config_.heading_lock_enabled &&
        (!config_.mit_mode_confirmed || !config_.heading_reference_confirmed ||
         !(drive_.info().capabilities & motor::CommandMit) || !(drive_.info().capabilities & motor::CommandTorque) ||
         config_.heading.torque_max_nm > drive_.info().torque_limit_nm))
        config_error_ = -EINVAL;
    configured_ = !config_error_;
    return config_error_;
}
void BigYawExecutor::withdraw(WaitReason, int, bool) {
    heading_.reset();
    native_last_rate_rad_s_ = 0;
    native_slew_limited_ = false;
    native_enable_generation_ = 0;
    if (!config_.heading_lock_enabled && !config_.native_velocity_mode_confirmed)
        (void)axis_.reset();
    (void)drive_.disable();
    status_.ready = status_.requested = false;
}
RunStatus BigYawExecutor::publish(core::TimeUs now, RunState state, WaitReason reason, int error) {
    status_.state = state;
    status_.reason = reason;
    status_.error = error;
    status_.member_count = 1;
    status_.active_count = drive_.snapshot().state == motor::MotorState::Active;
    status_.waiting_count = 1 - status_.active_count;
    status_.stamp = {now / 1000, ++production_sequence_, true};
    return status_;
}
RunStatus BigYawExecutor::suspend(core::TimeUs now, WaitReason reason, int error, bool blocked) {
    withdraw(reason, error, blocked);
    return publish(now, blocked ? RunState::Blocked : RunState::Disabled, reason, error);
}
RunStatus BigYawExecutor::update(const BigYawExecutionInputs &in, core::TimeUs now) {
    const auto now_ms = now / 1000;
    const bool have_previous = previous_us_ != 0;
    const bool cycle = have_previous && now > previous_us_ && now - previous_us_ <= config_.max_cycle_us;
    const float dt = cycle ? float(now - previous_us_) / 1e6f : 0;
    previous_us_ = now;
    if (!configured_)
        return suspend(now, WaitReason::Configuration, config_error_ ? config_error_ : -EACCES, true);
    if (in.emergency_stop || in.clear_estop)
        return suspend(now, WaitReason::Command, in.emergency_stop ? -ECANCELED : 0);
    const auto &r = in.request;
    if (r.mode == BigYawMode::Disabled)
        return suspend(now, WaitReason::Command);
    if (r.mode != BigYawMode::FollowCenter || !std::isfinite(r.target_rate_rad_s) ||
        std::fabs(r.target_rate_rad_s) > config_.rate_abs_max_rad_s)
        return suspend(now, WaitReason::Command, -ERANGE);
    if (!in.peer_online || !in.transport_ready || !in.local_boot_id || r.receiver_boot_id != in.local_boot_id)
        return suspend(now, WaitReason::Transport, -ESTALE);
    const auto effective_age = std::max(r.command_age_ms, r.source_age_ms);
    if (!forwardedFresh(r.stamp, effective_age, now_ms, config_.command_timeout_ms))
        return suspend(now, WaitReason::Command, -ESTALE);
    if (config_.heading_lock_enabled && (!in.head.valid || in.head.receiver_boot_id != in.local_boot_id ||
                                         !in.peer_boot_id || in.head.sender_boot_id != in.peer_boot_id ||
                                         !forwardedFresh(in.head.stamp, in.head.source_age_ms, now_ms, 50)))
        return suspend(now, WaitReason::Reference, -ESTALE);
    status_.requested = true;
    status_.last_command_sequence = r.stamp.sequence;
    int error = drive_.enable();
    if (!have_previous || !cycle) {
        // Missed interval: keep the enable request pending and stage no new
        // output. Do not disable the drive or reset the heading controller.
        status_.ready = false;
        return publish(now, RunState::Recovering, WaitReason::Cycle, error);
    }
    if (config_.heading_lock_enabled) {
        const auto view = drive_.snapshot();
        if (view.state != motor::MotorState::Active || !view.feedback_fresh || !view.enabled_requested ||
            !view.output_permitted || !(view.feedback.valid & motor::FeedbackVelocity) ||
            !std::isfinite(view.feedback.velocity_rad_s)) {
            heading_.reset();
            (void)drive_.invalidateComputedEffort();
            status_.ready = false;
            return publish(now, RunState::Recovering, WaitReason::Feedback, error);
        }
        if (heading_enable_generation_ != view.enable_generation) {
            heading_.reset();
            heading_enable_generation_ = view.enable_generation;
        }
        float torque = 0;
        const int calculated = heading_.step(in.head, r.target_rate_rad_s, dt, now_ms, torque);
        if (calculated < 0)
            return suspend(now, WaitReason::Reference, calculated);
        // Torque at the joint -> motor shaft via external ratio and sign once.
        const int staged = drive_.setTorque(torque * config_.direction / config_.motor_to_joint_ratio,
                                            view.enable_generation);
        if (staged < 0)
            return suspend(now, WaitReason::Drive, staged);
        status_.ready = true;
        return publish(now, RunState::Active, WaitReason::None, error);
    }
    if (config_.native_velocity_mode_confirmed) {
        if (error < 0)
            return suspend(now, WaitReason::Drive, error);
        const auto view = drive_.snapshot();
        const bool feedback_ready = view.state == motor::MotorState::Active && view.enabled_requested &&
                                    view.output_permitted && view.feedback_fresh &&
                                    (view.feedback.valid & motor::FeedbackVelocity) &&
                                    std::isfinite(view.feedback.velocity_rad_s);
        if (!feedback_ready) {
            native_last_rate_rad_s_ = 0;
            native_enable_generation_ = 0;
            // An active motor losing its prerequisites must discard its staged
            // velocity immediately; an in-progress enable may keep waiting.
            if (view.state == motor::MotorState::Active)
                return suspend(now, WaitReason::Feedback, -ESTALE);
            status_.ready = false;
            return publish(now, RunState::Recovering, WaitReason::Feedback, error);
        }
        if (native_enable_generation_ != view.enable_generation) {
            native_enable_generation_ = view.enable_generation;
            native_last_rate_rad_s_ = 0;
        }
        const float target = r.target_rate_rad_s * config_.motor_to_joint_ratio * config_.direction;
        const float max_change = config_.native_velocity_acceleration_rad_s2 * dt;
        native_slew_limited_ = std::fabs(target - native_last_rate_rad_s_) > max_change;
        const float staged_rate = native_last_rate_rad_s_ +
                                  std::clamp(target - native_last_rate_rad_s_, -max_change, max_change);
        const int staged = drive_.setVelocity(staged_rate);
        if (staged < 0)
            return suspend(now, WaitReason::Drive, staged);
        native_last_rate_rad_s_ = staged_rate;
        status_.ready = true;
        return publish(now, RunState::Active, WaitReason::None);
    }
    const int ret = axis_.update(r.target_rate_rad_s * config_.motor_to_joint_ratio * config_.direction, dt);
    if (!error)
        error = ret;
    status_.ready = axis_.telemetry().output_valid;
    return publish(now, status_.ready ? RunState::Active : RunState::Recovering,
                   !cycle          ? WaitReason::Cycle
                   : status_.ready ? WaitReason::None
                                   : WaitReason::Feedback,
                   error);
}
BigYawFeedback BigYawExecutor::feedback() const {
    const auto mapped = wireFeedback(status_);
    BigYawFeedback out{};
    out.execution_state = mapped.execution_state;
    out.ready = status_.ready;
    out.armed = mapped.armed;
    out.active_reasons = mapped.active_reasons;
    out.last_command_sequence = status_.last_command_sequence;
    out.stamp = status_.stamp;
    const auto view = drive_.snapshot();
    out.valid = configured_ && status_.stamp.valid && view.feedback_fresh &&
                (view.feedback.valid & motor::FeedbackVelocity) && std::isfinite(view.feedback.velocity_rad_s);
    if (out.valid)
        out.actual_rate_rad_s = view.feedback.velocity_rad_s * config_.direction / config_.motor_to_joint_ratio;
    return out;
}
} // namespace skywalker::robotics
