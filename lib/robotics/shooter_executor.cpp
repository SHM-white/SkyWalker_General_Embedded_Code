#include <robotics/shooter/shooter_executor.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
namespace skywalker::robotics {
ShooterExecutor::ShooterExecutor(motor::Motor &left, motor::Motor &right, motor::Motor &dial,
    motor::Group &friction_group, motor::Group &dial_group,
    const control::VelocityMotor::Config &friction_control,
    const control::PositionMotor::Config &dial_control, const Config &config)
    : left_(left), right_(right), dial_(dial), friction_group_(friction_group), dial_group_(dial_group),
      left_control_(left, friction_control), right_control_(right, friction_control),
      dial_control_(dial, dial_control), config_(config) {}
int ShooterExecutor::begin() {
    if (begin_attempted_) return -EALREADY;
    begin_attempted_ = true;
    const bool valid = config_.max_cycle_us && config_.max_cycle_us <= 20000 &&
        config_.command_timeout_ms && config_.source_timeout_us &&
        config_.permission_timeout_ms && config_.heat_timeout_ms && config_.gimbal_timeout_ms &&
        config_.friction_dwell_ms && config_.dial_settle_ms && config_.jam_timeout_ms &&
        std::isfinite(config_.friction_speed_rad_s) && config_.friction_speed_rad_s > 0 &&
        std::isfinite(config_.friction_tolerance_rad_s) && config_.friction_tolerance_rad_s > 0 &&
        std::isfinite(config_.dial_step_rad) && config_.dial_step_rad > 0 &&
        std::isfinite(config_.dial_tolerance_rad) && config_.dial_tolerance_rad > 0 &&
        std::isfinite(config_.dial_settle_velocity_rad_s) && config_.dial_settle_velocity_rad_s > 0 &&
        std::isfinite(config_.heat_per_round) && config_.heat_per_round > 0 &&
        std::isfinite(config_.max_fire_rate_hz) && config_.max_fire_rate_hz > 0 &&
        std::fabs(config_.friction_direction[0]) == 1 && std::fabs(config_.friction_direction[1]) == 1 &&
        std::fabs(config_.dial_direction) == 1;
    int ret = valid ? left_control_.configure() : -EINVAL;
    if (ret == 0) ret = right_control_.configure();
    if (ret == 0) ret = dial_control_.configure();
    configuration_error_ = ret;
    configured_ = ret == 0;
    return ret;
}
void ShooterExecutor::stopFriction(WaitReason reason, int error) {
    friction_group_.disable(); friction_good_since_ms_ = 0;
    status_.friction_ready = false; status_.friction.requested = false;
    status_.friction.state = RunState::Disabled; status_.friction.reason = reason; status_.friction.error = error;
    status_.friction.ready = false;
}
void ShooterExecutor::stopFeed(WaitReason reason, int error) {
    dial_group_.disable(); status_.dial_busy = false;
    step_started_ms_ = dial_good_since_ms_ = next_shot_ms_ = 0;
    status_.feed.requested = false; status_.feed.ready = false;
    status_.feed.state = RunState::Disabled; status_.feed.reason = reason; status_.feed.error = error;
}
void ShooterExecutor::discardEvent(const ShooterCommand &command) {
    if (command.fire_event_id && (!status_.last_event_id ||
        sequenceAfter(command.fire_event_id, status_.last_event_id))) status_.last_event_id = command.fire_event_id;
}
void ShooterExecutor::publish(core::TimeUs now) {
    const MessageStamp stamp{now / 1000, ++production_sequence_, true};
    status_.stamp = status_.friction.stamp = status_.feed.stamp = stamp;
    const auto f = friction_group_.status(), d = dial_group_.status();
    status_.friction.member_count = f.member_count; status_.friction.active_count = f.active_count;
    status_.friction.waiting_count = f.member_count - f.active_count;
    status_.feed.member_count = d.member_count; status_.feed.active_count = d.active_count;
    status_.feed.waiting_count = d.member_count - d.active_count;
}
ShooterStatus ShooterExecutor::suspend(core::TimeUs now, WaitReason reason, int error) {
    stopFriction(reason, error); stopFeed(reason, error); publish(now); return status_;
}
ShooterStatus ShooterExecutor::update(const ShooterExecutionInputs &in, core::TimeUs now_us) {
    const auto now = now_us / 1000;
    const bool cycle = have_time_ && now_us > previous_us_ && now_us - previous_us_ <= config_.max_cycle_us;
    const float dt = cycle ? float(now_us - previous_us_) / 1e6f : 0;
    previous_us_ = now_us; have_time_ = true;
    const auto finish = [&] { publish(now_us); return status_; };
    if (in.clear_estop && !in.emergency_stop) {
        status_.jammed = false; discardEvent(in.command);
        return suspend(now_us, WaitReason::Command);
    }
    if (!configured_ || in.emergency_stop) {
        discardEvent(in.command);
        return suspend(now_us, !configured_ ? WaitReason::Configuration : WaitReason::Command,
            !configured_ ? configuration_error_ : -ECANCELED);
    }
    const bool requested = in.command.mode != ShooterMode::Disabled &&
        in.command.mode <= ShooterMode::FireContinuous && std::isfinite(in.command.fire_rate_hz) &&
        in.command.fire_rate_hz >= 0 && isFresh(in.command.stamp, now, config_.command_timeout_ms) &&
        core::fresh(in.source_stamp, now_us, config_.source_timeout_us);
    const bool permitted = !in.require_permission || (in.permission.valid && in.permission.enabled &&
        isFresh(in.permission.stamp, now, config_.permission_timeout_ms));
    if (!in.transport_ready || !requested || !permitted) {
        discardEvent(in.command);
        return suspend(now_us, !in.transport_ready ? WaitReason::Transport : !permitted ? WaitReason::Power : WaitReason::Command);
    }
    status_.friction.requested = true;
    int ret = friction_group_.enable();
    const int lr = left_control_.update(config_.friction_speed_rad_s * config_.friction_direction[0], dt);
    const int rr = right_control_.update(config_.friction_speed_rad_s * config_.friction_direction[1], dt);
    if (!ret) ret = lr < 0 ? lr : rr;
    status_.friction.ready = left_control_.telemetry().output_valid || right_control_.telemetry().output_valid;
    status_.friction.state = status_.friction.ready ? RunState::Active : RunState::Recovering;
    status_.friction.reason = !cycle ? WaitReason::Cycle : status_.friction.ready ? WaitReason::None : WaitReason::Feedback;
    status_.friction.error = ret; status_.friction.last_command_sequence = in.command.stamp.sequence;
    const auto lv = left_.snapshot(), rv = right_.snapshot();
    const bool speeds_good = lv.feedback_fresh && rv.feedback_fresh &&
        (lv.feedback.valid & motor::FeedbackVelocity) && (rv.feedback.valid & motor::FeedbackVelocity) &&
        std::fabs(lv.feedback.velocity_rad_s - config_.friction_speed_rad_s * config_.friction_direction[0]) <= config_.friction_tolerance_rad_s &&
        std::fabs(rv.feedback.velocity_rad_s - config_.friction_speed_rad_s * config_.friction_direction[1]) <= config_.friction_tolerance_rad_s;
    if (!speeds_good) friction_good_since_ms_ = 0;
    else if (!friction_good_since_ms_) friction_good_since_ms_ = now;
    status_.friction_ready = speeds_good && now >= friction_good_since_ms_ &&
        now - friction_good_since_ms_ >= config_.friction_dwell_ms;
    // These are the existing feed/heat/jam business prerequisites, not motor recovery authorization.
    const bool aiming = !in.require_gimbal || (isFresh(in.gimbal.stamp, now, config_.gimbal_timeout_ms) &&
        in.gimbal.state == RunState::Active && in.gimbal.ready);
    const bool heat_good = !in.require_heat || (isFresh(in.heat.stamp, now, config_.heat_timeout_ms) &&
        std::isfinite(in.heat.heat) && in.heat.heat >= 0 && std::isfinite(in.heat.limit) && in.heat.limit > 0 &&
        std::isfinite(in.heat.cooling_per_s) && in.heat.cooling_per_s >= 0);
    if (!in.allow_feed || !status_.friction_ready || !aiming || !heat_good || status_.jammed) {
        discardEvent(in.command);
        stopFeed(!aiming ? WaitReason::Reference : !heat_good ? WaitReason::Power : WaitReason::Drive, -EAGAIN);
        if (status_.jammed) status_.feed.state = RunState::Blocked;
        return finish();
    }
    status_.feed.requested = true;
    (void)dial_group_.enable();
    auto dial = dial_.snapshot();
    if (!dial.position_reference_valid && dial.feedback_fresh) {
        const bool home = core::fresh(in.dial_home_reference.stamp, now_us, config_.source_timeout_us) &&
            std::isfinite(in.dial_home_reference.value);
        // A relative zero is only a first-start option. Lost multi-turn coordinates require a real reference.
        if (home || (!have_dial_reference_ && config_.allow_relative_dial_reseed)) {
            (void)dial_.reseedPosition(home ? in.dial_home_reference.value : 0);
            dial = dial_.snapshot();
        }
    }
    if (!have_dial_reference_ && dial.feedback_fresh && dial.position_reference_valid &&
        (dial.feedback.valid & motor::FeedbackPosition)) {
        status_.dial_target_rad = dial.feedback.position_rad;
        dial_reference_ = dial.reference_generation; have_dial_reference_ = true;
    }
    const bool dial_available = dial.feedback_fresh && dial.position_reference_valid &&
        (dial.feedback.valid & (motor::FeedbackPosition | motor::FeedbackVelocity)) ==
        (motor::FeedbackPosition | motor::FeedbackVelocity) && dial.state == motor::MotorState::Active;
    if (!have_dial_reference_ || !dial_available || !cycle) {
        discardEvent(in.command); // Do not replay a discrete event received while unavailable.
        (void)dial_control_.update(status_.dial_target_rad, dt);
        status_.feed.state = RunState::Recovering; status_.feed.ready = false;
        status_.feed.reason = !cycle ? WaitReason::Cycle : !dial.position_reference_valid ? WaitReason::Reference : WaitReason::Feedback;
        // A communication gap is not a mechanical jam timeout.
        if (status_.dial_busy) step_started_ms_ = now;
        return finish();
    }
    if (in.require_heat && in.heat.stamp.sequence != heat_sequence_) {
        heat_sequence_ = in.heat.stamp.sequence;
        // TODO(referee): validate measurement latency before consuming reservations.
        if (in.heat.stamp.timestamp_ms > last_shot_ms_) status_.reserved_heat = 0;
    }
    if (status_.dial_busy) {
        const bool settled = std::fabs(static_cast<double>(dial.feedback.position_rad) - status_.dial_target_rad) <= static_cast<double>(config_.dial_tolerance_rad) &&
            std::fabs(dial.feedback.velocity_rad_s) <= config_.dial_settle_velocity_rad_s;
        if (!settled) dial_good_since_ms_ = 0;
        else if (!dial_good_since_ms_) dial_good_since_ms_ = now;
        if (settled && now - dial_good_since_ms_ >= config_.dial_settle_ms) status_.dial_busy = false;
        if (status_.dial_busy && now - step_started_ms_ >= config_.jam_timeout_ms) {
            status_.jammed = true; discardEvent(in.command); stopFeed(WaitReason::Drive, -ETIMEDOUT);
            status_.feed.state = RunState::Blocked; return finish();
        }
    }
    const bool new_single = in.command.mode == ShooterMode::FireSingle && in.command.fire_event_id &&
        (!status_.last_event_id || sequenceAfter(in.command.fire_event_id, status_.last_event_id)) &&
        isFresh(in.command.fire_event_stamp, now, config_.command_timeout_ms);
    const bool continuous = in.command.mode == ShooterMode::FireContinuous && in.command.fire_rate_hz > 0 && now >= next_shot_ms_;
    const bool budget = !in.require_heat || in.heat.heat + status_.reserved_heat + config_.heat_per_round <= in.heat.limit;
    if ((new_single || continuous) && !status_.dial_busy && budget) {
        status_.dial_target_rad += static_cast<double>(config_.dial_step_rad * config_.dial_direction);
        status_.dial_busy = true; step_started_ms_ = last_shot_ms_ = now; dial_good_since_ms_ = 0;
        ++status_.shots; status_.reserved_heat += config_.heat_per_round;
        const float rate = std::clamp(in.command.fire_rate_hz, 0.1f, config_.max_fire_rate_hz);
        next_shot_ms_ = now + static_cast<std::uint64_t>(1000.0f / rate);
    }
    discardEvent(in.command); // Busy/denied single events are dropped, never replayed.
    ret = dial_control_.update(status_.dial_target_rad, dt);
    if (ret < 0) { stopFeed(WaitReason::Drive, ret); return finish(); }
    status_.feed.state = dial_control_.telemetry().output_valid ? RunState::Active : RunState::Recovering;
    status_.feed.reason = budget ? WaitReason::None : WaitReason::Power;
    status_.feed.ready = dial_control_.telemetry().output_valid;
    status_.feed.last_command_sequence = in.command.stamp.sequence;
    return finish();
}
} // namespace skywalker::robotics
