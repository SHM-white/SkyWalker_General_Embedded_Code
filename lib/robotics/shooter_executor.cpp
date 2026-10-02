#include <robotics/shooter/shooter_executor.hpp>
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
void disable(motor::Group &group) {
    const auto s = group.status();
    if (s.active || s.enable_pending) group.disable();
}
RunStatus run(RecoveryGate &gate, RunState state, WaitReason reason, int error = 0) {
    RunStatus out{};
    out.state = state; out.reason = reason; out.error = error;
    out.generation = gate.generation();
    out.ready = gate.stage() != RecoveryGate::Stage::WaitingPrerequisites &&
                gate.stage() != RecoveryGate::Stage::Blocked;
    return out;
}
int fault(motor::Motor &m, bool explicit_clear) {
    const auto v = m.snapshot();
    if (v.state != motor::MotorState::Fault) return 0;
    if (!explicit_clear && !transient(v.last_fault.reason)) return -EPERM;
    return v.feedback_fresh ? -EAGAIN : -ESTALE;
}
}
ShooterExecutor::ShooterExecutor(motor::Motor &left, motor::Motor &right, motor::Motor &dial,
    motor::Group &friction_group, motor::Group &dial_group,
    const control::VelocityMotor::Config &friction_control,
    const control::PositionMotor::Config &dial_control, const Config &config)
    : left_(left), right_(right), dial_(dial), friction_group_(friction_group), dial_group_(dial_group),
      left_control_(left, friction_control), right_control_(right, friction_control),
      dial_control_(dial, dial_control), config_(config), friction_gate_(config.recovery), feed_gate_(config.recovery) {}

int ShooterExecutor::begin() {
    if (begin_attempted_) return -EALREADY;
    begin_attempted_ = true;
    const bool valid = config_.max_cycle_us && config_.max_cycle_us <= 20000 &&
        config_.recovery.command_timeout_ms && config_.recovery.source_timeout_us &&
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
    disable(friction_group_);
    friction_gate_.withdraw(reason, error);
    friction_good_since_ms_ = 0;
    status_.friction_ready = false;
    status_.friction = run(friction_gate_, RunState::Recovering, reason, error);
}
void ShooterExecutor::stopFeed(WaitReason reason, int error) {
    disable(dial_group_);
    feed_gate_.withdraw(reason, error);
    status_.dial_busy = false;
    step_started_ms_ = dial_good_since_ms_ = next_shot_ms_ = 0;
    have_dial_reference_ = false;
    status_.feed = run(feed_gate_, RunState::Recovering, reason, error);
}
void ShooterExecutor::discardEvent(const ShooterCommand &command) {
    if (command.fire_event_id && (!status_.last_event_id ||
        sequenceAfter(command.fire_event_id, status_.last_event_id))) status_.last_event_id = command.fire_event_id;
}
void ShooterExecutor::publish(core::TimeUs now) {
    const MessageStamp stamp{now / 1000, ++production_sequence_, true};
    status_.stamp = status_.friction.stamp = status_.feed.stamp = stamp;
    status_.friction.generation = friction_gate_.generation();
    status_.feed.generation = feed_gate_.generation();
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
    if (in.emergency_stop) emergency_latched_ = true;
    if (in.clear_fault && !in.emergency_stop) {
        stopFriction(WaitReason::Reference, 0); stopFeed(WaitReason::Reference, 0);
        emergency_latched_ = false; status_.jammed = false; clear_authorized_ = true;
        discardEvent(in.command);
    }
    if (!configured_ || emergency_latched_) {
        discardEvent(in.command);
        suspend(now_us, !configured_ ? WaitReason::Configuration : WaitReason::Drive,
                !configured_ ? (configuration_error_ ? configuration_error_ : -EACCES) : -ECANCELED);
        status_.friction.state = status_.feed.state = RunState::Blocked;
        return finish();
    }
    const bool requested = in.command.mode != ShooterMode::Disabled &&
        in.command.mode <= ShooterMode::FireContinuous && std::isfinite(in.command.fire_rate_hz) &&
        in.command.fire_rate_hz >= 0 && isFresh(in.command.stamp, now, config_.recovery.command_timeout_ms) &&
        core::fresh(in.source_stamp, now_us, config_.recovery.source_timeout_us);
    const bool permitted = !in.require_permission || (in.permission.valid && in.permission.enabled &&
        isFresh(in.permission.stamp, now, config_.permission_timeout_ms));
    if (!in.transport_ready || !cycle || !requested || !permitted) {
        discardEvent(in.command);
        return suspend(now_us, !in.transport_ready ? WaitReason::Transport : !cycle ? WaitReason::Cycle :
                       !permitted ? WaitReason::Power : WaitReason::Command, -EAGAIN);
    }
    const int lf = fault(left_, clear_authorized_), rf = fault(right_, clear_authorized_);
    if (lf || rf) {
        stopFriction(WaitReason::Drive, lf ? lf : rf);
        stopFeed(WaitReason::Drive, -EAGAIN); discardEvent(in.command);
        if (lf == -EPERM || rf == -EPERM) status_.friction.state = RunState::Blocked;
        else if (left_.snapshot().feedback_fresh && right_.snapshot().feedback_fresh)
            (void)friction_group_.clearFault();
        return finish();
    }
    if (!left_.snapshot().feedback_fresh || !right_.snapshot().feedback_fresh) {
        stopFriction(WaitReason::Feedback, -ESTALE); stopFeed(WaitReason::Feedback, -ESTALE);
        discardEvent(in.command); return finish();
    }
    auto friction = friction_group_.status();
    if ((friction_gate_.stage() == RecoveryGate::Stage::Active && !friction.active) ||
        (friction_gate_.stage() == RecoveryGate::Stage::Enabling && !friction.active && !friction.enable_pending))
        stopFriction(WaitReason::Drive, -EAGAIN);
    if (friction_gate_.stage() == RecoveryGate::Stage::WaitingPrerequisites) {
        if (!friction.ready) { stopFeed(WaitReason::Drive, -EAGAIN); discardEvent(in.command); return finish(); }
        int ret = left_control_.reset();
        if (ret == 0) ret = right_control_.reset();
        if (ret < 0) { stopFriction(WaitReason::Reference, ret); return finish(); }
        friction_gate_.prepared(now);
    }
    const auto previous = friction_gate_.stage();
    if (!friction_gate_.accept(in.command.stamp, in.source_stamp, now)) {
        if (previous == RecoveryGate::Stage::Active || previous == RecoveryGate::Stage::Enabling)
            stopFriction(WaitReason::Command, -ESTALE);
        stopFeed(WaitReason::Command, -ESTALE); discardEvent(in.command); return finish();
    }
    if (previous == RecoveryGate::Stage::WaitingCommand) {
        const int ret = friction_group_.enable();
        if (ret < 0) stopFriction(WaitReason::Drive, ret);
        status_.friction = run(friction_gate_, RunState::Recovering, WaitReason::Drive, ret);
        discardEvent(in.command); return finish();
    }
    if (!friction_group_.active()) { discardEvent(in.command); return finish(); }
    friction_gate_.enabled();
    int ret = left_control_.update(config_.friction_speed_rad_s * config_.friction_direction[0], dt);
    if (ret == 0) ret = right_control_.update(config_.friction_speed_rad_s * config_.friction_direction[1], dt);
    if (ret < 0) { discardEvent(in.command); return suspend(now_us, WaitReason::Drive, ret); }
    status_.friction = run(friction_gate_, RunState::Active, WaitReason::None);
    status_.friction.last_command_sequence = in.command.stamp.sequence;
    const bool speeds_good = std::fabs(left_.snapshot().feedback.velocity_rad_s -
        config_.friction_speed_rad_s * config_.friction_direction[0]) <= config_.friction_tolerance_rad_s &&
        std::fabs(right_.snapshot().feedback.velocity_rad_s -
        config_.friction_speed_rad_s * config_.friction_direction[1]) <= config_.friction_tolerance_rad_s;
    if (!speeds_good) friction_good_since_ms_ = 0;
    else if (!friction_good_since_ms_) friction_good_since_ms_ = now;
    status_.friction_ready = speeds_good && now >= friction_good_since_ms_ &&
        now - friction_good_since_ms_ >= config_.friction_dwell_ms;
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
    const auto dial = dial_.snapshot();
    const int df = fault(dial_, clear_authorized_);
    if (df) {
        stopFeed(WaitReason::Drive, df); discardEvent(in.command);
        if (df == -EPERM) status_.feed.state = RunState::Blocked;
        else if (dial.feedback_fresh) (void)dial_group_.clearFault();
        return finish();
    }
    clear_authorized_ = false;
    if (!dial.feedback_fresh) { stopFeed(WaitReason::Feedback, -ESTALE); discardEvent(in.command); return finish(); }
    if (have_dial_reference_ && dial.reference_generation != dial_reference_) {
        stopFeed(WaitReason::Reference, -ESTALE); discardEvent(in.command); return finish();
    }
    auto feed = dial_group_.status();
    if ((feed_gate_.stage() == RecoveryGate::Stage::Active && !feed.active) ||
        (feed_gate_.stage() == RecoveryGate::Stage::Enabling && !feed.active && !feed.enable_pending)) {
        stopFeed(WaitReason::Drive, -EAGAIN); discardEvent(in.command); return finish();
    }
    if (feed_gate_.stage() == RecoveryGate::Stage::WaitingPrerequisites) {
        discardEvent(in.command);
        if (!feed.ready) return finish();
        if (!dial.position_reference_valid) {
            const bool home = core::fresh(in.dial_home_reference.stamp, now_us, config_.recovery.source_timeout_us) &&
                              std::isfinite(in.dial_home_reference.value);
            if (!home && !config_.allow_relative_dial_reseed) {
                stopFeed(WaitReason::Reference, -ENODATA); return finish();
            }
            ret = dial_.reseedPosition(home ? in.dial_home_reference.value : 0);
            if (ret < 0) { stopFeed(WaitReason::Reference, ret); return finish(); }
        }
        ret = dial_control_.reset();
        if (ret < 0) { stopFeed(WaitReason::Reference, ret); return finish(); }
        const auto view = dial_.snapshot();
        status_.dial_target_rad = view.feedback.position_rad;
        dial_reference_ = view.reference_generation; have_dial_reference_ = true;
        feed_gate_.prepared(now);
    }
    const auto feed_previous = feed_gate_.stage();
    if (!feed_gate_.accept(in.command.stamp, in.source_stamp, now)) {
        if (feed_previous == RecoveryGate::Stage::Active || feed_previous == RecoveryGate::Stage::Enabling)
            stopFeed(WaitReason::Command, -ESTALE);
        discardEvent(in.command); return finish();
    }
    if (feed_previous == RecoveryGate::Stage::WaitingCommand) {
        ret = dial_group_.enable();
        if (ret < 0) stopFeed(WaitReason::Drive, ret);
        status_.feed = run(feed_gate_, RunState::Recovering, WaitReason::Drive, ret);
        discardEvent(in.command); return finish();
    }
    if (!dial_group_.active()) { discardEvent(in.command); return finish(); }
    feed_gate_.enabled();
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
        isFresh(in.command.fire_event_stamp, now, config_.recovery.command_timeout_ms) &&
        in.command.fire_event_stamp.timestamp_ms > feed_gate_.boundaryUs() / 1000;
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
    status_.feed = run(feed_gate_, RunState::Active, budget ? WaitReason::None : WaitReason::Power);
    status_.feed.last_command_sequence = in.command.stamp.sequence;
    return finish();
}
} // namespace skywalker::robotics
