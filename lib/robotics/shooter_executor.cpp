#include <robotics/shooter/shooter_executor.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>

namespace skywalker::robotics {
namespace {
control::PositionMotor::Config dialPositionConfig(control::PositionMotor::Config config, float speed_limit) {
    const float limit = std::min(speed_limit, config.loop.velocity.requested_velocity_abs_max_rad_s);
    config.loop.velocity.requested_velocity_abs_max_rad_s = limit;
    config.loop.position.output_min = std::max(config.loop.position.output_min, -limit);
    config.loop.position.output_max = std::min(config.loop.position.output_max, limit);
    return config;
}
}
ShooterExecutor::ShooterExecutor(motor::Motor &left, motor::Motor &right, motor::Motor &dial,
                                 motor::Group &friction_group, motor::Group &dial_group,
                                 const control::VelocityMotor::Config &friction_control,
                                 const control::PositionMotor::Config &dial_control, const Config &config)
    : left_(left), right_(right), dial_(dial), friction_group_(friction_group), dial_group_(dial_group),
      left_control_(left, friction_control), right_control_(right, friction_control),
      dial_control_(dial, dialPositionConfig(dial_control, config.dial_speed_rad_s), this),
      dial_velocity_control_(dial, {dial_control.loop.velocity, dial_control.effort_unit}, this), config_(config),
      dial_velocity_limit_rad_s_(std::min(config.dial_speed_rad_s,
                                          dial_control.loop.velocity.requested_velocity_abs_max_rad_s)),
      dial_deceleration_rad_s2_(std::min(dial_control.loop.velocity.reference_slew.rising_rate_per_s,
                                        dial_control.loop.velocity.reference_slew.falling_rate_per_s)) {
}

int ShooterExecutor::begin() {
    if (begin_attempted_)
        return -EALREADY;
    begin_attempted_ = true;
    const bool valid = config_.max_cycle_us && config_.max_cycle_us <= 20000 && config_.command_timeout_ms &&
                       config_.event_timeout_ms && config_.source_timeout_us && config_.permission_timeout_ms &&
                       config_.heat_timeout_ms && config_.gimbal_timeout_ms && config_.friction_dwell_ms &&
                       config_.dial_settle_ms && config_.jam_timeout_ms && config_.dial_brake_timeout_ms &&
                       std::isfinite(config_.friction_speed_rad_s) && config_.friction_speed_rad_s > 0 &&
                       std::isfinite(config_.friction_tolerance_rad_s) && config_.friction_tolerance_rad_s > 0 &&
                       std::isfinite(config_.dial_step_rad) && config_.dial_step_rad > 0 &&
                       std::isfinite(dial_velocity_limit_rad_s_) && dial_velocity_limit_rad_s_ > 0 &&
                       std::isfinite(dial_deceleration_rad_s2_) && dial_deceleration_rad_s2_ > 0 &&
                       std::isfinite(config_.dial_tolerance_rad) && config_.dial_tolerance_rad > 0 &&
                       std::isfinite(config_.dial_settle_velocity_rad_s) && config_.dial_settle_velocity_rad_s > 0 &&
                       std::isfinite(config_.dial_phase_tolerance_rad) && config_.dial_phase_tolerance_rad >= 0 &&
                       config_.dial_phase_tolerance_rad < config_.dial_step_rad * 0.5f &&
                       std::isfinite(config_.heat_per_round) && config_.heat_per_round > 0 &&
                       std::isfinite(config_.max_fire_rate_hz) && config_.max_fire_rate_hz > 0 &&
                       std::fabs(config_.friction_direction[0]) == 1 &&
                       std::fabs(config_.friction_direction[1]) == 1 && std::fabs(config_.dial_direction) == 1 &&
                       dial_control_.reference() == control::PositionReference::DriverContinuous;
    int ret = valid ? left_control_.configure() : -EINVAL;
    if (!ret)
        ret = right_control_.configure();
    if (!ret)
        ret = dial_control_.configure();
    if (!ret)
        ret = dial_velocity_control_.configure();
    configuration_error_ = ret;
    configured_ = ret == 0;
    return ret;
}

void ShooterExecutor::stopFriction(WaitReason reason, int error) {
    friction_group_.disable();
    friction_good_since_ms_ = 0;
    status_.friction_ready = false;
    status_.friction.requested = false;
    status_.friction.state = RunState::Disabled;
    status_.friction.reason = reason;
    status_.friction.error = error;
    status_.friction.ready = false;
}

void ShooterExecutor::setDialMode(DialControlMode mode) {
    const auto positionMode = [](DialControlMode m) {
        return m == DialControlMode::SinglePosition || m == DialControlMode::HoldPosition;
    };
    if (mode != status_.dial_mode &&
        (mode == DialControlMode::Disabled || status_.dial_mode == DialControlMode::Disabled ||
         positionMode(mode) != positionMode(status_.dial_mode))) {
        // Shared producer identity does not allow concurrent writers. Switching
        // invalidates the previous staged effort and both integral histories.
        (void)dial_control_.reset();
        (void)dial_velocity_control_.reset();
    }
    status_.dial_mode = mode;
}

void ShooterExecutor::stopFeed(WaitReason reason, int error) {
    dial_group_.disable();
    setDialMode(DialControlMode::Disabled);
    status_.dial_requested_velocity_rad_s = 0;
    status_.dial_position_error_rad = 0;
    status_.dial_busy = false;
    status_.active_single_event_id = 0;
    single_started_us_ = dial_settled_since_us_ = brake_started_us_ = progress_since_us_ = 0;
    continuous_budget_end_rad_ = 0;
    const auto dial = dial_.snapshot();
    if (dial.feedback_fresh && dial.position_reference_valid && (dial.feedback.valid & motor::FeedbackPosition) &&
        std::isfinite(dial.feedback.position_rad))
        status_.dial_target_rad = dial.feedback.position_rad;
    status_.feed.requested = false;
    status_.feed.ready = false;
    status_.feed.state = RunState::Disabled;
    status_.feed.reason = reason;
    status_.feed.error = error;
}

bool ShooterExecutor::newEvent(const ShooterCommand &command) const {
    return command.fire_event_id &&
           (!status_.last_event_id || sequenceAfter(command.fire_event_id, status_.last_event_id));
}

void ShooterExecutor::consumeEvent(const ShooterCommand &command, bool accepted) {
    if (!newEvent(command))
        return;
    status_.last_event_id = command.fire_event_id;
    if (accepted) {
        status_.last_accepted_event_id = command.fire_event_id;
        ++status_.accepted_single_events;
    }
    else {
        status_.last_rejected_event_id = command.fire_event_id;
        ++status_.rejected_single_events;
    }
}

double ShooterExecutor::nextIndex(double measured_position) const {
    const double step = config_.dial_step_rad;
    const double phase = static_cast<double>(config_.dial_direction) *
                         (measured_position - dial_phase_origin_rad_) / step;
    const double nearest = std::round(phase);
    // A stopped phase close to an index starts the following full index.
    // Between indexes, advance only to the next forward mechanical phase.
    const double next = std::fabs(phase - nearest) * step <= static_cast<double>(config_.dial_phase_tolerance_rad)
                            ? nearest + 1
                            : std::floor(phase) + 1;
    return dial_phase_origin_rad_ + static_cast<double>(config_.dial_direction) * next * step;
}

void ShooterExecutor::publish(core::TimeUs now) {
    const MessageStamp stamp{now / 1000, ++production_sequence_, true};
    status_.stamp = status_.friction.stamp = status_.feed.stamp = stamp;
    const auto f = friction_group_.status(), d = dial_group_.status();
    status_.friction.member_count = f.member_count;
    status_.friction.active_count = f.active_count;
    status_.friction.waiting_count = f.member_count - f.active_count;
    status_.feed.member_count = d.member_count;
    status_.feed.active_count = d.active_count;
    status_.feed.waiting_count = d.member_count - d.active_count;
    const auto dial = dial_.snapshot();
    status_.dial_feedback_valid = dial.feedback_fresh && dial.position_reference_valid &&
                                  (dial.feedback.valid & (motor::FeedbackPosition | motor::FeedbackVelocity)) ==
                                      (motor::FeedbackPosition | motor::FeedbackVelocity) &&
                                  std::isfinite(dial.feedback.position_rad) &&
                                  std::isfinite(dial.feedback.velocity_rad_s);
    if (status_.dial_feedback_valid) {
        status_.dial_position_rad = dial.feedback.position_rad;
        status_.dial_actual_velocity_rad_s = dial.feedback.velocity_rad_s;
    }
}

ShooterStatus ShooterExecutor::suspend(core::TimeUs now, WaitReason reason, int error) {
    stopFriction(reason, error);
    stopFeed(reason, error);
    publish(now);
    return status_;
}

ShooterStatus ShooterExecutor::update(const ShooterExecutionInputs &in, core::TimeUs now_us) {
    const auto now = now_us / 1000;
    const bool cycle = have_time_ && now_us > previous_us_ && now_us - previous_us_ <= config_.max_cycle_us;
    const float dt = cycle ? float(now_us - previous_us_) / 1e6f : 0;
    previous_us_ = now_us;
    have_time_ = true;
    status_.dial_requested_velocity_rad_s = 0;
    const auto finish = [&] {
        publish(now_us);
        return status_;
    };
    if (in.clear_estop && !in.emergency_stop) {
        status_.jammed = false;
        consumeEvent(in.command);
        return suspend(now_us, WaitReason::Command);
    }
    if (!configured_ || in.emergency_stop) {
        consumeEvent(in.command);
        return suspend(now_us, !configured_ ? WaitReason::Configuration : WaitReason::Command,
                       !configured_ ? configuration_error_ : -ECANCELED);
    }
    const bool requested = in.command.mode != ShooterMode::Disabled && in.command.mode <= ShooterMode::FireContinuous &&
                           std::isfinite(in.command.fire_rate_hz) && in.command.fire_rate_hz >= 0 &&
                           isFresh(in.command.stamp, now, config_.command_timeout_ms) &&
                           core::fresh(in.source_stamp, now_us, config_.source_timeout_us);
    const bool permitted = !in.require_permission || (in.permission.valid && in.permission.enabled &&
                                                      isFresh(in.permission.stamp, now, config_.permission_timeout_ms));
    if (!in.transport_ready || !requested || !permitted || !cycle) {
        consumeEvent(in.command);
        return suspend(now_us, !in.transport_ready ? WaitReason::Transport
                               : !permitted        ? WaitReason::Power
                               : !cycle            ? WaitReason::Cycle
                                                   : WaitReason::Command);
    }

    status_.friction.requested = true;
    int ret = friction_group_.enable();
    const int lr = left_control_.update(config_.friction_speed_rad_s * config_.friction_direction[0], dt);
    const int rr = right_control_.update(config_.friction_speed_rad_s * config_.friction_direction[1], dt);
    if (!ret)
        ret = lr < 0 ? lr : rr;
    const auto lv = left_.snapshot(), rv = right_.snapshot();
    if (ret < 0 || lv.state == motor::MotorState::Fault || rv.state == motor::MotorState::Fault) {
        consumeEvent(in.command);
        return suspend(now_us, WaitReason::Drive, ret < 0 ? ret : -EIO);
    }
    status_.friction.ready = left_control_.telemetry().output_valid && right_control_.telemetry().output_valid;
    status_.friction.state = status_.friction.ready ? RunState::Active : RunState::Recovering;
    status_.friction.reason = status_.friction.ready ? WaitReason::None : WaitReason::Feedback;
    status_.friction.error = ret;
    status_.friction.last_command_sequence = in.command.stamp.sequence;
    const bool speeds_good = lv.feedback_fresh && rv.feedback_fresh && (lv.feedback.valid & motor::FeedbackVelocity) &&
                             (rv.feedback.valid & motor::FeedbackVelocity) &&
                             std::fabs(lv.feedback.velocity_rad_s -
                                       config_.friction_speed_rad_s * config_.friction_direction[0]) <=
                                 config_.friction_tolerance_rad_s &&
                             std::fabs(rv.feedback.velocity_rad_s -
                                       config_.friction_speed_rad_s * config_.friction_direction[1]) <=
                                 config_.friction_tolerance_rad_s;
    if (!speeds_good)
        friction_good_since_ms_ = 0;
    else if (!friction_good_since_ms_)
        friction_good_since_ms_ = now;
    status_.friction_ready = speeds_good && status_.friction.ready && now >= friction_good_since_ms_ &&
                             now - friction_good_since_ms_ >= config_.friction_dwell_ms;
    const bool aiming = !in.require_gimbal || (isFresh(in.gimbal.stamp, now, config_.gimbal_timeout_ms) &&
                                               in.gimbal.state == RunState::Active && in.gimbal.ready);
    const bool heat_good = !in.require_heat ||
                           (isFresh(in.heat.stamp, now, config_.heat_timeout_ms) && std::isfinite(in.heat.heat) &&
                            in.heat.heat >= 0 && std::isfinite(in.heat.limit) && in.heat.limit > 0 &&
                            std::isfinite(in.heat.cooling_per_s) && in.heat.cooling_per_s >= 0);
    if (!in.allow_feed || !status_.friction_ready || !aiming || !heat_good || status_.jammed) {
        consumeEvent(in.command);
        stopFeed(!aiming ? WaitReason::Reference : !heat_good ? WaitReason::Power : WaitReason::Drive, -EAGAIN);
        if (status_.jammed)
            status_.feed.state = RunState::Blocked;
        return finish();
    }

    auto dial = dial_.snapshot();
    if (dial.state == motor::MotorState::Fault) {
        consumeEvent(in.command);
        return suspend(now_us, WaitReason::Drive, -EIO);
    }
    if (!dial.position_reference_valid && dial.feedback_fresh) {
        const bool home = core::fresh(in.dial_home_reference.stamp, now_us, config_.source_timeout_us) &&
                          std::isfinite(in.dial_home_reference.value);
        // Relative reseeding remains restricted to the first empty-bench start.
        if (home || (!have_dial_reference_ && config_.allow_relative_dial_reseed)) {
            (void)dial_.reseedPosition(home ? in.dial_home_reference.value : 0);
            dial = dial_.snapshot();
        }
    }
    const bool dial_available = dial.feedback_fresh && dial.position_reference_valid &&
                                (dial.feedback.valid & (motor::FeedbackPosition | motor::FeedbackVelocity)) ==
                                    (motor::FeedbackPosition | motor::FeedbackVelocity) &&
                                std::isfinite(dial.feedback.position_rad) &&
                                std::isfinite(dial.feedback.velocity_rad_s);
    if (!dial_available) {
        consumeEvent(in.command);
        stopFeed(!dial.position_reference_valid ? WaitReason::Reference : WaitReason::Feedback, -EAGAIN);
        status_.feed.state = RunState::Recovering;
        return finish();
    }
    if (!have_dial_reference_ || dial.reference_generation != dial_reference_) {
        const bool changed = have_dial_reference_;
        stopFeed(WaitReason::Reference, 0);
        dial_reference_ = dial.reference_generation;
        dial_phase_origin_rad_ = dial.feedback.position_rad;
        status_.dial_target_rad = dial.feedback.position_rad;
        have_dial_reference_ = true;
        if (changed) {
            // Old positions and accepted travel never survive coordinate changes.
            consumeEvent(in.command);
            status_.feed.state = RunState::Recovering;
            return finish();
        }
    }
    ret = dial_group_.enable();
    if (ret < 0) {
        consumeEvent(in.command);
        return suspend(now_us, WaitReason::Drive, ret);
    }
    dial = dial_.snapshot();
    if (dial.state != motor::MotorState::Active || !dial.output_permitted) {
        consumeEvent(in.command);
        status_.feed.state = RunState::Recovering;
        status_.feed.ready = false;
        status_.feed.reason = WaitReason::Drive;
        status_.feed.error = 0;
        return finish();
    }

    if (in.require_heat) {
        if (!have_heat_model_) {
            heat_model_ = in.heat.heat + status_.reserved_heat;
            have_heat_model_ = true;
        }
        else if (now_us > heat_updated_us_) {
            heat_model_ = std::max(0.0f, heat_model_ - in.heat.cooling_per_s *
                                                            float(now_us - heat_updated_us_) / 1e6f);
        }
        // A new referee sample can raise the conservative prediction, never
        // erase unacknowledged reservations merely because its stamp is newer.
        if (in.heat.stamp.sequence != heat_sequence_) {
            heat_sequence_ = in.heat.stamp.sequence;
            heat_model_ = std::max(heat_model_, in.heat.heat);
        }
        heat_updated_us_ = now_us;
        status_.reserved_heat = std::max(0.0f, heat_model_ - in.heat.heat);
    }
    const auto heatAvailable = [&] {
        return !in.require_heat || heat_model_ + config_.heat_per_round <= in.heat.limit;
    };
    const auto reserveRound = [&] {
        ++status_.shots;
        if (in.require_heat) {
            heat_model_ += config_.heat_per_round;
            status_.reserved_heat = std::max(0.0f, heat_model_ - in.heat.heat);
        }
        else
            status_.reserved_heat += config_.heat_per_round;
    };
    const bool continuous = in.command.mode == ShooterMode::FireContinuous && in.command.fire_rate_hz > 0;
    const float continuous_speed = std::min(dial_velocity_limit_rad_s_,
                                            std::min(in.command.fire_rate_hz, config_.max_fire_rate_hz) *
                                                config_.dial_step_rad);
    const auto settledFor = [&](bool settled) {
        if (!settled)
            dial_settled_since_us_ = 0;
        else if (!dial_settled_since_us_)
            dial_settled_since_us_ = now_us;
        return settled && now_us >= dial_settled_since_us_ &&
               now_us - dial_settled_since_us_ >= core::TimeUs(config_.dial_settle_ms) * 1000;
    };
    const auto startBraking = [&](WaitReason reason) {
        setDialMode(DialControlMode::BrakingVelocity);
        braking_reason_ = reason;
        brake_started_us_ = now_us;
        dial_settled_since_us_ = 0;
        status_.dial_target_rad = dial.feedback.position_rad;
    };

    if (status_.dial_mode == DialControlMode::Disabled) {
        status_.dial_target_rad = dial.feedback.position_rad;
        setDialMode(DialControlMode::HoldPosition);
    }
    if (status_.dial_mode == DialControlMode::SinglePosition) {
        const bool arrived = std::fabs(status_.dial_target_rad - static_cast<double>(dial.feedback.position_rad)) <=
                                 static_cast<double>(config_.dial_tolerance_rad) &&
                             std::fabs(dial.feedback.velocity_rad_s) <= config_.dial_settle_velocity_rad_s;
        if (settledFor(arrived)) {
            ++status_.completed_single_events;
            status_.active_single_event_id = 0;
            dial_settled_since_us_ = 0;
            setDialMode(DialControlMode::HoldPosition);
        }
        else if (now_us >= single_started_us_ &&
                 now_us - single_started_us_ >= core::TimeUs(config_.jam_timeout_ms) * 1000) {
            status_.jammed = true;
            consumeEvent(in.command);
            stopFeed(WaitReason::Drive, -ETIMEDOUT);
            status_.feed.state = RunState::Blocked;
            return finish();
        }
    }
    if (status_.dial_mode == DialControlMode::ContinuousVelocity && !continuous)
        startBraking(WaitReason::Command);
    if (status_.dial_mode == DialControlMode::BrakingVelocity) {
        if (settledFor(std::fabs(dial.feedback.velocity_rad_s) <= config_.dial_settle_velocity_rad_s)) {
            // Hold this measured stopping position; no accumulated position
            // trajectory is allowed to restart motion after continuous fire.
            status_.dial_target_rad = dial.feedback.position_rad;
            continuous_budget_end_rad_ = 0;
            dial_settled_since_us_ = 0;
            setDialMode(DialControlMode::HoldPosition);
        }
        else if (now_us >= brake_started_us_ &&
                 now_us - brake_started_us_ >= core::TimeUs(config_.dial_brake_timeout_ms) * 1000) {
            status_.jammed = true;
            consumeEvent(in.command);
            stopFeed(WaitReason::Drive, -ETIMEDOUT);
            status_.feed.state = RunState::Blocked;
            return finish();
        }
    }

    if (newEvent(in.command)) {
        const bool accept = in.command.mode == ShooterMode::FireSingle &&
                            isFresh(in.command.fire_event_stamp, now, config_.event_timeout_ms) &&
                            status_.dial_mode == DialControlMode::HoldPosition &&
                            std::fabs(dial.feedback.velocity_rad_s) <= config_.dial_settle_velocity_rad_s &&
                            heatAvailable();
        consumeEvent(in.command, accept);
        if (accept) {
            reserveRound();
            status_.dial_target_rad = nextIndex(dial.feedback.position_rad);
            status_.active_single_event_id = in.command.fire_event_id;
            single_started_us_ = now_us;
            dial_settled_since_us_ = 0;
            setDialMode(DialControlMode::SinglePosition);
        }
    }

    WaitReason feed_reason = WaitReason::None;
    if (continuous && status_.dial_mode == DialControlMode::HoldPosition) {
        if (heatAvailable()) {
            reserveRound();
            continuous_budget_end_rad_ = static_cast<double>(config_.dial_direction) *
                                         nextIndex(dial.feedback.position_rad);
            progress_position_rad_ = dial.feedback.position_rad;
            progress_since_us_ = now_us;
            setDialMode(DialControlMode::ContinuousVelocity);
        }
        else
            feed_reason = WaitReason::Power;
    }
    if (status_.dial_mode == DialControlMode::ContinuousVelocity) {
        const double position = config_.dial_direction * dial.feedback.position_rad;
        const double speed = std::max(static_cast<double>(continuous_speed),
                                      std::max(0.0, static_cast<double>(config_.dial_direction) *
                                                        static_cast<double>(dial.feedback.velocity_rad_s)));
        // Reserve each forthcoming mechanical index, including the predicted
        // stopping distance. These are authorizations, not detected projectiles.
        // 调参：减速距离使用速度参考斜率的保守较小值；实际机械刹停距离仍需无弹标定。
        const double lookahead = speed * speed / (2.0 * static_cast<double>(dial_deceleration_rad_s2_)) +
                                 speed * static_cast<double>(dt) +
                                 static_cast<double>(config_.dial_phase_tolerance_rad);
        std::uint32_t reservations_this_cycle = 0;
        while (position + lookahead >= continuous_budget_end_rad_) {
            // Bound work even if invalid mechanical scaling or a discontinuous
            // feedback coordinate would otherwise request an enormous catch-up.
            if (++reservations_this_cycle > 64)
                return suspend(now_us, WaitReason::Configuration, -ERANGE);
            if (!heatAvailable()) {
                startBraking(WaitReason::Power);
                break;
            }
            reserveRound();
            continuous_budget_end_rad_ += static_cast<double>(config_.dial_step_rad);
        }
        if (status_.dial_mode == DialControlMode::ContinuousVelocity) {
            if (static_cast<double>(config_.dial_direction) *
                    (static_cast<double>(dial.feedback.position_rad) - progress_position_rad_) >=
                static_cast<double>(config_.dial_tolerance_rad)) {
                progress_position_rad_ = dial.feedback.position_rad;
                progress_since_us_ = now_us;
            }
            else if (now_us >= progress_since_us_ &&
                     now_us - progress_since_us_ >= core::TimeUs(config_.jam_timeout_ms) * 1000) {
                status_.jammed = true;
                stopFeed(WaitReason::Drive, -ETIMEDOUT);
                status_.feed.state = RunState::Blocked;
                return finish();
            }
        }
    }

    const bool velocity_mode = status_.dial_mode == DialControlMode::ContinuousVelocity ||
                               status_.dial_mode == DialControlMode::BrakingVelocity;
    bool output_valid = false;
    if (velocity_mode) {
        status_.dial_target_rad = dial.feedback.position_rad;
        status_.dial_position_error_rad = 0;
        status_.dial_requested_velocity_rad_s = status_.dial_mode == DialControlMode::ContinuousVelocity
                                                   ? config_.dial_direction * continuous_speed
                                                   : 0;
        ret = dial_velocity_control_.update(status_.dial_requested_velocity_rad_s, dt);
        output_valid = dial_velocity_control_.telemetry().output_valid;
        if (status_.dial_mode == DialControlMode::BrakingVelocity)
            feed_reason = braking_reason_;
    }
    else {
        status_.dial_position_error_rad = status_.dial_target_rad - static_cast<double>(dial.feedback.position_rad);
        ret = dial_control_.update(status_.dial_target_rad, dt);
        const auto telemetry = dial_control_.telemetry();
        output_valid = telemetry.output_valid;
        status_.dial_requested_velocity_rad_s = output_valid ? telemetry.output.velocity.velocity_reference_rad_s : 0;
    }
    if (ret < 0)
        return suspend(now_us, WaitReason::Drive, ret);
    status_.dial_busy = status_.dial_mode == DialControlMode::SinglePosition || velocity_mode;
    status_.feed.requested = status_.dial_busy;
    status_.feed.state = output_valid ? RunState::Active : RunState::Recovering;
    status_.feed.reason = output_valid ? feed_reason : WaitReason::Drive;
    status_.feed.error = ret;
    status_.feed.ready = output_valid;
    status_.feed.last_command_sequence = in.command.stamp.sequence;
    return finish();
}
} // namespace skywalker::robotics
