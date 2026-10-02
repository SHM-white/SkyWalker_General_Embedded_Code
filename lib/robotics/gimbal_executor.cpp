#include <robotics/gimbal/gimbal_executor.hpp>

#include <cerrno>
#include <cmath>

namespace skywalker::robotics {
namespace {
bool transient(motor::FaultReason reason) {
    switch (reason) {
    case motor::FaultReason::FeedbackExpired:
    case motor::FaultReason::CommandExpired:
    case motor::FaultReason::UnexpectedDisabled:
    case motor::FaultReason::EnableTimeout:
    case motor::FaultReason::TransportError:
    case motor::FaultReason::RxOverflow:
        return true;
    default:
        return false;
    }
}
bool validCommand(const GimbalCommand &command) {
    return command.mode <= GimbalMode::AbsoluteAngle && std::isfinite(command.yaw_target_rad) &&
           std::isfinite(command.pitch_target_rad) && std::isfinite(command.yaw_rate_rad_s) &&
           std::isfinite(command.pitch_rate_rad_s);
}
} // namespace

GimbalExecutor::GimbalExecutor(motor::Motor &yaw_drive, motor::Motor &pitch_drive, motor::Group &group,
                               const control::PositionMotor::Config &yaw_motor, const GimbalAxisConfig &yaw_axis,
                               const control::PositionMotor::Config &pitch_motor, const GimbalAxisConfig &pitch_axis,
                               const Config &config)
    : yaw_drive_(yaw_drive), pitch_drive_(pitch_drive), group_(group), yaw_(yaw_drive, yaw_motor, yaw_axis),
      pitch_(pitch_drive, pitch_motor, pitch_axis), config_(config), recovery_(config.recovery) {}

int GimbalExecutor::begin() {
    if (begin_attempted_)
        return -EALREADY;
    begin_attempted_ = true;
    int ret = config_.max_cycle_us == 0 || config_.max_cycle_us > 20000 || config_.fault_retry_ms == 0 ||
                      config_.permission_timeout_ms == 0 || config_.recovery.command_timeout_ms == 0 ||
                      config_.recovery.source_timeout_us == 0 ? -EINVAL : yaw_.validate();
    if (ret == 0)
        ret = pitch_.validate();
    if (ret == 0)
        ret = yaw_.begin();
    if (ret == 0)
        ret = pitch_.begin();
    config_error_ = ret;
    configured_ = ret == 0;
    return ret;
}

void GimbalExecutor::withdraw(WaitReason reason, int error, bool blocked) {
    const auto group = group_.status();
    const auto yaw_state = yaw_drive_.snapshot().state;
    const auto pitch_state = pitch_drive_.snapshot().state;
    if (group.active || group.enable_pending || yaw_state == motor::MotorState::Active ||
        yaw_state == motor::MotorState::Enabling || pitch_state == motor::MotorState::Active ||
        pitch_state == motor::MotorState::Enabling)
        group_.disable();
    // Repeated disable while already stopped would restart protocol stopping
    // and cancel an asynchronous clear-fault handshake.
    recovery_.withdraw(reason, error, blocked);
    have_reference_ = false;
    status_.ready = false;
    if (blocked) {
        hard_error_ = error == 0 ? -EIO : error;
        hard_reason_ = reason;
    }
}

RunStatus GimbalExecutor::publish(core::TimeUs now_us, RunState state, WaitReason reason, int error) {
    status_.state = state;
    status_.reason = reason;
    status_.error = error;
    status_.generation = recovery_.generation();
    status_.stamp = {now_us / 1000, ++production_sequence_, true};
    return status_;
}

RunStatus GimbalExecutor::suspend(core::TimeUs now_us, WaitReason reason, int error, bool blocked) {
    withdraw(reason, error, blocked);
    return publish(now_us, blocked ? RunState::Blocked : RunState::Recovering, reason, error);
}

RunStatus GimbalExecutor::update(const GimbalExecutionInputs &inputs, core::TimeUs now_us) {
    const auto now_ms = now_us / 1000;
    const bool cycle_valid = have_time_ && now_us > previous_us_ && now_us - previous_us_ <= config_.max_cycle_us;
    const float dt_s = cycle_valid ? float(now_us - previous_us_) / 1000000.0f : 0.0f;
    previous_us_ = now_us;
    have_time_ = true;
    status_.ready = false;

    if (inputs.emergency_stop)
        emergency_latched_ = true;
    if (inputs.clear_fault && !inputs.emergency_stop) {
        withdraw(WaitReason::Reference);
        const int ret = configured_ ? group_.clearFault() : -EACCES;
        if (ret == 0) {
            emergency_latched_ = false;
            hard_error_ = 0;
            explicit_clear_pending_ = true;
            recovery_.withdraw(WaitReason::Reference);
            have_reference_ = false;
            retry_ms_ = now_ms + config_.fault_retry_ms;
        }
        else {
            withdraw(WaitReason::Drive, ret, true);
        }
    }
    if (!configured_) {
        const int error = config_error_ == 0 ? -EACCES : config_error_;
        withdraw(WaitReason::Configuration, error, true);
        return publish(now_us, RunState::Blocked, WaitReason::Configuration, error);
    }
    if (emergency_latched_) {
        withdraw(WaitReason::Drive, -ECANCELED, true);
        return publish(now_us, RunState::Blocked, WaitReason::Drive, -ECANCELED);
    }
    if (hard_error_ < 0) {
        withdraw(hard_reason_, hard_error_, true);
        return publish(now_us, RunState::Blocked, hard_reason_, hard_error_);
    }
    if (!inputs.transport_ready)
        return suspend(now_us, WaitReason::Transport, -EAGAIN);
    if (!cycle_valid)
        return suspend(now_us, WaitReason::Cycle, -ESTALE);
    if (!validCommand(inputs.command))
        return suspend(now_us, WaitReason::Command, -EINVAL, true);
    if (inputs.require_permission && (!inputs.permission.valid || !inputs.permission.enabled ||
        !isFresh(inputs.permission.stamp, now_ms, config_.permission_timeout_ms)))
        return suspend(now_us, WaitReason::Power, -EACCES);

    auto yaw_view = yaw_drive_.snapshot();
    auto pitch_view = pitch_drive_.snapshot();
    const bool fault = yaw_view.state == motor::MotorState::Fault || pitch_view.state == motor::MotorState::Fault;
    if (fault) {
        const auto check = [&](const motor::MotorSnapshot &view) {
            return view.state != motor::MotorState::Fault || transient(view.last_fault.reason);
        };
        const bool recoverable = explicit_clear_pending_ || (check(yaw_view) && check(pitch_view));
        const auto &info = yaw_view.state == motor::MotorState::Fault ? yaw_view.last_fault : pitch_view.last_fault;
        withdraw(WaitReason::Drive, info.error, !recoverable);
        if (recoverable && yaw_view.feedback_fresh && pitch_view.feedback_fresh && now_ms >= retry_ms_) {
            retry_ms_ = now_ms + config_.fault_retry_ms;
            const int ret = group_.clearFault();
            if (ret < 0 && ret != -EAGAIN && ret != -EBUSY && ret != -EALREADY)
                return publish(now_us, RunState::Recovering, WaitReason::Drive, ret);
        }
        return publish(now_us, recoverable ? RunState::Recovering : RunState::Blocked,
                       WaitReason::Drive, info.error);
    }
    explicit_clear_pending_ = false;
    if (have_reference_ && (yaw_view.reference_generation != yaw_reference_ ||
                            pitch_view.reference_generation != pitch_reference_))
        return suspend(now_us, WaitReason::Reference, -ESTALE);

    const auto yaw_status = yaw_.poll(now_ms);
    const auto pitch_status = pitch_.poll(now_ms);
    if (!yaw_status.feedback_healthy || !pitch_status.feedback_healthy) {
        const int ret = !yaw_status.feedback_healthy ? yaw_status.error : pitch_status.error;
        return suspend(now_us, WaitReason::Feedback, ret, ret == -ERANGE || ret == -EINVAL);
    }

    const bool requested = inputs.command.mode != GimbalMode::Disabled;
    auto group = group_.status();
    if (!requested && (group.active || group.enable_pending)) {
        withdraw(WaitReason::Command);
        group = group_.status();
    }
    // A dropped driver handshake/active state requires a new reference and
    // source production boundary even if no protocol fault was exposed.
    if ((recovery_.stage() == RecoveryGate::Stage::Active && !group.active) ||
        (recovery_.stage() == RecoveryGate::Stage::Enabling && !group.active && !group.enable_pending))
        return suspend(now_us, WaitReason::Drive, -EAGAIN);

    if (recovery_.stage() == RecoveryGate::Stage::WaitingPrerequisites) {
        if (!yaw_status.ready_for_enable || !pitch_status.ready_for_enable || !group.ready || now_ms < retry_ms_)
            return publish(now_us, RunState::Recovering, WaitReason::Reference, -EAGAIN);
        int ret = yaw_.reset();
        if (ret == 0)
            ret = pitch_.reset();
        if (ret < 0)
            return suspend(now_us, WaitReason::Reference, ret);
        yaw_reference_ = yaw_drive_.snapshot().reference_generation;
        pitch_reference_ = pitch_drive_.snapshot().reference_generation;
        have_reference_ = true;
        recovery_.prepared(now_ms);
        if (recovery_.stage() == RecoveryGate::Stage::Blocked)
            return suspend(now_us, recovery_.reason(), recovery_.error(), true);
    }
    status_.ready = true;
    if (!requested)
        return publish(now_us, RunState::Disabled, WaitReason::Command);
    const auto previous_stage = recovery_.stage();
    if (!recovery_.accept(inputs.command.stamp, inputs.source_stamp, now_ms)) {
        if (previous_stage == RecoveryGate::Stage::Active || previous_stage == RecoveryGate::Stage::Enabling) {
            group_.disable();
            have_reference_ = false;
            status_.ready = false;
        }
        return publish(now_us, RunState::Recovering, WaitReason::Command, -ESTALE);
    }
    if (previous_stage == RecoveryGate::Stage::WaitingCommand) {
        const int ret = group_.enable();
        if (ret < 0) {
            retry_ms_ = now_ms + config_.fault_retry_ms;
            return suspend(now_us, WaitReason::Drive, ret);
        }
        return publish(now_us, RunState::Recovering, WaitReason::Drive);
    }
    group = group_.status();
    if (!group.active)
        return publish(now_us, RunState::Recovering, WaitReason::Drive);
    recovery_.enabled();
    int ret = yaw_.update({inputs.command.mode, inputs.command.yaw_target_rad, inputs.command.yaw_rate_rad_s},
                          SafetyAction::Active, dt_s);
    if (ret == 0)
        ret = pitch_.update({inputs.command.mode, inputs.command.pitch_target_rad, inputs.command.pitch_rate_rad_s},
                            SafetyAction::Active, dt_s);
    if (ret < 0)
        return suspend(now_us, WaitReason::Drive, ret, ret == -EINVAL || ret == -ERANGE);
    status_.last_command_sequence = inputs.command.stamp.sequence;
    return publish(now_us, RunState::Active, WaitReason::None);
}

} // namespace skywalker::robotics
