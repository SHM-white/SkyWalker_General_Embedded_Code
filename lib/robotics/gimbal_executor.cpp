#include <robotics/gimbal/gimbal_executor.hpp>
#include <cerrno>
#include <cmath>
namespace skywalker::robotics {
GimbalExecutor::GimbalExecutor(motor::Motor &yaw_drive, motor::Motor &pitch_drive, motor::Group &group,
                               const control::PositionMotor::Config &yaw_motor, const GimbalAxisConfig &yaw_axis,
                               const control::PositionMotor::Config &pitch_motor, const GimbalAxisConfig &pitch_axis,
                               const Config &config)
    : yaw_drive_(yaw_drive), pitch_drive_(pitch_drive), group_(group), yaw_(yaw_drive, yaw_motor, yaw_axis),
      pitch_(pitch_drive, pitch_motor, pitch_axis), config_(config) {
}

int GimbalExecutor::begin() {
    if (begin_attempted_)
        return -EALREADY;
    begin_attempted_ = true;
    int ret = !config_.max_cycle_us || !config_.permission_timeout_ms || !config_.command_timeout_ms ||
                      !config_.source_timeout_us
                  ? -EINVAL
                  : yaw_.validate();
    if (!ret)
        ret = pitch_.validate();
    if (!ret)
        ret = yaw_.begin();
    if (!ret)
        ret = pitch_.begin();
    config_error_ = ret;
    configured_ = !ret;
    return ret;
}
void GimbalExecutor::withdraw(WaitReason, int, bool) {
    group_.disable();
    yaw_.withdraw();
    pitch_.withdraw();
    status_.ready = status_.requested = false;
}
RunStatus GimbalExecutor::publish(core::TimeUs now, RunState state, WaitReason reason, int error) {
    status_.state = state;
    status_.reason = reason;
    status_.error = error;
    const auto members = group_.status();
    status_.member_count = members.member_count;
    status_.active_count = members.active_count;
    status_.waiting_count = members.member_count - members.active_count;
    status_.stamp = {now / 1000, ++production_sequence_, true};
    return status_;
}
RunStatus GimbalExecutor::suspend(core::TimeUs now, WaitReason reason, int error, bool blocked) {
    withdraw(reason, error, blocked);
    return publish(now, blocked ? RunState::Blocked : RunState::Disabled, reason, error);
}
RunStatus GimbalExecutor::update(const GimbalExecutionInputs &in, core::TimeUs now) {
    const auto now_ms = now / 1000;
    const bool have_previous = previous_us_ != 0;
    const bool cycle = have_previous && now > previous_us_ && now - previous_us_ <= config_.max_cycle_us;
    const float dt = cycle ? float(now - previous_us_) / 1e6f : 0;
    previous_us_ = now;
    have_time_ = true;
    if (!configured_)
        return suspend(now, WaitReason::Configuration, config_error_ ? config_error_ : -EACCES, true);
    if (in.emergency_stop || in.clear_estop)
        return suspend(now, WaitReason::Command, in.emergency_stop ? -ECANCELED : 0);
    if (in.command.mode > GimbalMode::AbsoluteAngle)
        return suspend(now, WaitReason::Command, -EINVAL);
    if (in.command.mode == GimbalMode::Disabled || !isFresh(in.command.stamp, now_ms, config_.command_timeout_ms) ||
        !core::fresh(in.source_stamp, now, config_.source_timeout_us))
        return suspend(now, WaitReason::Command);
    if (!in.transport_ready)
        return suspend(now, WaitReason::Transport, -ESTALE);
    // Missing or expired referee permission is allowed; only an explicitly
    // valid and disabled permission restricts this executor.
    if (in.require_permission && in.permission.valid && in.permission.stamp.valid &&
        isFresh(in.permission.stamp, now_ms, config_.permission_timeout_ms) && !in.permission.enabled)
        return suspend(now, WaitReason::Power, -EACCES);

    status_.requested = true;
    status_.last_command_sequence = in.command.stamp.sequence;
    int error = group_.enable();
    if (!have_previous || !cycle) {
        // Keep the enable request pending; do not disable or reset the group for
        // a missed interval. No new axis output is staged this cycle.
        status_.ready = false;
        return publish(now, RunState::Recovering, WaitReason::Cycle, error);
    }

    const bool yaw_trajectory_ok = in.yaw_trajectory.valid && std::isfinite(in.yaw_trajectory.position_rad) &&
                                   std::isfinite(in.yaw_trajectory.rate_rad_s) &&
                                   std::isfinite(in.yaw_trajectory.acceleration_rad_s2) &&
                                   core::fresh(in.yaw_trajectory.stamp, now,
                                               static_cast<core::TimeUs>(config_.command_timeout_ms) * 1000ULL);
    const bool yaw_input_ok = yaw_trajectory_ok ||
                              (in.command.mode == GimbalMode::Rate            ? std::isfinite(in.command.yaw_rate_rad_s)
                               : in.command.mode == GimbalMode::AbsoluteAngle ? std::isfinite(in.command.yaw_target_rad)
                                                                              : true);
    const bool pitch_input_ok = in.command.mode == GimbalMode::Rate ? std::isfinite(in.command.pitch_rate_rad_s)
                                : in.command.mode == GimbalMode::AbsoluteAngle
                                    ? std::isfinite(in.command.pitch_target_rad)
                                    : true;
    int yr = 0, pr = 0;
    if (yaw_trajectory_ok)
        yr = yaw_.updateTrajectory(in.yaw_trajectory, SafetyAction::Active, in.yaw_output_valid ? dt : 0);
    else if (yaw_input_ok)
        yr = yaw_.update({in.command.mode, in.command.yaw_target_rad, in.command.yaw_rate_rad_s}, SafetyAction::Active,
                         in.yaw_output_valid ? dt : 0);
    else if (!error)
        error = -EINVAL;
    if (pitch_input_ok)
        pr = pitch_.update({in.command.mode, in.command.pitch_target_rad, in.command.pitch_rate_rad_s},
                           SafetyAction::Active, in.pitch_output_valid ? dt : 0);
    else if (!error)
        error = -EINVAL;
    const int axis_error = yr < 0 ? yr : pr;
    if (!error)
        error = axis_error;
    status_.ready = (yaw_input_ok && yaw_.telemetry().output_valid) ||
                    (pitch_input_ok && pitch_.telemetry().output_valid);
    return publish(now, status_.ready ? RunState::Active : RunState::Recovering,
                   status_.ready ? WaitReason::None : (error ? WaitReason::Command : WaitReason::Feedback), error);
}
} // namespace skywalker::robotics
