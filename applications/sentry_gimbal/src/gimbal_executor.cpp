#include "gimbal_executor.hpp"
#include <cerrno>
#include <cmath>
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
bool permanent(int error) {
    return error == -EINVAL || error == -ENOTSUP || error == -ENODEV || error == -ERANGE ||
           error == -EBUSY || error == -ENOSPC;
}
}
RunStatus GimbalExecutor::begin() {
    if (checked_) return status_;
    return update({}, core::monotonicTimeUs());
}
void GimbalExecutor::suspend() {
    const auto state = drive_.snapshot().state;
    if (state == motor::MotorState::Active || state == motor::MotorState::Enabling)
        (void)drive_.disable();
}
RunStatus GimbalExecutor::update(const GimbalCommand &command, core::TimeUs now_us) {
    const auto now = now_us / 1000;
    const bool cycle_valid = have_time_ && now_us > previous_us_ && now_us - previous_us_ <= 20000;
    const float dt = cycle_valid ? float(now_us - previous_us_) / 1000000.0f : 0;
    previous_us_ = now_us;
    have_time_ = true;
    status_.ready = false;
    const auto finish = [&](RunState state, WaitReason reason, int error = 0) {
        status_.state = state;
        status_.reason = reason;
        status_.error = error;
        status_.generation = static_cast<std::uint32_t>(drive_.snapshot().enable_generation);
        return status_;
    };
    const bool requested = command.mode != GimbalMode::Disabled &&
                           isFresh(command.stamp, now, board_config::command_timeout_ms);
    if (!requested) suspend(); // Never wait for a retry deadline to withdraw output.
    if (!checked_) {
        checked_ = true;
        config_error_ = board_config::connections_configured ? axis_.validate() : -ENODEV;
    }
    if (config_error_ < 0) return finish(RunState::Blocked, WaitReason::Configuration, config_error_);
    if (!configured_) {
        if (now < retry_ms_) return finish(RunState::Recovering, WaitReason::Transport, -EAGAIN);
        int ret = 0;
        if (!attached_) { ret = bus_.attach(drive_); attached_ = ret == 0; }
        if (ret == 0 && !started_) { ret = bus_.start(); started_ = ret == 0; }
        if (ret == 0) { ret = axis_.begin(); configured_ = ret == 0; }
        if (ret < 0) {
            retry_ms_ = now + 100;
            if (permanent(ret)) config_error_ = ret;
            return finish(config_error_ < 0 ? RunState::Blocked : RunState::Recovering, WaitReason::Configuration, ret);
        }
    }
    auto view = drive_.snapshot();
    if (view.state == motor::MotorState::Fault) {
        const auto reason = view.last_fault.reason;
        const bool transient = reason == motor::FaultReason::UnexpectedDisabled || reason == motor::FaultReason::EnableTimeout;
        if (!transient) return finish(RunState::Blocked, WaitReason::Drive, view.last_fault.error);
        if (view.feedback_fresh && now >= retry_ms_) {
            retry_ms_ = now + 100;
            const int ret = drive_.clearFault();
            if (ret < 0 && ret != -EAGAIN && ret != -EBUSY)
                return finish(RunState::Recovering, WaitReason::Drive, ret);
        }
        return finish(RunState::Recovering, WaitReason::Drive, view.last_fault.error);
    }
    const auto axis_status = axis_.poll(now);
    const bool active = drive_.active();
    status_.ready = axis_status.ready_for_enable || active || view.state == motor::MotorState::Enabling;
    if (!requested) return finish(RunState::Disabled, WaitReason::Command);
    if (command.mode > GimbalMode::AbsoluteAngle || !std::isfinite(command.yaw_target_rad) ||
        !std::isfinite(command.yaw_rate_rad_s)) {
        suspend();
        return finish(RunState::Blocked, WaitReason::Command, -EINVAL);
    }
    if (!axis_status.feedback_healthy) {
        suspend();
        status_.ready = false;
        return finish(RunState::Recovering, WaitReason::Feedback, axis_status.error);
    }
    if (active) {
        if (!cycle_valid) {
            suspend();
            status_.ready = false;
            return finish(RunState::Recovering, WaitReason::Cycle, -ESTALE);
        }
        int ret = axis_.update({command.mode, command.yaw_target_rad, command.yaw_rate_rad_s}, SafetyAction::Active, dt);
        if (ret == 0) ret = bus_.commit().error;
        if (ret < 0) {
            suspend();
            status_.ready = false;
            return finish(RunState::Recovering, WaitReason::Drive, ret);
        }
        status_.last_command_sequence = command.stamp.sequence;
        return finish(RunState::Active, WaitReason::None);
    }
    if (view.state == motor::MotorState::Enabling)
        return finish(RunState::Recovering, WaitReason::Drive);
    // The command must have been produced after this preparation boundary.
    if (!axis_status.ready_for_enable)
        return finish(RunState::Recovering, WaitReason::Reference, axis_status.error);
    if (command.stamp.timestamp_ms <= axis_status.ready_since_ms)
        return finish(RunState::Recovering, WaitReason::Command, -EAGAIN);
    if (now < retry_ms_) return finish(RunState::Recovering, WaitReason::Drive, -EAGAIN);
    int ret = axis_.reset();
    if (ret == 0) ret = drive_.enable();
    retry_ms_ = now + 100;
    return finish(RunState::Recovering, ret < 0 ? WaitReason::Reference : WaitReason::Drive, ret);
}
