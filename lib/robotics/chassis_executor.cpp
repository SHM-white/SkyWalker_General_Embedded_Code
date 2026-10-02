#include <robotics/chassis/chassis_executor.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>

namespace skywalker::robotics {
namespace {
bool transientFault(motor::FaultReason reason) {
    switch (reason) {
    case motor::FaultReason::FeedbackExpired: case motor::FaultReason::CommandExpired:
    case motor::FaultReason::UnexpectedDisabled: case motor::FaultReason::EnableTimeout:
    case motor::FaultReason::TransportError: case motor::FaultReason::RxOverflow: return true;
    default: return false;
    }
}
}

int SwerveHardware::begin() {
    if (!config_.hardware_confirmed) return -ENODEV;
    if (!config_.feedback_stable_ms || !std::isfinite(config_.velocity_safety_rad_s) ||
        config_.velocity_safety_rad_s <= 0 || !std::isfinite(config_.temperature_limit_c)) return -EINVAL;
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        if (!motors_[i] || (config_.directions[i] != 1 && config_.directions[i] != -1)) return -EINVAL;
        for (std::size_t j = 0; j < i; ++j) if (motors_[i] == motors_[j]) return -EINVAL;
        const auto info = motors_[i]->info();
        auto required = motor::CommandCurrent | motor::FeedbackVelocity | motor::FeedbackCurrent | motor::FeedbackPosition;
        if (i < 4) required |= motor::FeedbackAbsolutePosition;
        if ((info.capabilities & required) != required || !std::isfinite(info.current_limit_a) ||
            info.current_limit_a <= 0) return -ENOTSUP;
    }
    configured_ = true;
    return 0;
}

int SwerveHardware::validateFeedback(std::size_t index, const motor::MotorSnapshot &view, bool reference) const {
    const auto &f = view.feedback;
    if (!view.feedback_fresh) return -ESTALE;
    if (view.state == motor::MotorState::Offline || view.state == motor::MotorState::Fault) return -EHOSTDOWN;
    auto required = motor::FeedbackVelocity | motor::FeedbackCurrent;
    if (index < 4) required |= motor::FeedbackAbsolutePosition;
    if (reference) required |= motor::FeedbackPosition;
    if ((f.valid & required) != required || !std::isfinite(f.velocity_rad_s) || !std::isfinite(f.current_a) ||
        (index < 4 && !std::isfinite(f.absolute_position_rad)) ||
        (reference && (!view.position_reference_valid || !std::isfinite(f.position_rad)))) return -ENODATA;
    if (reference && seeded_ && view.reference_generation != references_[index]) return -ESTALE;
    if (std::fabs(f.velocity_rad_s) > config_.velocity_safety_rad_s ||
        ((f.valid & motor::FeedbackTemperature) && (!std::isfinite(f.temperature_c) ||
        f.temperature_c >= config_.temperature_limit_c))) return -ERANGE;
    return 0;
}

int SwerveHardware::read(ChassisFeedback &out) {
    if (!configured_) return -EACCES;
    ChassisFeedback next{};
    float sum = 0;
    std::uint64_t oldest = std::numeric_limits<std::uint64_t>::max();
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        const auto view = motors_[i]->snapshot();
        const int ret = validateFeedback(i, view, true);
        if (ret < 0) return ret;
        const auto &f = view.feedback;
        const float sign = config_.directions[i];
        if (i < 4) next.module[i] = {sign * f.absolute_position_rad, sign * f.position_rad, sign * f.velocity_rad_s, 0};
        else next.module[i - 4].drive_velocity_rad_s = sign * f.velocity_rad_s;
        sum += std::fabs(f.current_a);
        oldest = std::min(oldest, f.timestamp_ms);
    }
    absolute_current_sum_a_ = sum;
    oldest_feedback_ms_ = oldest;
    out = next;
    return 0;
}

int SwerveHardware::prepare(std::uint64_t now_ms, ChassisFeedback &out) {
    if (!configured_) return -EACCES;
    const auto state = group_.status();
    if (state.active || state.enable_pending || !state.ready) return -EAGAIN;
    if (!seeded_) {
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            const auto view = motors_[i]->snapshot();
            const int valid = validateFeedback(i, view, false);
            if (valid < 0) return valid;
            const int ret = motors_[i]->reseedPosition(i < 4 ? view.feedback.absolute_position_rad : 0);
            if (ret < 0) return ret;
            references_[i] = motors_[i]->snapshot().reference_generation;
            baseline_stamps_[i] = view.feedback.timestamp_ms;
        }
        seeded_ = true;
        stable_since_ms_ = now_ms;
        return -EAGAIN;
    }
    const int ret = read(out);
    if (ret < 0) return ret;
    for (std::size_t i = 0; i < motors_.size(); ++i)
        if (motors_[i]->snapshot().feedback.timestamp_ms <= baseline_stamps_[i]) return -EAGAIN;
    if (now_ms < stable_since_ms_ || now_ms - stable_since_ms_ < config_.feedback_stable_ms) return -EAGAIN;
    ready_ = true;
    return 0;
}

void SwerveHardware::suspend() {
    const auto state = group_.status();
    if (state.active || state.enable_pending) group_.disable();
    seeded_ = ready_ = false;
    absolute_current_sum_a_ = 0;
}

int SwerveHardware::stage(const ChassisOutput &out, float scale) {
    if (!ready_ || !group_.active()) return -EACCES;
    if (!std::isfinite(scale) || scale < 0 || scale > 1) return -EINVAL;
    std::array<float, 8> current{};
    for (std::size_t i = 0; i < 4; ++i) {
        current[i] = out.module[i].steer_effort * scale * config_.directions[i];
        current[i + 4] = out.module[i].drive_effort * scale * config_.directions[i + 4];
    }
    for (std::size_t i = 0; i < motors_.size(); ++i)
        if (!std::isfinite(current[i]) || std::fabs(current[i]) > motors_[i]->info().current_limit_a) return -ERANGE;
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        const int ret = motors_[i]->setCurrent(current[i]);
        if (ret < 0) { suspend(); return ret; }
    }
    return 0;
}

int ChassisExecutor::begin() {
    if (begin_attempted_) return -EALREADY;
    begin_attempted_ = true;
    int ret = chassis_.validate();
    if (ret == 0 && (!config_.max_cycle_us || config_.max_cycle_us > 20000 || !config_.measurement_timeout_us ||
        !config_.permission_timeout_ms || !config_.fault_retry_ms || !config_.recovery.command_timeout_ms ||
        !config_.recovery.source_timeout_us || !std::isfinite(config_.bench_effort_scale) ||
        config_.bench_effort_scale < 0 || config_.bench_effort_scale > 1 ||
        (!config_.power_control_calibrated && config_.bench_effort_scale > 0.15f) ||
        !std::isfinite(config_.estimate_idle_power_w) || config_.estimate_idle_power_w < 0 ||
        !std::isfinite(config_.estimate_w_per_abs_amp) || config_.estimate_w_per_abs_amp < 0 ||
        (config_.allow_estimated_power && (!config_.power_model_calibrated || config_.estimate_w_per_abs_amp <= 0))))
        ret = -EINVAL;
    if (ret == 0) ret = hardware_.begin();
    config_error_ = ret;
    configured_ = ret == 0;
    return ret;
}

void ChassisExecutor::withdraw(WaitReason reason, int error, bool blocked) {
    hardware_.suspend();
    recovery_.withdraw(reason, error, blocked);
    (void)limiter_.reset();
    output_ = {};
    effort_scale_ = 0;
    status_.ready = false;
    if (blocked) { hard_error_ = error == 0 ? -EIO : error; hard_reason_ = reason; }
}

RunStatus ChassisExecutor::publish(core::TimeUs now_us, RunState state, WaitReason reason, int error) {
    status_.state = state; status_.reason = reason; status_.error = error;
    status_.generation = recovery_.generation();
    status_.stamp = {now_us / 1000, ++production_sequence_, true};
    return status_;
}

RunStatus ChassisExecutor::suspend(core::TimeUs now_us, WaitReason reason, int error, bool blocked) {
    withdraw(reason, error, blocked);
    return publish(now_us, blocked ? RunState::Blocked : RunState::Recovering, reason, error);
}

int ChassisExecutor::power(const ChassisExecutionInputs &inputs, core::TimeUs now_us, float dt_s) {
    effort_scale_ = config_.bench_effort_scale;
    selected_power_ = inputs.measured_power;
    const auto valid_measurement = [&] {
        return selected_power_.valid && selected_power_.source != PowerMeasurementSource::None &&
            core::fresh(selected_power_.stamp, now_us, config_.measurement_timeout_us) &&
            std::isfinite(selected_power_.power_w) && selected_power_.power_w >= 0 &&
            (selected_power_.source != PowerMeasurementSource::Estimated ||
             (config_.power_model_calibrated && config_.allow_estimated_power)) &&
            (!selected_power_.voltage_current_valid || (std::isfinite(selected_power_.bus_voltage_v) &&
                selected_power_.bus_voltage_v > 0 && std::isfinite(selected_power_.bus_current_a)));
    };
    if (!valid_measurement() && config_.allow_estimated_power && config_.power_model_calibrated) {
        selected_power_ = {config_.estimate_idle_power_w + hardware_.absoluteCurrentSumA() * config_.estimate_w_per_abs_amp,
                           0, 0, true, false, PowerMeasurementSource::Estimated,
                           {hardware_.oldestFeedbackMs() * 1000, status_.stamp.sequence, true}};
    }
    if (!config_.require_power_budget) return 0;
    const auto now_ms = now_us / 1000;
    const auto &budget = inputs.power_budget;
    if (!isFresh(budget.stamp, now_ms, config_.permission_timeout_ms) ||
        !isFresh(budget.limit_stamp, now_ms, config_.permission_timeout_ms) ||
        budget.stamp.timestamp_ms * 1000 <= recovery_.boundaryUs() ||
        budget.limit_stamp.timestamp_ms * 1000 <= recovery_.boundaryUs() || !valid_measurement()) return -ESTALE;
    ChassisPowerDecision decision{};
    const int ret = limiter_.step({selected_power_.power_w, budget.chassis_power_limit_w, budget.buffer_energy_j}, dt_s, decision);
    if (ret == 0) effort_scale_ *= decision.effort_scale;
    return ret;
}

RunStatus ChassisExecutor::update(const ChassisExecutionInputs &inputs, core::TimeUs now_us) {
    const auto now_ms = now_us / 1000;
    const bool cycle = have_time_ && now_us > previous_us_ && now_us - previous_us_ <= config_.max_cycle_us;
    const float dt_s = cycle ? float(now_us - previous_us_) / 1000000.0f : 0;
    previous_us_ = now_us; have_time_ = true; status_.ready = false;
    if (inputs.emergency_stop) emergency_latched_ = true;
    if (inputs.clear_fault && !inputs.emergency_stop && configured_) {
        withdraw(WaitReason::Reference);
        const int ret = hardware_.group().clearFault();
        if (ret == 0 || ret == -EALREADY) {
            emergency_latched_ = false; hard_error_ = 0; explicit_clear_pending_ = true;
            recovery_.withdraw(WaitReason::Reference); retry_ms_ = now_ms + config_.fault_retry_ms;
        } else return suspend(now_us, WaitReason::Drive, ret, true);
    }
    if (!configured_) return suspend(now_us, WaitReason::Configuration, config_error_ ? config_error_ : -EACCES, true);
    if (emergency_latched_) return suspend(now_us, WaitReason::Drive, -ECANCELED, true);
    if (hard_error_ < 0) return suspend(now_us, hard_reason_, hard_error_, true);
    if (!inputs.transport_ready) return suspend(now_us, WaitReason::Transport, -EAGAIN);
    if (!cycle) return suspend(now_us, WaitReason::Cycle, -ESTALE);
    if (inputs.command.mode > ChassisMode::Spin || !std::isfinite(inputs.command.vx_m_s) ||
        !std::isfinite(inputs.command.vy_m_s) || !std::isfinite(inputs.command.wz_rad_s))
        return suspend(now_us, WaitReason::Command, -EINVAL, true);
    if (inputs.require_permission && (!inputs.permission.valid || !inputs.permission.enabled ||
        !isFresh(inputs.permission.stamp, now_ms, config_.permission_timeout_ms)))
        return suspend(now_us, WaitReason::Power, -EACCES);
    bool fault = false, recoverable = true, fresh = true;
    int fault_error = 0;
    for (std::size_t i = 0; i < 8; ++i) {
        const auto view = hardware_.motorAt(i).snapshot();
        fresh = fresh && view.feedback_fresh;
        if (view.state != motor::MotorState::Fault) continue;
        fault = true; recoverable = recoverable && transientFault(view.last_fault.reason);
        fault_error = view.last_fault.error;
    }
    if (fault) {
        recoverable = recoverable || explicit_clear_pending_;
        withdraw(WaitReason::Drive, fault_error, !recoverable);
        if (recoverable && fresh && now_ms >= retry_ms_) {
            retry_ms_ = now_ms + config_.fault_retry_ms;
            const int ret = hardware_.group().clearFault();
            if (ret < 0 && ret != -EAGAIN && ret != -EBUSY && ret != -EALREADY)
                return publish(now_us, RunState::Recovering, WaitReason::Drive, ret);
        }
        return publish(now_us, recoverable ? RunState::Recovering : RunState::Blocked, WaitReason::Drive, fault_error);
    }
    explicit_clear_pending_ = false;
    const bool requested = inputs.command.mode != ChassisMode::Disabled;
    auto group = hardware_.group().status();
    if (!requested && (group.active || group.enable_pending)) { withdraw(WaitReason::Command); group = hardware_.group().status(); }
    if ((recovery_.stage() == RecoveryGate::Stage::Active && !group.active) ||
        (recovery_.stage() == RecoveryGate::Stage::Enabling && !group.active && !group.enable_pending))
        return suspend(now_us, WaitReason::Drive, -EAGAIN);
    if (recovery_.stage() == RecoveryGate::Stage::WaitingPrerequisites) {
        if (now_ms < retry_ms_) return publish(now_us, RunState::Recovering, WaitReason::Drive, -EAGAIN);
        const int ret = hardware_.prepare(now_ms, feedback_);
        if (ret == -EAGAIN) return publish(now_us, RunState::Recovering, WaitReason::Reference, ret);
        if (ret < 0) return suspend(now_us, WaitReason::Feedback, ret, ret == -ERANGE || ret == -EINVAL);
        const int reset = chassis_.reset(feedback_);
        if (reset < 0) return suspend(now_us, WaitReason::Reference, reset, true);
        (void)limiter_.reset();
        recovery_.prepared(now_ms);
        if (recovery_.stage() == RecoveryGate::Stage::Blocked)
            return suspend(now_us, recovery_.reason(), recovery_.error(), true);
    }
    const int read = hardware_.read(feedback_);
    if (read < 0) return suspend(now_us, read == -ESTALE ? WaitReason::Reference : WaitReason::Feedback, read, read == -ERANGE);
    status_.ready = true;
    if (!requested) return publish(now_us, RunState::Disabled, WaitReason::Command);
    if (!inputs.recovery_context_valid) {
        if (group.active || group.enable_pending) return suspend(now_us, WaitReason::Reference, -ESTALE);
        return publish(now_us, RunState::Recovering, WaitReason::Reference, -ESTALE);
    }
    if (inputs.require_permission && inputs.permission.stamp.timestamp_ms * 1000 <= recovery_.boundaryUs()) {
        if (group.active || group.enable_pending) return suspend(now_us, WaitReason::Power, -ESTALE);
        return publish(now_us, RunState::Recovering, WaitReason::Power, -ESTALE);
    }
    const int power_result = power(inputs, now_us, dt_s);
    if (power_result < 0) {
        if (group.active || group.enable_pending) return suspend(now_us, WaitReason::Power, power_result);
        return publish(now_us, RunState::Recovering, WaitReason::Power, power_result);
    }
    const auto previous_stage = recovery_.stage();
    if (!recovery_.accept(inputs.command.stamp, inputs.source_stamp, now_ms)) {
        if (previous_stage == RecoveryGate::Stage::Active || previous_stage == RecoveryGate::Stage::Enabling) {
            hardware_.suspend(); output_ = {}; effort_scale_ = 0; status_.ready = false; (void)limiter_.reset();
        }
        return publish(now_us, RunState::Recovering, WaitReason::Command, -ESTALE);
    }
    if (previous_stage == RecoveryGate::Stage::WaitingCommand) {
        const int ret = hardware_.group().enable();
        if (ret < 0) return suspend(now_us, WaitReason::Drive, ret);
        return publish(now_us, RunState::Recovering, WaitReason::Drive);
    }
    if (!hardware_.group().active()) return publish(now_us, RunState::Recovering, WaitReason::Drive);
    recovery_.enabled();
    int ret = chassis_.step(inputs.command, feedback_, dt_s, output_);
    if (ret == 0) ret = hardware_.stage(output_, effort_scale_);
    if (ret < 0) return suspend(now_us, WaitReason::Drive, ret, ret == -EINVAL || ret == -ERANGE);
    status_.last_command_sequence = inputs.command.stamp.sequence;
    return publish(now_us, RunState::Active, WaitReason::None);
}

} // namespace skywalker::robotics
