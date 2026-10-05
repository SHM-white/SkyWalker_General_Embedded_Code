#include <robotics/chassis/chassis_executor.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>

namespace skywalker::robotics {
int SwerveHardware::begin() {
    if (!config_.hardware_confirmed) return -ENODEV;
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        if (!motors_[i] || (config_.directions[i] != 1 && config_.directions[i] != -1)) return -EINVAL;
        for (std::size_t j = 0; j < i; ++j) if (motors_[i] == motors_[j]) return -EINVAL;
        const auto info = motors_[i]->info();
        const auto required = motor::CommandCurrent | motor::FeedbackVelocity |
            (i < 4 ? motor::FeedbackAbsolutePosition : 0u);
        if ((info.capabilities & required) != required || !std::isfinite(info.current_limit_a) ||
            info.current_limit_a <= 0) return -ENOTSUP;
    }
    configured_ = true;
    return 0;
}
int SwerveHardware::read(ChassisFeedback &out) {
    if (!configured_) return -EACCES;
    out = {};
    absolute_current_sum_a_ = 0;
    oldest_feedback_ms_ = std::numeric_limits<std::uint64_t>::max();
    bool have_current = false;
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        const auto view = motors_[i]->snapshot();
        const auto &f = view.feedback;
        const bool active = view.state == motor::MotorState::Active && view.feedback_fresh;
        auto &m = out.module[i < 4 ? i : i - 4];
        const float sign = config_.directions[i];
        if (i < 4) {
            m.steer_absolute_rad = sign * f.absolute_position_rad;
            m.steer_velocity_rad_s = sign * f.velocity_rad_s;
            const auto fields = motor::FeedbackAbsolutePosition | motor::FeedbackVelocity;
            m.steer_valid = active && (f.valid & fields) == fields &&
                std::isfinite(m.steer_absolute_rad) && std::isfinite(m.steer_velocity_rad_s);
            m.steer_feedback_stamp_ms = f.timestamp_ms;
            m.steer_enable_generation = view.enable_generation;
            m.steer_reference_generation = view.reference_generation;
        } else {
            m.drive_velocity_rad_s = sign * f.velocity_rad_s;
            m.drive_valid = active && (f.valid & motor::FeedbackVelocity) &&
                std::isfinite(m.drive_velocity_rad_s);
            m.drive_enable_generation = view.enable_generation;
        }
        if (view.feedback_fresh && (f.valid & motor::FeedbackCurrent) && std::isfinite(f.current_a)) {
            absolute_current_sum_a_ += std::fabs(f.current_a);
            oldest_feedback_ms_ = std::min(oldest_feedback_ms_, f.timestamp_ms);
            have_current = true;
        }
    }
    if (!have_current) oldest_feedback_ms_ = 0;
    return 0;
}
void SwerveHardware::suspend() { group_.disable(); }
int SwerveHardware::stage(const ChassisOutput &out, float steer_scale, float drive_scale) {
    if (!configured_) return -EACCES;
    if (!std::isfinite(steer_scale) || steer_scale < 0 || steer_scale > 1 ||
        !std::isfinite(drive_scale) || drive_scale < 0 || drive_scale > 1) return -EINVAL;
    int first_error = 0;
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        const auto &m = out.module[i < 4 ? i : i - 4];
        const bool valid = i < 4 ? m.steer_output_valid : m.drive_output_valid;
        const float value = config_.directions[i] * (i < 4 ? m.steer_effort * steer_scale : m.drive_effort * drive_scale);
        const auto version = i < 4 ? m.steer_enable_generation : m.drive_enable_generation;
        const int ret = valid ? motors_[i]->setCurrent(value, version) : motors_[i]->invalidateComputedEffort();
        if (ret < 0 && !first_error) first_error = ret;
    }
    return first_error;
}
int ChassisExecutor::begin() {
    if (begin_attempted_) return -EALREADY;
    begin_attempted_ = true;
    int ret = chassis_.validate();
    if (ret == 0 && (!config_.max_cycle_us || config_.max_cycle_us > 20000 || !config_.measurement_timeout_us ||
        !config_.permission_timeout_ms || !config_.command_timeout_ms ||
        !config_.source_timeout_us || !std::isfinite(config_.bench_effort_scale) ||
        config_.bench_effort_scale < 0 || config_.bench_effort_scale > 1 ||
        !std::isfinite(config_.bench_steer_effort_scale) || config_.bench_steer_effort_scale < 0 ||
        config_.bench_steer_effort_scale > 1 || !std::isfinite(config_.bench_drive_current_limit_a) ||
        config_.bench_drive_current_limit_a <= 0 || !std::isfinite(config_.bench_steer_current_limit_a) ||
        config_.bench_steer_current_limit_a <= 0 ||
        !std::isfinite(config_.estimate_idle_power_w) || config_.estimate_idle_power_w < 0 ||
        !std::isfinite(config_.estimate_w_per_abs_amp) || config_.estimate_w_per_abs_amp < 0 ||
        (config_.allow_estimated_power && (!config_.power_model_calibrated || config_.estimate_w_per_abs_amp <= 0))))
        ret = -EINVAL;
    if (ret == 0) ret = hardware_.begin();
    config_error_ = ret;
    configured_ = ret == 0;
    return ret;
}

void ChassisExecutor::withdraw(WaitReason, int, bool) {
    hardware_.suspend();
    output_ = {};
    effort_scale_ = steer_effort_scale_ = 0;
    status_.ready = status_.requested = false;
    (void)limiter_.reset();
}
RunStatus ChassisExecutor::publish(core::TimeUs now, RunState state, WaitReason reason, int error) {
    status_.state = state; status_.reason = reason; status_.error = error;
    const auto members = hardware_.group().status();
    status_.member_count = members.member_count;
    status_.active_count = members.active_count;
    status_.waiting_count = members.member_count - members.active_count;
    status_.stamp = {now / 1000, ++production_sequence_, true};
    return status_;
}
RunStatus ChassisExecutor::suspend(core::TimeUs now, WaitReason reason, int error, bool blocked) {
    withdraw(reason, error, blocked);
    return publish(now, blocked ? RunState::Blocked : RunState::Disabled, reason, error);
}
int ChassisExecutor::power(const ChassisExecutionInputs &inputs, core::TimeUs now_us, float dt_s) {
    effort_scale_ = config_.bench_effort_scale;
    steer_effort_scale_ = config_.bench_steer_effort_scale;
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
        !valid_measurement()) return -ESTALE;
    ChassisPowerDecision decision{};
    const int ret = limiter_.step({selected_power_.power_w, budget.chassis_power_limit_w, budget.buffer_energy_j}, dt_s, decision);
    if (ret == 0) {
        // Measurement includes steering. Total-budget reduction must also
        // constrain it; independent bench scales never exempt steering power.
        effort_scale_ *= decision.effort_scale;
        steer_effort_scale_ *= decision.effort_scale;
    }
    return ret;
}

RunStatus ChassisExecutor::update(const ChassisExecutionInputs &in, core::TimeUs now) {
    const auto now_ms = now / 1000;
    const bool cycle = have_time_ && now > previous_us_ && now - previous_us_ <= config_.max_cycle_us;
    const float dt = cycle ? float(now - previous_us_) / 1e6f : 0;
    previous_us_ = now; have_time_ = true;
    if (!configured_) return suspend(now, WaitReason::Configuration, config_error_ ? config_error_ : -EACCES, true);
    if (in.emergency_stop || in.clear_estop) return suspend(now, WaitReason::Command, in.emergency_stop ? -ECANCELED : 0);
    if (in.command.mode > ChassisMode::Spin || !std::isfinite(in.command.vx_m_s) ||
        !std::isfinite(in.command.vy_m_s) || !std::isfinite(in.command.wz_rad_s))
        return suspend(now, WaitReason::Command, -EINVAL);
    const bool fresh = isFresh(in.command.stamp, now_ms, config_.command_timeout_ms) &&
        core::fresh(in.source_stamp, now, config_.source_timeout_us);
    if (!fresh || in.command.mode == ChassisMode::Disabled) return suspend(now, WaitReason::Command, fresh ? 0 : -ESTALE);
    if (!in.transport_ready) return suspend(now, WaitReason::Transport, -ESTALE);
    if (in.require_permission && (!in.permission.valid || !in.permission.enabled ||
        !isFresh(in.permission.stamp, now_ms, config_.permission_timeout_ms)))
        return suspend(now, WaitReason::Power, -EACCES);
    status_.requested = true;
    status_.last_command_sequence = in.command.stamp.sequence;
    int first_error = hardware_.group().enable();
    const int read = hardware_.read(feedback_);
    if (read < 0 && !first_error) first_error = read;
    const int calculated = chassis_.step(in.command, feedback_, dt, output_);
    if (calculated < 0 && !first_error) first_error = calculated;
    // Keep enabled power control; missing measurements affect output, not input intent.
    const int limited = cycle ? power(in, now, dt) : 0;
    if (limited < 0) {
        effort_scale_ = steer_effort_scale_ = 0;
        for (auto &m : output_.module) m.steer_output_valid = m.drive_output_valid = false;
    }
    auto staged = output_;
    if (!config_.power_control_calibrated) {
        for (auto &m : staged.module) {
            m.steer_effort = std::clamp(m.steer_effort, -config_.bench_steer_current_limit_a, config_.bench_steer_current_limit_a);
            m.drive_effort = std::clamp(m.drive_effort, -config_.bench_drive_current_limit_a, config_.bench_drive_current_limit_a);
        }
    }
    const int sent = hardware_.stage(staged, steer_effort_scale_, effort_scale_);
    if (sent < 0 && !first_error) first_error = sent;
    output_ = staged;
    std::size_t outputs = 0;
    for (const auto &m : output_.module) outputs += unsigned(m.steer_output_valid) + unsigned(m.drive_output_valid);
    status_.ready = outputs != 0;
    return publish(now, outputs ? RunState::Active : RunState::Recovering,
        limited < 0 ? WaitReason::Power : !cycle ? WaitReason::Cycle : outputs ? WaitReason::None : WaitReason::Feedback,
        first_error ? first_error : limited);
}
} // namespace skywalker::robotics
