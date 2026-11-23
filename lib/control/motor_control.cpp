#include <control/position_motor.hpp>
#include <control/velocity_motor.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <control/angle.h>
#include <control/feedforward.h>

namespace skywalker::control {
namespace {
bool finiteFloat(double value) {
    return std::isfinite(value) && std::fabs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}
int validateEffort(const motor::MotorInfo &info, EffortUnit unit, float requested) {
    if (!std::isfinite(requested) || requested < 0)
        return -EINVAL;
    const auto required = unit == EffortUnit::Ampere ? motor::CommandCurrent : motor::CommandTorque;
    if (unit != EffortUnit::Ampere && unit != EffortUnit::NewtonMeter)
        return -EINVAL;
    if (!(info.capabilities & required))
        return -ENOTSUP;
    const float available = unit == EffortUnit::Ampere ? info.current_limit_a : info.torque_limit_nm;
    return std::isfinite(available) && available > 0 && requested <= available ? 0 : -ERANGE;
}
ControlCheck checkMeasurement(const motor::MotorSnapshot &s, std::uint32_t required) {
    if (!s.feedback_fresh)
        return {ControlIssue::FeedbackStale, -ESTALE};
    const auto &f = s.feedback;
    if ((f.valid & required) != required)
        return {ControlIssue::MissingFeedback, -ENODATA};
    if (!std::isfinite(f.velocity_rad_s) || ((required & motor::FeedbackPosition) && !std::isfinite(f.position_rad)) ||
        ((required & motor::FeedbackAbsolutePosition) && !std::isfinite(f.absolute_position_rad)))
        return {ControlIssue::InvalidMeasurement, -EINVAL};
    return {};
}
}

VelocityMotor::VelocityMotor(motor::Motor &m, const Config &c, const void *owner)
    : motor_(m), producer_(owner ? owner : this), config_(c) {
}
int VelocityMotor::configure() {
    if (configured_)
        return -EALREADY;
    int ret = control_motor_velocity_validate(&config_.loop);
    if (ret < 0)
        return ret;
    const auto info = motor_.info();
    ret = validateEffort(info, config_.effort_unit, config_.loop.effort_abs_max);
    if (ret < 0)
        return ret;
    if (!(info.capabilities & motor::FeedbackVelocity))
        return -ENOTSUP;
    ret = motor_.bindProducer(producer_);
    if (ret < 0)
        return ret;
    configured_ = true;
    return 0;
}
int VelocityMotor::resetFrom(const motor::MotorSnapshot &s) {
    control_motor_velocity_state next{};
    const int ret = control_motor_velocity_reset(&next, s.feedback.velocity_rad_s, 0);
    if (ret < 0)
        return ret;
    state_ = next;
    observed_enable_generation_ = s.enable_generation;
    history_valid_ = true;
    return 0;
}
int VelocityMotor::reset() {
    if (!configured_)
        return -EACCES;
    history_valid_ = false;
    (void)motor_.invalidateComputedEffortFrom(producer_);
    return 0;
}
void VelocityMotor::publish(const Telemetry &next) {
    const auto key = k_spin_lock(&telemetry_lock_);
    telemetry_ = next;
    k_spin_unlock(&telemetry_lock_, key);
}
VelocityMotor::Telemetry VelocityMotor::telemetry() const {
    const auto key = k_spin_lock(&telemetry_lock_);
    const auto copy = telemetry_;
    k_spin_unlock(&telemetry_lock_, key);
    return copy;
}
int VelocityMotor::stageEffort(float effort, std::uint64_t generation) {
    return config_.effort_unit == EffortUnit::Ampere ? motor_.setCurrentFrom(producer_, effort, generation)
                                                     : motor_.setTorqueFrom(producer_, effort, generation);
}
int VelocityMotor::fail(int error, const motor::MotorSnapshot &s, ControlIssue issue) {
    history_valid_ = false;
    if (configured_)
        (void)motor_.invalidateComputedEffortFrom(producer_);
    Telemetry next{};
    next.motor = s;
    next.target_rad_s = latest_target_rad_s_;
    next.dt_s = latest_dt_s_;
    next.effort_unit = config_.effort_unit;
    next.target_valid = target_sequence_ != 0;
    next.target_sequence = target_sequence_;
    next.error = error;
    next.issue = issue;
    publish(next);
    return error;
}
int VelocityMotor::wait(const motor::MotorSnapshot &s, ControlIssue issue) {
    return fail(0, s, issue);
}
int VelocityMotor::update(float target, float dt) {
    const auto snapshot = motor_.snapshot();
    if (!configured_)
        return fail(-EACCES, snapshot, ControlIssue::NotConfigured);
    if (!std::isfinite(target))
        return fail(-EINVAL, snapshot, ControlIssue::InvalidTarget);
    if (std::fabs(target) > config_.loop.requested_velocity_abs_max_rad_s)
        return fail(-ERANGE, snapshot, ControlIssue::InvalidTarget);
    if (!std::isfinite(dt) || dt < 0)
        return fail(-EINVAL, snapshot, ControlIssue::InvalidPeriod);
    if (target_sequence_ == std::numeric_limits<std::uint64_t>::max())
        return fail(-EOVERFLOW, snapshot, ControlIssue::InvalidTarget);
    latest_target_rad_s_ = target;
    latest_dt_s_ = dt;
    ++target_sequence_;
    const auto measurement = checkMeasurement(snapshot, motor::FeedbackVelocity);
    if (measurement.error < 0)
        return wait(snapshot, measurement.issue);
    if (snapshot.state != motor::MotorState::Active)
        return wait(snapshot, ControlIssue::NotActive);
    const auto &pid = config_.loop.regulator.feedback;
    if (dt < pid.dt_min_s || dt > pid.dt_max_s)
        return wait(snapshot, ControlIssue::InvalidPeriod);
    const bool initialize = !history_valid_ || snapshot.enable_generation != observed_enable_generation_;
    control_motor_velocity_output output{};
    if (initialize) {
        const int ret = resetFrom(snapshot);
        if (ret < 0)
            return fail(ret, snapshot, ControlIssue::InvalidMeasurement);
    }
    else {
        auto next = state_;
        const control_motor_velocity_input input{target, snapshot.feedback.velocity_rad_s, 0, dt, false};
        const int ret = control_motor_velocity_step(&next, &config_.loop, &input, &output);
        if (ret < 0)
            return fail(ret, snapshot);
        state_ = next;
    }
    const int ret = stageEffort(output.effort_command, snapshot.enable_generation);
    if (ret < 0)
        return fail(ret, snapshot);
    Telemetry next{};
    next.motor = snapshot;
    next.output = output;
    next.target_rad_s = target;
    next.dt_s = dt;
    next.effort_command = output.effort_command;
    next.effort_unit = config_.effort_unit;
    next.target_valid = next.output_valid = true;
    next.target_sequence = target_sequence_;
    publish(next);
    return 0;
}

PositionMotor::PositionMotor(motor::Motor &m, const Config &c, const void *owner)
    : motor_(m), producer_(owner ? owner : this), config_(c) {
}
int PositionMotor::configure() {
    if (configured_)
        return -EALREADY;
    int ret = control_motor_position_validate(&config_.loop);
    if (ret < 0)
        return ret;
    const auto &a = config_.loop.position, &b = config_.loop.velocity.regulator.feedback;
    if (std::max(a.dt_min_s, b.dt_min_s) > std::min(a.dt_max_s, b.dt_max_s))
        return -ERANGE;
    if (config_.reference != PositionReference::StartupRelative &&
        config_.reference != PositionReference::DriverContinuous &&
        config_.reference != PositionReference::AbsoluteNearest)
        return -EINVAL;
    const auto info = motor_.info();
    ret = validateEffort(info, config_.effort_unit, config_.loop.velocity.effort_abs_max);
    if (ret < 0)
        return ret;
    if (config_.has_external_velocity) {
        ret = control_motor_velocity_validate(&config_.external_velocity);
        if (ret < 0)
            return ret;
        if (!std::isfinite(config_.external_acceleration_effort_gain) ||
            config_.external_acceleration_effort_gain < 0.0f)
            return -EINVAL;
        ret = validateEffort(info, config_.effort_unit, config_.external_velocity.effort_abs_max);
        if (ret < 0)
            return ret;
    }
    const auto required = motor::FeedbackVelocity |
                          (config_.reference == PositionReference::AbsoluteNearest ? motor::FeedbackAbsolutePosition
                                                                                   : motor::FeedbackPosition);
    if ((info.capabilities & required) != required)
        return -ENOTSUP;
    ret = motor_.bindProducer(producer_);
    if (ret < 0)
        return ret;
    configured_ = true;
    return 0;
}
int PositionMotor::resetFrom(const motor::MotorSnapshot &s, double position) {
    control_motor_position_state next{};
    const int ret = control_motor_position_reset(&next, &config_.loop, 0, s.feedback.velocity_rad_s);
    if (ret < 0)
        return ret;
    if (config_.reference == PositionReference::StartupRelative && !startup_origin_valid_) {
        initial_position_rad_ = position;
        startup_origin_valid_ = true;
    }
    state_ = next;
    coordinate_origin_rad_ = position;
    observed_enable_generation_ = s.enable_generation;
    observed_reference_generation_ = s.reference_generation;
    history_valid_ = true;
    return 0;
}
int PositionMotor::resolveMeasurement(const motor::MotorSnapshot &s, double &position) {
    if (config_.reference != PositionReference::AbsoluteNearest) {
        position = s.feedback.position_rad;
        return 0;
    }
    const float angle = s.feedback.absolute_position_rad;
    const bool epoch = !absolute_local_valid_ || s.enable_generation != observed_enable_generation_ ||
                       s.reference_generation != observed_reference_generation_;
    if (epoch) {
        absolute_local_position_rad_ = angle;
        previous_absolute_rad_ = angle;
        previous_absolute_stamp_ms_ = s.feedback.timestamp_ms;
        absolute_local_valid_ = true;
        history_valid_ = false;
    }
    else if (s.feedback.timestamp_ms > previous_absolute_stamp_ms_) {
        float delta = 0;
        const int ret = control_shortest_angle_error(angle, previous_absolute_rad_, &delta);
        if (ret < 0)
            return ret;
        absolute_local_position_rad_ += static_cast<double>(delta);
        previous_absolute_rad_ = angle;
        previous_absolute_stamp_ms_ = s.feedback.timestamp_ms;
    }
    position = absolute_local_position_rad_;
    return std::isfinite(position) ? 0 : -ERANGE;
}
int PositionMotor::reset() {
    if (!configured_)
        return -EACCES;
    history_valid_ = false;
    external_history_valid_ = false;
    active_loop_ = ActiveLoop::None;
    absolute_local_valid_ = false;
    (void)motor_.invalidateComputedEffortFrom(producer_);
    return 0;
}
void PositionMotor::publish(const Telemetry &next) {
    const auto key = k_spin_lock(&telemetry_lock_);
    telemetry_ = next;
    k_spin_unlock(&telemetry_lock_, key);
}
PositionMotor::Telemetry PositionMotor::telemetry() const {
    const auto key = k_spin_lock(&telemetry_lock_);
    const auto copy = telemetry_;
    k_spin_unlock(&telemetry_lock_, key);
    return copy;
}
int PositionMotor::stageEffort(float effort, std::uint64_t generation) {
    return config_.effort_unit == EffortUnit::Ampere ? motor_.setCurrentFrom(producer_, effort, generation)
                                                     : motor_.setTorqueFrom(producer_, effort, generation);
}
int PositionMotor::fail(int error, const motor::MotorSnapshot &s, ControlIssue issue) {
    history_valid_ = false;
    external_history_valid_ = false;
    active_loop_ = ActiveLoop::None;
    absolute_local_valid_ = false;
    if (configured_)
        (void)motor_.invalidateComputedEffortFrom(producer_);
    Telemetry next{};
    next.motor = s;
    next.requested_position_rad = latest_target_rad_;
    next.dt_s = latest_dt_s_;
    next.effort_unit = config_.effort_unit;
    next.target_valid = target_sequence_ != 0;
    next.target_sequence = target_sequence_;
    next.error = error;
    next.issue = issue;
    publish(next);
    return error;
}
int PositionMotor::wait(const motor::MotorSnapshot &s, ControlIssue issue) {
    return fail(0, s, issue);
}
int PositionMotor::update(double target, float dt) {
    return updateImpl(target, 0, 0, false, dt);
}
int PositionMotor::update(double target, float reference_velocity_rad_s, float reference_acceleration_rad_s2,
                          float dt) {
    return updateImpl(target, reference_velocity_rad_s, reference_acceleration_rad_s2, true, dt);
}
int PositionMotor::updateImpl(double target, float reference_velocity_rad_s, float reference_acceleration_rad_s2,
                              bool use_reference, float dt) {
    const auto snapshot = motor_.snapshot();
    if (!configured_)
        return fail(-EACCES, snapshot, ControlIssue::NotConfigured);
    if (!std::isfinite(target))
        return fail(-EINVAL, snapshot, ControlIssue::InvalidTarget);
    if (config_.reference == PositionReference::AbsoluteNearest && !finiteFloat(target))
        return fail(-ERANGE, snapshot, ControlIssue::InvalidTarget);
    if (use_reference && (!std::isfinite(reference_velocity_rad_s) || !std::isfinite(reference_acceleration_rad_s2)))
        return fail(-EINVAL, snapshot, ControlIssue::InvalidTarget);
    if (!std::isfinite(dt) || dt < 0)
        return fail(-EINVAL, snapshot, ControlIssue::InvalidPeriod);
    if (target_sequence_ == std::numeric_limits<std::uint64_t>::max())
        return fail(-EOVERFLOW, snapshot, ControlIssue::InvalidTarget);
    latest_target_rad_ = target;
    latest_dt_s_ = dt;
    ++target_sequence_;
    const bool absolute = config_.reference == PositionReference::AbsoluteNearest;
    const auto required = motor::FeedbackVelocity |
                          (absolute ? motor::FeedbackAbsolutePosition : motor::FeedbackPosition);
    // Position is not required for absolute-angle recovery after a feedback gap.
    if (!absolute && !snapshot.position_reference_valid)
        return wait(snapshot, ControlIssue::ReferenceLost);
    const auto measurement = checkMeasurement(snapshot, required);
    if (measurement.error < 0)
        return wait(snapshot, measurement.issue);
    if (snapshot.state != motor::MotorState::Active)
        return wait(snapshot, ControlIssue::NotActive);
    const auto &a = config_.loop.position, &b = config_.loop.velocity.regulator.feedback;
    if (dt < std::max(a.dt_min_s, b.dt_min_s) || dt > std::min(a.dt_max_s, b.dt_max_s))
        return wait(snapshot, ControlIssue::InvalidPeriod);
    double position = 0;
    int ret = resolveMeasurement(snapshot, position);
    if (ret < 0)
        return fail(ret, snapshot, ControlIssue::InvalidMeasurement);
    const bool same_gen = snapshot.enable_generation == observed_enable_generation_ &&
                          snapshot.reference_generation == observed_reference_generation_;
    const bool handover_from_external =
        active_loop_ == ActiveLoop::ExternalVelocity && external_history_valid_ && same_gen &&
        telemetry_.output_valid;
    const float prev_effort = telemetry_.effort_command;
    const bool initialize = !handover_from_external && (!history_valid_ || !same_gen);
    if (initialize || handover_from_external) {
        ret = resetFrom(snapshot, position);
        if (ret < 0)
            return fail(ret, snapshot, ControlIssue::InvalidMeasurement);
    }
    double resolved = target;
    if (config_.reference == PositionReference::StartupRelative)
        resolved += initial_position_rad_;
    if (absolute) {
        float error = 0;
        ret = control_shortest_angle_error(static_cast<float>(target), snapshot.feedback.absolute_position_rad, &error);
        if (ret < 0)
            return fail(ret, snapshot, ControlIssue::InvalidTarget);
        resolved = position + static_cast<double>(error);
    }
    if (!finiteFloat(resolved))
        return fail(-ERANGE, snapshot, ControlIssue::InvalidTarget);
    if (handover_from_external) {
        const float v_max = config_.loop.velocity.requested_velocity_abs_max_rad_s;
        const float init_vel = std::clamp(snapshot.feedback.velocity_rad_s, -v_max, v_max);
        ret = control_motor_velocity_reset(&state_.velocity, snapshot.feedback.velocity_rad_s, init_vel);
        if (ret < 0)
            return fail(ret, snapshot, ControlIssue::InvalidMeasurement);
        const control_feedforward_reference ff_ref{static_cast<float>(resolved), init_vel, 0.0f};
        float ff_out = 0.0f;
        if (control_feedforward_calculate(&config_.loop.velocity.regulator.feedforward, &ff_ref, &ff_out) == 0 &&
            std::isfinite(ff_out)) {
            const auto &vpid = config_.loop.velocity.regulator.feedback;
            state_.velocity.regulator.feedback.integral_output =
                std::clamp(prev_effort - ff_out, vpid.integral_min, vpid.integral_max);
        }
        external_history_valid_ = false;
    }
    control_motor_position_output output{};
    if (!initialize) {
        auto next = state_;
        double origin = coordinate_origin_rad_;
        if (std::fabs(position - origin) > 128) {
            const double shift = position - origin;
            if (!finiteFloat(shift))
                return fail(-ERANGE, snapshot, ControlIssue::InvalidMeasurement);
            next.position.previous_measurement -= static_cast<float>(shift);
            origin = position;
        }
        if (!finiteFloat(resolved - origin) || !finiteFloat(position - origin))
            return fail(-ERANGE, snapshot, ControlIssue::InvalidTarget);
        const control_motor_position_input input{static_cast<float>(resolved - origin),
                                                 static_cast<float>(position - origin),
                                                 snapshot.feedback.velocity_rad_s,
                                                 dt,
                                                 static_cast<float>(resolved),
                                                 true,
                                                 use_reference ? reference_velocity_rad_s : 0.0f,
                                                 use_reference,
                                                 use_reference ? reference_acceleration_rad_s2 : 0.0f,
                                                 use_reference};
        ret = control_motor_position_step(&next, &config_.loop, &input, &output);
        if (ret < 0)
            return fail(ret, snapshot);
        state_ = next;
        coordinate_origin_rad_ = origin;
    }
    ret = stageEffort(output.effort_command, snapshot.enable_generation);
    if (ret < 0)
        return fail(ret, snapshot);
    active_loop_ = ActiveLoop::EncoderPosition;
    Telemetry next{};
    next.motor = snapshot;
    next.output = output;
    next.requested_position_rad = target;
    next.target_position_rad = resolved;
    next.position_rad = absolute
                            ? static_cast<double>(snapshot.feedback.absolute_position_rad)
                            : position -
                                  (config_.reference == PositionReference::StartupRelative ? initial_position_rad_ : 0);
    next.dt_s = dt;
    next.effort_command = output.effort_command;
    next.effort_unit = config_.effort_unit;
    next.target_valid = next.output_valid = true;
    next.target_sequence = target_sequence_;
    publish(next);
    return 0;
}

int PositionMotor::updateExternalVelocity(const ExternalVelocityInput &ext, float dt) {
    const auto snapshot = motor_.snapshot();
    if (!configured_)
        return fail(-EACCES, snapshot, ControlIssue::NotConfigured);
    if (!config_.has_external_velocity)
        return fail(-ENOTSUP, snapshot, ControlIssue::InvalidTarget);
    if (!std::isfinite(ext.velocity_request_rad_s) || !std::isfinite(ext.measured_velocity_rad_s) ||
        !std::isfinite(ext.position_reference_rad) || !std::isfinite(ext.reference_velocity_rad_s) ||
        !std::isfinite(ext.acceleration_ff_rad_s2) || !std::isfinite(ext.outer_position_error_rad) ||
        !std::isfinite(ext.outer_output_rad_s))
        return fail(-EINVAL, snapshot, ControlIssue::InvalidTarget);
    if (!std::isfinite(dt) || dt < 0)
        return fail(-EINVAL, snapshot, ControlIssue::InvalidPeriod);
    if (target_sequence_ == std::numeric_limits<std::uint64_t>::max())
        return fail(-EOVERFLOW, snapshot, ControlIssue::InvalidTarget);
    latest_dt_s_ = dt;
    ++target_sequence_;
    const bool absolute = config_.reference == PositionReference::AbsoluteNearest;
    const auto required = motor::FeedbackVelocity |
                          (absolute ? motor::FeedbackAbsolutePosition : motor::FeedbackPosition);
    if (!absolute && !snapshot.position_reference_valid)
        return wait(snapshot, ControlIssue::ReferenceLost);
    const auto measurement = checkMeasurement(snapshot, required);
    if (measurement.error < 0)
        return wait(snapshot, measurement.issue);
    if (snapshot.state != motor::MotorState::Active)
        return wait(snapshot, ControlIssue::NotActive);
    const auto &ext_pid = config_.external_velocity.regulator.feedback;
    if (dt < ext_pid.dt_min_s || dt > ext_pid.dt_max_s)
        return wait(snapshot, ControlIssue::InvalidPeriod);
    double position = 0;
    int ret = resolveMeasurement(snapshot, position);
    if (ret < 0)
        return fail(ret, snapshot, ControlIssue::InvalidMeasurement);
    latest_target_rad_ = position;
    const bool same_gen = snapshot.enable_generation == observed_enable_generation_ &&
                          snapshot.reference_generation == observed_reference_generation_;
    const bool handover_from_encoder =
        active_loop_ == ActiveLoop::EncoderPosition && history_valid_ && same_gen && telemetry_.output_valid;
    const bool need_init =
        !external_history_valid_ || !same_gen || active_loop_ != ActiveLoop::ExternalVelocity;
    const float v_max = config_.external_velocity.requested_velocity_abs_max_rad_s;
    if (need_init) {
        const float prev_effort = telemetry_.effort_command;
        const float init_vel = std::clamp(ext.measured_velocity_rad_s, -v_max, v_max);
        ret = control_motor_velocity_reset(&external_state_, ext.measured_velocity_rad_s, init_vel);
        if (ret < 0)
            return fail(ret, snapshot, ControlIssue::InvalidMeasurement);
        if (handover_from_encoder) {
            const control_feedforward_reference ff_ref{ext.position_reference_rad, init_vel, 0.0f};
            float ff_out = 0.0f;
            if (control_feedforward_calculate(&config_.external_velocity.regulator.feedforward, &ff_ref, &ff_out) == 0 &&
                std::isfinite(ff_out)) {
                external_state_.regulator.feedback.integral_output =
                    std::clamp(prev_effort - ff_out, ext_pid.integral_min, ext_pid.integral_max);
            }
        }
        observed_enable_generation_ = snapshot.enable_generation;
        observed_reference_generation_ = snapshot.reference_generation;
        external_history_valid_ = true;
        history_valid_ = false;
    }
    const float velocity_request = std::clamp(ext.velocity_request_rad_s, -v_max, v_max);
    const control_motor_velocity_input vel_input{
        velocity_request, ext.measured_velocity_rad_s, ext.position_reference_rad, dt, false};
    control_motor_velocity_state next_ext = external_state_;
    const control_motor_velocity_state state_before = next_ext;
    control_motor_position_output output{};
    ret = control_motor_velocity_step(&next_ext, &config_.external_velocity, &vel_input, &output.velocity);
    if (ret < 0)
        return fail(ret, snapshot);
    const float effort_ff = config_.external_acceleration_effort_gain * ext.acceleration_ff_rad_s2;
    if (!std::isfinite(effort_ff))
        return fail(-ERANGE, snapshot);
    float effort_total = output.velocity.effort_command + effort_ff;
    if (std::fabs(effort_total) > config_.external_velocity.effort_abs_max) {
        control_motor_velocity_input retry = vel_input;
        retry.freeze_integrator = true;
        next_ext = state_before;
        ret = control_motor_velocity_step(&next_ext, &config_.external_velocity, &retry, &output.velocity);
        if (ret < 0)
            return fail(ret, snapshot);
        effort_total = output.velocity.effort_command + effort_ff;
    }
    effort_total = std::clamp(effort_total, -config_.external_velocity.effort_abs_max,
                              config_.external_velocity.effort_abs_max);
    if (!std::isfinite(effort_total))
        return fail(-ERANGE, snapshot);
    ret = stageEffort(effort_total, snapshot.enable_generation);
    if (ret < 0)
        return fail(ret, snapshot);
    external_state_ = next_ext;
    active_loop_ = ActiveLoop::ExternalVelocity;
    output.position.error = ext.outer_position_error_rad;
    output.position.effective_error = ext.outer_position_error_rad;
    output.position.output = ext.outer_output_rad_s;
    output.position.saturated = ext.outer_saturated;
    output.effort_command = effort_total;
    output.velocity_request_raw_rad_s = ext.velocity_request_rad_s;
    output.velocity_feedforward_rad_s = ext.reference_velocity_rad_s;
    output.velocity_request_rad_s = velocity_request;
    output.velocity_request_limited = ext.velocity_request_limited || (velocity_request != ext.velocity_request_rad_s);
    Telemetry next{};
    next.motor = snapshot;
    next.output = output;
    next.requested_position_rad = position;
    next.target_position_rad = position;
    next.position_rad = absolute
                            ? static_cast<double>(snapshot.feedback.absolute_position_rad)
                            : position -
                                  (config_.reference == PositionReference::StartupRelative ? initial_position_rad_ : 0);
    next.dt_s = dt;
    next.effort_command = effort_total;
    next.effort_unit = config_.effort_unit;
    next.target_valid = next.output_valid = true;
    next.target_sequence = target_sequence_;
    publish(next);
    return 0;
}
}
