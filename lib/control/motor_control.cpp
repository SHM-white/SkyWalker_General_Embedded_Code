#include <control/position_motor.hpp>
#include <control/velocity_motor.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>

#include <control/angle.h>

namespace skywalker::control {
namespace {

bool finiteFloat(double value) {
    return std::isfinite(value) &&
           std::fabs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}

int validateSafety(const MotorSafety &safety, std::uint32_t capabilities) {
    if (!std::isfinite(safety.velocity_abs_max_rad_s) || safety.velocity_abs_max_rad_s <= 0.0f ||
        !std::isfinite(safety.temperature_max_c) || safety.temperature_max_c < 0.0f)
        return -EINVAL;
    if (safety.temperature_max_c > 0.0f && (capabilities & motor::FeedbackTemperature) == 0u)
        return -ENOTSUP;
    return 0;
}

int validateEffort(const motor::MotorInfo &info, EffortUnit unit, float requested_limit) {
    if (!std::isfinite(requested_limit) || requested_limit < 0.0f)
        return -EINVAL;
    std::uint32_t required = 0;
    float available = 0.0f;
    switch (unit) {
    case EffortUnit::Ampere:
        required = motor::CommandCurrent;
        available = info.current_limit_a;
        break;
    case EffortUnit::NewtonMeter:
        required = motor::CommandTorque;
        available = info.torque_limit_nm;
        break;
    default:
        return -EINVAL;
    }
    if ((info.capabilities & required) == 0u)
        return -ENOTSUP;
    if (!std::isfinite(available) || available <= 0.0f || requested_limit > available)
        return -ERANGE;
    return 0;
}

int validateMeasurement(const motor::MotorSnapshot &snapshot, const MotorSafety &safety,
                        std::uint32_t required) {
    if (!snapshot.feedback_fresh)
        return -ESTALE;
    const auto &feedback = snapshot.feedback;
    if ((feedback.valid & required) != required)
        return -ENODATA;
    if (!std::isfinite(feedback.velocity_rad_s) ||
        ((required & motor::FeedbackPosition) != 0u && !std::isfinite(feedback.position_rad)) ||
        ((required & motor::FeedbackAbsolutePosition) != 0u &&
         !std::isfinite(feedback.absolute_position_rad)))
        return -EINVAL;
    if (std::fabs(feedback.velocity_rad_s) > safety.velocity_abs_max_rad_s)
        return -ERANGE;
    if (safety.temperature_max_c > 0.0f) {
        if ((feedback.valid & motor::FeedbackTemperature) == 0u)
            return -ENODATA;
        if (!std::isfinite(feedback.temperature_c))
            return -EINVAL;
        if (feedback.temperature_c >= safety.temperature_max_c)
            return -ERANGE;
        // DM exposes rotor temperature in the generic feedback and MOS
        // temperature separately. Both protected the old MIT bench loops.
        if (snapshot.native_temperatures_valid) {
            if (!std::isfinite(snapshot.native_mos_temperature_c) ||
                !std::isfinite(snapshot.native_rotor_temperature_c))
                return -EINVAL;
            if (snapshot.native_mos_temperature_c >= safety.temperature_max_c ||
                snapshot.native_rotor_temperature_c >= safety.temperature_max_c)
                return -ERANGE;
        }
    }
    return 0;
}

int validateDt(float dt_s, float minimum, float maximum) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0f)
        return -EINVAL;
    return dt_s >= minimum && dt_s <= maximum ? 0 : -ERANGE;
}

} // namespace

VelocityMotor::VelocityMotor(motor::Motor &motor, const Config &config)
    : motor_(motor), config_(config) {}

int VelocityMotor::configure() {
    if (configured_)
        return -EALREADY;
    int ret = control_motor_velocity_validate(&config_.loop);
    if (ret < 0)
        return ret;
    const motor::MotorInfo info = motor_.info();
    ret = validateSafety(config_.safety, info.capabilities);
    if (ret < 0)
        return ret;
    if (config_.loop.requested_velocity_abs_max_rad_s > config_.safety.velocity_abs_max_rad_s)
        return -ERANGE;
    ret = validateEffort(info, config_.effort_unit, config_.loop.effort_abs_max);
    if (ret < 0)
        return ret;
    if ((info.capabilities & motor::FeedbackVelocity) == 0u)
        return -ENOTSUP;
    ret = motor_.bindProducer(this, config_.safety.velocity_abs_max_rad_s,
                              config_.safety.temperature_max_c, motor::FeedbackVelocity, false);
    if (ret < 0)
        return ret;
    configured_ = true;
    return 0;
}

int VelocityMotor::resetFrom(const motor::MotorSnapshot &snapshot) {
    control_motor_velocity_state next_state{};
    const int ret = control_motor_velocity_reset(&next_state, snapshot.feedback.velocity_rad_s, 0.0f);
    if (ret < 0)
        return ret;
    state_ = next_state;
    observed_enable_generation_ = snapshot.enable_generation;
    observed_reference_generation_ = snapshot.reference_generation;
    history_valid_ = true;
    return 0;
}

int VelocityMotor::reset() {
    if (!configured_)
        return -EACCES;
    const motor::MotorSnapshot snapshot = motor_.snapshot();
    if (snapshot.state == motor::MotorState::Active || snapshot.state == motor::MotorState::Enabling)
        return -EBUSY;
    if (!snapshot.feedback_fresh)
        return -EAGAIN;
    const int ret = validateMeasurement(snapshot, config_.safety, motor::FeedbackVelocity);
    if (ret < 0)
        return ret;
    history_valid_ = false;
    const int reset_error = resetFrom(snapshot);
    if (reset_error == 0) {
        Telemetry next{};
        next.motor = snapshot;
        next.effort_unit = config_.effort_unit;
        publish(next);
    }
    return reset_error;
}

int VelocityMotor::fail(int error, const motor::MotorSnapshot &snapshot, bool active) {
    Telemetry next{};
    next.motor = snapshot;
    next.effort_unit = config_.effort_unit;
    next.error = error;
    publish(next);
    if (active)
        motor_.rejectControl(error);
    return error;
}

void VelocityMotor::publish(const Telemetry &next) {
    const k_spinlock_key_t key = k_spin_lock(&telemetry_lock_);
    telemetry_ = next;
    k_spin_unlock(&telemetry_lock_, key);
}

VelocityMotor::Telemetry VelocityMotor::telemetry() const {
    const k_spinlock_key_t key = k_spin_lock(&telemetry_lock_);
    const Telemetry copy = telemetry_;
    k_spin_unlock(&telemetry_lock_, key);
    return copy;
}

int VelocityMotor::update(float target_rad_s, float dt_s) {
    const motor::MotorSnapshot snapshot = motor_.snapshot();
    if (!configured_ || snapshot.state != motor::MotorState::Active)
        return fail(-EACCES, snapshot, false);
    if (!snapshot.feedback_fresh)
        return fail(-ESTALE, snapshot, true);
    if (!snapshot.output_permitted || !motor_.active())
        return fail(-EACCES, snapshot, false);
    if (!std::isfinite(target_rad_s))
        return fail(-EINVAL, snapshot, true);
    if (std::fabs(target_rad_s) > config_.loop.requested_velocity_abs_max_rad_s)
        return fail(-ERANGE, snapshot, true);
    const auto &pid = config_.loop.regulator.feedback;
    int ret = validateDt(dt_s, pid.dt_min_s, pid.dt_max_s);
    if (ret < 0)
        return fail(ret, snapshot, true);
    ret = validateMeasurement(snapshot, config_.safety, motor::FeedbackVelocity);
    if (ret < 0)
        return fail(ret, snapshot, true);

    const bool new_generation = !history_valid_ ||
        snapshot.enable_generation != observed_enable_generation_ ||
        snapshot.reference_generation != observed_reference_generation_;
    if (new_generation) {
        ret = resetFrom(snapshot);
        if (ret < 0)
            return fail(ret, snapshot, true);
        ret = config_.effort_unit == EffortUnit::Ampere
                  ? motor_.setCurrentFrom(this, 0.0f)
                  : motor_.setTorqueFrom(this, 0.0f);
        if (ret < 0)
            return fail(ret, snapshot, true);
        Telemetry next{};
        next.motor = snapshot;
        next.target_rad_s = target_rad_s;
        next.dt_s = dt_s;
        next.effort_unit = config_.effort_unit;
        next.valid = true;
        publish(next);
        return 0;
    }

    control_motor_velocity_state next_state = state_;
    control_motor_velocity_output output{};
    const control_motor_velocity_input input = {
        .requested_velocity_rad_s = target_rad_s,
        .measured_velocity_rad_s = snapshot.feedback.velocity_rad_s,
        .position_reference_rad = 0.0f,
        .dt_s = dt_s,
        .freeze_integrator = false,
    };
    ret = control_motor_velocity_step(&next_state, &config_.loop, &input, &output);
    if (ret < 0)
        return fail(ret, snapshot, true);
    ret = config_.effort_unit == EffortUnit::Ampere
              ? motor_.setCurrentFrom(this, output.effort_command)
              : motor_.setTorqueFrom(this, output.effort_command);
    if (ret < 0)
        return fail(ret, snapshot, true);
    state_ = next_state;
    Telemetry next{};
    next.motor = snapshot;
    next.output = output;
    next.target_rad_s = target_rad_s;
    next.dt_s = dt_s;
    next.effort_command = output.effort_command;
    next.effort_unit = config_.effort_unit;
    next.valid = true;
    publish(next);
    return 0;
}

PositionMotor::PositionMotor(motor::Motor &motor, const Config &config)
    : motor_(motor), config_(config) {}

int PositionMotor::configure() {
    if (configured_)
        return -EALREADY;
    int ret = control_motor_position_validate(&config_.loop);
    if (ret < 0)
        return ret;
    const auto &position_pid = config_.loop.position;
    const auto &velocity_pid = config_.loop.velocity.regulator.feedback;
    if (std::max(position_pid.dt_min_s, velocity_pid.dt_min_s) >
            std::min(position_pid.dt_max_s, velocity_pid.dt_max_s) ||
        config_.loop.velocity.requested_velocity_abs_max_rad_s > config_.safety.velocity_abs_max_rad_s)
        return -ERANGE;
    if (config_.reference != PositionReference::StartupRelative &&
        config_.reference != PositionReference::DriverContinuous &&
        config_.reference != PositionReference::AbsoluteNearest)
        return -EINVAL;

    const motor::MotorInfo info = motor_.info();
    ret = validateSafety(config_.safety, info.capabilities);
    if (ret < 0)
        return ret;
    ret = validateEffort(info, config_.effort_unit, config_.loop.velocity.effort_abs_max);
    if (ret < 0)
        return ret;
    std::uint32_t required = motor::FeedbackPosition | motor::FeedbackVelocity;
    if (config_.reference == PositionReference::AbsoluteNearest)
        required |= motor::FeedbackAbsolutePosition;
    if ((info.capabilities & required) != required)
        return -ENOTSUP;
    ret = motor_.bindProducer(this, config_.safety.velocity_abs_max_rad_s,
                              config_.safety.temperature_max_c, required, true);
    if (ret < 0)
        return ret;
    configured_ = true;
    return 0;
}

int PositionMotor::resetFrom(const motor::MotorSnapshot &snapshot, bool explicit_reset) {
    control_motor_position_state next_state{};
    const int ret = control_motor_position_reset(&next_state, &config_.loop, 0.0f,
                                                 snapshot.feedback.velocity_rad_s);
    if (ret < 0)
        return ret;
    state_ = next_state;
    if (explicit_reset || !explicit_reset_anchor_valid_ ||
        anchor_reference_generation_ != snapshot.reference_generation) {
        initial_position_rad_ = snapshot.feedback.position_rad;
        anchor_reference_generation_ = snapshot.reference_generation;
        explicit_reset_anchor_valid_ = explicit_reset;
    }
    coordinate_origin_rad_ = snapshot.feedback.position_rad;
    observed_enable_generation_ = snapshot.enable_generation;
    observed_reference_generation_ = snapshot.reference_generation;
    history_valid_ = true;
    return 0;
}

int PositionMotor::reset() {
    if (!configured_)
        return -EACCES;
    const motor::MotorSnapshot snapshot = motor_.snapshot();
    if (snapshot.state == motor::MotorState::Active || snapshot.state == motor::MotorState::Enabling)
        return -EBUSY;
    if (!snapshot.feedback_fresh)
        return -EAGAIN;
    if (!snapshot.position_reference_valid)
        return -ENODATA;
    std::uint32_t required = motor::FeedbackPosition | motor::FeedbackVelocity;
    if (config_.reference == PositionReference::AbsoluteNearest)
        required |= motor::FeedbackAbsolutePosition;
    const int ret = validateMeasurement(snapshot, config_.safety, required);
    if (ret < 0)
        return ret;
    history_valid_ = false;
    const int reset_error = resetFrom(snapshot, true);
    if (reset_error == 0) {
        Telemetry next{};
        next.motor = snapshot;
        next.effort_unit = config_.effort_unit;
        publish(next);
    }
    return reset_error;
}

int PositionMotor::fail(int error, const motor::MotorSnapshot &snapshot, bool active) {
    Telemetry next{};
    next.motor = snapshot;
    next.effort_unit = config_.effort_unit;
    next.error = error;
    publish(next);
    if (active)
        motor_.rejectControl(error);
    return error;
}

void PositionMotor::publish(const Telemetry &next) {
    const k_spinlock_key_t key = k_spin_lock(&telemetry_lock_);
    telemetry_ = next;
    k_spin_unlock(&telemetry_lock_, key);
}

PositionMotor::Telemetry PositionMotor::telemetry() const {
    const k_spinlock_key_t key = k_spin_lock(&telemetry_lock_);
    const Telemetry copy = telemetry_;
    k_spin_unlock(&telemetry_lock_, key);
    return copy;
}

int PositionMotor::update(double target_position_rad, float dt_s) {
    const motor::MotorSnapshot snapshot = motor_.snapshot();
    if (!configured_ || snapshot.state != motor::MotorState::Active)
        return fail(-EACCES, snapshot, false);
    if (!snapshot.feedback_fresh)
        return fail(-ESTALE, snapshot, true);
    if (!snapshot.output_permitted || !motor_.active())
        return fail(-EACCES, snapshot, false);
    if (!std::isfinite(target_position_rad))
        return fail(-EINVAL, snapshot, true);
    const auto &position_pid = config_.loop.position;
    const auto &velocity_pid = config_.loop.velocity.regulator.feedback;
    int ret = validateDt(dt_s, std::max(position_pid.dt_min_s, velocity_pid.dt_min_s),
                         std::min(position_pid.dt_max_s, velocity_pid.dt_max_s));
    if (ret < 0)
        return fail(ret, snapshot, true);
    if (!snapshot.position_reference_valid)
        return fail(-ENODATA, snapshot, true);
    std::uint32_t required = motor::FeedbackPosition | motor::FeedbackVelocity;
    if (config_.reference == PositionReference::AbsoluteNearest)
        required |= motor::FeedbackAbsolutePosition;
    ret = validateMeasurement(snapshot, config_.safety, required);
    if (ret < 0)
        return fail(ret, snapshot, true);

    const bool new_generation = !history_valid_ ||
        snapshot.enable_generation != observed_enable_generation_ ||
        snapshot.reference_generation != observed_reference_generation_;
    if (new_generation) {
        ret = resetFrom(snapshot, false);
        if (ret < 0)
            return fail(ret, snapshot, true);
    }

    double resolved = target_position_rad;
    if (config_.reference == PositionReference::StartupRelative)
        resolved += initial_position_rad_;
    if (config_.reference == PositionReference::AbsoluteNearest) {
        if (!finiteFloat(target_position_rad))
            return fail(-ERANGE, snapshot, true);
        float error = 0.0f;
        ret = control_shortest_angle_error(static_cast<float>(target_position_rad),
                                           snapshot.feedback.absolute_position_rad, &error);
        if (ret < 0)
            return fail(ret, snapshot, true);
        resolved = static_cast<double>(snapshot.feedback.position_rad) + static_cast<double>(error);
    }
    if (!finiteFloat(resolved))
        return fail(-ERANGE, snapshot, true);

    if (new_generation) {
        ret = config_.effort_unit == EffortUnit::Ampere
                  ? motor_.setCurrentFrom(this, 0.0f)
                  : motor_.setTorqueFrom(this, 0.0f);
        if (ret < 0)
            return fail(ret, snapshot, true);
        Telemetry next{};
        next.motor = snapshot;
        next.requested_position_rad = target_position_rad;
        next.target_position_rad = resolved;
        next.position_rad = static_cast<double>(snapshot.feedback.position_rad) -
                            (config_.reference == PositionReference::StartupRelative ? initial_position_rad_ : 0.0);
        next.dt_s = dt_s;
        next.effort_unit = config_.effort_unit;
        next.valid = true;
        publish(next);
        return 0;
    }

    control_motor_position_state next_state = state_;
    double next_origin = coordinate_origin_rad_;
    if (std::fabs(static_cast<double>(snapshot.feedback.position_rad) - next_origin) > 128.0) {
        const double shift = static_cast<double>(snapshot.feedback.position_rad) - next_origin;
        if (!finiteFloat(shift))
            return fail(-ERANGE, snapshot, true);
        next_state.position.previous_measurement -= static_cast<float>(shift);
        next_origin = snapshot.feedback.position_rad;
    }
    const double local_target = resolved - next_origin;
    const double local_position = static_cast<double>(snapshot.feedback.position_rad) - next_origin;
    if (!finiteFloat(local_target) || !finiteFloat(local_position))
        return fail(-ERANGE, snapshot, true);
    const control_motor_position_input input = {
        .continuous_target_rad = static_cast<float>(local_target),
        .continuous_position_rad = static_cast<float>(local_position),
        .measured_velocity_rad_s = snapshot.feedback.velocity_rad_s,
        .dt_s = dt_s,
        .position_reference_rad = static_cast<float>(resolved),
        .has_position_reference = true,
    };
    control_motor_position_output output{};
    ret = control_motor_position_step(&next_state, &config_.loop, &input, &output);
    if (ret < 0)
        return fail(ret, snapshot, true);
    ret = config_.effort_unit == EffortUnit::Ampere
              ? motor_.setCurrentFrom(this, output.effort_command)
              : motor_.setTorqueFrom(this, output.effort_command);
    if (ret < 0)
        return fail(ret, snapshot, true);
    state_ = next_state;
    coordinate_origin_rad_ = next_origin;
    Telemetry next{};
    next.motor = snapshot;
    next.output = output;
    next.requested_position_rad = target_position_rad;
    next.target_position_rad = resolved;
    next.position_rad = static_cast<double>(snapshot.feedback.position_rad) -
                        (config_.reference == PositionReference::StartupRelative ? initial_position_rad_ : 0.0);
    next.dt_s = dt_s;
    next.effort_command = output.effort_command;
    next.effort_unit = config_.effort_unit;
    next.valid = true;
    publish(next);
    return 0;
}

} // namespace skywalker::control
