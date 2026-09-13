#include <algorithm>
#include <cerrno>
#include <cmath>
#include <zephyr/kernel.h>
#include <control/angle.h>
#include <control/position_motor.hpp>
#include <control/velocity_motor.hpp>

namespace skywalker::control {

MotorRuntime::MotorRuntime(MotorBackend &backend, EffortUnit unit, MotorSafety safety)
    : backend_(backend), unit_(unit), safety_(safety) {
}

int MotorRuntime::block(int error) {
    status_.state = MotorRunState::ConfigBlocked;
    status_.error = error;
    return error;
}
int MotorRuntime::configure(float effort_limit, std::uint32_t required) {
    if (owns_backend_ || status_.state == MotorRunState::ConfigBlocked)
        return -EALREADY;
    if (!std::isfinite(effort_limit) || effort_limit < 0 || !std::isfinite(safety_.velocity_abs_max_rad_s) ||
        safety_.velocity_abs_max_rad_s <= 0 || !std::isfinite(safety_.temperature_max_c) ||
        safety_.temperature_max_c < 0 || !safety_.recovery_retry_ms || !safety_.recovery_poll_ms ||
        safety_.recovery_retry_ms > 1000 || safety_.recovery_poll_ms > safety_.recovery_retry_ms)
        return block(-EINVAL);
    if (backend_.claimed_)
        return block(-EBUSY);
    int ret = backend_.describe(info_);
    if (ret < 0)
        return block(ret);
    if (unit_ == EffortUnit::Unspecified || unit_ != info_.effort_unit)
        return block(-EINVAL);
    if (!std::isfinite(info_.effort_limit) || info_.effort_limit <= 0 || effort_limit > info_.effort_limit ||
        !info_.feedback_timeout_ms)
        return block(-ERANGE);
    required_ = required | motor::FeedbackVelocity;
    if (safety_.temperature_max_c > 0)
        required_ |= motor::FeedbackTemperature;
    if ((info_.capabilities & required_) != required_)
        return block(-ENOTSUP);
    backend_.claimed_ = true;
    owns_backend_ = true;
    prepare_attempted_ = true;
    ret = backend_.configure();
    if (ret < 0)
        return block(ret);
    status_.state = MotorRunState::Waiting;
    return 0;
}
int MotorRuntime::prepare(float effort_limit, std::uint32_t required, MotorMeasurement &first) {
    int ret = configure(effort_limit, required);
    if (ret < 0)
        return ret;
    const auto deadline = k_uptime_get() + 3000;
    do {
        ret = poll(k_uptime_get(), first);
        if (ret == 0)
            return 0;
        k_sleep(K_MSEC(5));
    } while (k_uptime_get() < deadline);
    return ret;
}
ExecutionState MotorRuntime::state() const {
    switch (status_.state) {
    case MotorRunState::Running:
        return ExecutionState::Active;
    case MotorRunState::Ready:
        return ExecutionState::Ready;
    case MotorRunState::Recovering:
        return ExecutionState::Recovering;
    case MotorRunState::EStopLatched:
        return ExecutionState::EStopLatched;
    case MotorRunState::ConfigBlocked:
        return ExecutionState::ConfigBlocked;
    default:
        return ExecutionState::Waiting;
    }
}
int MotorRuntime::suspend(PauseReason reason) {
    if (status_.state == MotorRunState::ConfigBlocked || !owns_backend_)
        return -EACCES;
    if (status_.state == MotorRunState::EStopLatched)
        return 0;
    const bool transition = status_.state != MotorRunState::Waiting;
    if (transition || reason == PauseReason::EmergencyStop) {
        reference_reset_ = false;
        stable_started_ = false;
        next_retry_ms_ = 0;
        status_.reason = reason;
        status_.state = reason == PauseReason::EmergencyStop ? MotorRunState::EStopLatched : MotorRunState::Waiting;
        status_.stop_error = backend_.stop();
    }
    return status_.stop_error;
}
int MotorRuntime::clearEmergencyStop(bool released) {
    if (!released)
        return -EBUSY;
    if (status_.state == MotorRunState::EStopLatched) {
        status_.state = MotorRunState::Waiting;
        next_retry_ms_ = 0;
    }
    return 0;
}
int MotorRuntime::poll(std::uint64_t now, MotorMeasurement &measurement) {
    if (!owns_backend_ || status_.state == MotorRunState::ConfigBlocked ||
        status_.state == MotorRunState::EStopLatched || status_.state == MotorRunState::Stopped)
        return -EACCES;
    if (status_.state == MotorRunState::Running)
        return -EALREADY;
    if (now < next_retry_ms_)
        return -EAGAIN;
    const bool was_ready = status_.state == MotorRunState::Ready;
    int ret = backend_.pollPrepare(now);
    if (ret == 0 && !reference_reset_) {
        ret = backend_.resetMeasurementReference();
        if (ret == 0)
            reference_reset_ = true;
    }
    MotorMeasurement next{};
    if (ret == 0)
        ret = read(next);
    if (ret < 0) {
        if (was_ready)
            suspend(PauseReason::FeedbackTimeout);
        stable_started_ = false;
        reference_reset_ = false;
        status_.state = MotorRunState::Recovering;
        status_.last_recovery_error = ret;
        ++status_.recovery_attempts;
        next_retry_ms_ = now + ((ret == -EAGAIN || ret == -EINPROGRESS) ? safety_.recovery_poll_ms
                                                                        : safety_.recovery_retry_ms);
        return ret;
    }
    next_retry_ms_ = 0;
    if (!stable_started_ || now < stable_since_ms_ || next.feedback.timestamp_ms < last_feedback_ms_ ||
        next.feedback.timestamp_ms - last_feedback_ms_ > info_.feedback_timeout_ms) {
        if (was_ready) {
            suspend(PauseReason::FeedbackTimeout);
            status_.state = MotorRunState::Recovering;
            return -EAGAIN;
        }
        stable_started_ = true;
        stable_since_ms_ = now;
        last_feedback_ms_ = next.feedback.timestamp_ms;
    }
    else if (next.feedback.timestamp_ms != last_feedback_ms_) {
        last_feedback_ms_ = next.feedback.timestamp_ms;
        if (now - stable_since_ms_ >= safety_.recovery_stable_ms && !was_ready) {
            ++status_.resume_generation;
            status_.state = MotorRunState::Ready;
            status_.error = 0;
            status_.last_recovery_error = 0;
        }
    }
    if (safety_.recovery_stable_ms == 0 && !was_ready && status_.state != MotorRunState::Ready) {
        status_.state = MotorRunState::Ready;
        ++status_.resume_generation;
    }
    if (status_.state != MotorRunState::Ready) {
        status_.state = MotorRunState::Recovering;
        return -EAGAIN;
    }
    measurement = next;
    return 0;
}

int MotorRuntime::read(MotorMeasurement &measurement) {
    MotorMeasurement next{};
    int ret = backend_.read(next);
    if (ret < 0)
        return ret;
    const auto &fb = next.feedback;
    const auto now = static_cast<std::uint64_t>(k_uptime_get());
    if (fb.timestamp_ms == 0 || now < fb.timestamp_ms || now - fb.timestamp_ms > info_.feedback_timeout_ms)
        return -ESTALE;
    if ((fb.valid & required_) != required_)
        return -ENODATA;
    if (!std::isfinite(fb.velocity_rad_s))
        return -EINVAL;
    if ((required_ & motor::FeedbackPosition) && !std::isfinite(next.position_rad))
        return -EINVAL;
    if ((required_ & motor::FeedbackAbsolutePosition) && !std::isfinite(fb.absolute_position_rad))
        return -EINVAL;
    if (std::fabs(fb.velocity_rad_s) > safety_.velocity_abs_max_rad_s)
        return -ERANGE;
    if (safety_.temperature_max_c > 0.0f) {
        if (!std::isfinite(fb.temperature_c) ||
            (next.driver_temperature_valid && !std::isfinite(next.driver_temperature_c)))
            return -EINVAL;
        if (fb.temperature_c >= safety_.temperature_max_c ||
            (next.driver_temperature_valid && next.driver_temperature_c >= safety_.temperature_max_c))
            return -ERANGE;
    }
    measurement = next;
    return 0;
}

int MotorRuntime::arm() {
    if (status_.state != MotorRunState::Ready)
        return -EAGAIN;
    MotorMeasurement current{};
    int check = read(current);
    if (check < 0)
        return fail(check);
    const int ret = backend_.arm();
    if (ret < 0)
        return fail(ret);
    started_ms_ = previous_ms_ = k_uptime_get();
    status_.state = MotorRunState::Running;
    return 0;
}

int MotorRuntime::cycle(float dt_min_s, float dt_max_s, MotorMeasurement &measurement, float &dt_s) {
    if (status_.state != MotorRunState::Running)
        return -EACCES;
    const auto now = k_uptime_get();
    if (now <= previous_ms_)
        return fail(-ERANGE, PauseReason::InvalidCycle);
    dt_s = static_cast<float>(now - previous_ms_) / 1000.0f;
    if (dt_s < dt_min_s || dt_s > dt_max_s)
        return fail(-ERANGE, PauseReason::InvalidCycle);
    const int ret = read(measurement);
    if (ret < 0)
        return fail(ret, PauseReason::FeedbackTimeout);
    previous_ms_ = now;
    return 0;
}

int MotorRuntime::send(float effort) {
    int ret = backend_.write(effort);
    if (ret == 0)
        ret = backend_.flush();
    return ret < 0 ? fail(ret) : 0;
}

int MotorRuntime::fail(int error, PauseReason reason) {
    if (!owns_backend_)
        return block(error);
    if (status_.error == 0)
        status_.error = error;
    suspend(reason);
    return error;
}
int MotorRuntime::stop() {
    const int ret = owns_backend_ && prepare_attempted_ ? backend_.stop() : 0;
    status_.stop_error = ret;
    status_.state = MotorRunState::Stopped;
    return ret;
}

std::int64_t MotorRuntime::elapsedMs() const {
    return status_.state == MotorRunState::Running ? k_uptime_get() - started_ms_ : 0;
}

VelocityMotor::VelocityMotor(MotorBackend &backend, const Config &config)
    : config_(config), runtime_(backend, config.effort_unit, config.safety) {
}

int VelocityMotor::configure() {
    if (begin_attempted_)
        return -EALREADY;
    begin_attempted_ = true;
    int ret = control_motor_velocity_validate(&config_.loop);
    if (ret < 0)
        return runtime_.block(ret);
    if (config_.loop.requested_velocity_abs_max_rad_s > config_.safety.velocity_abs_max_rad_s)
        return runtime_.block(-ERANGE);
    return runtime_.configure(config_.loop.effort_abs_max, 0);
}
int VelocityMotor::poll(std::uint64_t now_ms) {
    const int ret = runtime_.poll(now_ms, prepared_);
    if (ret == 0)
        telemetry_.measurement = prepared_;
    return ret;
}
int VelocityMotor::suspend(PauseReason reason) {
    telemetry_.valid = false;
    return runtime_.suspend(reason);
}
int VelocityMotor::resume() {
    if (runtime_.state() != ExecutionState::Ready)
        return -EAGAIN;
    int fresh = runtime_.poll(k_uptime_get(), prepared_);
    if (fresh < 0)
        return fresh;
    int ret = control_motor_velocity_reset(&state_, prepared_.feedback.velocity_rad_s, 0);
    if (ret < 0)
        return runtime_.fail(ret);
    return runtime_.arm();
}
int VelocityMotor::begin() {
    int ret = configure();
    if (ret < 0)
        return ret;
    const auto deadline = k_uptime_get() + 3000;
    do {
        ret = poll(k_uptime_get());
        if (ret == 0)
            return resume();
        k_sleep(K_MSEC(5));
    } while (k_uptime_get() < deadline);
    return ret;
}

int VelocityMotor::update(float target_velocity_rad_s) {
    telemetry_.valid = false;
    if (status().state != MotorRunState::Running)
        return -EACCES;
    if (!std::isfinite(target_velocity_rad_s))
        return runtime_.fail(-EINVAL);
    Telemetry next{};
    const auto &pid = config_.loop.regulator.feedback;
    int ret = runtime_.cycle(pid.dt_min_s, pid.dt_max_s, next.measurement, next.dt_s);
    if (ret < 0)
        return ret;
    const control_motor_velocity_input input = {
        .requested_velocity_rad_s = target_velocity_rad_s,
        .measured_velocity_rad_s = next.measurement.feedback.velocity_rad_s,
        .position_reference_rad = 0.0f,
        .dt_s = next.dt_s,
        .freeze_integrator = false,
    };
    ret = control_motor_velocity_step(&state_, &config_.loop, &input, &next.output);
    if (ret < 0)
        return runtime_.fail(ret);
    ret = runtime_.send(next.output.effort_command);
    if (ret < 0)
        return ret;
    next.target_rad_s = target_velocity_rad_s;
    next.valid = true;
    telemetry_ = next;
    return 0;
}

int VelocityMotor::stop() {
    telemetry_.valid = false;
    return runtime_.stop();
}

PositionMotor::PositionMotor(MotorBackend &backend, const Config &config)
    : config_(config), runtime_(backend, config.effort_unit, config.safety) {
}

int PositionMotor::configure() {
    if (begin_attempted_)
        return -EALREADY;
    begin_attempted_ = true;
    int ret = control_motor_position_validate(&config_.loop);
    if (ret < 0)
        return runtime_.block(ret);
    const auto &position_pid = config_.loop.position;
    const auto &velocity_pid = config_.loop.velocity.regulator.feedback;
    if (std::max(position_pid.dt_min_s, velocity_pid.dt_min_s) >
            std::min(position_pid.dt_max_s, velocity_pid.dt_max_s) ||
        config_.loop.velocity.requested_velocity_abs_max_rad_s > config_.safety.velocity_abs_max_rad_s)
        return runtime_.block(-ERANGE);
    if (config_.reference != PositionReference::StartupRelative &&
        config_.reference != PositionReference::DriverContinuous &&
        config_.reference != PositionReference::AbsoluteNearest)
        return runtime_.block(-EINVAL);
    std::uint32_t required = motor::FeedbackPosition;
    if (config_.reference == PositionReference::AbsoluteNearest)
        required |= motor::FeedbackAbsolutePosition;
    return runtime_.configure(config_.loop.velocity.effort_abs_max, required);
}
int PositionMotor::poll(std::uint64_t now_ms) {
    const int ret = runtime_.poll(now_ms, prepared_);
    if (ret == 0)
        telemetry_.measurement = prepared_;
    return ret;
}
int PositionMotor::suspend(PauseReason reason) {
    telemetry_.valid = false;
    return runtime_.suspend(reason);
}
int PositionMotor::resume() {
    if (runtime_.state() != ExecutionState::Ready)
        return -EAGAIN;
    int fresh = runtime_.poll(k_uptime_get(), prepared_);
    if (fresh < 0)
        return fresh;
    initial_position_rad_ = coordinate_origin_rad_ = prepared_.position_rad;
    int ret = control_motor_position_reset(&state_, &config_.loop, 0, prepared_.feedback.velocity_rad_s);
    if (ret < 0)
        return runtime_.fail(ret);
    return runtime_.arm();
}
int PositionMotor::begin() {
    int ret = configure();
    if (ret < 0)
        return ret;
    const auto deadline = k_uptime_get() + 3000;
    do {
        ret = poll(k_uptime_get());
        if (ret == 0)
            return resume();
        k_sleep(K_MSEC(5));
    } while (k_uptime_get() < deadline);
    return ret;
}

int PositionMotor::update(double target_position_rad) {
    telemetry_.valid = false;
    if (status().state != MotorRunState::Running)
        return -EACCES;
    if (!std::isfinite(target_position_rad))
        return runtime_.fail(-EINVAL);
    Telemetry next{};
    const auto &pid = config_.loop.position;
    const auto &velocity_pid = config_.loop.velocity.regulator.feedback;
    int ret = runtime_.cycle(std::max(pid.dt_min_s, velocity_pid.dt_min_s),
                             std::min(pid.dt_max_s, velocity_pid.dt_max_s), next.measurement, next.dt_s);
    if (ret < 0)
        return ret;
    double resolved = target_position_rad;
    if (config_.reference == PositionReference::StartupRelative)
        resolved += initial_position_rad_;
    if (config_.reference == PositionReference::AbsoluteNearest) {
        float error = 0.0f;
        ret = control_shortest_angle_error(static_cast<float>(target_position_rad),
                                           next.measurement.feedback.absolute_position_rad, &error);
        if (ret < 0)
            return runtime_.fail(ret);
        resolved = next.measurement.position_rad + static_cast<double>(error);
    }
    // Rebase only after many turns. Shift PID measurement history by the same
    // amount, so changing coordinates does not create a derivative impulse.
    if (std::fabs(next.measurement.position_rad - coordinate_origin_rad_) > 128.0) {
        const double shift = next.measurement.position_rad - coordinate_origin_rad_;
        state_.position.previous_measurement -= static_cast<float>(shift);
        coordinate_origin_rad_ = next.measurement.position_rad;
    }
    control_motor_position_input input = {
        .continuous_target_rad = static_cast<float>(resolved - coordinate_origin_rad_),
        .continuous_position_rad = static_cast<float>(next.measurement.position_rad - coordinate_origin_rad_),
        .measured_velocity_rad_s = next.measurement.feedback.velocity_rad_s,
        .dt_s = next.dt_s,
    };
    // The C kernel uses target coordinates for gravity feedforward. Keep that
    // coordinate independent of the PID origin (see the optional reference).
    input.position_reference_rad = static_cast<float>(resolved);
    input.has_position_reference = true;
    ret = control_motor_position_step(&state_, &config_.loop, &input, &next.output);
    if (ret < 0)
        return runtime_.fail(ret);
    ret = runtime_.send(next.output.effort_command);
    if (ret < 0)
        return ret;
    next.requested_position_rad = target_position_rad;
    next.target_position_rad = resolved;
    next.position_rad = next.measurement.position_rad -
                        (config_.reference == PositionReference::StartupRelative ? initial_position_rad_ : 0.0);
    next.valid = true;
    telemetry_ = next;
    return 0;
}

int PositionMotor::stop() {
    telemetry_.valid = false;
    return runtime_.stop();
}

} // namespace skywalker::control
