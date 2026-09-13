#include <cerrno>
#include <control/can_recovery.hpp>
#include <cmath>
#include <zephyr/kernel.h>
#include <control/dm_motor_backend.hpp>

namespace skywalker::control {

int DmMotorBackend::describe(MotorInfo &info) {
    if (motor_ == nullptr || !device_is_ready(motor_))
        return -ENODEV;
    const int ret = motor::dm::describe(motor_, descriptor_);
    if (ret < 0)
        return ret;
    if (descriptor_.mode != motor::dm::ControlMode::Mit)
        return -ENOTSUP;
    const auto caps = motor::capabilities(motor_);
    if ((caps & motor::CommandTorque) == 0)
        return -ENOTSUP;
    info = {EffortUnit::NewtonMeter, descriptor_.torque_limit_nm, caps, CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS};
    return 0;
}

int DmMotorBackend::configure() {
    if (configured_)
        return -EALREADY;
    int ret = bus_.init(descriptor_.can);
    if (ret < 0)
        return ret;
    ret = bus_.attach(motor_);
    if (ret < 0)
        return ret;
    if (power_on_) {
        ret = power_on_();
        if (ret < 0)
            return ret;
    }
    configured_ = true;
    return 0;
}
int DmMotorBackend::prepare() {
    if (!configured_) {
        int ret = configure();
        if (ret < 0)
            return ret;
    }
    return pollPrepare(k_uptime_get());
}
int DmMotorBackend::pollPrepare(std::uint64_t now_ms) {
    if (!configured_)
        return -EACCES;
    int ret = pollCanRecovery(descriptor_.can);
    if (ret < 0)
        return ret;
    if (bus_.state() == motor::dm::BusState::Fault) {
        ret = bus_.recover(report_);
        if (ret < 0)
            return ret;
    }
    // Disabled DM firmware may only reply to host probes. Probe often enough to
    // establish multiple fresh samples across the 30 ms preparation interval.
    if (now_ms >= next_probe_ms_) {
        next_probe_ms_ = now_ms + 10;
        if (motor::getState(motor_) == motor::State::Fault)
            ret = bus_.recover(report_);
        else
            ret = bus_.stop(report_);
        if (ret < 0)
            return ret;
    }
    motor::dm::RawFeedback raw{};
    ret = motor::dm::readRawFeedback(motor_, raw);
    const auto observed_now = static_cast<std::uint64_t>(k_uptime_get());
    if (ret == 0 && raw.timestamp_ms && observed_now >= raw.timestamp_ms &&
        observed_now - raw.timestamp_ms <= CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS &&
        raw.status == motor::dm::DriveStatus::Disabled && motor::getState(motor_) == motor::State::Ready)
        return 0;
    return -EAGAIN;
}
int DmMotorBackend::resetMeasurementReference() {
    position_initialized_ = false;
    return 0;
}

int DmMotorBackend::read(MotorMeasurement &measurement) {
    MotorMeasurement next{};
    int ret = motor::readFeedback(motor_, next.feedback);
    if (ret < 0)
        return ret;
    motor::dm::RawFeedback raw{};
    ret = motor::dm::readRawFeedback(motor_, raw);
    if (ret < 0)
        return ret;
    if (motor::getState(motor_) != motor::State::Ready)
        return -EHOSTDOWN;
    const auto now = static_cast<std::uint64_t>(k_uptime_get());
    if (raw.timestamp_ms == 0 || now < raw.timestamp_ms ||
        now - raw.timestamp_ms > CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS)
        return -ESTALE;
    const bool disabled_before_arm = bus_.state() == motor::dm::BusState::Safe &&
                                     raw.status == motor::dm::DriveStatus::Disabled;
    if (raw.status != motor::dm::DriveStatus::Enabled && !disabled_before_arm)
        return -EHOSTDOWN;
    const auto &fb = next.feedback;
    constexpr auto required = motor::FeedbackPosition | motor::FeedbackVelocity | motor::FeedbackTorque |
                              motor::FeedbackTemperature;
    if ((fb.valid & required) != required)
        return -ENODATA;
    if (fb.timestamp_ms == 0 || now < fb.timestamp_ms ||
        now - fb.timestamp_ms > CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS)
        return -ESTALE;
    if (!std::isfinite(fb.position_rad) || !std::isfinite(fb.velocity_rad_s) || !std::isfinite(fb.torque_nm) ||
        !std::isfinite(fb.temperature_c))
        return -EINVAL;
    // Requires actual firmware wrap at +/-PMAX and less than PMAX movement
    // between consumed feedback frames. Never use 2*pi as the protocol period.
    if (!position_initialized_) {
        continuous_position_rad_ = fb.position_rad;
        position_initialized_ = true;
    }
    else {
        continuous_position_rad_ += static_cast<double>(
            std::remainder(fb.position_rad - previous_position_rad_, 2.0f * descriptor_.limits.position_max_rad));
    }
    previous_position_rad_ = fb.position_rad;
    next.position_rad = continuous_position_rad_;
    next.driver_temperature_c = raw.mos_temperature_c;
    next.driver_temperature_valid = true;
    measurement = next;
    return 0;
}

int DmMotorBackend::arm() {
    return bus_.arm(report_);
}
int DmMotorBackend::write(float effort_nm) {
    return motor::setTorque(motor_, effort_nm);
}
int DmMotorBackend::flush() {
    return bus_.flush(report_);
}

int DmMotorBackend::stop() {
    if (bus_.state() == motor::dm::BusState::Uninitialized)
        return 0;
    return bus_.stop(stop_report_);
}

} // namespace skywalker::control
