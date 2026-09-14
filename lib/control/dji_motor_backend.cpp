#include <cerrno>
#include <control/can_recovery.hpp>
#include <zephyr/kernel.h>
#include <control/dji_motor_backend.hpp>

namespace skywalker::control {

int DjiMotorBackend::describe(MotorInfo &info) {
    if (motor_ == nullptr || !device_is_ready(motor_))
        return -ENODEV;
    const int ret = motor::dji::describe(motor_, descriptor_);
    if (ret < 0)
        return ret;
    const auto caps = motor::capabilities(motor_);
    if ((caps & motor::CommandCurrent) == 0)
        return -ENOTSUP;
    info = {EffortUnit::Ampere, descriptor_.configured_current_limit_a, caps, CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS};
    return 0;
}

int DjiMotorBackend::configure() {
    if (configured_)
        return -EALREADY;
    int ret = bus_.init(descriptor_.can);
    if (ret < 0)
        return ret;
    ret = bus_.attach(motor_);
    if (ret < 0)
        return ret;
    configured_ = true;
    return 0;
}
int DjiMotorBackend::prepare() {
    if (!configured_) {
        int ret = configure();
        if (ret < 0)
            return ret;
    }
    return pollPrepare(k_uptime_get());
}
int DjiMotorBackend::pollPrepare(std::uint64_t now_ms) {
    if (!configured_)
        return -EACCES;
    int ret = pollCanRecovery(descriptor_.can);
    if (ret < 0)
        return ret;
    if (bus_.state() == motor::dji::BusState::Fault) {
        ret = bus_.recover(report_);
        if (ret < 0)
            return ret;
    }
    (void)now_ms;
    return motor::getState(motor_) == motor::State::Ready ? 0 : -EAGAIN;
}
int DjiMotorBackend::resetMeasurementReference() {
    return motor::dji::resetMeasurementReference(motor_);
}

int DjiMotorBackend::read(MotorMeasurement &measurement) {
    MotorMeasurement next{};
    const int ret = motor::readFeedback(motor_, next.feedback);
    if (ret < 0)
        return ret;
    if (motor::getState(motor_) != motor::State::Ready)
        return -EHOSTDOWN;
    next.position_rad = next.feedback.position_rad;
    measurement = next;
    return 0;
}

int DjiMotorBackend::arm() {
    const int ret = bus_.arm(report_);
    return ret < 0 ? ret : (report_.zero_sent ? 0 : -EIO);
}

int DjiMotorBackend::write(float effort_a) {
    return motor::setCurrent(motor_, effort_a);
}
int DjiMotorBackend::flush() {
    return bus_.flush(report_);
}

int DjiMotorBackend::stop() {
    if (bus_.state() == motor::dji::BusState::Uninitialized)
        return 0;
    const int ret = bus_.stop(stop_report_);
    return ret < 0 ? ret : (stop_report_.zero_sent ? 0 : -EIO);
}

} // namespace skywalker::control
