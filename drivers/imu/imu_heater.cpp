#include <drivers/imu/imu_heater.hpp>
#include <cmath>
#include <cerrno>
namespace skywalker::imu {
void ImuHeater::record(float duty, int error, int off_error) {
    auto k = k_spin_lock(&lock_);
    status_ = {duty, error, off_error};
    k_spin_unlock(&lock_, k);
}
ImuHeater::Snapshot ImuHeater::snapshot() const {
    auto k = k_spin_lock(&lock_);
    auto s = status_;
    k_spin_unlock(&lock_, k);
    return s;
}
int ImuHeater::init() {
    if (initialized_)
        return -EALREADY;
    const float period = float(config_.control_period_us) * 1e-6f;
    if (!config_.pwm.dev || !config_.pwm.period || !config_.temperature_timeout_us ||
        !std::isfinite(config_.target_c) || !std::isfinite(config_.maximum_c) ||
        config_.target_c >= config_.maximum_c || !config_.control_period_us || control_pid_validate(&config_.pid) < 0 ||
        config_.pid.output_min < 0 || config_.pid.output_max > 1 || period < config_.pid.dt_min_s ||
        period > config_.pid.dt_max_s)
        return -EINVAL;
    if (!pwm_is_ready_dt(&config_.pwm))
        return -ENODEV;
    const int r = pwm_set_dt(&config_.pwm, config_.pwm.period, 0);
    record(0, r, r);
    if (!r)
        initialized_ = true;
    return r;
}
int ImuHeater::fail(int error) {
    const int r = pwm_set_dt(&config_.pwm, config_.pwm.period, 0);
    const float duty = r < 0 ? snapshot().duty : 0;
    pid_ = {};
    have_time_ = false;
    record(duty, error, r);
    return r < 0 ? r : error;
}
int ImuHeater::disable() {
    if (!initialized_)
        return -EACCES;
    return fail(0);
}
int ImuHeater::update(const core::Measurement<float> &t, core::TimeUs now) {
    if (!initialized_)
        return -EACCES;
    if (!core::fresh(t.stamp, now, config_.temperature_timeout_us))
        return fail(-ESTALE);
    if (!std::isfinite(t.value) || t.value >= config_.maximum_c || t.value < -40)
        return fail(-ERANGE);
    if (have_time_ && now < previous_us_)
        return fail(-ESTALE);
    if (have_time_ && now - previous_us_ < config_.control_period_us)
        return -EAGAIN;
    const float dt = float(have_time_ ? now - previous_us_ : config_.control_period_us) * 1e-6f;
    const control_pid_input in{config_.target_c, t.value, dt, false};
    control_pid_result result{};
    const int r = control_pid_step(&pid_, &config_.pid, &in, &result);
    if (r < 0)
        return fail(r);
    const auto pulse = static_cast<std::uint32_t>(double(config_.pwm.period) * double(result.output));
    const int pr = pwm_set_dt(&config_.pwm, config_.pwm.period, pulse);
    if (pr < 0)
        return fail(pr);
    previous_us_ = now;
    have_time_ = true;
    record(result.output, 0);
    return 0;
}
}
