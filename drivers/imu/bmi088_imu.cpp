#include <drivers/imu/bmi088_imu.hpp>
#include <core/attitude.hpp>
#include <core/clock.hpp>
#include <zephyr/drivers/sensor.h>
#include <cerrno>

namespace skywalker::imu {
int Bmi088Imu::init() {
    if (initialized_)
        return -EALREADY;
    if (!config_.sample_period_us || !config_.temperature_period_us || !config_.max_input_skew_us ||
        !core::normalize(config_.sensor_to_body))
        return -EINVAL;
    if (!accel_ || !gyro_ || !device_is_ready(accel_) || !device_is_ready(gyro_))
        return -ENODEV;
    if (estimator_) {
#ifdef CONFIG_SKYWALKER_ATTITUDE_EKF
        const int r = estimator_->init();
        if (r < 0)
            return r;
#else
        return -ENOTSUP;
#endif
    }
    const int ret = state_.init(Accel | Gyro | Temperature | (estimator_ ? Orientation : 0u), config_.reference);
    if (ret < 0)
        return ret;
    if (estimator_)
        state_.resetOrientation(config_.reference);
    initialized_ = true;
    return 0;
}
int Bmi088Imu::service() {
    if (!initialized_)
        return -EACCES;
    const auto now = core::monotonicTimeUs();
    if (now < next_sample_)
        return -EAGAIN;
    next_sample_ = now + config_.sample_period_us;
    int error = 0;
    auto report = [&](int r) {
        if (r < 0) {
            state_.error(r, true);
            if (!error)
                error = r;
        }
    };
    auto read = [](const device *dev, sensor_channel ch, core::Measurement<core::Vec3> &m) {
        int r = sensor_sample_fetch(dev);
        sensor_value v[3]{};
        if (!r)
            r = sensor_channel_get(dev, ch, v);
        if (r < 0)
            return r;
        m.value = {sensor_value_to_float(&v[0]), sensor_value_to_float(&v[1]), sensor_value_to_float(&v[2])};
        m.stamp = {core::monotonicTimeUs(), 0, true};
        return core::finite(m.value) ? 0 : -ERANGE;
    };
    core::Measurement<core::Vec3> a{}, g{};
    const int ar = read(accel_, SENSOR_CHAN_ACCEL_XYZ, a);
    report(ar);
    const int gr = read(gyro_, SENSOR_CHAN_GYRO_XYZ, g);
    report(gr);
    Update u{};
    if (!ar) {
        u.sample.accel_m_s2 = a;
        u.sample.accel_m_s2.value = core::rotate(config_.sensor_to_body, a.value);
        u.updated_mask |= Accel;
    }
    if (!gr) {
        u.sample.gyro_rad_s = g;
        u.sample.gyro_rad_s.value = core::rotate(config_.sensor_to_body, g.value);
        u.updated_mask |= Gyro;
    }
    if (u.updated_mask)
        report(state_.publish(u));
    // In the pinned BMI08x driver DIE_TEMP performs its own SPI read, not a cache get.
    if (now >= next_temperature_) {
        next_temperature_ = now + config_.temperature_period_us;
        sensor_value t{};
        const int tr = sensor_channel_get(accel_, SENSOR_CHAN_DIE_TEMP, &t);
        if (!tr) {
            Update temp{};
            temp.updated_mask = Temperature;
            temp.sample.temperature_c = {sensor_value_to_float(&t), {core::monotonicTimeUs(), 0, true}};
            report(state_.publish(temp));
        }
        else
            report(tr);
    }
#ifdef CONFIG_SKYWALKER_ATTITUDE_EKF
    if (estimator_ && !ar && !gr) {
        const auto skew = a.stamp.time_us > g.stamp.time_us ? a.stamp.time_us - g.stamp.time_us
                                                            : g.stamp.time_us - a.stamp.time_us;
        if (skew <= config_.max_input_skew_us) {
            const auto generation = estimator_->generation();
            const int er = estimator_->update({a.value, g.value, std::max(a.stamp.time_us, g.stamp.time_us)});
            if (generation != estimator_->generation()) {
                if (++config_.reference.epoch == 0)
                    ++config_.reference.epoch;
                state_.resetOrientation(config_.reference);
            }
            if (!er) {
                Update q{};
                q.updated_mask = Orientation;
                q.sample.reference = config_.reference;
                q.sample.orientation = {core::multiply(estimator_->attitude(), core::conjugate(config_.sensor_to_body)),
                                        {std::max(a.stamp.time_us, g.stamp.time_us), 0, true}};
                q.sample.attitude_quality = estimator_->quality() == control::QuaternionEkf::Quality::Tracking
                                                ? AttitudeQuality::Tracking
                                                : AttitudeQuality::Degraded;
                report(state_.publish(q));
            }
            else if (er != -EAGAIN)
                report(er);
        }
        else
            report(-ESTALE);
    }
#endif
    return error;
}
}
