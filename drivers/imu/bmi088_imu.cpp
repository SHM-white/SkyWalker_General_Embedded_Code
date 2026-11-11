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
        (config_.planar_yaw_enabled && config_.correct_gyro_bias) ||
        (config_.correct_gyro_bias && !estimator_) || !core::normalize(config_.sensor_to_body))
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
    const int ret = state_.init(Accel | Gyro | Temperature | (estimator_ ? Orientation : 0u) |
                                   (config_.planar_yaw_enabled ? PlanarYaw : 0u), config_.reference);
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
    Update accel_update{}, gyro_update{};
    if (!ar) {
        accel_update.sample.accel_m_s2 = a;
        accel_update.sample.accel_m_s2.value = core::rotate(config_.sensor_to_body, a.value);
        accel_update.updated_mask |= Accel;
    }
    if (!gr) {
        auto gyro = g.value;
        if (config_.correct_gyro_bias && estimator_->quality() != control::QuaternionEkf::Quality::Initializing) {
            const auto bias = estimator_->gyroBiasRadS();
            if (core::finite(bias))
                gyro = {gyro.x - bias.x, gyro.y - bias.y, gyro.z - bias.z};
        }
        gyro_update.sample.gyro_rad_s = g;
        gyro_update.sample.gyro_rad_s.value = core::rotate(config_.sensor_to_body, gyro);
        gyro_update.updated_mask |= Gyro;
    }
    if (config_.planar_yaw_enabled) {
        // Raw gyro is already rotated into the carrier frame. The independent
        // fixed bias never changes when the diagnostic EKF degrades or resets.
        const auto &p = planar_yaw_.update(gyro_update.sample.gyro_rad_s, accel_update.sample.accel_m_s2);
        auto &out = gyro_update.sample.planar_yaw;
        out.yaw_rad = p.yaw_rad;
        out.rate_rad_s = p.rate_rad_s;
        out.bias_rad_s = p.bias_rad_s;
        out.generation = p.generation;
        using S = control::PlanarYawIntegrator::State;
        switch (p.state) {
        case S::Calibrating: out.state = PlanarYawState::Calibrating; break;
        case S::Ready: out.state = PlanarYawState::Ready; break;
        case S::GyroInvalid: out.state = PlanarYawState::GyroInvalid; break;
        case S::SampleGap: out.state = PlanarYawState::SampleGap; break;
        case S::TimeReset: out.state = PlanarYawState::TimeReset; break;
        case S::RateInvalid: out.state = PlanarYawState::RateInvalid; break;
        }
        // Also publish invalidity immediately when a gyro read fails, without
        // renewing the stamp of the previous successful gyro measurement.
        gyro_update.updated_mask |= PlanarYaw;
    }
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
    if (accel_update.updated_mask)
        report(state_.publish(accel_update));
    if (gyro_update.updated_mask)
        report(state_.publish(gyro_update));
#ifdef CONFIG_SKYWALKER_ATTITUDE_EKF
    if (estimator_ && !ar && !gr) {
        const auto skew = a.stamp.time_us > g.stamp.time_us ? a.stamp.time_us - g.stamp.time_us
                                                            : g.stamp.time_us - a.stamp.time_us;
        if (skew <= config_.max_input_skew_us) {
            const auto generation = estimator_->generation();
            const int er = estimator_->update({a.value, g.value, std::max(a.stamp.time_us, g.stamp.time_us)});
            const auto &ed = estimator_->diagnostics();
            const EstimatorDiagnostics estimator_diagnostic{
                .time_us = std::max(a.stamp.time_us, g.stamp.time_us),
                .result = er,
                .correction = static_cast<std::uint8_t>(ed.correction),
                .gravity_error_m_s2 = ed.gravity_error_m_s2,
                .innovation_nis = ed.innovation_nis};
            if (er)
                state_.estimatorDiagnostics(estimator_diagnostic);
            if (generation != estimator_->generation()) {
                if (++config_.reference.epoch == 0)
                    ++config_.reference.epoch;
                state_.resetOrientation(config_.reference);
            }
            if (!er) {
                Update attitude{};
                attitude.updated_mask = Orientation;
                attitude.estimator_updated = true;
                attitude.estimator = estimator_diagnostic;
                attitude.sample.reference = config_.reference;
                attitude.sample.orientation = {
                    core::multiply(estimator_->attitude(), core::conjugate(config_.sensor_to_body)),
                    {std::max(a.stamp.time_us, g.stamp.time_us), 0, true}};
                attitude.sample.attitude_quality = estimator_->quality() == control::QuaternionEkf::Quality::Tracking
                                                       ? AttitudeQuality::Tracking
                                                       : AttitudeQuality::Degraded;
                report(state_.publish(attitude));
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
