#include <drivers/imu/imu_state.hpp>
#include <core/attitude.hpp>
#include <core/clock.hpp>
#include <cerrno>

namespace skywalker::imu {
int ImuState::init(std::uint32_t caps, core::OrientationReference ref) {
    if (!ref.frame_id || !ref.epoch || (caps & ~(Accel | Gyro | Orientation | Temperature)) ||
        ((caps & Accel) && !freshness_.accel_us) || ((caps & Gyro) && !freshness_.gyro_us) ||
        ((caps & Orientation) && !freshness_.orientation_us) || ((caps & Temperature) && !freshness_.temperature_us))
        return -EINVAL;
    auto key = k_spin_lock(&lock_);
    if (value_.state != State::Uninitialized) {
        k_spin_unlock(&lock_, key);
        return -EALREADY;
    }
    value_.capabilities = caps;
    value_.sample.reference = ref;
    value_.state = State::Running;
    k_spin_unlock(&lock_, key);
    return 0;
}
int ImuState::publish(const Update &input) {
    Update u = input;
    const auto mask = u.updated_mask;
    auto &s = u.sample;
    const auto now = core::monotonicTimeUs();
    auto validStamp = [now](const core::Stamp &t) { return t.valid && t.time_us <= now; };
    if (!mask || (mask & ~(Accel | Gyro | Orientation | Temperature)) ||
        ((mask & Accel) && (!validStamp(s.accel_m_s2.stamp) || !core::finite(s.accel_m_s2.value))) ||
        ((mask & Gyro) && (!validStamp(s.gyro_rad_s.stamp) || !core::finite(s.gyro_rad_s.value))) ||
        ((mask & Temperature) && (!validStamp(s.temperature_c.stamp) || !std::isfinite(s.temperature_c.value))) ||
        ((mask & Orientation) &&
         (!validStamp(s.orientation.stamp) || !core::normalize(s.orientation.value) || !s.reference.frame_id ||
          !s.reference.epoch || s.attitude_quality == AttitudeQuality::Unavailable))) {
        error(-EINVAL);
        return -EINVAL;
    }
    auto key = k_spin_lock(&lock_);
    auto &old = value_.sample;
    int ret = 0;
    auto stale = [](const auto &a, const auto &b) { return b.stamp.valid && a.stamp.time_us < b.stamp.time_us; };
    if (value_.state != State::Running)
        ret = -EACCES;
    else if (mask & ~value_.capabilities)
        ret = -ENOTSUP;
    else if (((mask & Accel) && stale(s.accel_m_s2, old.accel_m_s2)) ||
             ((mask & Gyro) && stale(s.gyro_rad_s, old.gyro_rad_s)) ||
             ((mask & Temperature) && stale(s.temperature_c, old.temperature_c)) ||
             ((mask & Orientation) && (stale(s.orientation, old.orientation) || s.reference != old.reference)))
        ret = -ESTALE;
    if (!ret) {
        auto assign = [](auto &dst, const auto &src) {
            const auto seq = dst.stamp.sequence + 1;
            dst = src;
            dst.stamp.sequence = seq;
        };
        if (mask & Accel)
            assign(old.accel_m_s2, s.accel_m_s2);
        if (mask & Gyro)
            assign(old.gyro_rad_s, s.gyro_rad_s);
        if (mask & Temperature)
            assign(old.temperature_c, s.temperature_c);
        if (mask & Orientation) {
            assign(old.orientation, s.orientation);
            old.attitude_quality = s.attitude_quality;
        }
    }
    else {
        value_.diagnostics.last_error = ret;
        ++value_.diagnostics.invalid_updates;
    }
    k_spin_unlock(&lock_, key);
    return ret;
}
Snapshot ImuState::snapshot() const {
    const auto key = k_spin_lock(&lock_);
    auto out = value_;
    k_spin_unlock(&lock_, key);
    const auto now = core::monotonicTimeUs();
    const auto &s = out.sample;
    if (out.state == State::Running) {
        if (core::fresh(s.accel_m_s2.stamp, now, freshness_.accel_us))
            out.fresh_mask |= Accel;
        if (core::fresh(s.gyro_rad_s.stamp, now, freshness_.gyro_us))
            out.fresh_mask |= Gyro;
        if (core::fresh(s.orientation.stamp, now, freshness_.orientation_us))
            out.fresh_mask |= Orientation;
        if (core::fresh(s.temperature_c.stamp, now, freshness_.temperature_us))
            out.fresh_mask |= Temperature;
    }
    return out;
}
void ImuState::error(int code, bool io) {
    auto key = k_spin_lock(&lock_);
    value_.diagnostics.last_error = code;
    if (io)
        ++value_.diagnostics.io_errors;
    else
        ++value_.diagnostics.invalid_updates;
    k_spin_unlock(&lock_, key);
}
void ImuState::transportGap() {
    auto key = k_spin_lock(&lock_);
    ++value_.diagnostics.transport_gaps;
    k_spin_unlock(&lock_, key);
}
void ImuState::setState(State s) {
    auto key = k_spin_lock(&lock_);
    value_.state = s;
    k_spin_unlock(&lock_, key);
}
void ImuState::resetOrientation(core::OrientationReference ref, AttitudeQuality quality) {
    auto key = k_spin_lock(&lock_);
    value_.sample.orientation.stamp.valid = false;
    value_.sample.reference = ref;
    value_.sample.attitude_quality = quality;
    k_spin_unlock(&lock_, key);
}
}
