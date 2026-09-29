#include <drivers/imu/dm_imu_rs485.hpp>
#include <core/attitude.hpp>
#include <core/clock.hpp>
#include <cerrno>
namespace skywalker::imu {
int DmImuRs485Source::init() {
    if (initialized_)
        return -EALREADY;
    if (!core::normalize(config_.sensor_to_body) || !std::isfinite(config_.acceleration_scale) ||
        !std::isfinite(config_.angular_velocity_scale) || config_.acceleration_scale <= 0 ||
        config_.angular_velocity_scale <= 0)
        return -EINVAL;
    int r = parser_.init();
    if (r < 0)
        return r;
    r = state_.init(Accel | Gyro | Orientation, config_.reference);
    if (r < 0)
        return r;
    initialized_ = true;
    r = uart_.init();
    if (r < 0) {
        state_.error(r, true);
        state_.setState(State::Fault);
        retry_us_ = core::monotonicTimeUs() + 100000;
    }
    return r;
}
int DmImuRs485Source::service() {
    if (!initialized_)
        return -EACCES;
    const auto now = core::monotonicTimeUs();
    if (now < retry_us_)
        return -EAGAIN;
    int r = uart_.service(now / 1000);
    // -EACCES means callback registration never succeeded; no duplicate registration.
    if (r == -EACCES)
        r = uart_.init();
    if (r < 0 && r != -EAGAIN) {
        state_.error(r, true);
        state_.setState(State::Fault);
        retry_us_ = now + 100000;
        return r;
    }
    if (r == 0)
        state_.setState(State::Running);
    int updated = 0;
    communication::AsyncUart::RxChunk chunk{};
    for (unsigned budget = 0; budget < 8; ++budget) {
        const int rr = uart_.read(chunk);
        if (rr == -EAGAIN)
            break;
        if (rr == -EOVERFLOW) {
            parser_.discardPartial();
            state_.transportGap();
            continue;
        }
        if (rr < 0) {
            state_.error(rr, true);
            return rr;
        }
        const int n = parser_.consume(chunk.bytes, chunk.size, chunk.timestamp_ms * 1000, *this);
        if (n < 0)
            state_.error(n);
        else
            updated += n;
    }
    parser_.consume(nullptr, 0, core::monotonicTimeUs(), *this);
    const auto stats = parser_.statistics();
    const auto errors = stats.crc_errors + stats.invalid_frames + stats.assembly_timeouts;
    if (errors != parser_errors_) {
        state_.error(-EBADMSG);
        parser_errors_ = errors;
    }
    return updated ? 0 : -EAGAIN;
}
int DmImuRs485Source::acceptDm(const Update &input) {
    auto u = input;
    auto scaled = [](core::Vec3 v, float s) { return core::Vec3{v.x * s, v.y * s, v.z * s}; };
    if (u.updated_mask & Accel)
        u.sample.accel_m_s2.value = core::rotate(config_.sensor_to_body,
                                                 scaled(u.sample.accel_m_s2.value, config_.acceleration_scale));
    if (u.updated_mask & Gyro)
        u.sample.gyro_rad_s.value = core::rotate(config_.sensor_to_body,
                                                 scaled(u.sample.gyro_rad_s.value, config_.angular_velocity_scale));
    if (u.updated_mask & Orientation) {
        auto q = u.sample.orientation.value;
        if (config_.device_quaternion_is_world_to_sensor)
            q = core::conjugate(q);
        u.sample.orientation.value = core::multiply(q, core::conjugate(config_.sensor_to_body));
        u.sample.reference = config_.reference;
    }
    return state_.publish(u);
}
int DmImuRs485Source::resetReference() {
    if (!initialized_)
        return -EACCES;
    if (++config_.reference.epoch == 0)
        ++config_.reference.epoch;
    parser_.discardPartial();
    state_.resetOrientation(config_.reference, AttitudeQuality::Unavailable);
    return 0;
}
}
