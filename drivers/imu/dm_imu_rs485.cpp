#include <drivers/imu/dm_imu_rs485.hpp>
#include <core/attitude.hpp>
#include <core/clock.hpp>
#include <zephyr/logging/log.h>
#include <cerrno>
LOG_MODULE_REGISTER(dm_imu_rs485, LOG_LEVEL_INF);
namespace skywalker::imu {
int DmImuRs485Source::init() {
    if (initialized_)
        return -EALREADY;
    if (!core::normalize(config_.sensor_to_body) || !std::isfinite(config_.acceleration_scale) ||
        !std::isfinite(config_.angular_velocity_scale) || config_.acceleration_scale <= 0 ||
        config_.angular_velocity_scale <= 0 || !config_.response_timeout_us || config_.max_retries > 3)
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
void DmImuRs485Source::finishRequest(bool retry) {
    if (retry && retries_ < config_.max_retries)
        ++retries_;
    else {
        rid_ = rid_ == 3 ? 1 : rid_ == 1 ? 0 : 3;
        retries_ = 0;
    }
    phase_ = Phase::Idle;
    next_request_us_ = core::monotonicTimeUs() + 1000;
}
void DmImuRs485Source::logStatistics(core::TimeUs now) {
    if (now < next_log_us_)
        return;
    next_log_us_ = now + 1000000;
    const auto stats = parser_.statistics();
    LOG_INF("id=%u tx=%u rx_bytes=%u replies=%u timeouts=%u invalid=%u partial_timeouts=%u response=%u",
            config_.protocol.id, requests_, rx_bytes_, replies_, timeouts_, stats.invalid_frames,
            stats.assembly_timeouts, last_response_code_);
}
int DmImuRs485Source::service() {
    if (!initialized_)
        return -EACCES;
    const auto now = core::monotonicTimeUs();
    logStatistics(now);
    if (now < retry_us_)
        return -EAGAIN;
    int r = uart_.service(now / 1000);
    // -EACCES means callback registration never succeeded; no duplicate registration.
    if (r == -EACCES)
        r = uart_.init();
    if (r < 0 && r != -EAGAIN) {
        state_.error(r, true);
        state_.setState(State::Fault);
        if (phase_ != Phase::Idle)
            finishRequest(true);
        parser_.discardPartial();
        retry_us_ = now + 100000;
        return r;
    }
    if (r == -EAGAIN)
        return r;
    state_.setState(State::Running);
    response_error_ = 0;
    if (phase_ == Phase::Sending && !uart_.txBusy()) {
        const int tx_error = uart_.txError();
        if (tx_error < 0) {
            state_.error(tx_error, true);
            response_error_ = tx_error;
            finishRequest(true);
            parser_.discardPartial();
        }
        else {
            phase_ = Phase::AwaitReply;
            response_deadline_us_ = now + config_.response_timeout_us;
        }
    }
    // A fast reply may already be queued before TX completion is observed. Leave it
    // queued until the next call rather than parsing it while still in Sending.
    if (phase_ == Phase::Sending)
        return -EAGAIN;
    int updated = 0;
    communication::AsyncUart::RxChunk chunk{};
    for (unsigned budget = 0; budget < 8; ++budget) {
        const int rr = uart_.read(chunk);
        if (rr == -EAGAIN)
            break;
        if (rr == -EOVERFLOW) {
            parser_.discardPartial();
            state_.transportGap();
            if (phase_ != Phase::Idle)
                finishRequest(true);
            continue;
        }
        if (rr < 0) {
            state_.error(rr, true);
            return rr;
        }
        rx_bytes_ += chunk.size;
        const int n = parser_.consume(chunk.bytes, chunk.size, chunk.timestamp_ms * 1000, *this);
        if (n < 0)
            state_.error(n);
        else
            updated += n;
    }
    parser_.consume(nullptr, 0, core::monotonicTimeUs(), *this);
    const auto stats = parser_.statistics();
    const auto errors = stats.invalid_frames + stats.assembly_timeouts;
    if (errors != parser_errors_) {
        state_.error(-EBADMSG);
        parser_errors_ = errors;
    }
    const auto current = core::monotonicTimeUs();
    if (phase_ == Phase::AwaitReply && current >= response_deadline_us_) {
        ++timeouts_;
        state_.error(-ETIMEDOUT, true);
        response_error_ = -ETIMEDOUT;
        finishRequest(true);
        parser_.discardPartial();
    }
    if (phase_ == Phase::Idle && !uart_.txBusy() && current >= next_request_us_) {
        std::uint8_t frame[DmImuRs485Parser::frame_size];
        const int length = parser_.encodeRead(rid_, frame, sizeof(frame));
        if (length < 0)
            return length;
        // Remove leftovers before starting a new transaction, never in the sink callback.
        parser_.discardPartial();
        request_us_ = current;
        const int sent = uart_.send(frame, static_cast<std::size_t>(length));
        if (sent == 0) {
            phase_ = Phase::Sending;
            ++requests_;
        }
        else if (sent != -EAGAIN) {
            state_.error(sent, true);
            finishRequest(true);
            next_request_us_ = current + 100000;
            return sent;
        }
    }
    return response_error_ ? response_error_ : updated ? 0 : -EAGAIN;
}
int DmImuRs485Source::acceptDmRs485(std::uint8_t rid, std::uint8_t code, const Update &u,
                                   core::TimeUs first_rx_us, core::TimeUs last_rx_us) {
    // Queue timestamps have millisecond precision. No transaction sequence exists on the wire.
    if (phase_ != Phase::AwaitReply || rid != rid_ || first_rx_us < (request_us_ / 1000) * 1000 ||
        last_rx_us > response_deadline_us_)
        return 0;
    ++replies_;
    last_response_code_ = code;
    finishRequest(false);
    if (code != 0) {
        state_.error(-EIO);
        response_error_ = -EIO;
        return 0;
    }
    const int result = acceptDm(u);
    if (result < 0) {
        response_error_ = result;
        return result;
    }
    return 1;
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
    // Ignore an in-flight reply from the old reference session.
    phase_ = Phase::Idle;
    retries_ = 0;
    next_request_us_ = core::monotonicTimeUs() + config_.response_timeout_us;
    state_.resetOrientation(config_.reference, AttitudeQuality::Unavailable);
    return 0;
}
}
