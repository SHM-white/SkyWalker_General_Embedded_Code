#include <drivers/imu/dm_imu_can.hpp>
#include <core/attitude.hpp>
#include <core/byte_codec.hpp>
#include <core/clock.hpp>
#include <zephyr/logging/log.h>
#include <cerrno>

LOG_MODULE_REGISTER(dm_imu_can, LOG_LEVEL_INF);

namespace skywalker::imu {
int DmImuCanSource::init() {
    if (initialized_)
        return -EALREADY;
    if (!config_.can || !device_is_ready(config_.can))
        return -ENODEV;
    if (config_.feedback_id > CAN_STD_ID_MASK || config_.slave_id > CAN_STD_ID_MASK ||
        config_.feedback_id == config_.slave_id ||
        (config_.mode != Mode::ActiveReport && config_.mode != Mode::RequestReply) ||
        (config_.attitude != Attitude::Quaternion && config_.attitude != Attitude::Euler) ||
        !core::normalize(config_.sensor_to_body) || !config_.request_interval_us ||
        !config_.response_timeout_us || !config_.recovery_retry_us || config_.max_retries > 3)
        return -EINVAL;
    const int ret = state_.init(Accel | Gyro | Orientation | Temperature, config_.reference);
    if (ret < 0)
        return ret;
    initialized_ = true;
    rid_ = static_cast<std::uint8_t>(config_.attitude);
    const int started = startController();
    if (started < 0) {
        state_.error(started, true);
        state_.setState(State::Fault);
        retry_us_ = core::monotonicTimeUs() + config_.recovery_retry_us;
    }
    return started;
}

int DmImuCanSource::startController() {
    can_state bus_state{};
    int ret = can_get_state(config_.can, &bus_state, nullptr);
    if (ret < 0)
        return ret;
    if (bus_state != CAN_STATE_STOPPED)
        return -EBUSY;
    ret = can_set_mode(config_.can, CAN_MODE_NORMAL);
    if (ret < 0)
        return ret;
    can_filter filter{};
    filter.id = config_.feedback_id;
    filter.mask = CAN_STD_ID_MASK;
    filter_ = can_add_rx_filter(config_.can, onRx, this, &filter);
    if (filter_ < 0)
        return filter_;
    clearRx();
    atomic_set(&accept_rx_, 1);
    ret = can_start(config_.can);
    if (ret < 0) {
        atomic_clear(&accept_rx_);
        can_remove_rx_filter(config_.can, filter_);
        filter_ = -1;
        return ret == -EALREADY ? -EBUSY : ret;
    }
    owns_controller_ = true;
    state_.setState(State::Running);
    return 0;
}

void DmImuCanSource::onRx(const device *, can_frame *frame, void *context) {
    auto &self = *static_cast<DmImuCanSource *>(context);
    if (!atomic_get(&self.accept_rx_))
        return;
    const auto key = k_spin_lock(&self.rx_lock_);
    if (atomic_get(&self.accept_rx_)) {
        if (self.rx_count_ == rx_depth) {
            self.rx_head_ = self.rx_count_ = 0;
            self.rx_gap_ = true;
        }
        auto &event = self.rx_[(self.rx_head_ + self.rx_count_) % rx_depth];
        event.frame = *frame;
        event.received_us = core::monotonicTimeUs();
        ++self.rx_count_;
    }
    k_spin_unlock(&self.rx_lock_, key);
}

void DmImuCanSource::onTx(const device *, int error, void *context) {
    auto &self = *static_cast<DmImuCanSource *>(context);
    self.tx_finished_us_ = core::monotonicTimeUs();
    atomic_set(&self.tx_error_, error);
    atomic_set(&self.tx_done_, 1);
}

void DmImuCanSource::clearRx() {
    const auto key = k_spin_lock(&rx_lock_);
    rx_head_ = rx_count_ = 0;
    rx_gap_ = false;
    k_spin_unlock(&rx_lock_, key);
}

int DmImuCanSource::fail(int error, core::TimeUs now) {
    atomic_clear(&accept_rx_);
    state_.error(error, true);
    state_.setState(State::Fault);
    state_.transportGap();
    clearRx();
    awaiting_reply_ = false;
    recovering_ = true;
    stopped_ = false;
    retry_us_ = now + config_.recovery_retry_us;
    return error;
}

int DmImuCanSource::recover(core::TimeUs now) {
    if (!stopped_) {
        const int ret = can_stop(config_.can);
        if (ret < 0 && ret != -EALREADY)
            return ret;
        stopped_ = true;
    }
    // The canceled TX must retire before its frame/context can be reused.
    if (in_flight_) {
        if (!atomic_get(&tx_done_))
            return -EAGAIN;
        in_flight_ = false;
        atomic_clear(&tx_done_);
    }
    if (now < retry_us_)
        return -EAGAIN;
    retry_us_ = now + config_.recovery_retry_us;
    clearRx();
    atomic_set(&accept_rx_, 1);
    const int ret = can_start(config_.can);
    if (ret < 0) {
        atomic_clear(&accept_rx_);
        state_.error(ret, true);
        return ret;
    }
    recovering_ = stopped_ = false;
    retries_ = 0;
    rid_ = static_cast<std::uint8_t>(config_.attitude);
    next_request_us_ = now + config_.request_interval_us;
    state_.setState(State::Running);
    return 0;
}

void DmImuCanSource::finishRequest(bool retry, core::TimeUs now) {
    awaiting_reply_ = false;
    if (retry && retries_ < config_.max_retries)
        ++retries_;
    else {
        rid_ = rid_ == 1 ? static_cast<std::uint8_t>(config_.attitude) : rid_ == 2 ? 1 : 2;
        retries_ = 0;
    }
    next_request_us_ = now + config_.request_interval_us;
}

int DmImuCanSource::consume(const RxEvent &event) {
    const auto &f = event.frame;
    ++frames_;
    if (f.id != config_.feedback_id || f.flags != 0 || f.dlc != 8 || f.data[0] < 1 || f.data[0] > 4) {
        ++invalid_;
        state_.error(-EBADMSG);
        return -EBADMSG;
    }
    if (event.received_us <= accept_after_us_)
        return 0;
    const auto type = f.data[0];
    if (config_.mode == Mode::RequestReply &&
        (!awaiting_reply_ || type != rid_ || event.received_us < request_us_ || event.received_us > deadline_us_))
        return 0;

    const auto *p = f.data;
    const core::Stamp stamp{event.received_us, 0, true};
    Update u{};
    // V1.3 appendix C: 16-bit vectors are unsigned little endian, in SI units.
    const auto vector = [p](float span, float offset) {
        return core::Vec3{core::wire::u16(p + 2) * (span / 65535.0f) + offset,
                          core::wire::u16(p + 4) * (span / 65535.0f) + offset,
                          core::wire::u16(p + 6) * (span / 65535.0f) + offset};
    };
    if (type == 1) {
        u.updated_mask = Accel | Temperature;
        u.sample.accel_m_s2 = {core::rotate(config_.sensor_to_body, vector(470.4f, -235.2f)), stamp};
        u.sample.temperature_c = {static_cast<float>(p[1]), stamp};
    }
    else if (type == 2) {
        u.updated_mask = Gyro;
        u.sample.gyro_rad_s = {core::rotate(config_.sensor_to_body, vector(69.76f, -34.88f)), stamp};
    }
    else if (type == static_cast<std::uint8_t>(config_.attitude)) {
        core::Quaternion q{};
        if (type == 4) {
            // Four consecutive 14-bit fields, W first, most significant bits first.
            // W uses ALL six upper bits of DATA[2], including bit 2.
            const auto decode = [](unsigned raw) { return raw * (2.0f / 16383.0f) - 1.0f; };
            q = {decode((unsigned(p[1]) << 6) | (p[2] >> 2)),
                 decode((unsigned(p[2] & 0x03) << 12) | (unsigned(p[3]) << 4) | (p[4] >> 4)),
                 decode((unsigned(p[4] & 0x0f) << 10) | (unsigned(p[5]) << 2) | (p[6] >> 6)),
                 decode((unsigned(p[6] & 0x3f) << 8) | p[7])};
        }
        else {
            constexpr float radians = 0.017453292519943295f;
            const float pitch = core::wire::u16(p + 2) * (180.0f / 65535.0f) - 90.0f;
            const float yaw = core::wire::u16(p + 4) * (360.0f / 65535.0f) - 180.0f;
            const float roll = core::wire::u16(p + 6) * (360.0f / 65535.0f) - 180.0f;
            q = core::fromEuler({roll * radians, pitch * radians, yaw * radians});
        }
        if (!core::normalize(q)) {
            ++invalid_;
            state_.error(-EBADMSG);
            return -EBADMSG;
        }
        if (config_.device_quaternion_is_world_to_sensor)
            q = core::conjugate(q);
        u.updated_mask = Orientation;
        u.sample.orientation = {core::multiply(q, core::conjugate(config_.sensor_to_body)), stamp};
        u.sample.reference = config_.reference;
        u.sample.attitude_quality = AttitudeQuality::Unknown;
    }
    if (!u.updated_mask)
        return 0; // Never mix Euler and quaternion attitude streams.
    const int ret = state_.publish(u);
    if (ret < 0) {
        ++invalid_;
        return ret;
    }
    ++updates_;
    if (config_.mode == Mode::RequestReply) {
        ++replies_;
        finishRequest(false, core::monotonicTimeUs());
    }
    return 1;
}

void DmImuCanSource::logStatistics(core::TimeUs now) {
    if (now < next_log_us_)
        return;
    next_log_us_ = now + 1000000;
    LOG_INF("mode=%s slave=%x feedback=%x rx=%u updates=%u tx=%u replies=%u timeouts=%u invalid=%u",
            config_.mode == Mode::ActiveReport ? "active" : "request", config_.slave_id, config_.feedback_id,
            frames_, updates_, requests_, replies_, timeouts_, invalid_);
}

int DmImuCanSource::service() {
    if (!initialized_)
        return -EACCES;
    const auto now = core::monotonicTimeUs();
    logStatistics(now);
    if (!owns_controller_) {
        if (now < retry_us_)
            return -EAGAIN;
        retry_us_ = now + config_.recovery_retry_us;
        const int ret = startController();
        if (ret < 0)
            state_.error(ret, true);
        return ret;
    }
    if (recovering_)
        return recover(now);
    can_state bus_state{};
    const int bus_ret = can_get_state(config_.can, &bus_state, nullptr);
    if (bus_ret < 0)
        return fail(bus_ret, now);
    if (bus_state == CAN_STATE_BUS_OFF || bus_state == CAN_STATE_STOPPED)
        return fail(-ENETDOWN, now);
    if (in_flight_) {
        if (!atomic_get(&tx_done_)) {
            if (now >= deadline_us_) {
                ++timeouts_;
                return fail(-ETIMEDOUT, now);
            }
            return -EAGAIN;
        }
        const int error = static_cast<int>(atomic_get(&tx_error_));
        in_flight_ = false;
        atomic_clear(&tx_done_);
        if (error < 0)
            return fail(error, now);
        awaiting_reply_ = true;
        deadline_us_ = tx_finished_us_ + config_.response_timeout_us;
    }

    int updated = 0, error = 0;
    for (std::size_t budget = 0; budget < rx_depth; ++budget) {
        RxEvent event{};
        const auto key = k_spin_lock(&rx_lock_);
        const bool gap = rx_gap_, have = rx_count_ != 0;
        rx_gap_ = false;
        if (have) {
            event = rx_[rx_head_];
            rx_head_ = (rx_head_ + 1) % rx_depth;
            --rx_count_;
        }
        k_spin_unlock(&rx_lock_, key);
        if (gap) {
            state_.transportGap();
            if (awaiting_reply_)
                finishRequest(true, now);
        }
        if (!have)
            break;
        const int ret = consume(event);
        if (ret < 0)
            error = ret;
        else
            updated += ret;
    }
    const auto current = core::monotonicTimeUs();
    if (awaiting_reply_ && current >= deadline_us_) {
        ++timeouts_;
        state_.error(-ETIMEDOUT, true);
        error = -ETIMEDOUT;
        finishRequest(true, current);
    }
    if (config_.mode == Mode::RequestReply && !awaiting_reply_ && current >= next_request_us_) {
        // Vendor MC02 example: slave ID, classic DLC 8, CC RID READ DD + uint32(0).
        tx_frame_ = {};
        tx_frame_.id = config_.slave_id;
        tx_frame_.dlc = 8;
        tx_frame_.data[0] = 0xcc;
        tx_frame_.data[1] = rid_;
        tx_frame_.data[3] = 0xdd;
        clearRx();
        request_us_ = current;
        deadline_us_ = current + config_.response_timeout_us;
        atomic_clear(&tx_done_);
        in_flight_ = true;
        const int ret = can_send(config_.can, &tx_frame_, K_NO_WAIT, onTx, this);
        if (ret < 0) {
            in_flight_ = false;
            if (ret != -EAGAIN)
                return fail(ret, current);
            next_request_us_ = current + config_.request_interval_us;
        }
        else
            ++requests_;
    }
    return error ? error : updated ? 0 : -EAGAIN;
}

int DmImuCanSource::resetReference() {
    if (!initialized_)
        return -EACCES;
    if (in_flight_ || awaiting_reply_)
        return -EBUSY;
    if (++config_.reference.epoch == 0)
        ++config_.reference.epoch;
    clearRx();
    accept_after_us_ = core::monotonicTimeUs() + config_.response_timeout_us;
    next_request_us_ = accept_after_us_ + config_.request_interval_us;
    retries_ = 0;
    state_.resetOrientation(config_.reference, AttitudeQuality::Unavailable);
    return 0;
}
}
