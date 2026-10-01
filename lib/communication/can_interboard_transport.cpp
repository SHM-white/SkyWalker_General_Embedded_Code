#include <communication/interboard/can_interboard_transport.hpp>
#include <communication/interboard/interboard_protocol.hpp>
#include <communication/wire.hpp>
#include <zephyr/random/random.h>
#include <algorithm>
#include <cerrno>
#include <cstring>

namespace skywalker::communication {
int CanInterBoardTransport::initialize() {
    if (!config_.can || !device_is_ready(config_.can)) return -ENODEV;
    const auto mask = config_.extended_id ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK;
    if (config_.tx_id > mask || config_.rx_id > mask || config_.tx_id == config_.rx_id ||
        !config_.reassembly_timeout_ms || config_.reassembly_timeout_ms > 1000 ||
        !config_.recovery_retry_ms || config_.recovery_retry_ms > 10000) return -EINVAL;
    can_state state{};
    int ret = can_get_state(config_.can, &state, nullptr);
    if (ret < 0) return ret;
    if (state != CAN_STATE_STOPPED) return -EBUSY;
    ret = can_set_mode(config_.can, CAN_MODE_NORMAL);
    if (ret < 0) return ret;
    can_filter filter{};
    filter.id = config_.rx_id;
    filter.mask = mask;
    filter.flags = config_.extended_id ? CAN_FILTER_IDE : 0;
    filter_ = can_add_rx_filter(config_.can, onRx, this, &filter);
    if (filter_ < 0) return filter_;
    ret = can_start(config_.can);
    if (ret < 0) {
        can_remove_rx_filter(config_.can, filter_);
        filter_ = -1;
        return ret == -EALREADY ? -EBUSY : ret;
    }
    tx_token_ = static_cast<std::uint8_t>(sys_rand32_get());
    initialized_ = true;
    atomic_set(&accept_rx_, 1);
    return 0;
}
void CanInterBoardTransport::onRx(const device *, can_frame *frame, void *context) {
    auto &self = *static_cast<CanInterBoardTransport *>(context);
    if (!atomic_get(&self.accept_rx_)) return;
    const auto key = k_spin_lock(&self.rx_lock_);
    if (!atomic_get(&self.accept_rx_)) { k_spin_unlock(&self.rx_lock_, key); return; }
    if (self.raw_count_ == kRawDepth) {
        self.raw_head_ = self.raw_count_ = 0;
        self.raw_gap_ = true;
    }
    auto &event = self.raw_[(self.raw_head_ + self.raw_count_) % kRawDepth];
    event.frame = *frame;
    event.timestamp_ms = k_uptime_get();
    ++self.raw_count_;
    k_spin_unlock(&self.rx_lock_, key);
}
void CanInterBoardTransport::onTx(const device *, int error, void *context) {
    auto &self = *static_cast<CanInterBoardTransport *>(context);
    atomic_set(&self.tx_error_, error);
    atomic_set(&self.tx_done_, 1);
}
void CanInterBoardTransport::resetRx() {
    const auto key = k_spin_lock(&rx_lock_);
    raw_head_ = raw_count_ = 0;
    raw_gap_ = false;
    k_spin_unlock(&rx_lock_, key);
    rx_size_ = rx_expected_ = 0;
    rx_.gap();
}
int CanInterBoardTransport::fail(int error, std::uint64_t now) {
    atomic_clear(&accept_rx_);
    recovering_ = true;
    stopped_ = false;
    retry_ms_ = now + config_.recovery_retry_ms;
    tx_size_ = tx_offset_ = 0;
    resetRx();
    return error;
}
int CanInterBoardTransport::recover(std::uint64_t now) {
    if (!stopped_) {
        const int ret = can_stop(config_.can);
        if (ret < 0 && ret != -EALREADY) return ret;
        stopped_ = true;
    }
    // Do not reuse a callback context until its canceled operation has retired.
    if (in_flight_) {
        if (!atomic_get(&tx_done_)) return -EAGAIN;
        in_flight_ = false;
        atomic_clear(&tx_done_);
    }
    if (now < retry_ms_) return -EAGAIN;
    retry_ms_ = now + config_.recovery_retry_ms;
    const int ret = can_start(config_.can);
    if (ret < 0) return ret;
    resetRx();
    recovering_ = stopped_ = false;
    atomic_set(&accept_rx_, 1);
    return 0;
}
void CanInterBoardTransport::consume(const RxEvent &event) {
    const auto &f = event.frame;
    const auto bad = [&]() { rx_size_ = rx_expected_ = 0; rx_.gap(); };
    if (f.id != config_.rx_id || f.flags != (config_.extended_id ? CAN_FRAME_IDE : 0) ||
        f.dlc < 3 || f.dlc > 8) { bad(); return; }
    const bool first = (f.data[0] & 0x80) != 0, last = (f.data[0] & 0x40) != 0;
    const auto index = f.data[0] & 0x3f;
    const std::size_t count = f.dlc - 2;
    if (first) {
        if (rx_expected_) bad();
        if (index != 0 || count != 6 || f.data[2] != 1) { bad(); return; }
        const auto size = wire::loadLe16(f.data + 3);
        if (!size || size > kTxCapacity) { bad(); return; }
        rx_expected_ = size + 5;
        rx_size_ = 0;
        rx_token_ = f.data[1];
        rx_index_ = 0;
        rx_started_ms_ = event.timestamp_ms;
    }
    if (!rx_expected_) return;
    if (f.data[1] != rx_token_ || index != rx_index_ || event.timestamp_ms < rx_started_ms_ ||
        event.timestamp_ms - rx_started_ms_ >= config_.reassembly_timeout_ms ||
        count != std::min<std::size_t>(6, rx_expected_ - rx_size_) ||
        last != (rx_size_ + count == rx_expected_)) { bad(); return; }
    std::memcpy(rx_packet_ + rx_size_, f.data + 2, count);
    rx_size_ += count;
    ++rx_index_;
    if (last) {
        const auto total = rx_expected_;
        if (interboardCrc16(rx_packet_, total - 2) != wire::loadLe16(rx_packet_ + total - 2)) {
            bad(); return;
        }
        rx_.push(rx_packet_ + 3, total - 5, rx_started_ms_);
        rx_size_ = rx_expected_ = 0;
    }
}
int CanInterBoardTransport::service(std::uint64_t now) {
    if (!initialized_) {
        if (now < retry_ms_) return -EAGAIN;
        retry_ms_ = now + 100;
        const int ret = initialize();
        if (ret < 0) return ret;
    }
    if (recovering_) return recover(now);
    if (in_flight_ && atomic_get(&tx_done_)) {
        const int ret = static_cast<int>(atomic_get(&tx_error_));
        atomic_clear(&tx_done_);
        in_flight_ = false;
        if (ret < 0) return fail(ret, now);
        tx_offset_ += flight_size_;
        ++tx_index_;
        if (tx_offset_ == tx_size_) tx_size_ = tx_offset_ = 0;
    }
    if (tx_size_ && now >= tx_deadline_) return fail(-ETIMEDOUT, now);
    can_state state{};
    int ret = can_get_state(config_.can, &state, nullptr);
    if (ret < 0) return fail(ret, now);
    if (state == CAN_STATE_BUS_OFF || state == CAN_STATE_STOPPED) return fail(-ENETDOWN, now);
    for (unsigned budget = 0; budget < kRawDepth; ++budget) {
        RxEvent event{};
        const auto key = k_spin_lock(&rx_lock_);
        const bool gap = raw_gap_, have = raw_count_ != 0;
        raw_gap_ = false;
        if (have) {
            event = raw_[raw_head_];
            raw_head_ = (raw_head_ + 1) % kRawDepth;
            --raw_count_;
        }
        k_spin_unlock(&rx_lock_, key);
        if (gap) { rx_size_ = rx_expected_ = 0; rx_.gap(); }
        if (!have) break;
        consume(event);
    }
    if (rx_expected_ && now >= rx_started_ms_ && now - rx_started_ms_ >= config_.reassembly_timeout_ms) {
        rx_size_ = rx_expected_ = 0;
        rx_.gap();
    }
    if (tx_size_ && !in_flight_) {
        flight_size_ = std::min<std::size_t>(6, tx_size_ - tx_offset_);
        tx_frame_ = {};
        tx_frame_.id = config_.tx_id;
        tx_frame_.flags = config_.extended_id ? CAN_FRAME_IDE : 0;
        tx_frame_.dlc = flight_size_ + 2;
        tx_frame_.data[0] = tx_index_ | (tx_offset_ == 0 ? 0x80 : 0) |
                            (tx_offset_ + flight_size_ == tx_size_ ? 0x40 : 0);
        tx_frame_.data[1] = tx_token_;
        std::memcpy(tx_frame_.data + 2, tx_packet_ + tx_offset_, flight_size_);
        atomic_clear(&tx_done_);
        in_flight_ = true;
        ret = can_send(config_.can, &tx_frame_, K_NO_WAIT, onTx, this);
        if (ret < 0) {
            in_flight_ = false;
            if (ret != -EAGAIN) return fail(ret, now);
        }
    }
    return 0;
}
int CanInterBoardTransport::send(const std::uint8_t *p, std::size_t n, std::uint32_t timeout) {
    if (!p || !n || !timeout || timeout > 1000) return -EINVAL;
    if (n > kTxCapacity) return -EMSGSIZE;
    if (!initialized_ || recovering_) return -EACCES;
    if (txBusy()) return -EAGAIN;
    tx_packet_[0] = 1;
    wire::storeLe16(tx_packet_ + 1, n);
    std::memcpy(tx_packet_ + 3, p, n);
    wire::storeLe16(tx_packet_ + n + 3, interboardCrc16(tx_packet_, n + 3));
    ++tx_token_;
    tx_index_ = 0;
    tx_offset_ = 0;
    tx_size_ = n + 5;
    tx_deadline_ = static_cast<std::uint64_t>(k_uptime_get()) + timeout;
    return 0;
}
}
