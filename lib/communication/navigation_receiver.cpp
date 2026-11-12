#include <communication/navigation/navigation_receiver.hpp>
#include <core/clock.hpp>
#include <cerrno>
#include <cstring>

namespace skywalker::communication::navigation {
BUILD_ASSERT(CONFIG_SKYWALKER_NAVIGATION_RX_PRIORITY < CONFIG_NUM_PREEMPT_PRIORITIES);
int NavigationReceiver::start() {
    if (!atomic_cas(&started_, 0, 1))
        return -EALREADY;
    k_thread_create(&thread_, stack_, K_KERNEL_STACK_SIZEOF(stack_), entry, this, nullptr, nullptr,
                    CONFIG_SKYWALKER_NAVIGATION_RX_PRIORITY, 0, K_NO_WAIT);
    return 0;
}
NavigationReceiver::Snapshot NavigationReceiver::snapshot() const {
    const auto key = k_spin_lock(&lock_);
    const auto copy = value_;
    k_spin_unlock(&lock_, key);
    return copy;
}
int NavigationReceiver::forwardRefereeFrame(const std::uint8_t *bytes, std::size_t size) {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    if (!bytes || !size || size > sizeof(ForwardFrame::bytes))
        return -EINVAL;
    if (!atomic_get(&started_))
        return -EACCES;
    ForwardFrame frame{};
    std::memcpy(frame.bytes, bytes, size);
    frame.size = static_cast<std::uint16_t>(size);
    frame.queued_us = core::monotonicTimeUs();
    if (k_msgq_put(&forward_queue_, &frame, K_NO_WAIT) != 0) {
        atomic_inc(&forward_dropped_);
        return -ENOSPC;
    }
    return 0;
}
void NavigationReceiver::entry(void *self, void *, void *) {
    static_cast<NavigationReceiver *>(self)->run();
}
void NavigationReceiver::run() {
    Snapshot current{};
    current.state = State::Starting;
    int error = uart_.init();
    core::TimeUs retry_us = 0;
    bool tx_pending = false;
    if (error < 0) {
        current.uart_error = error;
        current.state = State::InitFailed;
        retry_us = core::monotonicTimeUs() + 100000;
    }
    for (;;) {
        const auto now = core::monotonicTimeUs();
        if (tx_pending && !uart_.txBusy()) {
            tx_pending = false;
            current.tx_error = uart_.txError();
            if (current.tx_error < 0)
                ++current.forward_errors;
            else
                ++current.forwarded_frames;
        }
        if (now >= retry_us) {
            error = uart_.service(now / 1000);
            if (error == -EACCES)
                error = uart_.init();
            if (error == 0) {
                current.state = State::Running;
                current.uart_error = 0;
            }
            else if (error != -EAGAIN) {
                current.state = State::InitFailed;
                current.uart_error = error;
                protocol_.invalidate();
                ++current.resets;
                retry_us = now + 100000;
            }
            if (current.state == State::Running) {
                AsyncUart::RxChunk chunk{};
                for (unsigned budget = 0; budget < 8; ++budget) {
                    const int result = uart_.read(chunk);
                    if (result == -EAGAIN)
                        break;
                    if (result < 0) {
                        protocol_.invalidate();
                        ++current.resets;
                        if (result == -EOVERFLOW)
                            continue;
                        current.uart_error = result;
                        break;
                    }
                    ++current.rx_chunks;
                    protocol_.process(chunk.bytes, chunk.size, chunk.timestamp_ms * 1000);
                }
                // Drain RX first; discard old queued frames after a link outage.
                if (!tx_pending && !uart_.txBusy()) {
                    ForwardFrame outgoing{};
                    for (unsigned budget = 0; budget < 8; ++budget) {
                        if (k_msgq_get(&forward_queue_, &outgoing, K_NO_WAIT) != 0)
                            break;
                        const auto tx_now = core::monotonicTimeUs();
                        if (tx_now < outgoing.queued_us || tx_now - outgoing.queued_us > 300000) {
                            atomic_inc(&forward_dropped_);
                            continue;
                        }
                        current.tx_error = uart_.send(outgoing.bytes, outgoing.size, 50);
                        if (current.tx_error < 0)
                            ++current.forward_errors;
                        else
                            tx_pending = true;
                        break;
                    }
                }
            }
        }
        protocol_.process(nullptr, 0, core::monotonicTimeUs());
        current.protocol = protocol_.snapshot();
        current.dropped = static_cast<std::uint32_t>(uart_.droppedChunks());
        current.forward_dropped = static_cast<std::uint32_t>(atomic_get(&forward_dropped_));
        const auto key = k_spin_lock(&lock_);
        value_ = current;
        k_spin_unlock(&lock_, key);
        k_sleep(K_MSEC(1));
    }
}
} // namespace skywalker::communication::navigation
