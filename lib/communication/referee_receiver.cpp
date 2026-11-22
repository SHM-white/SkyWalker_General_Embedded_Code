#include <algorithm>
#include <cerrno>
#include <communication/referee/referee_receiver.hpp>
#include <core/clock.hpp>

namespace skywalker::communication {
BUILD_ASSERT(CONFIG_SKYWALKER_REFEREE_RX_PRIORITY >= 0 &&
             CONFIG_SKYWALKER_REFEREE_RX_PRIORITY < CONFIG_NUM_PREEMPT_PRIORITIES,
             "Referee receiver priority must be a valid preemptive priority");

int RefereeReceiver::setFrameHandler(RefereeParser::FrameHandler handler, void *context) {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    if (atomic_get(&started_))
        return -EBUSY;
    service_.setFrameHandler(handler, context);
    return 0;
}

int RefereeReceiver::start() {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    if (!timeout_ms_) {
        const auto key = k_spin_lock(&lock_);
        value_ = {};
        value_.state = State::InitFailed;
        value_.uart_error = -EINVAL;
        k_spin_unlock(&lock_, key);
        return -EINVAL;
    }
    if (!atomic_cas(&started_, 0, 1))
        return -EALREADY;
    {
        const auto key = k_spin_lock(&lock_);
        value_.state = State::Starting;
        value_.uart_error = 0;
        k_spin_unlock(&lock_, key);
    }
    std::uint32_t options = 0;
#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
    options |= K_FP_REGS;
#endif
    k_thread_create(&thread_, thread_stack_, K_KERNEL_STACK_SIZEOF(thread_stack_), threadEntry, this, nullptr, nullptr,
                    CONFIG_SKYWALKER_REFEREE_RX_PRIORITY, options, K_NO_WAIT);
    return 0;
}

RefereeReceiver::Snapshot RefereeReceiver::snapshot() const {
    Snapshot copy{};
    {
        const auto key = k_spin_lock(&lock_);
        copy = value_;
        k_spin_unlock(&lock_, key);
    }
    const auto now = core::monotonicTimeUs() / 1000;
    copy.referee.online = copy.referee.online && robotics::isFresh(copy.referee.stamp, now, timeout_ms_);
    return copy;
}

void RefereeReceiver::threadEntry(void *self, void *, void *) {
    static_cast<RefereeReceiver *>(self)->run();
}

void RefereeReceiver::run() {
    Snapshot current{};
    current.state = State::Starting;
    {
        const auto key = k_spin_lock(&lock_);
        value_ = current;
        k_spin_unlock(&lock_, key);
    }
    std::uint64_t retry_ms = 0;
    for (;;) {
        const auto now = core::monotonicTimeUs() / 1000;
        if (now >= retry_ms) {
            int ret = uart_.service(now);
            if (ret == -EACCES)
                ret = uart_.init();
            if (ret == 0) {
                current.state = State::Running;
                current.uart_error = 0;
                AsyncUart::RxChunk chunk{};
                for (unsigned budget = 0; budget < 8; ++budget) {
                    const int rr = uart_.read(chunk);
                    if (rr == -EOVERFLOW) {
                        service_.discardPartial();
                        ++current.resets;
                        continue;
                    }
                    if (rr == -EAGAIN)
                        break;
                    if (rr < 0) {
                        current.uart_error = rr;
                        break;
                    }
                    ++current.rx_chunks;
                    const int pr = service_.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
                    if (pr < 0)
                        current.uart_error = pr;
                }
            }
            else if (ret != -EAGAIN) {
                current.state = State::InitFailed;
                current.uart_error = ret;
                retry_ms = now + 100;
            }
        }
        const auto post_now = std::max(now, core::monotonicTimeUs() / 1000);
        (void)service_.processBytes(nullptr, 0, post_now);
        robotics::RefereeState state{};
        (void)service_.snapshot(post_now, state);
        current.referee = state;
        current.dropped = static_cast<std::uint32_t>(uart_.droppedChunks());
        {
            const auto key = k_spin_lock(&lock_);
            value_ = current;
            k_spin_unlock(&lock_, key);
        }
        k_sleep(K_MSEC(1));
    }
}
} // namespace skywalker::communication
