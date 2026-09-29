#include <communication/vision/vision_receiver.hpp>
#include <core/clock.hpp>
#include <cerrno>
namespace skywalker::communication::vision {
BUILD_ASSERT(CONFIG_SKYWALKER_VISION_RX_PRIORITY < CONFIG_NUM_PREEMPT_PRIORITIES);
int VisionReceiver::start() {
    if (!atomic_cas(&started_, 0, 1))
        return -EALREADY;
    const int r = link_.init();
    if (r < 0) {
        atomic_clear(&started_);
        return r;
    }
    k_thread_create(&thread_, stack_, K_KERNEL_STACK_SIZEOF(stack_), entry, this, nullptr, nullptr,
                    CONFIG_SKYWALKER_VISION_RX_PRIORITY, 0, K_NO_WAIT);
    return 0;
}
VisionReceiver::Snapshot VisionReceiver::snapshot() const {
    auto k = k_spin_lock(&lock_);
    auto s = value_;
    k_spin_unlock(&lock_, k);
    s.link.aim_fresh = core::fresh(s.link.aim.stamp, core::monotonicTimeUs(), config_.link.aim_timeout_us);
    return s;
}
void VisionReceiver::entry(void *self, void *, void *) {
    static_cast<VisionReceiver *>(self)->run();
}
void VisionReceiver::run() {
    Snapshot current{};
    current.state = State::Starting;
    int r = uart_.init();
    core::TimeUs retry = 0, next_tx = 0;
    if (r < 0) {
        current.uart_error = r;
        current.state = State::InitFailed;
        retry = core::monotonicTimeUs() + 100000;
    }
    for (;;) {
        const auto now = core::monotonicTimeUs();
        if (now >= retry) {
            r = uart_.service(now / 1000);
            if (r == -EACCES)
                r = uart_.init();
            if (r == 0) {
                current.state = State::Running;
                current.uart_error = 0;
            }
            else if (r != -EAGAIN) {
                current.uart_error = r;
                current.state = State::InitFailed;
                retry = now + 100000;
            }
            if (current.state == State::Running) {
                AsyncUart::RxChunk chunk{};
                for (unsigned budget = 0; budget < 8; ++budget) {
                    const int rr = uart_.read(chunk);
                    if (rr == -EAGAIN)
                        break;
                    if (rr == -EOVERFLOW) {
                        link_.discardPartial();
                        ++current.resets;
                        continue;
                    }
                    if (rr < 0) {
                        current.uart_error = rr;
                        break;
                    }
                    ++current.rx_chunks;
                    link_.processRxBytes(chunk.bytes, chunk.size, chunk.timestamp_ms * 1000);
                }
                if (config_.feedback_period_us && now >= next_tx) {
                    next_tx = now + config_.feedback_period_us;
                    std::uint8_t bytes[256];
                    const int size = link_.encodeFeedback(bytes, sizeof(bytes));
                    int tx = size;
                    if (size > 0)
                        tx = uart_.send(bytes, static_cast<std::size_t>(size));
                    current.tx_error = tx < 0 ? tx : 0;
                    if (tx < 0)
                        ++current.tx_skipped;
                    else
                        ++current.tx_frames;
                }
            }
        }
        link_.processRxBytes(nullptr, 0, core::monotonicTimeUs());
        current.link = link_.snapshot();
        current.dropped = static_cast<std::uint32_t>(uart_.droppedChunks());
        auto key = k_spin_lock(&lock_);
        value_ = current;
        k_spin_unlock(&lock_, key);
        k_sleep(K_MSEC(1));
    }
}
}
