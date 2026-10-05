#include <cerrno>
#include <communication/remote/remote_receiver.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(remote_receiver, LOG_LEVEL_INF);

namespace skywalker::communication {
BUILD_ASSERT(CONFIG_SKYWALKER_REMOTE_RX_PRIORITY < CONFIG_NUM_PREEMPT_PRIORITIES,
             "Remote receiver priority must be a valid preemptive priority");

int RemoteReceiver::start() {
    const auto &d = config_.decoder;
    if (d.channel_min < 0 || d.channel_max > 2047 || d.channel_min >= d.channel_center ||
        d.channel_center >= d.channel_max || d.center_deadband < 0 || config_.remote.offline_timeout_ms == 0 ||
        config_.remote.assembly_gap_ms == 0)
        return -EINVAL;
    if (!atomic_cas(&started_, 0, 1))
        return -EALREADY;
    k_thread_create(&thread_, thread_stack_, K_KERNEL_STACK_SIZEOF(thread_stack_), threadEntry, this, nullptr, nullptr,
                    CONFIG_SKYWALKER_REMOTE_RX_PRIORITY, 0, K_NO_WAIT);
    return 0;
}

int RemoteReceiver::snapshot(Snapshot &out) {
    Snapshot candidate{};
    const int ret = latest_.get(candidate);
    if (ret == 0)
        out = candidate;
    const auto now = static_cast<std::uint64_t>(k_uptime_get());
    out.remote.online = out.remote.online &&
                        robotics::isFresh(out.remote.stamp, now, config_.remote.offline_timeout_ms);
    return ret;
}

void RemoteReceiver::threadEntry(void *self, void *, void *) {
    static_cast<RemoteReceiver *>(self)->run();
}

void RemoteReceiver::run() {
    Snapshot current{};
    current.state = State::Starting;
    latest_.put(current);
    std::uint64_t retry_ms = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        if (now >= retry_ms) {
            int ret = uart_.service(now);
            if (ret == -EACCES)
                ret = uart_.init();
            if (ret == 0) {
                current.state = State::Running;
                current.uart_error = 0;
            }
            else if (ret != -EAGAIN) {
                current.state = State::InitFailed;
                current.uart_error = ret;
                retry_ms = now + 100;
            }
            AsyncUart::RxChunk chunk{};
            for (unsigned budget = 0; budget < 8 && ret == 0; ++budget) {
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
                service_.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
            }
        }
        robotics::RemoteState remote{};
        service_.snapshot(static_cast<std::uint64_t>(k_uptime_get()), remote);
        current.remote = remote;
        current.dropped = static_cast<unsigned>(uart_.droppedChunks());
        latest_.put(current);
        k_sleep(K_MSEC(1));
    }
}
} // namespace skywalker::communication
