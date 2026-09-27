#include <cerrno>
#include <cstdint>
#include <communication/async_uart.hpp>
#include <communication/remote/remote_service.hpp>
#include <latest.hpp>
#include <lib/vofa/vofa.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_config.hpp"

LOG_MODULE_REGISTER(dr16_bench, LOG_LEVEL_INF);
using namespace skywalker;

namespace {
struct BenchSnapshot {
    robotics::RemoteState remote{};
    std::uint32_t rx_chunks = 0, resets = 0;
    unsigned dropped = 0;
    int uart_error = 0;
};

Latest<BenchSnapshot> latest;

void remoteTask(void *, void *, void *) {
    static communication::AsyncUart::DmaBuffers dma_buffers __nocache;
    static communication::AsyncUart uart(bench::remote_uart, dma_buffers);
    communication::RemoteService service(bench::decoder, bench::remote);
    BenchSnapshot snapshot{};

    const int init_ret = uart.init();
    LOG_INF("DR16 init=%d", init_ret);
    if (init_ret < 0) {
        snapshot.uart_error = init_ret;
        latest.put(snapshot);
        LOG_ERR("DR16 UART initialization failed");
        return;
    }

    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const int sr = uart.service(now);
        if (sr < 0 && sr != -EAGAIN)
            snapshot.uart_error = sr;
        else if (sr == 0)
            snapshot.uart_error = 0;

        communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget = 0; budget < 8; ++budget) {
            const int rr = uart.read(chunk);
            if (rr == -EOVERFLOW) {
                service.discardPartial();
                ++snapshot.resets;
                continue;
            }
            if (rr < 0)
                break;
            ++snapshot.rx_chunks;
            service.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
        }

        service.snapshot(now, snapshot.remote);
        snapshot.dropped = static_cast<unsigned>(uart.droppedChunks());
        latest.put(snapshot);
        k_sleep(K_MSEC(1));
    }
}

void vofaTask(void *, void *, void *) {
    static Vofa vofa{};
    const int init_ret = vofa_init(&vofa, bench::telemetry_uart);
    LOG_INF("VOFA init=%d", init_ret);
    if (init_ret < 0) {
        LOG_ERR("VOFA telemetry UART initialization failed");
        return;
    }

    BenchSnapshot snapshot{};
    std::uint32_t last_resets = 0;
    unsigned last_dropped = 0;
    int last_uart_error = 0, last_send_error = 0;
    for (;;) {
        // Keep the previous snapshot on contention, then check its original age.
        latest.get(snapshot);
        const auto &state = snapshot.remote;
        const auto &a = state.analog;
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const bool fresh = state.online && robotics::isFresh(state.stamp, now, bench::remote.offline_timeout_ms);
        const float channels[] = {
            fresh ? 1.0f : 0.0f, float(a.right_x), float(a.right_y), float(a.left_x),
            float(a.left_y), float(unsigned(state.left_switch)), float(unsigned(state.right_switch)),
            float(a.wheel), float(state.mouse.x), float(state.mouse.y), float(state.mouse.z),
            state.mouse.left ? 1.0f : 0.0f, state.mouse.right ? 1.0f : 0.0f,
            float(state.keyboard.bits), float(state.stamp.sequence), float(snapshot.rx_chunks),
        };
        constexpr auto channel_count = sizeof(channels) / sizeof(channels[0]);
        static_assert(channel_count <= VOFA_MAX_FLOATS);
        const int send_ret = vofa_send(&vofa, channels, static_cast<std::uint8_t>(channel_count));
        if (send_ret != last_send_error) {
            if (send_ret < 0)
                LOG_WRN("VOFA send failed: %d", send_ret);
            last_send_error = send_ret;
        }
        if (snapshot.uart_error != last_uart_error) {
            if (snapshot.uart_error < 0)
                LOG_ERR("DR16 UART service failed: %d", snapshot.uart_error);
            last_uart_error = snapshot.uart_error;
        }
        if (snapshot.resets != last_resets || snapshot.dropped != last_dropped) {
            LOG_WRN("DR16 RX continuity: resets=%u dropped=%u", snapshot.resets, snapshot.dropped);
            last_resets = snapshot.resets;
            last_dropped = snapshot.dropped;
        }
        k_sleep(K_MSEC(100));
    }
}
} // namespace

K_THREAD_DEFINE(remote_thread, 3072, remoteTask, nullptr, nullptr, nullptr, 6, 0, 0);
K_THREAD_DEFINE(vofa_thread, 4096, vofaTask, nullptr, nullptr, nullptr, 7, 0, 0);

int main() {
    LOG_INF("DR16 receive and VOFA telemetry threads started");
    return 0;
}
