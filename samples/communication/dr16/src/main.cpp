#include <cerrno>
#include <cstring>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <communication/async_uart.hpp>
#include <communication/wire.hpp>
#include "board_config.hpp"

LOG_MODULE_REGISTER(dr16_bench, LOG_LEVEL_INF);

int main() {
    using namespace skywalker;
    static communication::AsyncUart::DmaBuffers dma_buffers __nocache;
    static communication::AsyncUart uart(bench::remote_uart, dma_buffers);
    communication::Dr16Decoder decoder(bench::decoder);
    robotics::RemoteState state{};
    std::uint8_t pending[18]{}, last_frame[18]{}, last_chunk[64]{};
    std::size_t used = 0, last_chunk_size = 0;
    std::uint32_t rx_bytes = 0, rx_chunks = 0, discontinuities = 0;
    std::uint64_t last_bytes_ms = 0, next_log = 0;
    int service_error = 0;
    const int init_ret = uart.init();
    LOG_INF("DR16 init=%d; console raw channel monitor", init_ret);
    if (init_ret < 0) {
        LOG_ERR("UART initialization failed; fix the reported device/DMA/buffer error before decoding");
        return init_ret;
    }

    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const int sr = uart.service(now);
        if (sr < 0 && sr != -EAGAIN)
            service_error = sr;
        else if (sr == 0)
            service_error = 0;

        communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget = 0; budget < 8; ++budget) {
            const int rr = uart.read(chunk);
            if (rr == -EOVERFLOW) {
                used = 0;
                ++discontinuities;
                continue;
            }
            if (rr < 0)
                break;
            rx_bytes += chunk.size;
            ++rx_chunks;
            last_chunk_size = chunk.size;
            std::memcpy(last_chunk, chunk.bytes, last_chunk_size);
            if (used && (chunk.timestamp_ms < last_bytes_ms ||
                         chunk.timestamp_ms - last_bytes_ms > bench::remote.assembly_gap_ms))
                used = 0;
            last_bytes_ms = chunk.timestamp_ms;
            for (std::size_t i = 0; i < chunk.size; ++i) {
                pending[used++] = chunk.bytes[i];
                if (used != sizeof(pending))
                    continue;
                if (decoder.decodeFrame(pending, sizeof(pending), chunk.timestamp_ms, state) == 0) {
                    std::memcpy(last_frame, pending, sizeof(last_frame));
                    used = 0;
                }
                else {
                    --used;
                    std::memmove(pending, pending + 1, used);
                }
            }
        }

        if (now >= next_log) {
            next_log = now + 100;
            const bool fresh = state.stamp.valid && now >= state.stamp.timestamp_ms &&
                               now - state.stamp.timestamp_ms <= bench::remote.offline_timeout_ms;
            LOG_INF("online=%d bytes=%u chunks=%u valid=%u rejected_windows=%u gaps=%u dropped=%u service=%d", fresh,
                    rx_bytes, rx_chunks, decoder.validFrameCount(), decoder.invalidFrameCount(), discontinuities,
                    static_cast<unsigned>(uart.droppedChunks()), service_error);
            if (state.stamp.valid) {
                // No deadband in this sample: adding center recovers raw CH0..CH3.
                const auto &a = state.analog;
                LOG_INF("CH0=%d CH1=%d CH2=%d CH3=%d d0=%d d1=%d d2=%d d3=%d",
                        a.right_x + bench::decoder.channel_center, a.right_y + bench::decoder.channel_center,
                        a.left_x + bench::decoder.channel_center, a.left_y + bench::decoder.channel_center, a.right_x,
                        a.right_y, a.left_x, a.left_y);
                LOG_INF("SW_HIGH=%u SW_LOW=%u TAIL16=%u mouse=%d/%d/%d buttons=%u/%u keys=%04x",
                        unsigned((last_frame[5] >> 6) & 3), unsigned((last_frame[5] >> 4) & 3),
                        unsigned(communication::wire::loadLe16(last_frame + 16)), state.mouse.x, state.mouse.y,
                        state.mouse.z, unsigned(state.mouse.left), unsigned(state.mouse.right),
                        unsigned(state.keyboard.bits));
            }
            if (!fresh && last_chunk_size) {
                LOG_HEXDUMP_INF(last_chunk, last_chunk_size, "last RX chunk (not necessarily one frame)");
            }
        }
        k_sleep(K_MSEC(1));
    }
}
