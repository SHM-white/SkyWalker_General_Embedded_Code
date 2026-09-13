#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <communication/async_uart.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(dr16_bench, LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    static communication::AsyncUart uart(bench::remote_uart);
    communication::RemoteService service(bench::decoder, bench::remote);
    int ret = uart.init();
    LOG_INF("DR16 physical UART init=%d; move sticks/switches, then unplug receiver", ret);
    std::uint64_t next_log = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        uart.service(now);
        communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget = 0; budget < 8; ++budget) {
            ret = uart.read(chunk);
            if (ret == -EOVERFLOW) {
                service.discardPartial();
                continue;
            }
            if (ret < 0)
                break;
            service.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
        }
        if (now >= next_log) {
            next_log = now + 100;
            robotics::RemoteState r{};
            const int status = service.snapshot(now, r);
            LOG_INF(
                "online=%d rc=%d seq=%u rx_age_ms=%llu R=(%d,%d) L=(%d,%d) wheel=%d switches=%u/%u mouse=%d/%d keys=%04x",
                r.online, status, r.stamp.sequence, r.stamp.valid ? now - r.stamp.timestamp_ms : 0, r.analog.right_x,
                r.analog.right_y, r.analog.left_x, r.analog.left_y, r.analog.wheel, unsigned(r.left_switch),
                unsigned(r.right_switch), r.mouse.x, r.mouse.y, r.keyboard.bits);
        }
        k_sleep(K_MSEC(1));
    }
}
