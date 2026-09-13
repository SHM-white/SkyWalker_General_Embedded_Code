#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <communication/async_uart.hpp>
#include <communication/referee/referee_parser.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(referee_bench,LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    static communication::AsyncUart uart(bench::referee_uart); communication::RefereeParser parser(bench::version);
    int ret=uart.init(); LOG_INF("Referee physical UART init=%d profile=%d",ret,int(bench::version));
    std::uint64_t next_log=0;
    for (;;) {
        const auto now=static_cast<std::uint64_t>(k_uptime_get()); uart.service(now); communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget=0;budget<8;++budget) {
            ret=uart.read(chunk); if (ret==-EOVERFLOW) { parser.discardPartial(); continue; } if (ret<0) break;
            parser.consume(chunk.bytes,chunk.size,chunk.timestamp_ms);
        }
        parser.consume(nullptr,0,now);
        if (now>=next_log) {
            next_log=now+200; const auto &r=parser.state(); const auto &stats=parser.stats();
            LOG_INF("online=%d frames=%u crc=%u/%u unknown=%u robot=%u outputs(g/c/s)=%d/%d/%d permission_fresh=%d limit_W=%.1f buffer_J=%.1f",
                robotics::isFresh(r.stamp,now,bench::offline_timeout_ms),stats.valid_frames,stats.crc8_errors,stats.crc16_errors,stats.unknown_commands,
                r.robot.robot_id,r.robot.gimbal_output.enabled,r.robot.chassis_output.enabled,r.robot.shooter_output.enabled,
                robotics::isFresh(r.robot.chassis_output.stamp,now,bench::permission_timeout_ms),double(r.power.chassis_power_limit_w),double(r.power.buffer_energy_j));
        }
        k_sleep(K_MSEC(1));
    }
}
