#include <zephyr/logging/log.h>
#include <robotics/command/command_manager.hpp>
#include <robotics/command/receiver_sources.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(command_insurance, LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    static communication::AsyncUart::DmaBuffers dma __nocache;
    static communication::RemoteReceiver receiver(bench::remote_uart, dma, {bench::decoder, bench::remote});
    static RemoteSource source(receiver);
    static const CommandManager::Config config = [] {
        CommandManager::Config value{};
        value.require_referee_for_motion = false;
        value.allow_auto = false;
        return value;
    }();
    static CommandManager manager(config);
    int ret = manager.registerSource(source);
    if (ret == 0) ret = manager.start();
    if (ret < 0) { LOG_ERR("command service start: %d", ret); return ret; }
    CommandSnapshot frame{};
    std::uint64_t next_log = 0;
    LOG_INF("Motor-free insurance bench: Down disables, Middle resumes with fresh RC; no reset key");
    for (;;) {
        const auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
        if (manager.snapshot(frame) == 0 && now_ms >= next_log) {
            next_log = now_ms + 100;
            const auto &decision = frame.decision;
            LOG_INF("online=%d mode=%u chassis=%u gimbal=%u reasons=%x error=%d seq=%u",
                    frame.observed.remote.online, unsigned(decision.operator_mode), unsigned(decision.command.chassis.mode),
                    unsigned(decision.command.gimbal.mode), decision.reasons(), decision.error, decision.command.stamp.sequence);
        }
        k_sleep(K_MSEC(10));
    }
}
