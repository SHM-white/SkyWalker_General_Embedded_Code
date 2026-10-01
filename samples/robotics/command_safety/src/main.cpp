#include <core/clock.hpp>
#include <zephyr/logging/log.h>
#include <communication/remote/remote_receiver.hpp>
#include <robotics/command/command_manager.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(command_insurance, LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    static communication::AsyncUart::DmaBuffers dma __nocache;
    static communication::RemoteReceiver receiver(bench::remote_uart, dma, {bench::decoder, bench::remote});
    CommandManager::Config config{};
    config.require_referee_for_motion = false;
    config.allow_auto = false;
    CommandManager manager(config);
    const int ret = receiver.start();
    if (ret < 0) { LOG_ERR("remote start: %d", ret); return ret; }
    communication::RemoteReceiver::Snapshot remote{};
    std::uint64_t next_log = 0;
    LOG_INF("Motor-free insurance bench: Down disables, Middle resumes with fresh RC; no reset key");
    for (;;) {
        receiver.snapshot(remote);
        CommandInputs input{};
        input.remote = remote.remote;
        input.now_us = core::monotonicTimeUs();
        const auto decision = manager.update(input);
        if (input.now_us / 1000 >= next_log) {
            next_log = input.now_us / 1000 + 100;
            LOG_INF("online=%d mode=%u chassis=%u gimbal=%u reasons=%x error=%d seq=%u",
                    input.remote.online, unsigned(decision.operator_mode), unsigned(decision.command.chassis.mode),
                    unsigned(decision.command.gimbal.mode), decision.reasons(), decision.error, decision.command.stamp.sequence);
        }
        k_sleep(K_MSEC(10));
    }
}
