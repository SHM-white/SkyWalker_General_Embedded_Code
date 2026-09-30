#include "board_config.hpp"
#include "telemetry.hpp"
#include <core/clock.hpp>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(command_manager_bench, LOG_LEVEL_INF);

namespace {
using namespace skywalker;
static communication::AsyncUart::DmaBuffers rc_dma __nocache;
static communication::AsyncUart::DmaBuffers vision_dma __nocache;
static communication::AsyncUart::DmaBuffers referee_dma __nocache;
static bench::InputSources sources(bench::inputs, rc_dma, vision_dma, referee_dma);
static robotics::CommandManager manager(bench::manager);
static bench::Telemetry telemetry;
}

int main() {
    const int checked = manager.validate();
    const int started = sources.start();
    const int output = telemetry.start(bench::telemetry_uart);
    LOG_INF("config=%d sources_start=%d telemetry=%d; command observation bench", checked, started, output);
    for (;;) {
        const auto frame = sources.poll();
        skywalker::robotics::CommandDecision decision{};
        const int ret = manager.step(frame.commands, skywalker::core::monotonicTimeUs(), decision);
        telemetry.emit(frame, decision, ret);
        k_sleep(K_MSEC(10));
    }
}
