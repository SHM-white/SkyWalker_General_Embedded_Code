#include "board_config.hpp"
#include "telemetry.hpp"
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
    const int checked = manager.configError();
    if (checked < 0) { LOG_ERR("invalid config: %d", checked); return checked; }
    const int started = sources.start();
    const int output = telemetry.start(bench::telemetry_uart);
    LOG_INF("config=%d sources_start=%d telemetry=%d; command observation bench", checked, started, output);
    for (;;) {
        const auto frame = sources.poll();
        const auto decision = manager.update(frame.inputs);
        telemetry.emit(frame, decision);
        k_sleep(K_MSEC(10));
    }
}
