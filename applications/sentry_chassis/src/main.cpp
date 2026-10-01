#include <core/clock.hpp>
#include <zephyr/logging/log.h>
#include "chassis_executor.hpp"
LOG_MODULE_REGISTER(sentry_chassis, LOG_LEVEL_INF);
using namespace skywalker;
namespace {
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::ConfiguredInterBoardTransport link_transport(board_config::interboard_transport, &link_dma);
communication::InterBoardEndpoint link(link_transport,
    {robotics::BoardRole::ChassisController, board_config::command_timeout_ms, board_config::heartbeat_timeout_ms});
void linkTask(void *, void *, void *) {
    for (;;) { link.poll(k_uptime_get()); k_sleep(K_MSEC(1)); }
}
void chassisTask(void *, void *, void *) {
    static ChassisExecutor executor(link);
    (void)executor.begin();
    std::uint64_t next_log = 0;
    for (;;) {
        const auto now = core::monotonicTimeUs();
        const auto status = executor.update(now);
        link.setStatus(status);
        if (now / 1000 >= next_log) {
            next_log = now / 1000 + 1000;
            LOG_INF("chassis=%u wait=%u ready=%d generation=%u error=%d", unsigned(status.state),
                    unsigned(status.reason), status.ready, status.generation, status.error);
        }
        k_sleep(K_MSEC(5));
    }
}
}
K_THREAD_DEFINE(link_thread, 6144, linkTask, nullptr, nullptr, nullptr, 5, 0, 0);
K_THREAD_DEFINE(chassis_thread, 8192, chassisTask, nullptr, nullptr, nullptr, 4, 0, 0);
int main() {
    LOG_INF("chassis configured=%d power_budget_required=%d", board_config::connections_configured,
            board_config::require_power_budget);
    return 0;
}
