#include <core/clock.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/command/receiver_sources.hpp>
#include <zephyr/logging/log.h>
#include "gimbal_executor.hpp"
LOG_MODULE_REGISTER(sentry_gimbal, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::AsyncUart::DmaBuffers referee_dma __nocache;
communication::RemoteReceiver remote(board_config::remote_uart, remote_dma, {});
communication::InterBoardEndpoint link(board_config::interboard_uart, link_dma,
    {BoardRole::GimbalController, board_config::command_timeout_ms, board_config::chassis_heartbeat_timeout_ms});
communication::RefereeReceiver referee(board_config::referee_uart, referee_dma, board_config::referee_version);
RemoteSource remote_source(remote);
RefereePermissionSource permission_source(referee);
CommandManager commands(board_config::command_policy);

void linkTask(void *, void *, void *) {
    CommandSnapshot frame{};
    std::uint64_t next_log = 0;
    for (;;) {
        const auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
        if (commands.snapshot(frame) == 0) {
            link.submit(frame.decision.command.chassis);
            link.setReferee(frame.observed.referee);
            if (now_ms >= next_log) {
                next_log = now_ms + 1000;
                LOG_INF("mode=%u reasons=%x error=%d", unsigned(frame.decision.operator_mode),
                        frame.decision.reasons(), frame.decision.error);
            }
        }
        link.poll(now_ms);
        k_sleep(K_MSEC(1));
    }
}
void gimbalTask(void *, void *, void *) {
    static GimbalExecutor executor;
    (void)executor.begin();
    RobotCommand command{};
    std::uint64_t next_log = 0;
    for (;;) {
        (void)commands.current(command);
        const auto now = core::monotonicTimeUs();
        const auto status = executor.update(command.gimbal, now);
        link.setStatus(status);
        if (now / 1000 >= next_log) {
            next_log = now / 1000 + 1000;
            LOG_INF("yaw=%u wait=%u ready=%d error=%d", unsigned(status.state), unsigned(status.reason),
                    status.ready, status.error);
        }
        k_sleep(K_MSEC(5));
    }
}
}
K_THREAD_DEFINE(link_thread, 8192, linkTask, nullptr, nullptr, nullptr, 5, 0, 0);
K_THREAD_DEFINE(gimbal_thread, 6144, gimbalTask, nullptr, nullptr, nullptr, 4, 0, 0);
int main() {
    int ret = commands.registerSource(remote_source);
    if (ret == 0) ret = commands.bindPermissions(permission_source);
    if (ret == 0) ret = commands.start();
    LOG_INF("gimbal configured=%d referee_required=%d command_start=%d", board_config::connections_configured,
            board_config::require_referee_for_motion, ret);
    return ret;
}
