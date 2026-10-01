#include <core/clock.hpp>
#include <communication/remote/remote_receiver.hpp>
#include <communication/referee/referee_receiver.hpp>
#include <robotics/command/command_manager.hpp>
#include <zephyr/logging/log.h>
#include "command_router.hpp"
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
Latest<RefereeState> referee_state;
Latest<GimbalCommand> local_command;
void refereeTask(void *, void *, void *) {
    for (;;) {
        const auto value = referee.poll(k_uptime_get());
        (void)referee_state.put(value);
        link.setReferee(value);
        k_sleep(K_MSEC(1));
    }
}
void linkTask(void *, void *, void *) {
    for (;;) { link.poll(k_uptime_get()); k_sleep(K_MSEC(1)); }
}
void commandTask(void *, void *, void *) {
    CommandManager::Config config{};
    config.require_referee_for_motion = board_config::require_referee_for_motion;
    config.permission_timeout_ms = board_config::permission_timeout_ms;
    config.input_timeout_ms = board_config::command_timeout_ms;
    config.allow_auto = false; // No vision receiver is assembled in this application.
    CommandManager manager(config);
    CommandRouter router(local_command, link);
    communication::RemoteReceiver::Snapshot rc{};
    RefereeState ref{};
    std::uint64_t next_log = 0;
    for (;;) {
        remote.snapshot(rc);
        referee_state.get(ref);
        CommandInputs inputs{};
        inputs.remote = rc.remote;
        inputs.referee = ref;
        inputs.now_us = core::monotonicTimeUs();
        const auto decision = manager.update(inputs);
        const auto routed = router.route(decision.command);
        if (inputs.now_us / 1000 >= next_log) {
            next_log = inputs.now_us / 1000 + 1000;
            LOG_INF("mode=%u reasons=%x error=%d publish=%d", unsigned(decision.operator_mode),
                    decision.reasons(), decision.error, routed.local_gimbal_result);
        }
        k_sleep(K_MSEC(10));
    }
}
void gimbalTask(void *, void *, void *) {
    static GimbalExecutor executor;
    (void)executor.begin();
    GimbalCommand command{};
    std::uint64_t next_log = 0;
    for (;;) {
        local_command.get(command);
        const auto now = core::monotonicTimeUs();
        const auto status = executor.update(command, now);
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
K_THREAD_DEFINE(referee_thread, 6144, refereeTask, nullptr, nullptr, nullptr, 6, 0, 0);
K_THREAD_DEFINE(link_thread, 8192, linkTask, nullptr, nullptr, nullptr, 5, 0, 0);
K_THREAD_DEFINE(command_thread, 4096, commandTask, nullptr, nullptr, nullptr, 5, 0, 0);
K_THREAD_DEFINE(gimbal_thread, 6144, gimbalTask, nullptr, nullptr, nullptr, 4, 0, 0);
int main() {
    const int ret = remote.start();
    LOG_INF("gimbal configured=%d referee_required=%d remote_start=%d", board_config::connections_configured,
            board_config::require_referee_for_motion, ret);
    return ret;
}
