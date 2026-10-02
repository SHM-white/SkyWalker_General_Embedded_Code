#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <communication/interboard/interboard_endpoint.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(interboard_bench, LOG_LEVEL_INF);

int main() {
    using namespace skywalker::communication;
    using namespace skywalker::robotics;
    static AsyncUart::DmaBuffers dma_buffers __nocache;
    static ConfiguredInterBoardTransport transport(bench::transport, &dma_buffers);
    static InterBoardEndpoint link(transport, [] {
        InterBoardEndpoint::Config c{bench::role}; c.enable_big_yaw = true;
        c.big_yaw_contract_version = CONFIG_BENCH_BIG_YAW_CONTRACT_VERSION; return c;
    }());
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    LOG_INF("transport=%u role=%u; p: pause command; s: pause execution status; g: wheel generation; b: big-Yaw generation",
            unsigned(transport.kind()), unsigned(bench::role));
    ChassisCommand command{};
    RunStatus status{};
    status.ready = true;
    status.generation = 1;
    std::uint64_t next_command = 0, next_log = 0;
    std::uint32_t sequence = 0, status_sequence = 0;
    BigYawRequest big_yaw_request{};
    BigYawFeedback big_yaw_feedback{};
    big_yaw_feedback.resume_generation = 1;
    bool produce = true, produce_status = true;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        unsigned char key = 0;
        if (uart_poll_in(console, &key) == 0) {
            if (key == 'p') produce = !produce;
            if (key == 's') produce_status = !produce_status;
            if (key == 'g') ++status.generation;
            if (key == 'b') ++big_yaw_feedback.resume_generation;
        }
        if (bench::role == BoardRole::GimbalController && produce && now >= next_command) {
            next_command = now + 10;
            command.mode = ChassisMode::BodyVelocity;
            command.source = ControlSource::Remote;
            command.vx_m_s = bench::target_vx_m_s;
            command.stamp = {now, ++sequence, true};
            link.submit(command);
            big_yaw_request.mode = BigYawMode::FollowCenter;
            big_yaw_request.target_rate_rad_s = 0.1f;
            big_yaw_request.source_sequence = sequence; big_yaw_request.stamp = command.stamp;
            big_yaw_request.permission = {true, true, command.stamp};
            link.submitBigYaw(big_yaw_request);
        }
        const auto previous = link.snapshot();
        const bool fresh = isFresh(previous.control.command.stamp, now, 100);
        const bool context = previous.local_boot_id &&
            previous.control.receiver_boot_id == previous.local_boot_id &&
            previous.control.resume_generation == status.generation;
        status.state = previous.online && fresh && context ? RunState::Active : RunState::Recovering;
        status.reason = previous.online ? WaitReason::Command : WaitReason::Transport;
        if (status.state == RunState::Active) status.reason = WaitReason::None;
        status.last_command_sequence = previous.control.command.stamp.sequence;
        if (produce_status) {
            status.stamp = {now, ++status_sequence, true};
            link.setStatus(status);
            // A no-output execution simulation with a distinct recovery context.
            const auto &r = previous.big_yaw_request;
            const bool big_yaw_active = previous.big_yaw_compatible &&
                forwardedFresh(r.stamp, r.source_age_ms, now, 100) &&
                forwardedFresh(r.stamp, r.command_age_ms, now, 100) &&
                r.receiver_boot_id == previous.local_boot_id &&
                r.resume_generation == big_yaw_feedback.resume_generation &&
                r.mode == BigYawMode::FollowCenter && r.permission.valid && r.permission.enabled &&
                forwardedFresh(r.permission.stamp, r.permission_age_ms, now, 300);
            big_yaw_feedback.execution_state = big_yaw_active ? ExecutionState::Active : ExecutionState::Ready;
            big_yaw_feedback.ready = true; big_yaw_feedback.armed = big_yaw_active; big_yaw_feedback.valid = true;
            big_yaw_feedback.last_command_sequence = r.stamp.sequence;
            big_yaw_feedback.actual_rate_rad_s = 0; // This protocol bench has no motor.
            big_yaw_feedback.stamp = status.stamp;
            link.setBigYawFeedback(big_yaw_feedback);
        }
        link.poll(now);
        if (now >= next_log) {
            next_log = now + 200;
            const auto rx = link.snapshot();
            LOG_INF("transport=%u error=%d peer_online=%d peer_boot=%llx peer_gen=%u local_gen=%u cmd_seq=%u cmd_fresh=%d peer_ready=%d status_producing=%d CRC=%u",
                    unsigned(rx.transport), rx.error, rx.online, rx.peer.sender_boot_id,
                    rx.peer.resume_generation, status.generation, rx.control.command.stamp.sequence,
                    isFresh(rx.control.command.stamp, now, 100), rx.peer.ready, produce_status,
                    rx.parser_stats.crc_errors);
            LOG_INF("bigYaw compatible=%d local_gen=%u peer_gen=%u req_source=%u req_age=%u peer_ready=%d peer_valid=%d advertised_contract=%u",
                    rx.big_yaw_compatible, big_yaw_feedback.resume_generation,
                    rx.big_yaw_feedback.resume_generation, rx.big_yaw_request.source_sequence,
                    rx.big_yaw_request.source_age_ms, rx.big_yaw_feedback.ready,
                    rx.big_yaw_feedback.valid, CONFIG_BENCH_BIG_YAW_CONTRACT_VERSION);
        }
        k_sleep(K_MSEC(1));
    }
}
