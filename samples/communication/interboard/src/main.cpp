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
    static InterBoardEndpoint link(transport, {bench::role});
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    LOG_INF("transport=%u role=%u; p: pause command; s: pause execution status; g: advance generation",
            unsigned(transport.kind()), unsigned(bench::role));
    ChassisCommand command{};
    RunStatus status{};
    status.ready = true;
    status.generation = 1;
    std::uint64_t next_command = 0, next_log = 0;
    std::uint32_t sequence = 0, status_sequence = 0;
    bool produce = true, produce_status = true;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        unsigned char key = 0;
        if (uart_poll_in(console, &key) == 0) {
            if (key == 'p') produce = !produce;
            if (key == 's') produce_status = !produce_status;
            if (key == 'g') ++status.generation;
        }
        if (bench::role == BoardRole::GimbalController && produce && now >= next_command) {
            next_command = now + 10;
            command.mode = ChassisMode::BodyVelocity;
            command.source = ControlSource::Remote;
            command.vx_m_s = bench::target_vx_m_s;
            command.stamp = {now, ++sequence, true};
            link.submit(command);
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
        }
        k_sleep(K_MSEC(1));
    }
}
