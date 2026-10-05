#include <algorithm>
#include <cerrno>
#include <cstring>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <communication/interboard/interboard_endpoint.hpp>
#include <communication/wire.hpp>
#include "../../../robotics/common/sample_diagnostics.hpp"
#include "board_config.hpp"
LOG_MODULE_REGISTER(interboard_bench, LOG_LEVEL_INF);

namespace {
// Fault injection belongs to this no-actuator diagnostic, never the endpoint.
class DiagnosticTransport final : public skywalker::communication::InterBoardTransport {
public:
    explicit DiagnosticTransport(InterBoardTransport &inner) : inner_(inner) {}
    bool invalid_protocol = false;
    skywalker::communication::InterBoardTransportKind kind() const override { return inner_.kind(); }
    int service(std::uint64_t now) override { return inner_.service(now); }
    int read(RxChunk &out) override { return inner_.read(out); }
    bool txBusy() const override { return inner_.txBusy(); }
    int send(const std::uint8_t *data, std::size_t size, std::uint32_t timeout) override {
        if (!invalid_protocol) return inner_.send(data, size, timeout);
        if (!data || !size || size > kTxCapacity) return -EINVAL;
        std::uint8_t bytes[kTxCapacity]{};
        std::memcpy(bytes, data, size);
        for (std::size_t offset = 0; offset < size;) {
            if (size - offset < 14) return -EINVAL;
            const auto total = std::size_t(skywalker::communication::wire::loadLe16(bytes + offset + 6)) + 14;
            if (total > size - offset) return -EINVAL;
            bytes[offset + 2] = 0xff;
            skywalker::communication::wire::storeLe16(bytes + offset + total - 2,
                skywalker::communication::interboardCrc16(bytes + offset, total - 2));
            offset += total;
        }
        return inner_.send(bytes, size, timeout);
    }
private:
    InterBoardTransport &inner_;
};
}

int main() {
    using namespace skywalker::communication;
    using namespace skywalker::robotics;
    static AsyncUart::DmaBuffers dma_buffers __nocache;
    static ConfiguredInterBoardTransport physical_transport(bench::transport, &dma_buffers);
    static DiagnosticTransport transport(physical_transport);
    static InterBoardEndpoint link(transport, {bench::role});
    skywalker::samples::control::SampleDiagnostics diagnostics;
    LOG_INF("transport=%u role=%u protocol=%u; autonomous protocol exercise, no actuators",
            unsigned(transport.kind()), unsigned(bench::role), kInterBoardProtocolVersion);
    ChassisCommand command{};
    RunStatus status{};
    status.ready = true;
    std::uint64_t next_command = 0, next_log = 0;
    std::uint32_t sequence = 0, status_sequence = 0;
    BigYawRequest big_yaw_request{};
    BigYawFeedback big_yaw_feedback{};
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const auto previous = link.snapshot();
        const auto diagnostic = diagnostics.update(now, previous.online, false);
        transport.invalid_protocol = diagnostic.invalid_protocol;
        if (bench::role == BoardRole::GimbalController && !diagnostic.input_paused && now >= next_command) {
            next_command = now + 10;
            command.mode = ChassisMode::BodyVelocity; command.source = ControlSource::Autonomous;
            command.vx_m_s = bench::target_vx_m_s; command.stamp = {now, ++sequence, true};
            link.submit(command);
            OperatorControl authority{};
            authority.run_allowed = previous.online;
            authority.source_sequence = sequence; authority.source_age_ms = 0;
            authority.receiver_boot_id = previous.peer.sender_boot_id;
            authority.stamp = command.stamp;
            (void)link.submitOperatorControl(authority);
            big_yaw_request.mode = BigYawMode::FollowCenter;
            big_yaw_request.target_rate_rad_s = 0.1f;
            big_yaw_request.source_sequence = sequence; big_yaw_request.stamp = command.stamp;
            big_yaw_request.permission = {true, true, command.stamp};
            link.submitBigYaw(big_yaw_request);
        }
        if (!diagnostic.execution_paused && !diagnostic.status_paused) {
            const bool fresh = isFresh(previous.control.command.stamp, now, 100);
            const bool context = previous.local_boot_id && previous.control.receiver_boot_id == previous.local_boot_id;
            const bool allowed = previous.operator_control_valid && previous.operator_control.run_allowed &&
                !previous.operator_control.emergency_stop;
            status.requested = previous.online && allowed && fresh && context;
            status.member_count = 1;
            status.active_count = status.requested ? 1 : 0;
            status.waiting_count = 0;
            status.state = status.requested ? RunState::Active : RunState::Disabled;
            status.reason = previous.online ? WaitReason::Command : WaitReason::Transport;
            if (status.state == RunState::Active) status.reason = WaitReason::None;
            status.last_command_sequence = previous.control.command.stamp.sequence;
            status.stamp = {now, ++status_sequence, true};
            link.setStatus(status);
            const auto &request = previous.big_yaw_request;
            const bool active = previous.online && allowed &&
                forwardedFresh(request.stamp, request.source_age_ms, now, 100) &&
                forwardedFresh(request.stamp, request.command_age_ms, now, 100) &&
                request.receiver_boot_id == previous.local_boot_id &&
                request.mode == BigYawMode::FollowCenter && request.permission.valid && request.permission.enabled &&
                forwardedFresh(request.permission.stamp, request.permission_age_ms, now, 300);
            big_yaw_feedback.execution_state = active ? ExecutionState::Active : ExecutionState::Ready;
            big_yaw_feedback.ready = true; big_yaw_feedback.armed = active; big_yaw_feedback.valid = true;
            big_yaw_feedback.last_command_sequence = request.stamp.sequence;
            big_yaw_feedback.actual_rate_rad_s = 0; // No motor exists in this sample.
            big_yaw_feedback.stamp = status.stamp;
            link.setBigYawFeedback(big_yaw_feedback);
        }
        link.poll(now);
        if (now >= next_log) {
            next_log = now + 200;
            const auto rx = link.snapshot();
            LOG_INF("transport=%u error=%d online=%d boot=%llx cmd=%u authority=%d inputPause=%d statusPause=%d CRC=%u versionErrors=%u",
                    unsigned(rx.transport), rx.error, rx.online, rx.peer.sender_boot_id,
                    rx.control.command.stamp.sequence,
                    rx.operator_control_valid, diagnostic.input_paused,
                    diagnostic.execution_paused || diagnostic.status_paused,
                    rx.parser_stats.crc_errors, rx.parser_stats.version_errors);
            LOG_INF("bigYaw source=%u age=%u ready=%d valid=%d",
                    rx.big_yaw_request.source_sequence, rx.big_yaw_request.source_age_ms,
                    rx.big_yaw_feedback.ready, rx.big_yaw_feedback.valid);
        }
        k_sleep(K_MSEC(1));
    }
}
