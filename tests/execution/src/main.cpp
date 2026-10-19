// Rewritten for the current execution contract: RecoveryGate was retired.
// Test live status publication, stale authority, snapshot isolation and link behavior.
#include <cerrno>
#include <cstring>
#include <communication/interboard/interboard_endpoint.hpp>
#include <robotics/execution/snapshot_cache.hpp>
#include <zephyr/ztest.h>
using namespace skywalker;
using namespace skywalker::robotics;

ZTEST(execution, test_empty_snapshot_does_not_overwrite_destination) {
    SnapshotCache<RunStatus> cache;
    RunStatus output{};
    output.error=314;
    zassert_equal(cache.snapshot(output),-EAGAIN);
    zassert_equal(output.error,314);
}

ZTEST(execution, test_multiple_readers_retain_original_production_stamp) {
    SnapshotCache<RunStatus> cache;
    RunStatus produced{};
    produced.state=RunState::Active;
    produced.ready=true;
    produced.stamp={100,22,true};
    produced.last_command_sequence=9;
    zassert_ok(cache.publish(produced));
    RunStatus a{},b{};
    zassert_ok(cache.snapshot(a));
    zassert_ok(cache.snapshot(b));
    zassert_equal(a.stamp.timestamp_ms,100u);
    zassert_equal(b.stamp.sequence,22u);
    zassert_true(wireFeedback(a,200,100).armed);
    zassert_false(wireFeedback(b,201,100).armed);
    zassert_equal(wireFeedback(b,201,100).active_reasons,FeedbackStale);
    zassert_false(wireFeedback(b,99,100).ready);
    produced.stamp={300,23,true};
    zassert_ok(cache.publish(produced));
    zassert_ok(cache.snapshot(a));
    zassert_equal(a.stamp.sequence,23u);
    zassert_equal(b.stamp.sequence,22u);
}

ZTEST(execution, test_wait_reasons_have_distinct_wire_reason_bits) {
    const struct { WaitReason reason; std::uint32_t expected; } cases[]={
        {WaitReason::Command,CommandStale},
        {WaitReason::Transport,TransportUnavailable},
        {WaitReason::Feedback,FeedbackStale},
        {WaitReason::Reference,RecoveryBoundary},
        {WaitReason::Cycle,RecoveryBoundary},
        {WaitReason::Drive,InvalidConfiguration},
        {WaitReason::Configuration,InvalidConfiguration},
        {WaitReason::Power,PowerBudgetStale}};
    for (const auto &test:cases) {
        RunStatus status{};
        status.state=RunState::Recovering;
        status.reason=test.reason;
        status.stamp={10,1,true};
        auto frame=wireFeedback(status);
        zassert_equal(frame.active_reasons,test.expected);
        zassert_false(frame.armed);
    }
}

namespace {
// No callback or worker: send() immediately copies into the real interboard session.
class CapturingTransport final : public communication::InterBoardTransport {
public:
    communication::InterBoardLink observer{BoardRole::GimbalController};
    std::uint64_t now = 0;
    unsigned sends = 0;
    communication::InterBoardTransportKind kind() const override {
        return communication::InterBoardTransportKind::Uart;
    }
    int service(std::uint64_t time) override {
        now = time;
        return 0;
    }
    int read(RxChunk &) override {
        return -EAGAIN;
    }
    int send(const std::uint8_t *bytes, std::size_t size, std::uint32_t) override {
        ++sends;
        return observer.processRxBytes(bytes, size, now);
    }
    bool txBusy() const override {
        return false;
    }
};
}

ZTEST(execution, test_live_endpoint_stopped_status_producer_withdraws_ready) {
    CapturingTransport transport;
    communication::InterBoardEndpoint endpoint(transport, {BoardRole::ChassisController});
    RunStatus status{};
    status.state = RunState::Active;
    status.ready = true;
    status.last_command_sequence = 42;
    status.stamp = {1000, 1, true};
    endpoint.setStatus(status);
    endpoint.poll(1000);
    BoardHeartbeat heartbeat{};
    ChassisFeedbackSummary feedback{};
    zassert_ok(transport.observer.latestHeartbeat(heartbeat));
    zassert_ok(transport.observer.latestChassisFeedback(feedback));
    zassert_true(heartbeat.ready);
    zassert_true(feedback.armed);
    zassert_equal(feedback.last_command_sequence, 42);
    endpoint.poll(1100);
    zassert_ok(transport.observer.latestHeartbeat(heartbeat));
    zassert_true(heartbeat.ready);
    // Heartbeat remains live, but the independent execution status is stale.
    endpoint.poll(1120);
    zassert_true(transport.observer.peerOnline(1120, 100));
    zassert_ok(transport.observer.latestHeartbeat(heartbeat));
    zassert_ok(transport.observer.latestChassisFeedback(feedback));
    zassert_false(heartbeat.ready);
    zassert_equal(heartbeat.active_reasons, FeedbackStale);
    zassert_false(feedback.ready);
    zassert_false(feedback.armed);
    zassert_equal(feedback.execution_state, ExecutionState::Waiting);
    status.stamp = {1140, 2, true};
    endpoint.setStatus(status);
    endpoint.poll(1140);
    zassert_ok(transport.observer.latestHeartbeat(heartbeat));
    zassert_ok(transport.observer.latestChassisFeedback(feedback));
    zassert_true(heartbeat.ready);
    zassert_true(feedback.armed);
    zassert_equal(transport.sends, 4);
}

ZTEST(execution, test_endpoint_invalid_timeouts_fail_closed_without_sends) {
    CapturingTransport transport;
    communication::InterBoardEndpoint endpoint(transport, {BoardRole::ChassisController, 0, 100, 60, 100});
    endpoint.poll(100);
    zassert_equal(endpoint.snapshot().error,-EINVAL);
    zassert_false(endpoint.snapshot().online);
    zassert_equal(transport.sends,0u);
}
ZTEST_SUITE(execution, nullptr, nullptr, nullptr, nullptr, nullptr);

