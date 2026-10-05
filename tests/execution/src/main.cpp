#include <cerrno>
#include <cstring>
#include <communication/interboard/interboard_endpoint.hpp>
#include <robotics/execution/recovery_gate.hpp>
#include <robotics/execution/snapshot_cache.hpp>
#include <zephyr/ztest.h>

using namespace skywalker;
using namespace skywalker::robotics;

ZTEST(execution, test_recovery_requires_new_original_input) {
    RecoveryGate gate({100, 100000});
    gate.prepared(100);
    zassert_equal(gate.generation(), 1);
    gate.prepared(101);
    zassert_equal(gate.boundaryUs(), 100000);
    // Fresh arbitration of a cached input cannot authorize recovery.
    zassert_false(gate.accept({110, 8, true}, {99000, 5, true}, 110));
    zassert_false(gate.accept({110, 9, true}, {100000, 6, true}, 110));
    zassert_equal(gate.stage(), RecoveryGate::Stage::WaitingCommand);
    zassert_true(gate.accept({111, 10, true}, {111000, 7, true}, 111));
    zassert_equal(gate.stage(), RecoveryGate::Stage::Enabling);
    gate.enabled();
    zassert_equal(gate.stage(), RecoveryGate::Stage::Active);
    // Arbitrator still producing, original source stopped.
    zassert_false(gate.accept({212, 11, true}, {111000, 7, true}, 212));
    zassert_equal(gate.stage(), RecoveryGate::Stage::WaitingPrerequisites);
    gate.prepared(212);
    zassert_equal(gate.generation(), 2);
    zassert_false(gate.accept({213, 12, true}, {111000, 7, true}, 213));
    zassert_true(gate.accept({214, 13, true}, {214000, 8, true}, 214));
}

ZTEST(execution, test_pending_enable_revokes_and_mechanisms_recover_independently) {
    RecoveryGate yaw({100, 100000}), wheels({100, 100000});
    yaw.prepared(10);
    wheels.prepared(10);
    zassert_true(yaw.accept({11, 1, true}, {11000, 1, true}, 11));
    zassert_true(wheels.accept({11, 1, true}, {11000, 1, true}, 11));
    wheels.enabled();
    zassert_false(yaw.accept({12, 2, false}, {12000, 2, true}, 12));
    zassert_equal(yaw.stage(), RecoveryGate::Stage::WaitingPrerequisites);
    yaw.enabled();
    zassert_equal(yaw.stage(), RecoveryGate::Stage::WaitingPrerequisites);
    yaw.prepared(12);
    zassert_equal(yaw.generation(), 2);
    zassert_equal(wheels.generation(), 1);
    zassert_equal(wheels.stage(), RecoveryGate::Stage::Active);
    yaw.withdraw(WaitReason::Drive, -EIO, true);
    yaw.prepared(13);
    zassert_equal(yaw.stage(), RecoveryGate::Stage::Blocked);
    zassert_false(yaw.accept({14, 3, true}, {14000, 3, true}, 14));
    yaw.withdraw(WaitReason::Reference);
    yaw.prepared(14);
    zassert_equal(yaw.generation(), 3);
    zassert_true(yaw.accept({15, 4, true}, {15000, 4, true}, 15));
}

ZTEST(execution, test_future_and_cached_command_stamps_reject_enable) {
    RecoveryGate gate({100, 100000});
    gate.prepared(100);
    zassert_false(gate.accept({101, 1, true}, {102000, 1, true}, 101));
    zassert_false(gate.accept({102, 2, true}, {101000, 2, true}, 101));
    zassert_false(gate.accept({99, 3, true}, {101000, 3, true}, 101));
    zassert_true(gate.accept({101, 4, true}, {101000, 4, true}, 101));
    // A stopped manager must expire even when a different source is current.
    zassert_false(gate.accept({101, 4, true}, {202000, 5, true}, 202));
}

ZTEST(execution, test_invalid_recovery_configuration_cannot_prepare) {
    RecoveryGate no_command_lifetime({0, 100000}), no_source_lifetime({100, 0});
    no_command_lifetime.prepared(10);
    no_source_lifetime.prepared(10);
    zassert_equal(no_command_lifetime.stage(), RecoveryGate::Stage::Blocked);
    zassert_equal(no_source_lifetime.stage(), RecoveryGate::Stage::Blocked);
    zassert_equal(no_command_lifetime.error(), -EINVAL);
    zassert_equal(no_source_lifetime.error(), -EINVAL);
    zassert_false(no_command_lifetime.accept({11, 1, true}, {11000, 1, true}, 11));
}

ZTEST(execution, test_source_stamp_preserves_selected_producer) {
    CommandSnapshot frame{};
    frame.observed.remote.online = true;
    frame.observed.remote.stamp = {10, 3, true};
    frame.decision.command.gimbal.stamp = {99, 100, true};
    const auto remote = sourceStamp(frame, ControlSource::Remote);
    zassert_equal(remote.time_us, 10000);
    zassert_equal(remote.sequence, 3);
    frame.decision.selected_vision.stamp = {20001, 7, true};
    const auto vision = sourceStamp(frame, ControlSource::Vision);
    zassert_equal(vision.time_us, 20001);
    zassert_equal(vision.sequence, 7);
    frame.observed.remote.online = false;
    zassert_false(sourceStamp(frame, ControlSource::Remote).valid);
    zassert_false(sourceStamp(frame, ControlSource::None).valid);
}

ZTEST(execution, test_status_cache_never_renews_age) {
    SnapshotCache<RunStatus> cache;
    RunStatus first{};
    first.error = 42;
    zassert_equal(cache.snapshot(first), -EAGAIN);
    zassert_equal(first.error, 42);
    RunStatus produced{};
    produced.state = RunState::Active;
    produced.ready = true;
    produced.generation = 5;
    produced.stamp = {50, 3, true};
    zassert_ok(cache.publish(produced));
    RunStatus second{};
    zassert_ok(cache.snapshot(first));
    zassert_ok(cache.snapshot(second));
    zassert_equal(first.stamp.timestamp_ms, 50);
    zassert_equal(second.stamp.sequence, 3);
    zassert_true(wireFeedback(first, 150, 100).armed);
    const auto stale = wireFeedback(second, 151, 100);
    zassert_false(stale.ready);
    zassert_false(stale.armed);
    zassert_equal(stale.execution_state, ExecutionState::Waiting);
    zassert_equal(stale.active_reasons, FeedbackStale);
    zassert_false(wireFeedback(second, 49, 100).ready);
    second.stamp.valid = false;
    zassert_false(wireFeedback(second, 50, 100).ready);
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
    status.generation = 7;
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
    zassert_equal(heartbeat.resume_generation, 7);
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

ZTEST_SUITE(execution, nullptr, nullptr, nullptr, nullptr, nullptr);
