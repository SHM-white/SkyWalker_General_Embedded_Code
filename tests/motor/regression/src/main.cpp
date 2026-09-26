#include <algorithm>
#include <cstdint>
#include <variant>
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/ztest.h>
// Inspect private state to force exact interleavings without a production test
// hook or scheduler timing guesses. Standard/Zephyr headers are loaded first.
#define private public
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
#undef private

using namespace skywalker::motor;
namespace {
std::uint64_t now() {
    return std::max<std::uint64_t>(1, k_uptime_get());
}
int sends;
can_frame last_frame;
int send(const device *, const can_frame *frame, k_timeout_t, can_tx_callback_t, void *) {
    ++sends;
    last_frame = *frame;
    return 0; // Completion is injected separately, just as by a real CAN ISR.
}
DEVICE_API(can, api) = {.send = send};
const device fake_can = [] {
    device d{};
    d.api = &api;
    return d;
}();

dji::Config djiConfig(unsigned id) {
    return dji::m3508(
        {.id = static_cast<std::uint8_t>(id), .current_limit_a = 2.0f, .gear_ratio = 1.0f, .timing = {20, 20, 5, 100}});
}
dm::Config dmConfig() {
    return dm::j4310Mit({.id = 1,
                         .master_id = 0x11,
                         .position_max_rad = 12.5f,
                         .velocity_max_rad_s = 30.0f,
                         .torque_max_nm = 10.0f,
                         .torque_limit_nm = 1.0f,
                         .timing = {20, 20, 5, 100}});
}
void active(Motor &m) {
    m.started_ = true;
    m.snapshot_.state = MotorState::Active;
    m.snapshot_.enable_generation = 1;
    m.snapshot_.output_permitted = true;
    m.snapshot_.feedback.timestamp_ms = now();
    m.snapshot_.stop.request_generation = 1;
    m.safe_prepared_ = true;
    m.activated_ms_ = now();
}
void attach(CanBus &bus, Motor &a, Motor &b) {
    zassert_ok(bus.attach(a, b));
    bus.status_.state = BusState::Running;
    bus.units_[0] = {CanBus::UnitKind::Dji, 0x200, 0};
    bus.unit_count_ = 1;
    active(a);
    active(b);
}
}

ZTEST(motor, test_stale_safety_frame_cannot_complete_new_stop) {
    static Motor a(djiConfig(1)), b(djiConfig(2));
    static CanBus bus(&fake_can);
    attach(bus, a, b);
    zassert_ok(a.setCurrent(1.0f));
    zassert_ok(bus.commit().error);
    b.requestDisable();
    bus.captureCandidate(0, TxPurpose::SafeOutput);
    zassert_ok(bus.buildSafety());
    zassert_true(bus.candidate_.motors[0].motion);
    a.requestDisable(); // The original F1 window, after encoding and before authorization.
    const int before = sends;
    zassert_equal(bus.submitCandidate(), -EAGAIN);
    zassert_equal(sends, before);
    zassert_equal(a.snapshot().stop.progress, StopProgress::Pending);
    bus.captureCandidate(0, TxPurpose::SafeOutput);
    zassert_ok(bus.buildSafety());
    zassert_ok(bus.submitCandidate());
    for (auto byte : last_frame.data)
        zassert_equal(byte, 0);
    bus.in_flight_.completed_ms = now();
    bus.in_flight_.completed_order = 1;
    bus.updateStopAfterTx(bus.in_flight_);
    zassert_equal(a.snapshot().stop.progress, StopProgress::TxComplete);
    zassert_equal(b.snapshot().stop.progress, StopProgress::TxComplete);
}

ZTEST(motor, test_authorized_residual_does_not_acknowledge_later_stop) {
    static Motor a(djiConfig(1)), b(djiConfig(2));
    static CanBus bus(&fake_can);
    attach(bus, a, b);
    zassert_ok(a.setCurrent(1.0f));
    zassert_ok(bus.commit().error);
    b.requestDisable();
    bus.captureCandidate(0, TxPurpose::SafeOutput);
    zassert_ok(bus.buildSafety());
    zassert_ok(bus.submitCandidate());
    a.requestDisable(); // Allowed one-frame residual, but it cannot acknowledge A.
    bus.updateStopAfterTx(bus.in_flight_);
    zassert_equal(a.snapshot().stop.progress, StopProgress::Pending);
    zassert_true(a.safe_pending_);
    zassert_equal(b.snapshot().stop.progress, StopProgress::TxComplete);
}

ZTEST(motor, test_snapshot_and_sequence_survive_new_commit) {
    static Motor a(djiConfig(1)), b(djiConfig(2));
    static CanBus bus(&fake_can);
    attach(bus, a, b);
    zassert_ok(a.setCurrent(1.0f));
    zassert_ok(b.setCurrent(1.0f));
    const auto first = bus.commit();
    bus.captureCandidate(0, TxPurpose::Target);
    zassert_ok(a.setCurrent(2.0f));
    zassert_ok(b.setCurrent(2.0f));
    zassert_ok(bus.commit().error);
    zassert_ok(bus.buildTarget(now()));
    zassert_equal(bus.candidate_.sequence, first.sequence);
    zassert_equal(bus.candidate_.frame.data[0], bus.candidate_.frame.data[2]);
    zassert_equal(bus.candidate_.frame.data[1], bus.candidate_.frame.data[3]);
    zassert_equal(bus.candidate_.motors[0].command.command.primary, 1.0f);
    zassert_equal(bus.candidate_.motors[1].command.command.primary, 1.0f);
}

ZTEST(motor, test_feedback_gap_revokes_group_before_new_timestamp) {
    static Motor a(djiConfig(1)), b(djiConfig(2));
    static Group group(a, b);
    active(a);
    active(b);
    group.active_ = true;
    group.enable_generation_ = 1;
    a.snapshot_.feedback.timestamp_ms = 100;
    std::get<Motor::DjiRuntime>(a.protocol_state_).has_encoder = true;
    dji::RawFeedback raw{};
    zassert_ok(a.acceptDjiFeedback(raw, 121));
    zassert_false(group.active_);
    zassert_false(a.snapshot_.output_permitted);
    zassert_false(b.snapshot_.output_permitted);
    zassert_equal(a.snapshot_.feedback.timestamp_ms, 121);
    zassert_equal(a.feedback_stable_since_ms_, 121);
    zassert_equal(group.last_fault_.reason, FaultReason::FeedbackExpired);
}

ZTEST(motor, test_dm_gap_during_enable_revokes_group) {
    static Motor a(dmConfig()), b(djiConfig(2));
    static Group group(a, b);
    active(a);
    active(b);
    a.snapshot_.state = MotorState::Enabling;
    a.enable_pending_ = true;
    group.enable_pending_ = true;
    group.enable_generation_ = 1;
    a.snapshot_.feedback.timestamp_ms = 100;
    std::get<Motor::DmRuntime>(a.protocol_state_).has_native_position = true;
    dm::DecodedFeedback feedback{};
    feedback.raw.motor_id = 1;
    feedback.raw.status = dm::DriveStatus::Disabled;
    zassert_ok(a.acceptDmFeedback(feedback, 121, 1));
    zassert_false(group.enable_pending_);
    zassert_false(b.snapshot_.output_permitted);
    zassert_equal(a.feedback_stable_since_ms_, 121);
}

ZTEST(motor, test_fast_recovery_restarts_stability_without_reseeding) {
    static Motor a(djiConfig(1)), dm(dmConfig());
    active(a);
    active(dm);
    a.snapshot_.feedback.timestamp_ms = dm.snapshot_.feedback.timestamp_ms = 100;
    a.snapshot_.position_reference_valid = dm.snapshot_.position_reference_valid = true;
    std::get<Motor::DjiRuntime>(a.protocol_state_).has_encoder = true;
    std::get<Motor::DmRuntime>(dm.protocol_state_).has_native_position = true;
    a.raiseFault({FaultReason::TransportError, -EIO, &a, 101});
    dm.raiseFault({FaultReason::RxOverflow, -ENOBUFS, &dm, 101});
    zassert_ok(a.acceptDjiFeedback({}, 102));
    dm::DecodedFeedback feedback{};
    feedback.raw.motor_id = 1;
    feedback.raw.status = dm::DriveStatus::Disabled;
    zassert_ok(dm.acceptDmFeedback(feedback, 102, 1));
    zassert_equal(a.feedback_stable_since_ms_, 102);
    zassert_equal(dm.feedback_stable_since_ms_, 102);
    zassert_false(a.snapshot_.position_reference_valid);
    zassert_false(dm.snapshot_.position_reference_valid);
    a.safe_prepared_ = dm.safe_prepared_ = true;
    a.snapshot_.state = dm.snapshot_.state = MotorState::Disabled;
    zassert_true(a.readyLocked(107));
    zassert_true(dm.readyLocked(107));
}

ZTEST(motor, test_transport_fault_preserves_explicit_clear_requirement) {
    static Motor a(djiConfig(1));
    active(a);
    a.raiseFault({FaultReason::InvalidCommand, -EINVAL, &a, now()});
    a.raiseFault({FaultReason::TransportError, -EIO, &a, now()});
    zassert_equal(a.snapshot_.state, MotorState::Fault);
    zassert_equal(a.snapshot_.last_fault.reason, FaultReason::InvalidCommand);
    zassert_equal(a.requestEnable(2), -EIO);
    zassert_ok(a.requestClearFault());
    a.markFaultCleared(a.snapshot_.stop.request_generation);
    zassert_equal(a.latched_fault_.reason, FaultReason::None);
    zassert_not_equal(a.snapshot_.state, MotorState::Fault);
}

ZTEST(motor, test_old_clear_completion_cannot_clear_new_request) {
    static Motor a(dmConfig());
    active(a);
    a.raiseFault({FaultReason::DriveFault, -EIO, &a, now()});
    zassert_ok(a.requestClearFault());
    const auto old = a.snapshot_.stop.request_generation;
    a.requestDisable();
    zassert_ok(a.requestClearFault());
    a.markClearTxComplete(old, now(), 1);
    a.markFaultCleared(old);
    zassert_false(a.clear_tx_done_);
    zassert_equal(a.snapshot_.state, MotorState::Fault);
}

ZTEST(motor, test_work_and_deadlines_shorten_wait) {
    static Motor a(djiConfig(1)), b(djiConfig(2));
    static CanBus bus(&fake_can);
    attach(bus, a, b);
    bus.rx_count_ = 1;
    zassert_equal(bus.nextWaitMs(now()), 0);
    bus.rx_count_ = 0;
    a.snapshot_.feedback.timestamp_ms = b.snapshot_.feedback.timestamp_ms = 100;
    a.activated_ms_ = b.activated_ms_ = 120;
    zassert_equal(bus.nextWaitMs(120), 1);
    bus.status_.state = BusState::Recovering;
    bus.in_flight_.busy = true;
    bus.in_flight_.submitted_ms = 1;
    bus.next_recovery_ms_ = 220;
    zassert_equal(bus.nextWaitMs(120), 100); // Expired TX must not spin during retry wait.
}

ZTEST(motor, test_group_revocation_is_checked_at_authorization) {
    static Motor a(djiConfig(1)), b(djiConfig(2));
    static CanBus bus(&fake_can);
    static Group group(a, b);
    attach(bus, a, b);
    group.active_ = true;
    group.enable_generation_ = 1;
    zassert_ok(a.setCurrent(1.0f));
    zassert_ok(bus.commit().error);
    bus.captureCandidate(0, TxPurpose::Target);
    zassert_ok(bus.buildTarget(now()));
    // Group.disable closes group permission before notifying each Motor.
    group.active_ = false;
    zassert_equal(bus.submitCandidate(), -EAGAIN);
}

ZTEST(motor, test_dm_enable_and_probe_cannot_cross_disable) {
    static Motor a(dmConfig());
    static CanBus bus(&fake_can);
    zassert_ok(bus.attach(a));
    bus.units_[0] = {CanBus::UnitKind::Dm, 1, 0};
    bus.unit_count_ = 1;
    active(a);
    a.snapshot_.state = MotorState::Enabling;
    a.snapshot_.output_permitted = false;
    a.enable_pending_ = true;
    a.enable_requested_at_ms_ = now();
    bus.captureCandidate(0, TxPurpose::Enable);
    a.requestDisable();
    zassert_equal(bus.submitCandidate(), -EAGAIN);
    // Even if the request was canceled before capture, no late Enable/Probe.
    bus.captureCandidate(0, TxPurpose::Enable);
    zassert_equal(bus.submitCandidate(), -EAGAIN);
    bus.captureCandidate(0, TxPurpose::Probe);
    zassert_equal(bus.submitCandidate(), -EAGAIN);
}

ZTEST(motor, test_dji_waiting_for_group_peer_does_not_busy_loop) {
    static Motor a(djiConfig(1)), b(djiConfig(2));
    static CanBus bus(&fake_can);
    attach(bus, a, b);
    a.snapshot_.state = b.snapshot_.state = MotorState::Enabling;
    a.enable_pending_ = b.enable_pending_ = true;
    a.enable_requested_at_ms_ = b.enable_requested_at_ms_ = now();
    // DJI has no Enable special command; after safe preparation it can wait.
    zassert_equal(bus.nextWaitMs(now()), 2);
}

ZTEST(motor, test_stale_dm_callback_does_not_raise_new_gap_fault) {
    static Motor a(dmConfig());
    active(a);
    a.feedback_event_order_ = 2;
    a.snapshot_.feedback.timestamp_ms = 100;
    dm::DecodedFeedback feedback{};
    feedback.raw.motor_id = 1;
    feedback.raw.status = dm::DriveStatus::Enabled;
    zassert_equal(a.acceptDmFeedback(feedback, 121, 1), -ESTALE);
    zassert_equal(a.snapshot_.state, MotorState::Active);
    zassert_equal(a.snapshot_.feedback.timestamp_ms, 100);
}

ZTEST(motor, test_multiple_groups_share_one_authorized_frame) {
    static Motor a(djiConfig(1)), b(djiConfig(2));
    static CanBus bus(&fake_can);
    static Group first(a), second(b);
    attach(bus, a, b);
    first.active_ = second.active_ = true;
    first.enable_generation_ = second.enable_generation_ = 1;
    zassert_ok(a.setCurrent(1.0f));
    zassert_ok(b.setCurrent(2.0f));
    zassert_ok(bus.commit().error);
    bus.captureCandidate(0, TxPurpose::Target);
    zassert_ok(bus.buildTarget(now()));
    zassert_ok(bus.submitCandidate()); // CONFIG_SPIN_VALIDATE checks lock ownership too.
    zassert_true(bus.in_flight_.busy);
    zassert_equal(bus.in_flight_.stop_generations[0], 0);
    zassert_equal(bus.in_flight_.stop_generations[1], 0);
}

ZTEST_SUITE(motor, nullptr, nullptr, nullptr, nullptr, nullptr);
