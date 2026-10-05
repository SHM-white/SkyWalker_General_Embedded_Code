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
#include <robotics/gimbal/gimbal_executor.hpp>

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

namespace {
using skywalker::robotics::GimbalExecutionInputs;
using skywalker::robotics::GimbalExecutor;
using skywalker::robotics::GimbalMode;
using skywalker::robotics::RunState;
using skywalker::robotics::WaitReason;

// The controller, driver, group and both buses are production objects. As in the
// existing CAN regressions, only protocol completion and received feedback are
// injected; no worker timing or external CAN hardware is required.
const device second_fake_can = [] {
    device d{};
    d.api = &api;
    return d;
}();

skywalker::control::PositionMotor::Config gimbalLoop(bool torque) {
    skywalker::control::PositionMotor::Config config{};
    config.effort_unit = torque ? skywalker::control::EffortUnit::NewtonMeter : skywalker::control::EffortUnit::Ampere;
    config.safety = {2.0f, 80.0f};
    config.reference = skywalker::control::PositionReference::DriverContinuous;
    config.loop.position = {.kp = 1.0f,
                            .integral_min = -0.1f,
                            .integral_max = 0.1f,
                            .output_min = -1.0f,
                            .output_max = 1.0f,
                            .dt_min_s = 0.001f,
                            .dt_max_s = 0.02f};
    config.loop.velocity.regulator.feedback = {.kp = 0.1f,
                                               .integral_min = -0.1f,
                                               .integral_max = 0.1f,
                                               .output_min = -0.5f,
                                               .output_max = 0.5f,
                                               .dt_min_s = 0.001f,
                                               .dt_max_s = 0.02f};
    config.loop.velocity.reference_slew = {10.0f, 10.0f};
    config.loop.velocity.requested_velocity_abs_max_rad_s = 1.0f;
    config.loop.velocity.effort_abs_max = 0.5f;
    return config;
}

skywalker::robotics::GimbalAxisConfig limitedAxis() {
    return {skywalker::robotics::AxisTopology::Limited,      -1.0f, 1.0f, 1.0f, true,
            skywalker::robotics::AxisReferenceInit::Preserve};
}

struct GimbalFixture {
    Motor yaw{djiConfig(1)}, pitch{dmConfig()};
    CanBus yaw_bus{&fake_can}, pitch_bus{&second_fake_can};
    Group group{yaw, pitch};
    GimbalExecutor executor{yaw,           pitch,
                            group,         gimbalLoop(false),
                            limitedAxis(), gimbalLoop(true),
                            limitedAxis(), GimbalExecutor::Config{.fault_retry_ms = 1}};
    GimbalExecutionInputs inputs{};
    std::uint64_t clock_us = 0;
    std::uint32_t sequence = 0;

    void ready(Motor &motor, bool dm) {
        motor.started_ = true;
        motor.snapshot_.state = MotorState::Disabled;
        motor.snapshot_.output_permitted = false;
        motor.snapshot_.position_reference_valid = true;
        if (motor.snapshot_.reference_generation == 0)
            motor.snapshot_.reference_generation = 1;
        motor.snapshot_.feedback.position_rad = 0;
        motor.snapshot_.feedback.velocity_rad_s = 0;
        motor.snapshot_.feedback.temperature_c = 25;
        motor.snapshot_.feedback.valid = FeedbackPosition | FeedbackVelocity | FeedbackTemperature;
        motor.snapshot_.feedback.timestamp_ms = now();
        motor.feedback_stable_since_ms_ = now() - 5;
        motor.safe_prepared_ = true;
        motor.safe_pending_ = false;
        if (dm) {
            motor.snapshot_.native_drive_status_valid = true;
            motor.snapshot_.native_drive_status = static_cast<std::uint32_t>(dm::DriveStatus::Disabled);
            motor.snapshot_.native_temperatures_valid = true;
            motor.snapshot_.native_mos_temperature_c = 25;
            motor.snapshot_.native_rotor_temperature_c = 25;
        }
    }

    void setup() {
        k_sleep(K_MSEC(10)); // Driver readiness requires a nonzero stable-time origin.
        zassert_ok(yaw_bus.attach(yaw));
        zassert_ok(pitch_bus.attach(pitch));
        yaw_bus.status_.state = pitch_bus.status_.state = BusState::Running;
        yaw_bus.units_[0] = {CanBus::UnitKind::Dji, 0x200, 0};
        pitch_bus.units_[0] = {CanBus::UnitKind::Dm, 1, 0};
        yaw_bus.unit_count_ = pitch_bus.unit_count_ = 1;
        ready(yaw, false);
        ready(pitch, true);
        zassert_ok(executor.begin());
        inputs.transport_ready = true;
        inputs.command.mode = GimbalMode::Rate;
        inputs.command.yaw_rate_rad_s = 0.1f;
        inputs.command.pitch_rate_rad_s = -0.1f;
        clock_us = now() * 1000;
    }

    skywalker::robotics::RunStatus tick(bool new_source = true) {
        clock_us += 5000;
        yaw.snapshot_.feedback.timestamp_ms = pitch.snapshot_.feedback.timestamp_ms = now();
        inputs.command.stamp = {clock_us / 1000, ++sequence, true};
        if (new_source)
            inputs.source_stamp = {clock_us, sequence, true};
        return executor.update(inputs, clock_us);
    }

    void prepare() {
        zassert_equal(tick().reason, WaitReason::Cycle);
        const auto prepared = tick();
        zassert_equal(prepared.state, RunState::Recovering);
        zassert_equal(prepared.reason, WaitReason::Command);
        zassert_true(prepared.ready);
        zassert_equal(prepared.generation, 1);
        zassert_false(group.status().enable_pending);
    }

    std::uint64_t requestEnable() {
        const auto status = tick();
        zassert_equal(status.state, RunState::Recovering);
        zassert_true(group.status().enable_pending, "state=%u reason=%u error=%d", unsigned(status.state),
                     unsigned(status.reason), status.error);
        zassert_false(yaw.snapshot().output_permitted);
        zassert_false(pitch.snapshot().output_permitted);
        return group.status().enable_generation;
    }

    void completeEnable(std::uint64_t generation) {
        yaw.markPrepared(generation);
        pitch.markPrepared(generation);
    }

    void activate() {
        prepare();
        completeEnable(requestEnable());
        zassert_equal(tick().state, RunState::Active);
        zassert_true(yaw.copyStaged().valid);
        zassert_true(pitch.copyStaged().valid);
    }

    void stopped() {
        zassert_false(group.status().active);
        zassert_false(group.status().enable_pending);
        zassert_false(yaw.snapshot().output_permitted);
        zassert_false(pitch.snapshot().output_permitted);
        zassert_false(yaw.copyStaged().valid);
        zassert_false(pitch.copyStaged().valid);
    }
};
} // namespace

ZTEST(motor, test_gimbal_two_can_fault_recovery_waits_for_new_source) {
    static GimbalFixture fixture;
    fixture.setup();
    fixture.activate();
    zassert_not_equal(fixture.yaw.bus_, fixture.pitch.bus_);
    const auto old_source = fixture.inputs.source_stamp;
    fixture.yaw.raiseFault({FaultReason::TransportError, -EIO, &fixture.yaw, now()});
    fixture.stopped(); // A yaw fault closes the shared group on both physical CANs.
    zassert_equal(fixture.tick().state, RunState::Recovering);
    // Communication loss is an Offline transition, not an acknowledged drive
    // fault. Recovery requires fresh feedback, safe stop and a new reference.
    zassert_equal(fixture.yaw.snapshot().state, MotorState::Offline);
    zassert_false(fixture.yaw.snapshot().position_reference_valid);
    zassert_false(fixture.yaw.clear_pending_);
    fixture.ready(fixture.yaw, false);
    fixture.ready(fixture.pitch, true);
    const auto recovered = fixture.tick();
    zassert_equal(recovered.generation, 2);
    zassert_true(recovered.ready);
    fixture.inputs.source_stamp = old_source;
    zassert_equal(fixture.tick(false).reason, WaitReason::Command);
    fixture.stopped(); // Fresh arbitration must not authorize a pre-recovery input.
    fixture.completeEnable(fixture.requestEnable());
    zassert_equal(fixture.tick().state, RunState::Active);
    zassert_equal(fixture.yaw.copyStaged().command.kind, CommandKind::Current);
    zassert_equal(fixture.pitch.copyStaged().command.kind, CommandKind::Torque);
    // Application-owned publication is distinct for the two physical buses.
    zassert_ok(fixture.yaw_bus.commit().error);
    zassert_ok(fixture.pitch_bus.commit().error);
}

ZTEST(motor, test_gimbal_command_cancels_pending_two_can_enable) {
    static GimbalFixture fixture;
    fixture.setup();
    fixture.prepare();
    const auto cancelled_generation = fixture.requestEnable();
    fixture.inputs.command.mode = GimbalMode::Disabled;
    zassert_not_equal(fixture.tick().state, RunState::Active);
    fixture.stopped();
    fixture.completeEnable(cancelled_generation);
    fixture.stopped(); // A late protocol callback cannot resurrect canceled output.
    zassert_false(fixture.yaw.enable_pending_);
    zassert_false(fixture.pitch.enable_pending_);
}

ZTEST(motor, test_gimbal_permission_cancels_pending_two_can_enable) {
    static GimbalFixture fixture;
    fixture.setup();
    fixture.prepare();
    const auto cancelled_generation = fixture.requestEnable();
    fixture.inputs.require_permission = true;
    fixture.inputs.permission.valid = true;
    fixture.inputs.permission.enabled = false;
    fixture.inputs.permission.stamp = {fixture.clock_us / 1000, 1, true};
    zassert_equal(fixture.tick().reason, WaitReason::Power);
    fixture.stopped();
    fixture.completeEnable(cancelled_generation);
    fixture.stopped();
    // Restoring a permit prepares a new context but does not restore the old target.
    fixture.ready(fixture.yaw, false);
    fixture.ready(fixture.pitch, true);
    fixture.inputs.permission.enabled = true;
    fixture.inputs.permission.stamp = {fixture.clock_us / 1000, 2, true};
    zassert_equal(fixture.tick(false).reason, WaitReason::Command);
    fixture.stopped();
}

ZTEST(motor, test_gimbal_reference_change_stops_both_can_axes) {
    static GimbalFixture fixture;
    fixture.setup();
    fixture.activate();
    ++fixture.pitch.snapshot_.reference_generation;
    const auto changed = fixture.tick();
    zassert_equal(changed.state, RunState::Recovering);
    zassert_equal(changed.reason, WaitReason::Reference);
    fixture.stopped();
    zassert_equal(changed.generation, 1); // No new context before preparation completes.
}

ZTEST_SUITE(motor, nullptr, nullptr, nullptr, nullptr, nullptr);
