#include <zephyr/ztest.h>
#include <algorithm>
#include <array>
#include <variant>
#include <cmath>
#include <limits>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
// Test-only access to inject I/O snapshots without a live CAN controller.
// Production feedback conversion, staging, fault and control code still run.
#define private public
#include <drivers/motor/can_bus.hpp>
#undef private
#include <robotics/vehicle/chassis_profile.hpp>
#include <robotics/chassis/reference_v4_motion.hpp>
#include <robotics/execution/big_yaw_executor.hpp>
#include <robotics/execution/big_yaw_heading_follower.hpp>
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
constexpr float pi = 3.14159265358979323846f;
SwerveChassis::Config config() {
    auto c = vehicle::swerveConfig();
    c.kinematics.max_wheel_velocity_m_s = 1;
    for (auto &m : c.modules) {
        m.wheel_radius_m = .1f;
        m.drive.requested_velocity_abs_max_rad_s = 20;
        m.drive.reference_slew = {10, 10};
        m.drive.regulator.feedback.ki = 1;
        m.drive.regulator.feedback.integral_min = -1;
        m.drive.regulator.feedback.integral_max = 1;
        m.idle_behavior = SwerveModule::IdleBehavior::Hold;
    }
    return c;
}
ChassisFeedback feedback(float angle = 0) {
    ChassisFeedback f{};
    for (auto &m : f.module) {
        m.steer_valid = m.drive_valid = true;
        m.steer_absolute_rad = angle;
        m.steer_feedback_stamp_ms = m.steer_enable_generation = m.drive_enable_generation = 1;
    }
    return f;
}
void ticks(SwerveChassis &c, const ChassisCommand &cmd, ChassisFeedback &f, ChassisOutput &o, unsigned n = 4) {
    for (unsigned k = 0; k < n; ++k) {
        for (auto &m : f.module)
            ++m.steer_feedback_stamp_ms;
        zassert_ok(c.step(cmd, f, .002f, o));
    }
}
}
ZTEST(sentry, test_low_speed_spin_and_translation_start_from_rest) {
    const auto config = vehicle::swerveConfig();
    zassert_within(config.kinematics.stationary_epsilon_m_s, .01f, 1e-6f);
    zassert_within(config.kinematics.resume_velocity_m_s, .02f, 1e-6f);
    SwerveKinematics kinematics(config.kinematics);
    ModuleTargets initial{}, result{};
    zassert_ok(kinematics.reset(initial));
    const ChassisCommand spin{.mode = ChassisMode::Spin, .wz_rad_s = .5f};
    zassert_ok(kinematics.solve(spin, result));
    for (const auto &wheel : result)
        zassert_true(wheel.wheel_velocity_m_s > .1f);
    zassert_ok(kinematics.reset(initial));
    const ChassisCommand forward{.mode = ChassisMode::BodyVelocity, .vx_m_s = .1f};
    zassert_ok(kinematics.solve(forward, result));
    for (const auto &wheel : result)
        zassert_true(wheel.wheel_velocity_m_s > .09f);
}

ZTEST(sentry, test_candidate_mapping_and_real_motor_sign_units) {
    constexpr unsigned ids[] = {1, 4, 2, 3}, zeros[] = {3060, 986, 2383, 4390};
    for (unsigned i = 0; i < 4; ++i) {
        const auto &sc = vehicle::reference_steer[i], &wc = vehicle::reference_wheel[i];
        zassert_equal(sc.id, ids[i]);
        zassert_equal(wc.id, ids[i]);
        zassert_equal(sc.encoder_zero_ticks, zeros[i]);
        zassert_equal(sc.direction, -1);
        zassert_equal(wc.direction, -1);
        motor::Motor s(motor::dji::gm6020(
            {.id = sc.id, .encoder_zero_ticks = sc.encoder_zero_ticks, .current_mode_confirmed = true}));
        zassert_ok(s.acceptDjiFeedback({.encoder = sc.encoder_zero_ticks}, 1));
        zassert_within(s.snapshot().feedback.absolute_position_rad, 0, 1e-6f);
        zassert_ok(
            s.acceptDjiFeedback({.encoder = static_cast<std::uint16_t>((zeros[i] + 1024) % 8192), .speed_rpm = 60}, 2));
        zassert_within(sc.direction * s.snapshot().feedback.absolute_position_rad, -pi / 4, 1e-5f);
        motor::Motor d(motor::dji::m3508({.id = wc.id, .gear_ratio = wc.gear_ratio}));
        zassert_ok(d.acceptDjiFeedback({.encoder = 0, .speed_rpm = 60}, 1));
        zassert_within(d.snapshot().feedback.velocity_rad_s, 2 * pi / wc.gear_ratio, 1e-6f);
    }
    motor::Motor s0(vehicle::steerHardware(0)), s1(vehicle::steerHardware(1)), s2(vehicle::steerHardware(2)),
        s3(vehicle::steerHardware(3));
    motor::Motor d0(vehicle::wheelHardware(0)), d1(vehicle::wheelHardware(1)), d2(vehicle::wheelHardware(2)),
        d3(vehicle::wheelHardware(3));
    std::array<motor::Motor *, 8> motors{&s0, &s1, &s2, &s3, &d0, &d1, &d2, &d3};
    motor::Group group(s0, s1, s2, s3, d0, d1, d2, d3);
    auto hc = vehicle::swerveHardwareConfig();
    hc.directions.fill(-1);
    SwerveHardware hw(motors, group, hc);
    zassert_ok(hw.begin());
    ChassisOutput o{};
    for (auto *m : motors) {
        m->snapshot_.state = motor::MotorState::Active;
        m->snapshot_.feedback_fresh = true;
        m->snapshot_.feedback.valid = motor::FeedbackAbsolutePosition | motor::FeedbackVelocity;
        m->snapshot_.feedback.absolute_position_rad = .2f;
        m->snapshot_.feedback.velocity_rad_s = .3f;
    }
    ChassisFeedback f{};
    zassert_ok(hw.read(f));
    for (auto &m : f.module) {
        zassert_within(m.steer_absolute_rad, -.2f, 1e-6f);
        zassert_within(m.drive_velocity_rad_s, -.3f, 1e-6f);
    }
    for (auto &m : o.module) {
        m.steer_output_valid = m.drive_output_valid = true;
        m.steer_effort = .1f;
        m.drive_effort = .2f;
    }
    zassert_ok(hw.stage(o, 1));
    zassert_within(s0.copyStaged().command.primary, -.1f, 1e-6f);
    zassert_within(d0.copyStaged().command.primary, -.2f, 1e-6f);
}
ZTEST(sentry, test_per_wheel_cosine_and_independent_feedback_recovery) {
    SwerveChassis c(config());
    auto f = feedback();
    ChassisOutput o{};
    ChassisCommand cmd{.mode = ChassisMode::BodyVelocity, .vy_m_s = .5f};
    ticks(c, cmd, f, o, 20);
    zassert_true(o.drive_permitted); // No chassis-wide steering alignment gate.
    zassert_within(o.module[0].drive_target_rad_s, 0, 1e-5f);
    for (auto &m : f.module)
        m.steer_absolute_rad = pi / 2;
    ticks(c, cmd, f, o, 20);
    zassert_true(o.drive_permitted);
    zassert_true(o.module[0].drive_effort > 0);
    f.module[2].steer_valid = false;
    ticks(c, cmd, f, o);
    zassert_false(o.drive_permitted); // Diagnostic summary, not an output gate.
    zassert_false(o.module[2].drive_output_valid);
    for (unsigned i : {0u, 1u, 3u})
        zassert_true(o.module[i].drive_output_valid);
    f.module[2].steer_valid = true;
    ticks(c, cmd, f, o);
    zassert_true(o.drive_permitted);
    f.module[1].drive_valid = false;
    ticks(c, cmd, f, o);
    zassert_false(o.drive_permitted);
    for (unsigned i : {0u, 2u, 3u})
        zassert_true(o.module[i].drive_output_valid);
    f.module[1].drive_valid = true;
    ++f.module[1].drive_enable_generation;
    ticks(c, cmd, f, o);
    zassert_true(o.drive_permitted);
}
ZTEST(sentry, test_cosine_zero_reaches_drive_pid_without_stale_slew) {
    SwerveModule m(vehicle::swerveConfig().modules[0]);
    auto f = feedback().module[0];
    ModuleOutput o{};
    zassert_ok(m.reset(f));
    for (unsigned i = 0; i < 400; ++i) {
        ++f.steer_feedback_stamp_ms;
        zassert_ok(m.step({0.0f, .3f}, f, .002f, o));
    }
    zassert_true(o.drive_effort > 0);
    ++f.steer_feedback_stamp_ms;
    zassert_ok(m.step({pi / 2, .3f}, f, .002f, o));
    zassert_within(o.cosine_scale, 0, 1e-6f);
    zassert_within(o.drive_target_rad_s, 0, 1e-6f);
    // Installed drive profile has no integral term; measured rate is zero.
    zassert_within(o.drive_effort, 0, 1e-6f);
}
ZTEST(sentry, test_cosine_reverse_flip_hysteresis_zero_and_invalid) {
    SwerveModule m(config().modules[0]);
    auto f = feedback().module[0];
    ModuleOutput o{};
    for (float a : {0.f, .3f, pi / 3, pi / 2, 1.6f, pi, -pi}) {
        zassert_ok(m.reset(f));
        zassert_ok(m.step({a, .5f}, f, .002f, o));
        const float expected = std::pow(std::max(std::cos(o.alignment_error_rad), 0.f), 3.0f);
        zassert_within(o.cosine_scale, expected, 1e-6f);
        zassert_within(o.drive_target_rad_s, o.optimized_wheel_velocity_m_s / .1f * expected, 1e-5f);
    }
    zassert_true(o.flipped);
    zassert_true(o.drive_target_rad_s < 0);
    zassert_ok(m.step({pi / 2 + .02f, .5f}, f, .002f, o));
    zassert_true(o.flipped);
    zassert_ok(m.step({pi / 2 - .02f, .5f}, f, .002f, o));
    zassert_true(o.flipped);
    zassert_ok(m.step({1.4f, .5f}, f, .002f, o));
    zassert_false(o.flipped);
    zassert_ok(m.step({1.4f, 0}, f, .002f, o));
    zassert_equal(o.drive_target_rad_s, 0);
    f.steer_valid = false;
    zassert_ok(m.step({0, .5f}, f, .002f, o));
    zassert_equal(o.drive_effort, 0);
    zassert_equal(m.step({NAN, .5f}, f, .002f, o), -EINVAL);
    zassert_equal(o.drive_effort, 0);
}
ZTEST(sentry, test_rotation_has_no_collective_alignment_gate) {
    for (bool mixed : {false, true}) {
        SwerveChassis c(config());
        auto f = feedback();
        ChassisOutput o{};
        ChassisCommand cmd{.mode = mixed ? ChassisMode::BodyVelocity : ChassisMode::Spin,
                           .vx_m_s = mixed ? .1f : 0,
                           .wz_rad_s = .8f};
        ticks(c, cmd, f, o);
        zassert_true(o.drive_permitted);
        for (auto &m : o.module)
            zassert_true(m.drive_output_valid);
        f.module[0].steer_valid = false;
        ticks(c, cmd, f, o);
        zassert_false(o.drive_permitted);
        zassert_false(o.module[0].drive_output_valid);
        for (unsigned i = 1; i < 4; ++i)
            zassert_true(o.module[i].drive_output_valid);
        f.module[0].steer_valid = true;
        ticks(c, cmd, f, o);
        zassert_true(o.drive_permitted);
        cmd.wz_rad_s = 0;
        cmd.mode = ChassisMode::BodyVelocity;
        cmd.vy_m_s = .5f;
        ticks(c, cmd, f, o);
        zassert_true(o.drive_permitted);
        ticks(c, {}, f, o);
        zassert_false(o.drive_permitted);
    }
}
namespace {
can_state fake_state = CAN_STATE_ERROR_ACTIVE;
int fakeStart(const device *) {
    fake_state = CAN_STATE_ERROR_ACTIVE;
    return 0;
}
int fakeStop(const device *) {
    fake_state = CAN_STATE_STOPPED;
    return 0;
}
int fakeState(const device *, can_state *s, can_bus_err_cnt *e) {
    if (s)
        *s = fake_state;
    if (e)
        *e = {};
    return 0;
}
static DEVICE_API(can, fake_api) = {.start = fakeStart, .stop = fakeStop, .get_state = fakeState};
static device fake_can{.name = "sentry-test-can", .api = &fake_api};
}
ZTEST(sentry, test_can_failure_and_feedback_timeout_with_real_recovery) {
    motor::Motor m(vehicle::wheelHardware(0));
    motor::CanBus bus(&fake_can);
    zassert_ok(bus.attach(m));
    m.markStarted();
    zassert_ok(m.enable());
    auto now = static_cast<std::uint64_t>(k_uptime_get());
    if (!now) {
        k_sleep(K_MSEC(1));
        now = k_uptime_get();
    }
    zassert_ok(m.acceptDjiFeedback({.encoder = 0}, now));
    m.markPrepared(m.protocol_generation_);
    const auto gen = m.snapshot().enable_generation;
    zassert_true(m.active());
    fake_state = CAN_STATE_BUS_OFF;
    bus.controller_started_ = true;
    bus.enterRecovery(-ENETUNREACH, motor::FaultReason::TransportError);
    zassert_equal(bus.status().state, motor::BusState::Recovering);
    zassert_false(m.active());
    zassert_equal(m.snapshot().last_fault.reason, motor::FaultReason::TransportError);
    bus.recoverController(UINT64_MAX);
    bus.recoverController(UINT64_MAX);
    zassert_equal(bus.status().state, motor::BusState::Running);
    zassert_false(m.active()); // Controller recovery alone cannot restore motor authority.
    zassert_ok(m.acceptDjiFeedback({.encoder = 1}, now + 1));
    m.markPrepared(m.protocol_generation_);
    zassert_true(m.snapshot().enable_generation > gen);
    zassert_ok(m.acceptDjiFeedback({.encoder = 2}, now + 100));
    zassert_false(m.snapshot().output_permitted);
    zassert_equal(m.snapshot().last_fault.reason, motor::FaultReason::FeedbackExpired);
}
ZTEST(sentry, test_chassis_executor_command_expiry_withdraws_motor_group) {
    motor::Motor s0(vehicle::steerHardware(0)), s1(vehicle::steerHardware(1)), s2(vehicle::steerHardware(2)),
        s3(vehicle::steerHardware(3));
    motor::Motor d0(vehicle::wheelHardware(0)), d1(vehicle::wheelHardware(1)), d2(vehicle::wheelHardware(2)),
        d3(vehicle::wheelHardware(3));
    std::array<motor::Motor *, 8> motors{&s0, &s1, &s2, &s3, &d0, &d1, &d2, &d3};
    motor::Group group(s0, s1, s2, s3, d0, d1, d2, d3);
    SwerveHardware hw(motors, group, vehicle::swerveHardwareConfig());
    ChassisExecutor e(hw, vehicle::swerveConfig(), vehicle::chassisExecutionConfig());
    zassert_ok(e.begin());
    auto time = static_cast<std::uint64_t>(k_uptime_get());
    if (!time) {
        k_sleep(K_MSEC(1));
        time = k_uptime_get();
    }
    for (auto *m : motors) {
        m->markStarted();
        zassert_ok(m->enable());
        m->snapshot_.feedback.timestamp_ms = time;
        m->snapshot_.feedback.valid = motor::FeedbackAbsolutePosition | motor::FeedbackVelocity;
        m->markPrepared(m->protocol_generation_);
    }
    ChassisExecutionInputs in{};
    in.command = {.mode = ChassisMode::BodyVelocity, .vx_m_s = .1f, .stamp = {1000, 1, true}};
    in.source_stamp = {1000000, 1, true};
    in.transport_ready = true;
    for (unsigned i = 0; i < 5; ++i)
        e.update(in, 1000000 + i * 2000);
    zassert_true(e.output().drive_permitted);
    const auto stopped = e.update(in, 1200000);
    zassert_equal(stopped.state, RunState::Disabled);
    zassert_equal(stopped.reason, WaitReason::Command);
    for (auto *m : motors)
        zassert_false(m->snapshot().enabled_requested);
}

namespace {
motor::dm::Config testBigYawMotor(bool native) {
    const motor::dm::J4310Options options{.id = 6,
                                          .master_id = 0x11,
                                          .position_max_rad = 12.5f,
                                          .velocity_max_rad_s = 30,
                                          .torque_max_nm = 10,
                                          .torque_limit_nm = 0.5f};
    return native ? motor::dm::j4310Velocity(options) : motor::dm::j4310Mit(options);
}
BigYawExecutionInputs freshBigYawInput(std::uint64_t now_ms) {
    BigYawExecutionInputs in{};
    in.local_boot_id = 17;
    in.peer_boot_id = 19;
    in.peer_online = in.transport_ready = true;
    in.request.receiver_boot_id = in.local_boot_id;
    in.request.mode = BigYawMode::FollowCenter;
    in.request.target_rate_rad_s = 0.8f;
    in.request.stamp = {now_ms, 1, true};
    in.request.permission = {true, true, {now_ms, 1, true}};
    return in;
}
}

ZTEST(sentry, test_native_big_yaw_requires_matching_motor_mode_and_confirmation) {
    BigYawExecutor::Config c{};
    c.native_velocity_mode_confirmed = true;
    c.rate_abs_max_rad_s = 1.0f;
    motor::Motor mit(testBigYawMotor(false));
    BigYawExecutor wrong_backend(mit, {}, c);
    zassert_equal(wrong_backend.begin(), -EINVAL);
    c.heading_lock_enabled = true;
    motor::Motor native(testBigYawMotor(true));
    BigYawExecutor conflicting_backend(native, {}, c);
    zassert_equal(conflicting_backend.begin(), -EINVAL);
    c.heading_lock_enabled = false;
    c.native_velocity_acceleration_rad_s2 = 0;
    BigYawExecutor invalid_slew(native, {}, c);
    zassert_equal(invalid_slew.begin(), -EINVAL);
}

ZTEST(sentry, test_native_big_yaw_slew_expiry_and_recovery) {
    motor::Motor drive(testBigYawMotor(true));
    BigYawExecutor::Config c{};
    c.native_velocity_mode_confirmed = true;
    c.native_velocity_acceleration_rad_s2 = 10.0f;
    c.motor_to_joint_ratio = 2.0f;
    c.direction = -1.0f;
    c.rate_abs_max_rad_s = 1.0f;
    BigYawExecutor executor(drive, {}, c);
    zassert_ok(executor.begin());
    auto in = freshBigYawInput(1000);
    // Seed cycle timing; first cycle may never stage a motor command.
    zassert_false(executor.update(in, 1000000).ready);
    drive.markStarted();
    zassert_ok(drive.enable());
    auto &view = drive.snapshot_;
    const auto now = k_uptime_get();
    view.feedback.timestamp_ms = now > 0 ? static_cast<std::uint64_t>(now) : 1U;
    view.feedback.valid = motor::FeedbackVelocity;
    view.feedback.velocity_rad_s = 0;
    view.state = motor::MotorState::Active;
    view.output_permitted = true;
    view.enable_generation = 4;
    in.request.stamp.timestamp_ms = 1002;
    in.request.permission.stamp.timestamp_ms = 1002;
    zassert_true(executor.update(in, 1002000).ready);
    const auto initial = drive.copyStaged();
    zassert_true(initial.valid);
    zassert_equal(initial.command.kind, motor::CommandKind::Velocity);
    // 10 motor rad/s² * 2 ms = 0.02; mounting sign is applied once.
    zassert_within(initial.command.primary, -0.02f, 1e-6f);
    // Loss of feedback revokes the staged velocity, not merely the telemetry.
    view.feedback.timestamp_ms = 0;
    in.request.stamp.timestamp_ms = 1004;
    in.request.permission.stamp.timestamp_ms = 1004;
    zassert_false(executor.update(in, 1004000).ready);
    zassert_false(drive.copyStaged().valid);
    zassert_false(drive.snapshot().enabled_requested);
    // A fresh drive generation cannot inherit an old slew target.
    zassert_ok(drive.enable());
    view.feedback.timestamp_ms = now > 0 ? static_cast<std::uint64_t>(now) : 1U;
    view.state = motor::MotorState::Active;
    view.output_permitted = true;
    view.enable_generation = 5;
    in.request.stamp.timestamp_ms = 1006;
    in.request.permission.stamp.timestamp_ms = 1006;
    zassert_true(executor.update(in, 1006000).ready);
    zassert_within(drive.copyStaged().command.primary, -0.02f, 1e-6f);
    // Permission has already been resolved by the sender. Its duplicate
    // wire field must not independently revoke an otherwise valid request.
    in.request.permission.enabled = false;
    zassert_true(executor.update(in, 1008000).ready);
    zassert_true(drive.copyStaged().valid);
}
namespace {
constexpr double twoPi = 6.2831853071795864769;
float wrapRad(float a) {
    return std::remainder(a, 2 * pi);
}
// Map a continuous native angle onto the J4310 feedback range [-PMAX, +PMAX).
float protocolRaw(float continuous_rad) {
    constexpr float span = 25.0f;
    float wrapped = std::fmod(continuous_rad + span / 2, span);
    if (wrapped < 0)
        wrapped += span;
    return wrapped - span / 2;
}
BigYawWorldHeadingInputs headingSample(float yaw, float rate, float desired, std::uint64_t now_ms, float body_rate = 0,
                                       std::uint64_t boot = 11, std::uint32_t generation = 1) {
    BigYawWorldHeadingInputs in{};
    in.head.valid = true;
    in.head.yaw_rad = yaw;
    in.head.yaw_rate_rad_s = rate;
    in.head.source_age_ms = 0;
    in.head.sample_sequence = 1;
    in.head.zero_generation = generation;
    in.head.receiver_boot_id = 17;
    in.head.sender_boot_id = boot;
    in.head.stamp = {now_ms, 1, true};
    in.now_ms = now_ms;
    in.body_rate_rad_s = body_rate;
    in.desired_world_rate_rad_s = desired;
    in.dt_s = 0.002f;
    return in;
}
BigYawWorldHeadingFollower::Config followerConfig() {
    return BigYawWorldHeadingFollower::Config{};
}
// testBigYawMotor uses CAN ID 6 and PMAX 12.5 rad.
motor::dm::DecodedFeedback dmFrame(float position_rad) {
    motor::dm::DecodedFeedback d{};
    d.raw.motor_id = 6;
    d.position_rad = position_rad;
    return d;
}
} // namespace

ZTEST(sentry, test_encoder_frame_zero_quadrants_and_spin_scaling) {
    constexpr double zero = -1.132792, halfPi = 1.5707963267948966;
    ChassisCommand c{.mode = ChassisMode::BodyVelocity, .vx_m_s = 1.0f};
    auto out = referenceV4ChassisToBody(c, true, zero, zero, -1, 1);
    zassert_within(out.vx_m_s, 1, 1e-6f);
    zassert_within(out.vy_m_s, 0, 1e-6f);
    // Joint rotated +90 deg from the body: forward becomes body +Y.
    out = referenceV4ChassisToBody(c, true, zero - halfPi, zero, -1, 1);
    zassert_within(out.vx_m_s, 0, 1e-6f);
    zassert_within(out.vy_m_s, 1, 1e-6f);
    out = referenceV4ChassisToBody(c, true, zero + halfPi, zero, -1, 1);
    zassert_within(out.vy_m_s, -1, 1e-6f);
    // direction=+1 flips the mechanical sign exactly once.
    out = referenceV4ChassisToBody(c, true, zero + halfPi, zero, 1, 1);
    zassert_within(out.vy_m_s, 1, 1e-6f);
    // gear_ratio divides the joint angle before wrapping.
    out = referenceV4ChassisToBody(c, true, zero + 2 * halfPi, zero, -1, 2);
    zassert_within(out.vx_m_s, 0, 1e-6f);
    zassert_within(out.vy_m_s, -1, 1e-6f);
    // Spin scaling changes magnitude only; the rotation command is untouched.
    c.wz_rad_s = 0.5f;
    out = referenceV4ChassisToBody(c, true, zero, zero, -1, 1);
    zassert_within(out.vx_m_s, .3f, 1e-6f);
    zassert_equal(out.wz_rad_s, .5f);
    c.vx_m_s = 0;
    c.vy_m_s = 1;
    out = referenceV4ChassisToBody(c, true, zero, zero, -1, 1);
    zassert_within(out.vy_m_s, .4f, 1e-6f);
    zassert_equal(out.wz_rad_s, .5f);
    // An invalid encoder zeroes translation but preserves the spin command.
    out = referenceV4ChassisToBody(c, false, zero, zero, -1, 1);
    zassert_equal(out.vx_m_s, 0);
    zassert_equal(out.vy_m_s, 0);
    zassert_equal(out.wz_rad_s, .5f);
}

ZTEST(sentry, test_dm_native_unwrap_stays_continuous_across_25_rad_spans) {
    motor::Motor drive(testBigYawMotor(true));
    constexpr double zero = -1.132792;
    std::uint64_t stamp = 1, order = 1;
    double previous_unwrapped = 0;
    float previous_new_theta = 0, previous_old_theta = 0;
    float worst_new_step = 0, worst_old_step = 0;
    bool first = true;
    // 80 rad of continuous travel, crossing both protocol boundaries twice per span.
    for (int i = 0; i <= 1600; ++i) {
        const float continuous = -40.0f + 0.05f * i;
        zassert_ok(drive.acceptDmFeedback(dmFrame(protocolRaw(continuous)), stamp++, order++));
        const auto view = drive.snapshot();
        zassert_true(view.native_unwrapped_position_valid);
        const double unwrapped = view.native_unwrapped_position_rad;
        // reference_big_yaw: direction=-1, ratio=1, forward zero at `zero`.
        const float new_theta = static_cast<float>(std::remainder(-(unwrapped - zero), twoPi));
        const float old_theta = static_cast<float>(
            std::remainder(-(static_cast<double>(view.native_position_rad) - zero), twoPi));
        if (!first) {
            zassert_within(unwrapped - previous_unwrapped, 0.05, 1e-3, "index %d", i);
            worst_new_step = std::max(worst_new_step, std::fabs(wrapRad(new_theta - previous_new_theta)));
            worst_old_step = std::max(worst_old_step, std::fabs(wrapRad(old_theta - previous_old_theta)));
        }
        first = false;
        previous_unwrapped = unwrapped;
        previous_new_theta = new_theta;
        previous_old_theta = old_theta;
    }
    // One 25 rad span is |25 - 4*2*pi| = 0.1327 rad in direction space; the
    // unwrapped coordinate keeps the per-step change at the commanded 0.05 rad.
    zassert_true(worst_new_step < 0.06f, "new step %f", static_cast<double>(worst_new_step));
    zassert_true(worst_old_step > 0.12f, "raw step %f", static_cast<double>(worst_old_step));
    zassert_true(drive.snapshot().native_wrap_count >= 2);
}

ZTEST(sentry, test_dm_native_unwrap_ignores_invalid_and_stale_frames) {
    motor::Motor drive(testBigYawMotor(true));
    zassert_ok(drive.acceptDmFeedback(dmFrame(1.0f), 10, 1));
    const auto baseline = drive.snapshot();
    zassert_true(baseline.native_unwrapped_position_valid);
    zassert_equal(drive.acceptDmFeedback(dmFrame(NAN), 11, 2), -EINVAL);
    auto bad_velocity = dmFrame(2.0f);
    bad_velocity.velocity_rad_s = INFINITY;
    zassert_equal(drive.acceptDmFeedback(bad_velocity, 12, 3), -EINVAL);
    // An older timestamp or a repeated callback order must not move history.
    zassert_equal(drive.acceptDmFeedback(dmFrame(-12.4f), 9, 4), -ESTALE);
    zassert_equal(drive.acceptDmFeedback(dmFrame(-12.4f), 13, 1), -ESTALE);
    const auto after = drive.snapshot();
    zassert_equal(after.native_unwrapped_position_rad, baseline.native_unwrapped_position_rad);
    zassert_equal(after.native_wrap_count, baseline.native_wrap_count);
    zassert_equal(after.native_position_rad, baseline.native_position_rad);
    // The next accepted frame continues from the last angle, not from zero.
    zassert_ok(drive.acceptDmFeedback(dmFrame(-5.0f), 14, 2));
    zassert_equal(drive.snapshot().native_wrap_count, baseline.native_wrap_count);
    zassert_within(drive.snapshot().native_unwrapped_position_rad, -5.0, 1e-5);
    zassert_within(drive.snapshot().native_position_rad, -5.0f, 1e-6f);
}

ZTEST(sentry, test_dm_native_unwrap_is_independent_of_position_reseed) {
    motor::Motor drive(testBigYawMotor(true));
    auto now = static_cast<std::uint64_t>(k_uptime_get());
    if (!now) {
        k_sleep(K_MSEC(1));
        now = k_uptime_get();
    }
    zassert_ok(drive.acceptDmFeedback(dmFrame(3.0f), now, 1));
    zassert_within(drive.snapshot().native_unwrapped_position_rad, 3.0, 1e-6);
    zassert_ok(drive.reseedPosition(100.0));
    // Reseeding owns the relative Feedback::position_rad only.
    zassert_within(drive.snapshot().feedback.position_rad, 100.0f, 1e-6f);
    zassert_within(drive.snapshot().native_position_rad, 3.0f, 1e-6f);
    zassert_within(drive.snapshot().native_unwrapped_position_rad, 3.0, 1e-6);
    zassert_ok(drive.acceptDmFeedback(dmFrame(3.5f), now + 1, 2));
    zassert_within(drive.snapshot().native_unwrapped_position_rad, 3.5, 1e-6);
    zassert_within(drive.snapshot().feedback.position_rad, 100.5f, 1e-5f);
}

ZTEST(sentry, test_follower_capture_feedforward_and_heading_sign) {
    BigYawWorldHeadingFollower follower(followerConfig());
    zassert_ok(follower.validate());
    float out = 1;
    // The first usable sample only captures the measured heading.
    zassert_ok(follower.step(headingSample(0, 0, 0, 1000), out));
    zassert_equal(out, 0);
    zassert_true(follower.initialized());
    zassert_ok(follower.step(headingSample(0, 0, 0, 1002), out));
    zassert_within(out, 0, 1e-6f);
    // A rotating body needs the opposite joint rate to hold the world heading.
    zassert_ok(follower.step(headingSample(0, 0, 0, 1004, .5f), out));
    zassert_within(out, -.5f, 1e-3f);
    // Measured heading above the captured target commands a negative rate.
    zassert_ok(follower.step(headingSample(.1f, 0, 0, 1006), out));
    zassert_true(out < -.1f, "out %f", static_cast<double>(out));
    // World intent and an opposing body rate add in the same direction.
    follower.reset();
    zassert_ok(follower.step(headingSample(0, 0, 1.0f, 1010), out));
    zassert_equal(out, 0);
    zassert_ok(follower.step(headingSample(.002f, 1.0f, 1.0f, 1012, -.4f), out));
    zassert_within(out, 1.4f, .01f);
}

ZTEST(sentry, test_follower_holds_world_heading_against_body_bias) {
    BigYawWorldHeadingFollower follower(followerConfig());
    constexpr float dt = .002f, body_bias = .03f, plant_gain = 1.1f;
    constexpr unsigned delay_steps = 10; // 20 ms at a 2 ms cycle
    std::array<float, delay_steps> pipeline{};
    float heading = 0, joint = 0, out = 0, peak = 0;
    std::uint64_t now_ms = 5000;
    // Two minutes: body gyro bias, 10% velocity gain error and 20 ms delay.
    for (unsigned i = 0; i < 60000; ++i) {
        const float delayed = pipeline[delay_steps - 1];
        for (unsigned k = delay_steps - 1; k > 0; --k)
            pipeline[k] = pipeline[k - 1];
        pipeline[0] = joint;
        const float head_rate = plant_gain * delayed + body_bias;
        heading = wrapRad(heading + head_rate * dt);
        zassert_ok(follower.step(headingSample(heading, head_rate, 0, now_ms, body_bias), out));
        joint = out;
        if (i > 5000)
            peak = std::max(peak, std::fabs(heading));
        now_ms += 2;
    }
    zassert_true(std::fabs(heading) < .01f, "heading %f", static_cast<double>(heading));
    zassert_true(peak < .01f, "peak %f", static_cast<double>(peak));
    // The integral absorbs the plant gain error: -0.03 / 1.1.
    zassert_within(joint, -body_bias / plant_gain, 5e-3f);
}

ZTEST(sentry, test_follower_pi_wrap_rotation_and_release_capture) {
    BigYawWorldHeadingFollower follower(followerConfig());
    float out = 0;
    // Continuous rotation across +/-pi must not flip the correction sign.
    float yaw = 3.10f;
    zassert_ok(follower.step(headingSample(yaw, 1.0f, 1.0f, 6000), out));
    zassert_equal(out, 0);
    float worst = 0;
    for (unsigned i = 1; i <= 100; ++i) {
        yaw = wrapRad(yaw + .002f);
        zassert_ok(follower.step(headingSample(yaw, 1.0f, 1.0f, 6000 + 2 * i), out));
        worst = std::max(worst, std::fabs(out - 1.0f));
    }
    zassert_true(worst < .05f, "tracking error %f", static_cast<double>(worst));
    // Releasing the stick captures the current heading instead of chasing it.
    follower.reset();
    zassert_ok(follower.step(headingSample(.50f, 0, 1.0f, 7000), out));
    zassert_ok(follower.step(headingSample(.52f, 0, 1.0f, 7002), out));
    zassert_true(out > .5f);
    zassert_ok(follower.step(headingSample(.60f, 0, 0, 7004), out));
    zassert_within(follower.headingErrorRad(), 0, 1e-6f);
    zassert_within(follower.rateIntegralRadS(), 0, 1e-6f);
    zassert_within(out, 0, 1e-6f);
}

ZTEST(sentry, test_follower_rejects_unusable_sample_and_recaptures) {
    BigYawWorldHeadingFollower follower(followerConfig());
    float out = 1;
    auto in = headingSample(0, 0, 0, 8000);
    in.head.valid = false;
    zassert_equal(follower.step(in, out), -ESTALE);
    zassert_equal(out, 0);
    zassert_false(follower.initialized());
    in = headingSample(0, 0, 0, 8000);
    in.head.stamp.timestamp_ms = 7900;
    zassert_equal(follower.step(in, out), -ESTALE);
    in = headingSample(0, 0, 0, 8000, 0, 11, 0);
    zassert_equal(follower.step(in, out), -ESTALE);
    in = headingSample(NAN, 0, 0, 8000);
    zassert_equal(follower.step(in, out), -ESTALE);
    in = headingSample(0, 0, 0, 8000);
    in.body_rate_rad_s = NAN;
    zassert_equal(follower.step(in, out), -ESTALE);
    in = headingSample(0, 0, 0, 8000);
    in.dt_s = -1;
    zassert_equal(follower.step(in, out), -ESTALE);
    // A usable sample recaptures instead of stepping from the failed session.
    zassert_ok(follower.step(headingSample(.3f, 0, 0, 8002), out));
    zassert_equal(out, 0);
    zassert_true(follower.initialized());
    // Gimbal software-zero regeneration and a gimbal reboot recapture too.
    zassert_ok(follower.step(headingSample(.5f, 0, 0, 8004, 0, 11, 2), out));
    zassert_equal(out, 0);
    zassert_ok(follower.step(headingSample(.5f, 0, 0, 8006, 0, 12, 2), out));
    zassert_equal(out, 0);
}

ZTEST(sentry, test_follower_anti_windup_at_output_limit) {
    BigYawWorldHeadingFollower follower(followerConfig());
    float out = 0;
    zassert_ok(follower.step(headingSample(0, 0, 0, 2000), out));
    zassert_ok(follower.step(headingSample(0, 0, 0, 2002, 20.0f), out));
    zassert_within(out, -10.0f, 1e-5f);
    for (unsigned i = 0; i < 100; ++i) {
        zassert_ok(follower.step(headingSample(0, 0, 0, 2004 + 2 * i, 20.0f), out));
        zassert_within(out, -10.0f, 1e-5f);
    }
    zassert_within(follower.rateIntegralRadS(), 0, 1e-6f);
}

ZTEST(sentry, test_follower_rate_reaches_native_motor_sign_and_ratio_once) {
    motor::Motor drive(testBigYawMotor(true));
    BigYawExecutor::Config c{};
    c.native_velocity_mode_confirmed = true;
    c.native_velocity_acceleration_rad_s2 = 1000.0f;
    c.motor_to_joint_ratio = 2.0f;
    c.direction = -1.0f;
    c.rate_abs_max_rad_s = 10.0f;
    BigYawExecutor executor(drive, {}, c);
    zassert_ok(executor.begin());
    auto in = freshBigYawInput(1000);
    // Seed cycle timing; the first update never stages a motor command.
    zassert_false(executor.update(in, 1000000).ready);
    drive.markStarted();
    zassert_ok(drive.enable());
    auto &view = drive.snapshot_;
    const auto now = k_uptime_get();
    view.feedback.timestamp_ms = now > 0 ? static_cast<std::uint64_t>(now) : 1U;
    view.feedback.valid = motor::FeedbackVelocity;
    view.feedback.velocity_rad_s = 0;
    view.state = motor::MotorState::Active;
    view.output_permitted = true;
    view.enable_generation = 4;
    BigYawWorldHeadingFollower follower(followerConfig());
    float joint = 0;
    zassert_ok(follower.step(headingSample(0, 0, 0, 1002), joint));
    zassert_ok(follower.step(headingSample(0, 0, 0, 1004, .5f), joint));
    zassert_within(joint, -.5f, 1e-3f);
    in.request.target_rate_rad_s = joint;
    in.request.stamp.timestamp_ms = 1004;
    in.request.permission.stamp.timestamp_ms = 1004;
    zassert_true(executor.update(in, 1004000).ready);
    // The executor applies direction=-1 and ratio=2 exactly once.
    zassert_equal(drive.copyStaged().command.kind, motor::CommandKind::Velocity);
    zassert_within(drive.copyStaged().command.primary, 1.0f, 1e-3f);
}

ZTEST_SUITE(sentry, nullptr, nullptr, nullptr, nullptr, nullptr);
