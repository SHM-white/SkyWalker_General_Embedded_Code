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
        for (auto &m : f.module) ++m.steer_feedback_stamp_ms;
        zassert_ok(c.step(cmd, f, .002f, o));
    }
}
}
ZTEST(sentry, test_candidate_mapping_and_real_motor_sign_units) {
    constexpr unsigned ids[] = {1,4,2,3}, zeros[] = {3060,986,2383,4390};
    for (unsigned i = 0; i < 4; ++i) {
        const auto &sc = vehicle::reference_steer[i], &wc = vehicle::reference_wheel[i];
        zassert_equal(sc.id, ids[i]); zassert_equal(wc.id, ids[i]);
        zassert_equal(sc.encoder_zero_ticks, zeros[i]);
        zassert_equal(sc.direction, -1); zassert_equal(wc.direction, -1);
        motor::Motor s(motor::dji::gm6020({.id=sc.id, .encoder_zero_ticks=sc.encoder_zero_ticks,
                                          .current_mode_confirmed=true}));
        zassert_ok(s.acceptDjiFeedback({.encoder=sc.encoder_zero_ticks}, 1));
        zassert_within(s.snapshot().feedback.absolute_position_rad, 0, 1e-6f);
        zassert_ok(s.acceptDjiFeedback({.encoder=static_cast<std::uint16_t>((zeros[i]+1024)%8192), .speed_rpm=60}, 2));
        zassert_within(sc.direction * s.snapshot().feedback.absolute_position_rad, -pi/4, 1e-5f);
        motor::Motor d(motor::dji::m3508({.id=wc.id,.gear_ratio=wc.gear_ratio}));
        zassert_ok(d.acceptDjiFeedback({.encoder=0,.speed_rpm=60}, 1));
        zassert_within(d.snapshot().feedback.velocity_rad_s, 2*pi/wc.gear_ratio, 1e-6f);
    }
    motor::Motor s0(vehicle::steerHardware(0)),s1(vehicle::steerHardware(1)),s2(vehicle::steerHardware(2)),s3(vehicle::steerHardware(3));
    motor::Motor d0(vehicle::wheelHardware(0)),d1(vehicle::wheelHardware(1)),d2(vehicle::wheelHardware(2)),d3(vehicle::wheelHardware(3));
    std::array<motor::Motor*,8> motors{&s0,&s1,&s2,&s3,&d0,&d1,&d2,&d3};
    motor::Group group(s0,s1,s2,s3,d0,d1,d2,d3);
    auto hc = vehicle::swerveHardwareConfig(); hc.directions.fill(-1);
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
    ChassisFeedback f{}; zassert_ok(hw.read(f));
    for (auto &m : f.module) { zassert_within(m.steer_absolute_rad,-.2f,1e-6f); zassert_within(m.drive_velocity_rad_s,-.3f,1e-6f); }
    for (auto &m : o.module) { m.steer_output_valid=m.drive_output_valid=true; m.steer_effort=.1f; m.drive_effort=.2f; }
    zassert_ok(hw.stage(o,1));
    zassert_within(s0.copyStaged().command.primary,-.1f,1e-6f);
    zassert_within(d0.copyStaged().command.primary,-.2f,1e-6f);
}
ZTEST(sentry, test_final_angle_interlock_hysteresis_and_pid_recovery) {
    SwerveChassis c(config()); auto f=feedback(); ChassisOutput o{};
    ChassisCommand cmd{.mode=ChassisMode::BodyVelocity,.vy_m_s=.5f};
    ticks(c,cmd,f,o,20);
    zassert_false(o.drive_permitted);
    zassert_true(o.module[0].steer_reference_rad < .2f); // Ramp is near feedback, final error is pi/2.
    for (auto &m:f.module) m.steer_absolute_rad=pi/2-.09f;
    ticks(c,cmd,f,o); zassert_true(o.drive_permitted); zassert_true(o.module[0].drive_effort>0);
    for (auto &m:f.module) m.steer_absolute_rad=pi/2-.3f;
    ticks(c,cmd,f,o); zassert_true(o.drive_permitted);
    for (auto &m:f.module) m.steer_absolute_rad=pi/2-.41f;
    ticks(c,cmd,f,o,500); zassert_false(o.drive_permitted);
    for(auto &m:o.module) { zassert_equal(m.drive_effort,0); zassert_equal(m.drive_target_rad_s,0); }
    for (auto &m:f.module) m.steer_absolute_rad=pi/2;
    ticks(c,cmd,f,o,1); zassert_equal(o.module[0].drive_effort,0);
    ticks(c,cmd,f,o,1); zassert_true(o.module[0].drive_effort < .1f); // Rebuilt slew/integral.
    f.module[2].steer_valid=false; ticks(c,cmd,f,o); zassert_false(o.drive_permitted);
    for(auto &m:o.module) zassert_equal(m.drive_effort,0);
    f.module[2].steer_valid=true; ticks(c,cmd,f,o); zassert_true(o.drive_permitted);
    f.module[1].drive_valid=false; ticks(c,cmd,f,o); zassert_false(o.drive_permitted);
    f.module[1].drive_valid=true; ++f.module[1].drive_enable_generation;
    ticks(c,cmd,f,o,1); zassert_false(o.drive_permitted);
    ticks(c,cmd,f,o); zassert_true(o.drive_permitted);
}
ZTEST(sentry, test_cosine_reverse_flip_hysteresis_zero_and_invalid) {
    SwerveModule m(config().modules[0]); auto f=feedback().module[0]; ModuleOutput o{};
    for(float a : {0.f,.3f,pi/3,pi/2,1.6f,pi, -pi}) {
        zassert_ok(m.reset(f));
        zassert_ok(m.step({a,.5f},f,.002f,o));
        const float expected=std::pow(std::max(std::cos(o.alignment_error_rad),0.f),3.0f);
        zassert_within(o.cosine_scale,expected,1e-6f);
        zassert_within(o.drive_target_rad_s,o.optimized_wheel_velocity_m_s/.1f*expected,1e-5f);
    }
    zassert_true(o.flipped); zassert_true(o.drive_target_rad_s<0);
    zassert_ok(m.step({pi/2+.02f,.5f},f,.002f,o)); zassert_true(o.flipped);
    zassert_ok(m.step({pi/2-.02f,.5f},f,.002f,o)); zassert_true(o.flipped);
    zassert_ok(m.step({1.4f,.5f},f,.002f,o)); zassert_false(o.flipped);
    zassert_ok(m.step({1.4f,0},f,.002f,o)); zassert_equal(o.drive_target_rad_s,0);
    f.steer_valid=false; zassert_ok(m.step({0,.5f},f,.002f,o)); zassert_equal(o.drive_effort,0);
    zassert_equal(m.step({NAN,.5f},f,.002f,o),-EINVAL); zassert_equal(o.drive_effort,0);
}
ZTEST(sentry, test_rotation_start_alignment_continuity_and_exit) {
    for (bool mixed : {false,true}) {
        SwerveChassis c(config()); auto f=feedback(); ChassisOutput o{};
        ChassisCommand cmd{.mode=mixed?ChassisMode::BodyVelocity:ChassisMode::Spin,.vx_m_s=mixed?.1f:0,.wz_rad_s=.8f};
        ticks(c,cmd,f,o); zassert_false(o.drive_permitted);
        for(unsigned i=0;i<4;++i) f.module[i].steer_absolute_rad=o.module[i].optimized_angle_rad;
        ticks(c,cmd,f,o); zassert_true(o.drive_permitted);
        for(unsigned i=0;i<4;++i) f.module[i].steer_absolute_rad=o.module[i].optimized_angle_rad-.6f;
        ticks(c,cmd,f,o,20); zassert_true(o.drive_permitted);
        for(auto &m:o.module) { zassert_true(std::fabs(m.drive_target_rad_s)>0); zassert_true(m.cosine_scale<1); }
        f.module[0].steer_valid=false; ticks(c,cmd,f,o); zassert_false(o.drive_permitted);
        f.module[0].steer_valid=true; ticks(c,cmd,f,o); zassert_false(o.drive_permitted);
        for(unsigned i=0;i<4;++i) f.module[i].steer_absolute_rad=o.module[i].optimized_angle_rad;
        ticks(c,cmd,f,o); zassert_true(o.drive_permitted);
        cmd.wz_rad_s=0; cmd.mode=ChassisMode::BodyVelocity; cmd.vy_m_s=.5f;
        ticks(c,cmd,f,o); zassert_false(o.drive_permitted);
        ticks(c,{},f,o); zassert_false(o.drive_permitted);
    }
}
namespace {
can_state fake_state=CAN_STATE_ERROR_ACTIVE;
int fakeStart(const device*) { fake_state=CAN_STATE_ERROR_ACTIVE; return 0; }
int fakeStop(const device*) { fake_state=CAN_STATE_STOPPED; return 0; }
int fakeState(const device*,can_state *s,can_bus_err_cnt *e) { if(s)*s=fake_state; if(e)*e={}; return 0; }
static DEVICE_API(can, fake_api) = { .start=fakeStart, .stop=fakeStop, .get_state=fakeState };
static device fake_can{.name="sentry-test-can",.api=&fake_api};
}
ZTEST(sentry, test_can_failure_and_feedback_timeout_with_real_recovery) {
    motor::Motor m(vehicle::wheelHardware(0)); motor::CanBus bus(&fake_can);
    zassert_ok(bus.attach(m)); m.markStarted(); zassert_ok(m.enable());
    auto now=static_cast<std::uint64_t>(k_uptime_get()); if (!now) { k_sleep(K_MSEC(1)); now=k_uptime_get(); }
    zassert_ok(m.acceptDjiFeedback({.encoder=0},now));
    m.markPrepared(m.protocol_generation_);
    const auto gen=m.snapshot().enable_generation;
    zassert_true(m.active());
    fake_state=CAN_STATE_BUS_OFF; bus.controller_started_=true;
    bus.enterRecovery(-ENETUNREACH,motor::FaultReason::TransportError);
    zassert_equal(bus.status().state,motor::BusState::Recovering); zassert_false(m.active());
    zassert_equal(m.snapshot().last_fault.reason,motor::FaultReason::TransportError);
    bus.recoverController(UINT64_MAX); bus.recoverController(UINT64_MAX);
    zassert_equal(bus.status().state,motor::BusState::Running);
    zassert_false(m.active()); // Controller recovery alone cannot restore motor authority.
    zassert_ok(m.acceptDjiFeedback({.encoder=1},now+1)); m.markPrepared(m.protocol_generation_);
    zassert_true(m.snapshot().enable_generation>gen);
    zassert_ok(m.acceptDjiFeedback({.encoder=2},now+100));
    zassert_false(m.snapshot().output_permitted);
    zassert_equal(m.snapshot().last_fault.reason,motor::FaultReason::FeedbackExpired);
}
ZTEST(sentry, test_chassis_executor_command_expiry_withdraws_motor_group) {
    motor::Motor s0(vehicle::steerHardware(0)),s1(vehicle::steerHardware(1)),s2(vehicle::steerHardware(2)),s3(vehicle::steerHardware(3));
    motor::Motor d0(vehicle::wheelHardware(0)),d1(vehicle::wheelHardware(1)),d2(vehicle::wheelHardware(2)),d3(vehicle::wheelHardware(3));
    std::array<motor::Motor*,8> motors{&s0,&s1,&s2,&s3,&d0,&d1,&d2,&d3};
    motor::Group group(s0,s1,s2,s3,d0,d1,d2,d3);
    SwerveHardware hw(motors,group,vehicle::swerveHardwareConfig());
    ChassisExecutor e(hw,vehicle::swerveConfig(),vehicle::chassisExecutionConfig());
    zassert_ok(e.begin());
    auto time=static_cast<std::uint64_t>(k_uptime_get());if(!time){ k_sleep(K_MSEC(1));time=k_uptime_get(); }
    for(auto *m:motors) {
        m->markStarted();zassert_ok(m->enable());
        m->snapshot_.feedback.timestamp_ms=time;
        m->snapshot_.feedback.valid=motor::FeedbackAbsolutePosition|motor::FeedbackVelocity;
        m->markPrepared(m->protocol_generation_);
    }
    ChassisExecutionInputs in{};in.command={.mode=ChassisMode::BodyVelocity,.vx_m_s=.1f,.stamp={1000,1,true}};
    in.source_stamp={1000000,1,true};in.transport_ready=true;
    for(unsigned i=0;i<5;++i)e.update(in,1000000+i*2000);
    zassert_true(e.output().drive_permitted);
    const auto stopped=e.update(in,1200000);
    zassert_equal(stopped.state,RunState::Disabled);zassert_equal(stopped.reason,WaitReason::Command);
    for(auto *m:motors)zassert_false(m->snapshot().enabled_requested);
}
ZTEST_SUITE(sentry,nullptr,nullptr,nullptr,nullptr,nullptr);
