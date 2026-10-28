#include <zephyr/ztest.h>
#include <algorithm>
#include <array>
#include <variant>
#include <cmath>
#include <limits>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#define private public
#include <drivers/motor/can_bus.hpp>
#undef private
#include <robotics/chassis/imu_heading.hpp>
#include <robotics/execution/big_yaw_executor.hpp>
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
constexpr float pi=3.14159265358979323846f;
GimbalImuYaw head(std::uint64_t now=100) {
    return {.valid=true,.yaw_rad=0,.yaw_rate_rad_s=0,.source_age_ms=0,.sample_sequence=1,
            .zero_generation=1,.receiver_boot_id=22,.sender_boot_id=11,.stamp={now,1,true}};
}
LocalHeading body(std::uint64_t now=100) { return {0,0,{now*1000,1,true},1,true}; }
imu::Snapshot imuSample(std::uint64_t seq, std::uint64_t now,float yaw=0) {
    imu::Snapshot s{}; s.state=imu::State::Running; s.fresh_mask=imu::Orientation|imu::Gyro;
    s.sample.orientation={core::fromEuler({0,0,yaw}),{now,seq,true}};
    s.sample.gyro_rad_s={{},{now,seq,true}};
    s.sample.reference={3,1}; s.sample.attitude_quality=imu::AttitudeQuality::Tracking; return s;
}
}
ZTEST(heading, test_zero_capture_gap_recovery_and_reference_change) {
    ImuHeadingReference r; LocalHeading h{};
    for(unsigned i=1;i<=19;++i) { h=r.update(imuSample(i,1000000+i*2000,2),true,1000000+i*2000); zassert_false(h.valid); }
    h=r.update(imuSample(20,1040000,2),true,1040000); zassert_true(h.valid); zassert_within(h.yaw_rad,0,1e-6f);
    h=r.update(imuSample(21,1042000,2.2f),false,1042000); zassert_within(h.yaw_rad,.2f,1e-5f);
    h=r.update({},false,1100000); zassert_false(h.valid);
    h=r.update(imuSample(22,1102000,2.3f),false,1102000); zassert_within(h.yaw_rad,.3f,1e-5f);
    auto changed=imuSample(23,1104000,2.3f); changed.sample.reference.epoch=2;
    h=r.update(changed,true,1104000); zassert_false(h.valid); zassert_equal(h.generation,2);
    for(unsigned i=24;i<50;++i) { changed=imuSample(i,1104000+i*2000); h=r.update(changed,true,1104000+i*2000); zassert_false(h.valid); }
}
ZTEST(heading, test_frames_prediction_mixed_motion_and_fail_closed) {
    ChassisMotionAdapter::Config c{}; c.frame=ChassisMotionAdapter::Frame::Gimbal;
    c.startup_alignment_confirmed=true; c.compensation_enabled=true;
    ChassisMotionAdapter a(c); auto b=body(); auto h=head(); h.yaw_rad=pi/2;
    ChassisCommand cmd{.mode=ChassisMode::BodyVelocity,.vx_m_s=1,.wz_rad_s=.8f};
    auto o=a.update(cmd,b,h,22,true,100); zassert_true(a.translationReady());
    zassert_within(o.vx_m_s,0,1e-6f); zassert_within(o.vy_m_s,.3f,1e-6f);
    b.rate_rad_s=2; o=a.update(cmd,b,h,22,false,100);
    zassert_within(o.vx_m_s,.3f*std::sin(.014f),1e-6f);
    h.valid=false; o=a.update(cmd,b,h,22,false,100); zassert_equal(o.vx_m_s,0); zassert_equal(o.vy_m_s,0); zassert_equal(o.wz_rad_s,.8f);
    h.valid=true; o=a.update(cmd,b,h,22,false,100); zassert_true(a.translationReady());
    h.sender_boot_id=33; o=a.update(cmd,b,h,22,true,100); zassert_false(a.translationReady());
    h.sender_boot_id=11; o=a.update(cmd,b,h,22,true,100); zassert_false(a.translationReady());
    c.frame=ChassisMotionAdapter::Frame::World; ChassisMotionAdapter world(c); b.yaw_rad=pi/2;b.rate_rad_s=0;
    o=world.update(cmd,b,{},22,true,100); zassert_within(o.vy_m_s,-.3f,1e-6f);
    c.frame=ChassisMotionAdapter::Frame::Body;c.compensation_enabled=false;ChassisMotionAdapter plain(c);
    o=plain.update(cmd,{}, {},0,false,100);zassert_equal(o.vx_m_s,1);zassert_equal(o.wz_rad_s,.8f);
}
ZTEST(heading, test_world_heading_crossing_release_saturation_and_continuous_rotation) {
    ImuHeadingLock lock({}); auto h=head(); float torque=123;
    h.yaw_rad=pi-.001f; zassert_ok(lock.step(h,1,.002f,100,torque)); zassert_equal(torque,0);
    h.yaw_rad=-pi+.001f; zassert_ok(lock.step(h,1,.002f,100,torque)); zassert_true(std::fabs(torque)<.1f);
    for(unsigned i=0;i<10000;++i) { h.yaw_rad=std::remainder(h.yaw_rad+.002f,2*pi);h.yaw_rate_rad_s=1;zassert_ok(lock.step(h,1,.002f,100,torque));zassert_true(std::fabs(torque)<=.5f); }
    h.yaw_rate_rad_s=0;zassert_ok(lock.step(h,0,.002f,100,torque));zassert_within(lock.targetRad(),h.yaw_rad,1e-6f);
    h.valid=false; zassert_equal(lock.step(h,0,.002f,100,torque),-ESTALE);zassert_equal(torque,0);
    h.valid=true;h.yaw_rad=1;zassert_ok(lock.step(h,0,.002f,100,torque));zassert_equal(torque,0);zassert_within(lock.targetRad(),1,1e-6f);
    ++h.zero_generation;zassert_ok(lock.step(h,0,.002f,100,torque));zassert_equal(torque,0);
    zassert_equal(lock.step(h,0,.002f,151,torque),-ESTALE);zassert_equal(torque,0);
}
ZTEST(heading, test_big_yaw_executor_mit_authority_timeout_and_recovery) {
    auto cfg=motor::dm::j4310Mit({.id=2,.master_id=0x12,.torque_limit_nm=.5f});
    motor::Motor drive(cfg);
    control::VelocityMotor::Config motor_cfg{};
    BigYawExecutor::Config c{}; c.heading_lock_enabled=true;c.mit_mode_confirmed=true;c.heading_reference_confirmed=true;
    BigYawExecutor e(drive,motor_cfg,c);zassert_ok(e.begin());
    drive.markStarted();
    BigYawExecutionInputs in{};in.peer_online=in.transport_ready=true;in.local_boot_id=22;in.peer_boot_id=11;in.head=head();
    in.request.mode=BigYawMode::ImuHeadingHold;in.request.receiver_boot_id=22;in.request.target_rate_rad_s=.5f;
    in.request.source_age_ms=in.request.command_age_ms=0;in.request.stamp={100,1,true};in.request.permission={true,true,{100,1,true}};
    e.update(in,100000);
    drive.snapshot_.state=motor::MotorState::Active;drive.snapshot_.feedback.timestamp_ms=static_cast<std::uint64_t>(k_uptime_get());
    if(!drive.snapshot_.feedback.timestamp_ms) { k_sleep(K_MSEC(1));drive.snapshot_.feedback.timestamp_ms=k_uptime_get(); }
    drive.snapshot_.feedback.valid=motor::FeedbackVelocity;drive.snapshot_.output_permitted=true;
    drive.snapshot_.enable_generation=1;
    zassert_equal(e.update(in,102000).state,RunState::Active);zassert_equal(drive.copyStaged().command.kind,motor::CommandKind::Torque);
    zassert_equal(drive.copyStaged().command.primary,0);
    zassert_equal(e.update(in,104000).state,RunState::Active);zassert_true(drive.copyStaged().command.primary>0);
    in.head.valid=false;zassert_equal(e.update(in,106000).state,RunState::Disabled);zassert_false(drive.snapshot().enabled_requested);
    in.head.valid=true;in.request.permission.enabled=false;zassert_equal(e.update(in,108000).reason,WaitReason::Power);
    in.request.permission.enabled=true;in.request.receiver_boot_id=21;zassert_equal(e.update(in,110000).reason,WaitReason::Transport);
    in.request.receiver_boot_id=22;in.request.stamp.timestamp_ms=0;zassert_equal(e.update(in,112000).reason,WaitReason::Command);
}
ZTEST_SUITE(heading,nullptr,nullptr,nullptr,nullptr,nullptr);
