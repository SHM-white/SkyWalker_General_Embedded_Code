#include <cerrno>
#include <cmath>
#include <cstdint>
#include <limits>
#include <drivers/motor/motor.hpp>
#include <drivers/motor/group.hpp>
#include <drivers/motor/can_bus.hpp>
#include <zephyr/ztest.h>

using namespace skywalker::motor;
namespace {
dji::Config djiConfig(unsigned id) {
    return dji::m3508({.id=static_cast<std::uint8_t>(id),.current_limit_a=2.0f,
                       .gear_ratio=1.0f,.timing={20,20,5,100}});
}
dm::Config dmConfig() {
    return dm::j4310Mit({.id=1,.master_id=0x11,.position_max_rad=12.5f,
                         .velocity_max_rad_s=30.0f,.torque_max_nm=10.0f,
                         .torque_limit_nm=1.0f,.timing={20,20,5,100}});
}
}
ZTEST(motor, test_initial_authority_fails_closed_without_bus_or_feedback) {
    Motor motor(djiConfig(1));
    const auto s=motor.snapshot();
    zassert_equal(s.state,MotorState::Offline);
    zassert_false(s.feedback_fresh);
    zassert_false(s.output_permitted);
    zassert_false(s.enabled_requested);
    zassert_false(motor.active());
    zassert_equal(motor.enable(),-EACCES);
    zassert_equal(motor.disable(),-EACCES);
}
ZTEST(motor, test_dji_command_rejection_preserves_revision) {
    Motor motor(djiConfig(2));
    const auto info=motor.info();
    zassert_true(info.capabilities & CommandCurrent);
    zassert_true(info.capabilities & FeedbackPosition);
    zassert_within(info.current_limit_a,2.0f,0);
    zassert_ok(motor.setCurrent(0.5f));
    const auto old=motor.snapshot().command_revision;
    zassert_true(old > 0);
    zassert_equal(motor.setCurrent(3.0f),-ERANGE);
    zassert_equal(motor.setCurrent(std::numeric_limits<float>::quiet_NaN()),-EINVAL);
    zassert_equal(motor.setTorque(0.5f),-ENOTSUP);
    zassert_equal(motor.setVelocity(0.5f),-ENOTSUP);
    zassert_equal(motor.snapshot().command_revision,old);
    zassert_ok(motor.setCurrent(-0.5f));
    zassert_equal(motor.snapshot().command_revision,old+1);
    zassert_false(motor.active());
}
ZTEST(motor, test_dm_mode_limits_and_command_atomicity) {
    Motor motor(dmConfig());
    const auto info=motor.info();
    zassert_true(info.capabilities & CommandMit);
    zassert_true(info.capabilities & CommandTorque);
    zassert_false(info.capabilities & CommandCurrent);
    zassert_equal(motor.setCurrent(0.1f),-ENOTSUP);
    zassert_equal(motor.setVelocity(1.0f),-ENOTSUP);
    zassert_equal(motor.setPositionVelocity(1,2),-ENOTSUP);
    zassert_equal(motor.setTorque(2.0f),-ERANGE);
    zassert_equal(motor.setTorque(std::numeric_limits<float>::infinity()),-EINVAL);
    zassert_ok(motor.setTorque(0.5f));
    const auto rev=motor.snapshot().command_revision;
    dm::MitCommand invalid{};
    invalid.kp=501;
    zassert_equal(motor.setMit(invalid),-ERANGE);
    zassert_equal(motor.snapshot().command_revision,rev);
    zassert_ok(motor.setMit({}));
}
ZTEST(motor, test_reseed_requires_fresh_feedback) {
    Motor motor(djiConfig(3));
    zassert_equal(motor.reseedPosition(std::numeric_limits<double>::quiet_NaN()),-EINVAL);
    zassert_equal(motor.reseedPosition(std::numeric_limits<double>::infinity()),-EINVAL);
    zassert_equal(motor.reseedPosition(0.0),-EAGAIN);
    zassert_false(motor.snapshot().position_reference_valid);
}
ZTEST(motor, test_bus_single_owner_attachment_and_unready_controller) {
    CanBus bus(nullptr);
    Motor a(djiConfig(1)),b(djiConfig(2));
    zassert_ok(bus.attach(a));
    zassert_equal(bus.attach(a),-EALREADY);
    zassert_ok(bus.attach(b));
    zassert_equal(bus.attach(b),-EALREADY);
    zassert_equal(bus.status().state,BusState::Unstarted);
    zassert_equal(bus.start(),-ENODEV);
    zassert_false(a.active());
}
ZTEST(motor, test_group_without_hardware_reports_no_authority) {
    Motor a(djiConfig(1)),b(djiConfig(2));
    Group group(a,b);
    auto s=group.status();
    zassert_equal(s.member_count,2u);
    zassert_equal(s.enabled_count,0u);
    zassert_equal(s.active_count,0u);
    zassert_equal(s.offline_count,2u);
    zassert_equal(group.enable(),-EACCES);
    group.disable();
    zassert_equal(group.status().active_count,0u);
    Group duplicate(a,a);
    zassert_equal(duplicate.enable(),-EINVAL);
    zassert_equal(duplicate.status().member_count,1u);
}
ZTEST_SUITE(motor,nullptr,nullptr,nullptr,nullptr,nullptr);
