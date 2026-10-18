#include <cerrno>
#include <cmath>
#include <limits>
#include <robotics/gimbal/yaw_centering.hpp>
#include <robotics/execution/run_status.hpp>
#include <zephyr/ztest.h>

namespace {
using namespace skywalker;
using namespace skywalker::robotics;
YawCenteringInputs ready(core::TimeUs at=100000) {
    YawCenteringInputs value{};
    value.enabled=true;
    value.head_stable=true;
    value.permission_valid=true;
    value.joint_yaw_rad=0.5f;
    value.joint_stamp={at,1,true};
    value.source_stamp={at,1,true};
    return value;
}
}

ZTEST(robotics_contracts, test_yaw_centering_fails_closed_until_valid_cycle) {
    using namespace skywalker::robotics;
    YawCenteringController controller({});
    auto input=ready();
    auto first=controller.update(input, 100000);
    zassert_false(first.enabled);
    zassert_equal(first.reason, WaitReason::Cycle);
    auto second=controller.update(input, 110000);
    zassert_true(second.enabled);
    zassert_equal(second.reason, WaitReason::None);
    zassert_within(second.velocity_rad_s, 0.004f, 0.0000001f);
    zassert_equal(second.stamp.sequence, 2u);
    zassert_equal(second.stamp.timestamp_ms, 110u);
    zassert_true(second.center_error_rad>0.49f);
}

ZTEST(robotics_contracts, test_yaw_centering_permission_and_feedback_revocation_immediate) {
    using namespace skywalker::robotics;
    YawCenteringController controller({});
    auto input=ready();
    (void)controller.update(input,100000);
    auto active=controller.update(input,110000);
    zassert_true(active.velocity_rad_s>0);
    input.permission_valid=false;
    auto blocked=controller.update(input,120000);
    zassert_false(blocked.enabled);
    zassert_equal(blocked.reason, WaitReason::Power);
    zassert_equal(blocked.velocity_rad_s,0.0f);
    input.permission_valid=true;
    input.joint_stamp={100000,1,true};
    auto stale=controller.update(input,160001);
    zassert_false(stale.enabled);
    zassert_equal(stale.reason, WaitReason::Feedback);
    input.joint_stamp={170000,2,true};
    input.source_stamp={170000,2,true};
    controller.reset();
    auto jump=controller.update(input,170000);
    zassert_false(jump.enabled);
    zassert_equal(jump.reason, WaitReason::Cycle);
    auto again=controller.update(input,180000);
    zassert_true(again.enabled);
    zassert_within(again.velocity_rad_s,0.004f,0.0000001f);
}

ZTEST(robotics_contracts, test_yaw_deadband_hysteresis_and_reset) {
    using namespace skywalker::robotics;
    YawCenteringController controller({});
    auto input=ready();
    input.joint_yaw_rad=0.01f;
    (void)controller.update(input,100000);
    const auto quiet=controller.update(input,110000);
    zassert_true(quiet.enabled);
    zassert_equal(quiet.velocity_rad_s,0.0f);
    input.joint_yaw_rad=0.08f;
    const auto following=controller.update(input,120000);
    zassert_true(following.velocity_rad_s>0.0f);
    input.joint_yaw_rad=0.01f;
    const auto decelerate=controller.update(input,130000);
    zassert_true(decelerate.enabled);
    zassert_within(decelerate.velocity_rad_s,0.0f,0.0000001f);
    controller.reset();
    zassert_equal(controller.update(input,140000).reason,WaitReason::Cycle);
}

ZTEST(robotics_contracts, test_yaw_configuration_and_nonfinite_input) {
    using namespace skywalker::robotics;
    YawCenteringController::Config config{};
    config.follow_direction=0.5f;
    YawCenteringController invalid(config);
    auto input=ready();
    zassert_equal(invalid.update(input,100000).reason,WaitReason::Configuration);
    YawCenteringController valid({});
    input.joint_yaw_rad=std::numeric_limits<float>::quiet_NaN();
    zassert_equal(valid.update(input,100000).reason,WaitReason::Feedback);
    input=ready();
    input.head_stable=false;
    zassert_equal(valid.update(input,110000).reason,WaitReason::Reference);
    input.head_stable=true;
    input.enabled=false;
    zassert_equal(valid.update(input,120000).reason,WaitReason::Command);
}

ZTEST(robotics_contracts, test_feedback_state_mapping_does_not_grant_stale_authority) {
    using namespace skywalker::robotics;
    RunStatus status{};
    status.stamp={100,2,true};
    status.state=RunState::Active;
    status.ready=true;
    status.last_command_sequence=9;
    auto active=wireFeedback(status,200,100);
    zassert_true(active.armed);
    zassert_true(active.ready);
    zassert_equal(active.last_command_sequence,9u);
    auto stale=wireFeedback(status,201,100);
    zassert_false(stale.armed);
    zassert_false(stale.ready);
    zassert_equal(stale.active_reasons,FeedbackStale);
    status.state=RunState::Blocked;
    status.reason=WaitReason::Configuration;
    auto blocked=wireFeedback(status);
    zassert_equal(blocked.execution_state, ExecutionState::ConfigBlocked);
    zassert_equal(blocked.active_reasons,InvalidConfiguration);
    status.state=RunState::Recovering;
    status.reason=WaitReason::Power;
    zassert_equal(wireFeedback(status).active_reasons,PowerBudgetStale);
}
ZTEST_SUITE(robotics_contracts,nullptr,nullptr,nullptr,nullptr,nullptr);
