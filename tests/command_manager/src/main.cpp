#include <benchmark.hpp>
#include <robotics/command/command_arbiter.hpp>
#include <robotics/command/manual_command_mapper.hpp>
#include <zephyr/ztest.h>

#include <array>
#include <cerrno>
#include <limits>

namespace {
using namespace skywalker;
using namespace skywalker::robotics;

void near(float actual, float expected)
{
    zassert_within(actual, expected, 1.0e-6f, "actual %f, expected %f",
                   static_cast<double>(actual), static_cast<double>(expected));
}

struct Scenario {
    CommandArbiter manager{CommandArbiter::Config{}};
    CommandInputs input{};
    CommandDecision out{};
    core::TimeUs now = 1000000;

    void refresh()
    {
        input.remote.stamp = {now / 1000, 1, true};
        const OutputPermission permit{true, true, {now / 1000, 1, true}};
        input.referee.robot.chassis_output = permit;
        input.referee.robot.gimbal_output = permit;
        input.referee.robot.shooter_output = permit;
    }

    void manual()
    {
        input.remote.online = true;
        input.remote.left_switch = RcSwitch::Middle;
        input.remote.right_switch = RcSwitch::Middle;
        refresh();
    }

    void vision(std::uint64_t sequence)
    {
        input.vision = {{true, true, {1, 1}, {0.25f, 0.1f, 1}, {-0.2f, 0, 0}},
                        {now, sequence, true}};
    }

    void step()
    {
        input.now_us = now;
        out = manager.update(input);
        zassert_ok(out.error);
        zassert_equal(out.command.stamp.timestamp_ms, now / 1000);
        zassert_true(out.command.stamp.valid);
    }

    void establishVision()
    {
        manual();
        input.remote.left_switch = RcSwitch::Up;
        vision(1);
        step();
        zassert_equal(out.command.gimbal.mode, GimbalMode::Hold);
        zassert_true(out.gimbal_reasons & WaitNewVision);
        now += 1000;
        refresh();
        vision(2);
        step();
        zassert_equal(out.command.gimbal.mode, GimbalMode::AbsoluteAngle);
    }
};

void disabled(const CommandDecision &out)
{
    zassert_equal(out.command.chassis.mode, ChassisMode::Disabled);
    zassert_equal(out.command.gimbal.mode, GimbalMode::Disabled);
    zassert_equal(out.command.shooter.mode, ShooterMode::Disabled);
    near(out.command.chassis.vx_m_s, 0);
    near(out.command.gimbal.yaw_rate_rad_s, 0);
    near(out.command.shooter.fire_rate_hz, 0);
    zassert_false(out.selected_vision.stamp.valid);
}
}

ZTEST(command_manager, test_manual_mapper_analytic_values)
{
    ManualCommandMapper mapper(ManualCommandMapper::Config{});
    Scenario s;
    s.manual();
    auto &remote = s.input.remote;
    remote.analog = {330, -660, 660, 330, 1320};
    OperatorIntent intent{};
    zassert_ok(mapper.map(remote, intent));
    // 330/660 = 0.5; rescaled deadband yields (0.5 - 0.03)/(1 - 0.03).
    constexpr float half = 0.47f / 0.97f;
    near(intent.chassis_vx_norm, half);
    near(intent.chassis_vy_norm, -1);
    near(intent.chassis_wz_norm, 1);
    near(intent.gimbal_yaw_rate_norm, -half);
    near(intent.gimbal_pitch_rate_norm, -1);
    zassert_equal(intent.mode, OperatorMode::Manual);
    zassert_equal(intent.source, ControlSource::Remote);
    zassert_equal(intent.stamp.timestamp_ms, remote.stamp.timestamp_ms);

    remote.analog = {19, -19, 0, 0, 0};
    zassert_ok(mapper.map(remote, intent));
    near(intent.gimbal_yaw_rate_norm, 0);
    near(intent.gimbal_pitch_rate_norm, 0);
    remote.analog.right_x = 20;
    zassert_ok(mapper.map(remote, intent));
    near(intent.gimbal_yaw_rate_norm, -(20.0f / 660.0f - 0.03f) / 0.97f);

    remote.right_switch = RcSwitch::Up;
    remote.keyboard.bits = (1u << 0) | (1u << 3);
    remote.mouse = {1000, -1000, 0, true, true};
    zassert_ok(mapper.map(remote, intent));
    zassert_equal(intent.source, ControlSource::KeyboardMouse);
    near(intent.chassis_vx_norm, 1);
    near(intent.chassis_vy_norm, -1);
    near(intent.gimbal_yaw_rate_norm, -1);
    near(intent.gimbal_pitch_rate_norm, 1);
    zassert_true(intent.friction_requested);
    zassert_true(intent.fire_requested);
    remote.keyboard.bits = 0x0f;
    zassert_ok(mapper.map(remote, intent));
    near(intent.chassis_vx_norm, 0);
    near(intent.chassis_vy_norm, 0);
}

ZTEST(command_manager, test_mapper_invalid_and_offline_inputs)
{
    std::array<ManualCommandMapper::Config, 3> invalid{};
    invalid[0].channel_range = 0;
    invalid[1].analog_deadband = 1;
    invalid[2].mouse_yaw_scale = std::numeric_limits<float>::quiet_NaN();
    for (const auto &config : invalid) {
        OperatorIntent output{};
        output.chassis_vx_norm = 0.25f;
        zassert_equal(ManualCommandMapper(config).map(RemoteState{}, output), -EINVAL);
        near(output.chassis_vx_norm, 0.25f);
    }
    ManualCommandMapper mapper(ManualCommandMapper::Config{});
    Scenario s;
    s.manual();
    OperatorIntent output{};
    s.input.remote.left_switch = RcSwitch::Unknown;
    zassert_equal(mapper.map(s.input.remote, output), -EINVAL);
    s.input.remote.online = false;
    zassert_ok(mapper.map(s.input.remote, output));
    zassert_equal(output.mode, OperatorMode::Safe);
    zassert_equal(output.source, ControlSource::None);
    near(output.chassis_vx_norm, 0);
}

ZTEST(command_manager, test_manual_scaling_and_publication_stamps)
{
    Scenario s;
    s.manual();
    s.input.remote.analog = {660, -660, 660, 660, -660};
    s.step();
    zassert_equal(s.out.command.chassis.mode, ChassisMode::BodyVelocity);
    near(s.out.command.chassis.vx_m_s, 3);
    near(s.out.command.chassis.vy_m_s, -3);
    near(s.out.command.chassis.wz_rad_s, -6);
    near(s.out.command.gimbal.yaw_rate_rad_s, -3);
    near(s.out.command.gimbal.pitch_rate_rad_s, -2);
    zassert_equal(s.out.command.shooter.mode, ShooterMode::Disabled);
    const auto stamp = s.out.command.stamp;
    const std::array<MessageStamp, 7> stamps{
        s.out.command.chassis.stamp, s.out.command.gimbal.stamp, s.out.command.shooter.stamp,
        s.out.requested.stamp, s.out.requested.chassis.stamp, s.out.requested.gimbal.stamp,
        s.out.requested.shooter.stamp};
    for (const auto &part : stamps) {
        zassert_true(part.valid);
        zassert_equal(part.sequence, stamp.sequence);
        zassert_equal(part.timestamp_ms, stamp.timestamp_ms);
    }
    s.step();
    zassert_equal(s.out.command.stamp.sequence, stamp.sequence + 1);
}

ZTEST(command_manager, test_operator_safety_and_timeout_boundaries)
{
    Scenario s;
    s.step();
    disabled(s.out);
    zassert_true(s.out.reasons() & RcUnavailable);
    s.manual();
    s.input.remote.stamp.timestamp_ms = s.now / 1000 - 100;
    s.step();
    zassert_equal(s.out.command.chassis.mode, ChassisMode::BodyVelocity);
    s.input.remote.stamp.timestamp_ms -= 1;
    s.step();
    disabled(s.out);
    zassert_true(s.out.reasons() & RcUnavailable);
    s.refresh();
    s.input.remote.stamp.timestamp_ms += 1;
    s.step();
    disabled(s.out);
    zassert_true(s.out.reasons() & RcUnavailable);
    s.refresh();
    s.input.remote.left_switch = RcSwitch::Down;
    s.step();
    disabled(s.out);
    zassert_true(s.out.reasons() & SafeRequested);
    s.input.remote.left_switch = RcSwitch::Unknown;
    s.input.now_us = s.now;
    s.out = s.manager.update(s.input);
    zassert_equal(s.out.error, -EINVAL);
    disabled(s.out);
    zassert_true(s.out.reasons() & InvalidInputs);

    CommandArbiter::Config config{};
    config.allow_auto = false;
    CommandArbiter no_auto(config);
    s.input.remote.left_switch = RcSwitch::Up;
    const auto result = no_auto.update(s.input);
    zassert_ok(result.error);
    disabled(result);
    zassert_true(result.reasons() & AutoUnavailable);
}

ZTEST(command_manager, test_permission_boundaries_and_individual_veto)
{
    Scenario s;
    s.manual();
    s.input.referee.robot.chassis_output.stamp.timestamp_ms = s.now / 1000 - 300;
    s.step();
    zassert_equal(s.out.command.chassis.mode, ChassisMode::BodyVelocity);
    s.input.referee.robot.chassis_output.stamp.timestamp_ms -= 1;
    s.step();
    zassert_equal(s.out.command.chassis.mode, ChassisMode::Disabled);
    zassert_true(s.out.chassis_reasons & PermissionStale);
    zassert_equal(s.out.requested.chassis.mode, ChassisMode::BodyVelocity);
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::Rate);
    s.refresh();
    s.input.referee.robot.gimbal_output.enabled = false;
    s.input.referee.robot.shooter_output.valid = false;
    s.step();
    zassert_equal(s.out.command.chassis.mode, ChassisMode::BodyVelocity);
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::Disabled);
    zassert_true(s.out.gimbal_reasons & PermissionDenied);
    zassert_true(s.out.shooter_reasons & PermissionMissing);
    s.refresh();
    s.input.referee.robot.chassis_output.stamp.timestamp_ms += 1;
    s.step();
    zassert_true(s.out.chassis_reasons & PermissionStale);
}

ZTEST(command_manager, test_vision_new_frame_and_value_limits)
{
    Scenario s;
    s.establishVision();
    s.input.vision.value.yaw = {4.0f, 100.0f, 100.0f};
    s.input.vision.value.pitch = {-2.0f, -100.0f, -100.0f};
    s.step();
    zassert_equal(s.out.command.gimbal.source, ControlSource::Vision);
    near(s.out.command.gimbal.yaw_target_rad, 4);
    near(s.out.command.gimbal.pitch_target_rad, -2);
    near(s.out.command.gimbal.yaw_rate_rad_s, 3);
    near(s.out.command.gimbal.pitch_rate_rad_s, -2);
    near(s.out.selected_vision.value.yaw.acceleration_rad_s2, 30);
    near(s.out.selected_vision.value.pitch.acceleration_rad_s2, -20);
    zassert_true(s.out.gimbal_reasons & ValueLimited);
    zassert_equal(s.out.selected_vision.stamp.time_us, s.input.vision.stamp.time_us);
    zassert_equal(s.out.selected_vision.stamp.sequence, s.input.vision.stamp.sequence);
    near(s.input.vision.value.yaw.rate_rad_s, 100);
    near(s.input.vision.value.pitch.acceleration_rad_s2, -100);
    zassert_equal(s.out.command.shooter.mode, ShooterMode::Disabled);
    zassert_true(s.out.shooter_reasons & ShooterNotArmed);
}

ZTEST(command_manager, test_vision_invalid_values_reference_and_time)
{
    Scenario s;
    s.establishVision();
    const auto good = s.input.vision;
    const auto expect_hold = [&s](std::uint32_t reason) {
        s.step();
        zassert_equal(s.out.command.gimbal.mode, GimbalMode::Hold);
        zassert_true(s.out.gimbal_reasons & reason);
        zassert_false(s.out.selected_vision.stamp.valid);
    };
    s.input.vision.stamp.valid = false;
    expect_hold(VisionMissing);
    s.input.vision = good;
    s.input.vision.value.reference.epoch = 2;
    expect_hold(VisionReferenceMismatch);
    s.input.vision = good;
    s.input.vision.value.yaw.angle_rad = std::numeric_limits<float>::quiet_NaN();
    expect_hold(InvalidVision);
    s.input.vision = good;
    s.input.vision.value.pitch.acceleration_rad_s2 = std::numeric_limits<float>::infinity();
    expect_hold(InvalidVision);
    s.input.vision = good;
    s.input.vision.value.control_requested = false;
    expect_hold(VisionStopped);
    s.input.vision = good;
    s.input.vision.stamp.time_us = s.now + 1;
    expect_hold(VisionStale);
    s.input.vision = good;
    s.input.vision.stamp.sequence = 1;
    expect_hold(InvalidVision);
    s.input.vision = good;
    s.now += 100000;
    s.refresh();
    s.step();
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::AbsoluteAngle);
    s.now += 1;
    s.refresh();
    expect_hold(VisionStale);
}

ZTEST(command_manager, test_invalid_config_clock_regression_and_reset)
{
    std::array<CommandArbiter::Config, 6> configs{};
    configs[0].max_chassis_vx_m_s = -1;
    configs[1].max_vision_yaw_acceleration_rad_s2 = 0;
    configs[2].input_timeout_ms = 0;
    configs[3].expected_vision_reference.frame_id = 0;
    configs[4].override_exit_norm = configs[4].override_enter_norm;
    configs[5].mapper.channel_range = std::numeric_limits<float>::quiet_NaN();
    for (const auto &config : configs) {
        CommandArbiter arbiter(config);
        zassert_equal(arbiter.configError(), -EINVAL);
        const auto out = arbiter.update(CommandInputs{});
        zassert_equal(out.error, -EINVAL);
        zassert_true(out.reasons() & InvalidManagerConfig);
        disabled(out);
    }
    Scenario s;
    s.establishVision();
    const auto sequence = s.out.command.stamp.sequence;
    s.input.now_us = s.now - 1;
    s.out = s.manager.update(s.input);
    zassert_equal(s.out.error, -ESTALE);
    zassert_true(s.out.reasons() & ClockRegression);
    zassert_equal(s.out.command.stamp.sequence, sequence + 1);
    disabled(s.out);
    s.manager.reset();
    s.now = 100000;
    s.refresh();
    s.vision(1);
    s.step();
    zassert_equal(s.out.command.stamp.sequence, 1);
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::Hold);
    zassert_true(s.out.gimbal_reasons & WaitNewVision);
    s.now += 1000;
    s.refresh();
    s.vision(2);
    s.step();
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::AbsoluteAngle);
}

ZTEST(command_manager, test_complete_arbitration_scenario)
{
    Scenario s;
    s.step();
    zassert_equal(s.out.command.chassis.mode, ChassisMode::Disabled);
    s.manual();
    s.input.remote.analog.left_y = 330;
    s.vision(1);
    s.step();
    zassert_equal(s.out.command.gimbal.source, ControlSource::Remote);
    zassert_true(s.out.command.chassis.vx_m_s > 0);
    zassert_equal(s.out.command.shooter.mode, ShooterMode::Disabled);
    s.input.remote.left_switch = RcSwitch::Up;
    s.step();
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::Hold);
    s.now += 10000;
    s.refresh();
    s.vision(2);
    s.step();
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::AbsoluteAngle);
    zassert_equal(s.out.selected_vision.stamp.sequence, 2);
    // Chassis input alone must leave visual ownership intact.
    s.input.remote.analog.left_x = 600;
    s.step();
    zassert_false(s.out.manual_override);
    s.input.remote.analog.right_x = 330;
    s.step();
    zassert_true(s.out.manual_override);
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::Rate);
    s.input.remote.analog.right_x = 0;
    s.now += 10000;
    s.refresh();
    s.step();
    zassert_true(s.out.override_quiet);
    near(s.out.command.gimbal.yaw_rate_rad_s, 0);
    s.now += 200000;
    s.refresh();
    s.vision(3);
    s.step();
    zassert_false(s.out.manual_override);
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::Hold);
    s.now += 10000;
    s.refresh();
    s.vision(4);
    s.step();
    zassert_equal(s.out.command.gimbal.source, ControlSource::Vision);
    // Even visual fire requires keyboard friction-wheel arming.
    s.input.remote.right_switch = RcSwitch::Up;
    s.input.remote.mouse.right = true;
    s.step();
    zassert_equal(s.out.command.shooter.mode, ShooterMode::FireContinuous);
    s.input.referee.robot.gimbal_output.enabled = false;
    s.step();
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::Disabled);
    zassert_false(s.out.selected_vision.stamp.valid);
    zassert_equal(s.out.command.shooter.mode, ShooterMode::Ready);
    near(s.out.command.shooter.fire_rate_hz, 0);
    s.refresh();
    s.input.vision.value.control_requested = false;
    s.input.vision.value.reference = {};
    s.input.vision.stamp = {s.now, 5, true};
    s.step();
    zassert_true(s.out.gimbal_reasons & VisionStopped);
    s.input.vision.value.control_requested = true;
    s.input.vision.value.reference = {1, 1};
    s.input.vision.stamp = {s.now, 4, true};
    s.step();
    zassert_true(s.out.gimbal_reasons & InvalidVision);
    s.input.vision.stamp = {s.now, 6, true};
    s.step();
    zassert_equal(s.out.command.gimbal.mode, GimbalMode::AbsoluteAngle);
    s.now += 100001;
    s.refresh();
    s.step();
    zassert_true(s.out.gimbal_reasons & VisionStale);
    // Overall referee online cannot extend individual permission lifetimes.
    s.now += 301000;
    s.input.remote.stamp = {s.now / 1000, 1, true};
    s.input.referee.online = true;
    s.input.referee.stamp = {s.now / 1000, 99, true};
    s.step();
    zassert_true(s.out.chassis_reasons & PermissionStale);
    s.input.remote.online = false;
    s.step();
    disabled(s.out);
    const auto sequence = s.out.command.stamp.sequence;
    s.input.now_us = s.now - 1;
    s.out = s.manager.update(s.input);
    zassert_equal(s.out.error, -ESTALE);
    zassert_equal(s.out.command.stamp.sequence, sequence + 1);
    zassert_true(s.out.reasons() & ClockRegression);
}

ZTEST(command_manager, test_algorithm_timings)
{
    Scenario manual;
    manual.manual();
    manual.input.remote.analog.left_y = 330;
    manual.input.now_us = manual.now;
    ManualCommandMapper mapper(ManualCommandMapper::Config{});
    OperatorIntent intent{};
    int error = 0;
    skywalker::test::benchmark("command.mapper", 2048, [&](std::uint32_t) {
        error |= mapper.map(manual.input.remote, intent);
    });
    zassert_ok(error);
    near(intent.chassis_vx_norm, 0.47f / 0.97f);
    skywalker::test::benchmark("command.arbiter.manual", 2048, [&](std::uint32_t) {
        manual.out = manual.manager.update(manual.input);
        error |= manual.out.error;
    });
    zassert_ok(error);
    zassert_equal(manual.out.command.chassis.mode, ChassisMode::BodyVelocity);
    near(manual.out.command.chassis.vx_m_s, 3.0f * 0.47f / 0.97f);
    Scenario automatic;
    automatic.establishVision();
    automatic.input.remote.right_switch = RcSwitch::Up;
    automatic.input.remote.mouse.right = true;
    skywalker::test::benchmark("command.arbiter.vision", 2048, [&](std::uint32_t) {
        automatic.out = automatic.manager.update(automatic.input);
        error |= automatic.out.error;
    });
    zassert_ok(error);
    zassert_equal(automatic.out.command.gimbal.mode, GimbalMode::AbsoluteAngle);
    zassert_equal(automatic.out.command.shooter.mode, ShooterMode::FireContinuous);
    near(automatic.out.command.gimbal.yaw_target_rad, 0.25f);
    near(automatic.out.command.shooter.fire_rate_hz, 5);
}

ZTEST_SUITE(command_manager, nullptr, nullptr, nullptr, nullptr, nullptr);
