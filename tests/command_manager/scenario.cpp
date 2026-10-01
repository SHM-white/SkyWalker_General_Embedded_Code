#include <robotics/command/command_manager.hpp>
#include <cassert>
#include <cerrno>
#include <cstdio>
using namespace skywalker;
using namespace robotics;
int main() {
    CommandManager manager(CommandManager::Config{});
    CommandInputs input{};
    CommandDecision out{};
    core::TimeUs now = 1000000;
    const auto refresh = [&] {
        input.remote.stamp = {now / 1000, 1, true};
        const OutputPermission permit{true, true, {now / 1000, 1, true}};
        input.referee.robot.chassis_output = input.referee.robot.gimbal_output = input.referee.robot
                                                                                     .shooter_output = permit;
    };
    const auto step = [&] {
        input.now_us = now;
        out = manager.update(input);
        assert(out.error == 0);
        assert(out.command.stamp.timestamp_ms == now / 1000);
    };
    step();
    assert(out.command.chassis.mode == ChassisMode::Disabled);
    input.remote.online = true;
    input.remote.left_switch = RcSwitch::Middle;
    input.remote.right_switch = RcSwitch::Middle;
    input.remote.analog.left_y = 330;
    input.vision = {{true, true, {1, 1}, {0.25f, 0.1f, 1}, {-0.2f, 0, 0}}, {now, 1, true}};
    refresh();
    step();
    assert(out.command.gimbal.source == ControlSource::Remote);
    assert(out.command.chassis.vx_m_s > 0);
    assert(out.command.shooter.mode == ShooterMode::Disabled);
    input.remote.left_switch = RcSwitch::Up;
    step();
    assert(out.command.gimbal.mode == GimbalMode::Hold);
    now += 10000;
    refresh();
    input.vision.stamp = {now, 2, true};
    step();
    assert(out.command.gimbal.mode == GimbalMode::AbsoluteAngle);
    assert(out.selected_vision.stamp.sequence == 2);
    // Chassis input alone cannot interrupt visual ownership.
    input.remote.analog.left_x = 600;
    step();
    assert(!out.manual_override);
    input.remote.analog.right_x = 330;
    step();
    assert(out.manual_override && out.command.gimbal.mode == GimbalMode::Rate);
    input.remote.analog.right_x = 0;
    now += 10000;
    refresh();
    step();
    assert(out.override_quiet && out.command.gimbal.yaw_rate_rad_s == 0);
    now += 200000;
    refresh();
    input.vision.stamp = {now, 3, true};
    step();
    assert(!out.manual_override && out.command.gimbal.mode == GimbalMode::Hold);
    now += 10000;
    refresh();
    input.vision.stamp = {now, 4, true};
    step();
    assert(out.command.gimbal.source == ControlSource::Vision);
    // Keyboard arming is mandatory even for visual fire.
    input.remote.right_switch = RcSwitch::Up;
    input.remote.mouse.right = true;
    step();
    assert(out.command.shooter.mode == ShooterMode::FireContinuous);
    input.referee.robot.gimbal_output.enabled = false;
    step();
    assert(out.command.gimbal.mode == GimbalMode::Disabled && !out.selected_vision.stamp.valid);
    assert(out.command.shooter.mode == ShooterMode::Ready && out.command.shooter.fire_rate_hz == 0);
    refresh();
    input.vision.value.control_requested = false;
    input.vision.value.reference = {};
    input.vision.stamp = {now, 5, true};
    step();
    assert(out.gimbal_reasons & VisionStopped);
    input.vision.value.control_requested = true;
    input.vision.value.reference = {1, 1};
    input.vision.stamp = {now, 4, true};
    step();
    assert(out.gimbal_reasons & InvalidVision);
    input.vision.stamp = {now, 6, true};
    step();
    assert(out.command.gimbal.mode == GimbalMode::AbsoluteAngle);
    now += 100001;
    refresh();
    step();
    assert(out.gimbal_reasons & VisionStale);
    // Overall referee online cannot extend institution permission lifetime.
    now += 301000;
    input.remote.stamp = {now / 1000, 1, true};
    input.referee.online = true;
    input.referee.stamp = {now / 1000, 99, true};
    step();
    assert(out.chassis_reasons & PermissionStale);
    input.remote.online = false;
    step();
    assert(out.command.chassis.mode == ChassisMode::Disabled && out.command.gimbal.mode == GimbalMode::Disabled);
    const auto seq = out.command.stamp.sequence;
    input.now_us = now - 1;
    out = manager.update(input);
    assert(out.error == -ESTALE);
    assert(out.command.stamp.sequence == seq + 1 && (out.reasons() & ClockRegression));
    std::puts("command manager complete arbitration scenario: PASS");
}
