#pragma once
#include "rc_controls.hpp"
#include <robotics/command/command_manager.hpp>
#include <robotics/shooter/shooter_executor.hpp>

namespace skywalker::samples::control {
// 执行线程只撤销已仲裁的请求；手势和射频归命令层所有，禁止在这里重新授权。
class RcShooterRequest {
public:
    robotics::ShooterCommand update(const RcControlState &rc, const robotics::CommandSnapshot &frame,
                                    const robotics::ShooterStatus &status, std::uint64_t now_ms) {
        using namespace robotics;
        ShooterCommand command = frame.decision.command.shooter;
        if (status.jammed)
            release_required_ = true;
        const bool released = rc.keyboard_mouse_selected ? !rc.remote.mouse.right && !rc.remote.mouse.left
                                                         : rc.remote.right_switch == RcSwitch::Down;
        if (rc.fresh && released)
            release_required_ = false;
        const auto selected = rc.keyboard_mouse_selected ? ControlSource::KeyboardMouse : ControlSource::Remote;
        if (!rc.fresh || !rc.run_allowed || !rc.friction_requested || release_required_ ||
            !isFresh(command.stamp, now_ms, 100) ||
            (command.source != selected && command.source != ControlSource::Vision)) {
            command.mode = ShooterMode::Disabled;
            return command; // 保留事件身份，执行器拒绝并消费，不能在恢复后补发。
        }
        if (rc.keyboard_mouse_selected) {
            // 右键电平直接停机；长按释放直接撤销，不等待命令线程下一轮。
            // RapidContinuous 的左键松开间隙必须继续供弹。
            if (command.mode == ShooterMode::FireContinuous &&
                frame.decision.mouse_fire.kind == MouseFeedKind::HeldContinuous && !rc.remote.mouse.left)
                command.mode = ShooterMode::Ready;
        }
        else if (!rc.continuous_requested && command.mode == ShooterMode::FireContinuous)
            command.mode = ShooterMode::Ready;
        return command;
    }

private:
    bool release_required_ = false;
};
} // namespace skywalker::samples::control
