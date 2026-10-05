#pragma once
#include "rc_controls.hpp"
#include <robotics/command/command_manager.hpp>
#include <robotics/shooter/shooter_executor.hpp>

namespace skywalker::samples::control {
// Execution-thread owner. RC edges keep their acquisition stamp while waiting
// at most one source timeout for the command worker to observe the same frame.
class RcShooterRequest {
public:
    robotics::ShooterCommand update(const RcControlState &rc, const robotics::CommandSnapshot &frame,
                                    const robotics::ShooterStatus &status, std::uint64_t now_ms) {
        using namespace robotics;
        ShooterCommand command = frame.decision.command.shooter;
        if (status.jammed) release_required_ = true;
        if (rc.fresh && rc.remote.right_switch == RcSwitch::Down) release_required_ = false;
        if (!rc.fresh || !rc.run_allowed || !rc.friction_requested || release_required_) {
            consumed_ = rc.shot_event_id;
            command.mode = ShooterMode::Disabled;
            return command;
        }
        const bool permitted = command.mode == ShooterMode::FireContinuous;
        if (command.mode != ShooterMode::Disabled) command.mode = ShooterMode::Ready;
        command.fire_event_id = 0;
        command.fire_event_stamp = {};
        command.fire_rate_hz = 2;
        const bool new_event = rc.shot_event_id && rc.shot_event_id != consumed_;
        const bool aligned = frame.observed.remote.stamp.valid && rc.shot_stamp.valid &&
            (frame.observed.remote.stamp.sequence == rc.shot_stamp.sequence ||
             sequenceAfter(frame.observed.remote.stamp.sequence, rc.shot_stamp.sequence));
        if (new_event && (!isFresh(rc.shot_stamp, now_ms, 100) || aligned)) {
            consumed_ = rc.shot_event_id;
            // Once arbitration has observed this edge, denial or a busy dial
            // consumes it immediately. A later readiness change cannot fire it.
            if (aligned && permitted && isFresh(rc.shot_stamp, now_ms, 100) &&
                status.friction_ready && status.feed.state == RunState::Active &&
                !status.dial_busy && !status.jammed) {
                command.mode = ShooterMode::FireSingle;
                command.fire_event_id = rc.shot_event_id;
                command.fire_event_stamp = rc.shot_stamp;
                return command;
            }
        }
        if (permitted && rc.continuous_requested) command.mode = ShooterMode::FireContinuous;
        return command;
    }
private:
    std::uint32_t consumed_ = 0;
    bool release_required_ = false;
};
} // namespace skywalker::samples::control
