#include <cerrno>
#include <robotics/safety/global_safety_manager.hpp>
#include <robotics/safety/chassis_local_safety.hpp>
#include <robotics/safety/gimbal_local_safety.hpp>
namespace skywalker::robotics {
int GlobalSafetyManager::clearEmergencyStop(bool released) {
    if (!released)
        return -EBUSY;
    estop_latched_ = false;
    return 0;
}
int GlobalSafetyManager::evaluate(const GlobalSafetyInputs &i, GlobalSafetyDecision &out) {
    if (!config_.permission_timeout_ms || !config_.chassis_heartbeat_timeout_ms || !config_.chassis_feedback_timeout_ms)
        return -EINVAL;
    estop_latched_ |= i.emergency_stop_requested;
    GlobalSafetyDecision n{};
    n.stamp = {i.now_ms, ++sequence_, true};
    if (estop_latched_) {
        n.state = SafetyState::EmergencyStop;
        n.active_reasons = EmergencyStop;
        out = n;
        return 0;
    }
    auto action = [&](const OutputPermission &p, bool hold) {
        if (p.valid && !p.enabled) {
            n.active_reasons |= PowerDisabled;
            return SafetyAction::Disable;
        }
        if ((config_.require_referee_for_motion || p.valid) &&
            (!p.valid || !isFresh(p.stamp, i.now_ms, config_.permission_timeout_ms))) {
            n.active_reasons |= PowerStale;
            return SafetyAction::Disable;
        }
        if (!i.operator_motion_enabled) {
            n.active_reasons |= OperatorDisabled;
            return SafetyAction::Disable;
        }
        if (!i.command_source_fresh) {
            n.active_reasons |= CommandStale;
            return hold ? SafetyAction::Hold : SafetyAction::Disable;
        }
        return SafetyAction::Active;
    };
    n.gimbal = action(i.gimbal_power, true);
    n.chassis = action(i.chassis_power, false);
    n.shooter = action(i.shooter_power, false);
    const bool chassis_heartbeat_fresh = isFresh(i.chassis_heartbeat_stamp, i.now_ms,
                                                 config_.chassis_heartbeat_timeout_ms);
    const bool chassis_feedback_fresh = isFresh(i.chassis_feedback_stamp, i.now_ms,
                                                config_.chassis_feedback_timeout_ms);
    if (!chassis_heartbeat_fresh) {
        n.chassis = SafetyAction::Disable;
        n.active_reasons |= TransportUnavailable;
    }
    else if (!chassis_feedback_fresh) {
        n.chassis = SafetyAction::Disable;
        n.active_reasons |= FeedbackStale;
    }
    else {
        switch (i.chassis_execution_state) {
        case ExecutionState::Ready:
        case ExecutionState::Active:
            break;
        case ExecutionState::ConfigBlocked:
            n.chassis = SafetyAction::Disable;
            n.active_reasons |= InvalidConfiguration;
            break;
        case ExecutionState::EStopLatched:
            n.chassis = SafetyAction::Disable;
            n.active_reasons |= EmergencyStop;
            break;
        case ExecutionState::Waiting:
        case ExecutionState::Recovering:
        default:
            n.chassis = SafetyAction::Disable;
            n.active_reasons |= i.chassis_active_reasons;
            break;
        }
    }
    // Waiting/Recovering may have no reason bits; actions still describe partial availability.
    const bool fully_active = n.gimbal == SafetyAction::Active && n.chassis == SafetyAction::Active &&
                              n.shooter == SafetyAction::Active && !n.active_reasons;
    n.state = fully_active ? SafetyState::Active : SafetyState::Waiting;
    if (!fully_active && (n.gimbal != SafetyAction::Disable || n.chassis != SafetyAction::Disable))
        n.state = SafetyState::Degraded;
    out = n;
    return 0;
}
int ChassisLocalSafety::clearEmergencyStop(bool released) {
    if (!released)
        return -EBUSY;
    estop_latched_ = false;
    count_ = 0;
    return 0;
}
int ChassisLocalSafety::evaluate(const LocalSafetyInputs &i, LocalSafetyDecision &out) {
    if (!config_.command_timeout_ms || !config_.heartbeat_timeout_ms || !config_.stable_command_count ||
        i.global_action > SafetyAction::Active)
        return -EINVAL;
    LocalSafetyDecision n{};
    estop_latched_ |= i.emergency_stop_requested;
    if (i.receiver_boot_id != boot_id_ || i.resume_generation != generation_) {
        count_ = 0;
        have_sequence_ = false;
        boot_id_ = i.receiver_boot_id;
        generation_ = i.resume_generation;
    }
    const bool heartbeat_fresh = isFresh(i.peer_heartbeat_stamp, i.now_ms, config_.heartbeat_timeout_ms);
    const bool command_fresh = isFresh(i.command_stamp, i.now_ms, config_.command_timeout_ms);
    const bool context = i.receiver_boot_id != 0 && i.command_boot_id == i.receiver_boot_id &&
                         i.command_generation == i.resume_generation;
    if (!heartbeat_fresh || !command_fresh || !context || !i.power_allowed || i.global_action != SafetyAction::Active) {
        count_ = 0;
        // Commands seen while the peer is offline cannot count toward its later recovery.
        if (!heartbeat_fresh && context && i.command_stamp.valid &&
            (!have_sequence_ || sequenceAfter(i.command_stamp.sequence, sequence_))) {
            sequence_ = i.command_stamp.sequence;
            have_sequence_ = true;
            last_command_ = i.command_stamp;
        }
    }
    else if (!have_sequence_ || sequenceAfter(i.command_stamp.sequence, sequence_)) {
        if (!isFresh(last_command_, i.now_ms, config_.command_timeout_ms))
            count_ = 0;
        if (count_ < config_.stable_command_count)
            ++count_;
        sequence_ = i.command_stamp.sequence;
        have_sequence_ = true;
        last_command_ = i.command_stamp;
    }
    if (!i.config_valid) {
        n.state = ExecutionState::ConfigBlocked;
        n.active_reasons = InvalidConfiguration;
    }
    else if (estop_latched_) {
        n.state = ExecutionState::EStopLatched;
        n.active_reasons = EmergencyStop;
    }
    else if (!heartbeat_fresh) {
        n.state = ExecutionState::Waiting;
        n.active_reasons = TransportUnavailable;
    }
    else if (!i.power_allowed)
        n.active_reasons = PowerDisabled;
    else if (!i.feedback_fresh || !i.hardware_ready) {
        n.state = ExecutionState::Recovering;
        n.active_reasons = FeedbackStale;
    }
    else {
        n.state = ExecutionState::Ready;
        if (!command_fresh)
            n.active_reasons |= CommandStale;
        if (!context || count_ < config_.stable_command_count)
            n.active_reasons |= RecoveryBoundary;
        if (i.global_action != SafetyAction::Active)
            n.active_reasons |= OperatorDisabled;
        if (!n.active_reasons) {
            n.action = SafetyAction::Active;
            n.state = i.armed ? ExecutionState::Active : ExecutionState::Ready;
        }
    }
    if (n.action != SafetyAction::Active && !i.hardware_ready)
        count_ = 0;
    out = n;
    return 0;
}
int GimbalLocalSafety::clearEmergencyStop(bool released) {
    if (!released)
        return -EBUSY;
    estop_latched_ = false;
    return 0;
}
int GimbalLocalSafety::evaluate(const LocalSafetyInputs &i, LocalSafetyDecision &out) {
    if (!config_.command_timeout_ms || i.global_action > SafetyAction::Active)
        return -EINVAL;
    estop_latched_ |= i.emergency_stop_requested;
    LocalSafetyDecision n{};
    if (!i.config_valid) {
        n.state = ExecutionState::ConfigBlocked;
        n.active_reasons = InvalidConfiguration;
    }
    else if (estop_latched_) {
        n.state = ExecutionState::EStopLatched;
        n.active_reasons = EmergencyStop;
    }
    else if (!i.power_allowed)
        n.active_reasons = PowerDisabled;
    else if (!i.feedback_fresh || !i.hardware_ready) {
        n.state = ExecutionState::Recovering;
        n.active_reasons = FeedbackStale;
    }
    else if (i.global_action == SafetyAction::Disable)
        n.active_reasons = OperatorDisabled;
    else {
        n.state = i.armed ? ExecutionState::Active : ExecutionState::Ready;
        n.action = isFresh(i.command_stamp, i.now_ms, config_.command_timeout_ms) ? i.global_action
                                                                                  : SafetyAction::Hold;
        if (n.action == SafetyAction::Hold)
            n.active_reasons = CommandStale;
    }
    out = n;
    return 0;
}
}
