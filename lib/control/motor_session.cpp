#include <control/motor_session.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <core/clock.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor_session, LOG_LEVEL_INF);

namespace skywalker::control {

auto MotorSession::collectMotors(std::span<const Member> members) {
    std::array<motor::Motor *, motor::kMaxGroupMembers> result{};
    for (std::size_t i = 0; i < std::min(members.size(), result.size()); ++i)
        result[i] = members[i].motor;
    return result;
}

MotorSession::MotorSession(std::span<const Member> members) : MotorSession(members, Config{}) {}

MotorSession::MotorSession(std::span<const Member> members, Config config)
    : motors_(collectMotors(members)), count_(std::min(members.size(), motors_.size())),
      group_(std::span<motor::Motor *const>(motors_.data(), count_)), config_(config) {
    std::copy_n(members.begin(), count_, members_.begin());
    if (members.empty() || members.size() > motors_.size()) configuration_error_ = -EINVAL;
    working_.member_count = count_;
}

SessionCause MotorSession::cause(SessionOperation op, int error, RecoveryAction action,
                                core::TimeUs now, std::size_t index) const {
    SessionCause result{};
    result.operation = op;
    result.error = error;
    result.action = action;
    result.occurred_us = now;
    if (index < count_) {
        const auto &member = members_[index];
        if (member.name) result.member = member.name;
        if (member.bus && member.bus->busDevice()) result.bus = member.bus->busDevice()->name;
    }
    return result;
}

int MotorSession::configure() {
    if (configure_attempted_) return configured_ ? -EALREADY : configuration_error_;
    configure_attempted_ = true;
    const auto now = core::monotonicTimeUs();
    auto fail = [&](int error, std::size_t index) {
        configuration_error_ = error;
        group_.disable();
        working_.state = SessionState::Blocked;
        working_.current_blocker = cause(SessionOperation::Configure, error, RecoveryAction::FixConfiguration, now, index);
        publish();
        const auto &reason = working_.current_blocker;
        LOG_ERR("SESSION configure member=%s bus=%s err=%d next=fix_configuration", reason.member, reason.bus, error);
        return error;
    };
    if (configuration_error_ || !config_.source_timeout_us || !config_.target_timeout_us ||
        !config_.max_cycle_us || !config_.prepare_timeout_us || !config_.enable_timeout_us)
        return fail(configuration_error_ ? configuration_error_ : -EINVAL, count_);
    // Validate all bindings before starting any worker. These buses are exclusively
    // owned by this session; shared-bus publication is a separate API migration.
    for (std::size_t i = 0; i < count_; ++i) {
        const auto &member = members_[i];
        if (!member.name || !member.motor || !member.bus || !member.bus->busDevice() ||
            !std::visit([&](auto *axis) { return axis && &axis->motor() == member.motor; }, member.controller))
            return fail(-EINVAL, i);
        for (std::size_t j = 0; j < i; ++j)
            if (members_[j].motor == member.motor) return fail(-EINVAL, i);
        const auto position = std::get_if<PositionMotor *>(&member.controller);
        if (unsigned(member.reference) > unsigned(ReferencePolicy::RequireExternalReference) ||
            (!position && member.reference != ReferencePolicy::NotRequired) ||
            (position && member.reference == ReferencePolicy::NotRequired) ||
            (member.reference == ReferencePolicy::CaptureOnExplicitStart &&
             (*position)->reference() != PositionReference::StartupRelative) ||
            (member.reference == ReferencePolicy::UseCalibratedAbsolute &&
             (*position)->reference() != PositionReference::AbsoluteNearest))
            return fail(-EINVAL, i);
        if (member.bus->status().state != motor::BusState::Unstarted) return fail(-EBUSY, i);
        if (std::find(buses_.begin(), buses_.begin() + bus_count_, member.bus) == buses_.begin() + bus_count_)
            buses_[bus_count_++] = member.bus;
    }
    for (std::size_t i = 0; i < count_; ++i) {
        const int error = members_[i].bus->attach(*members_[i].motor);
        if (error < 0) return fail(error, i);
    }
    for (std::size_t i = 0; i < bus_count_; ++i)
        if (!buses_[i]->exclusivelyOwnedBy(group_)) return fail(-ENOTSUP, count_);
    for (std::size_t i = 0; i < count_; ++i) {
        auto *bus = members_[i].bus;
        if (bus->status().state == motor::BusState::Unstarted) {
            const int error = bus->start();
            if (error < 0) return fail(error, i);
        }
        const int error = std::visit([](auto *axis) { return axis->configure(ControlFailurePolicy::ReportOnly); },
                                     members_[i].controller);
        if (error < 0) return fail(error, i);
    }
    configured_ = true;
    working_.current_blocker = readiness(now);
    working_.can_start = working_.current_blocker.error == 0;
    publish();
    return 0;
}

SessionCause MotorSession::readiness(core::TimeUs now) const {
    for (std::size_t i = 0; i < count_; ++i) {
        const auto &member = members_[i];
        const auto bus = member.bus->status();
        if (bus.state != motor::BusState::Running)
            return cause(SessionOperation::Transport, bus.last_error < 0 ? bus.last_error : -ENETDOWN,
                         bus.state == motor::BusState::ConfigBlocked ? RecoveryAction::FixConfiguration : RecoveryAction::WaitBus, now, i);
        const auto snapshot = member.motor->snapshot();
        if (snapshot.state == motor::MotorState::Fault) {
            auto result = cause(SessionOperation::DriverFault, snapshot.last_fault.error < 0 ? snapshot.last_fault.error : -EIO,
                                RecoveryAction::Acknowledge, now, i);
            result.driver_reason = snapshot.last_fault.reason;
            return result;
        }
        if (!member.motor->ready())
            return cause(SessionOperation::Feedback, -EAGAIN, RecoveryAction::WaitFeedback, now, i);
        auto check = std::visit([](auto *axis) { return axis->preflight(); }, member.controller);
        if (check.issue == ControlIssue::ReferenceLost &&
            member.reference == ReferencePolicy::CaptureOnExplicitStart) continue;
        if (check.issue == ControlIssue::ReferenceLost &&
            member.reference == ReferencePolicy::UseCalibratedAbsolute &&
            (snapshot.feedback.valid & motor::FeedbackAbsolutePosition) &&
            std::isfinite(snapshot.feedback.absolute_position_rad)) continue;
        if (check.error < 0) {
            auto result = cause(check.issue == ControlIssue::ReferenceLost ? SessionOperation::Reference : SessionOperation::Feedback,
                                check.error, check.issue == ControlIssue::ReferenceLost ? RecoveryAction::SupplyReference : RecoveryAction::WaitFeedback, now, i);
            result.control_issue = check.issue;
            return result;
        }
    }
    return {};
}

SessionCause MotorSession::driverFailure(core::TimeUs now) const {
    const auto status = group_.status();
    for (std::size_t i = 0; i < count_; ++i) {
        if (members_[i].motor != status.last_fault.source_motor) continue;
        const auto &fault = status.last_fault;
        auto result = cause(SessionOperation::DriverFault, fault.error < 0 ? fault.error : -EIO,
                            RecoveryAction::NewStart, fault.occurred_ms * 1000, i);
        result.driver_reason = fault.reason;
        if (fault.reason == motor::FaultReason::TransportError || fault.reason == motor::FaultReason::RxOverflow) {
            result.operation = SessionOperation::Transport;
            const auto recovery = members_[i].bus->status().last_recovery;
            if (recovery.count && recovery.occurred_ms >= attempt_us_ / 1000 &&
                recovery.reason == fault.reason && recovery.error == fault.error) {
                result.has_recovery = true;
                result.recovery = recovery;
            }
            result.action = RecoveryAction::WaitBus;
        } else if (fault.reason == motor::FaultReason::DriveFault) {
            result.action = RecoveryAction::Acknowledge;
        }
        return result;
    }
    return cause(SessionOperation::Enable, -ECANCELED, RecoveryAction::NewStart, now);
}

void MotorSession::stop(const SessionCause &reason) {
    group_.disable();
    working_.phase = SessionPhase::Idle;
    working_.state = reason.error < 0 ? SessionState::Blocked : SessionState::Stopped;
    working_.can_start = false;
    working_.current_blocker = reason;
    if (reason.error < 0) {
        working_.last_stop = reason;
        ++working_.stop_event;
    }
}

void MotorSession::advance(const Input &input, core::TimeUs now) {
    const auto dt = have_previous_ && now >= previous_us_ ? now - previous_us_ : 0;
    previous_us_ = now;
    have_previous_ = true;
    if (!configured_) return;
    const bool new_start = input.start_sequence > start_seen_;
    const bool new_ack = input.acknowledge_sequence > acknowledge_seen_;
    start_seen_ = std::max(start_seen_, input.start_sequence);
    acknowledge_seen_ = std::max(acknowledge_seen_, input.acknowledge_sequence);
    const bool source_fresh = core::fresh(input.source, now, config_.source_timeout_us);
    const bool engaged = working_.state == SessionState::Starting || working_.state == SessionState::Running;
    // Capture a driver's first group fault before application cancellation changes state.
    if ((working_.state == SessionState::Running && !group_.active()) ||
        (working_.state == SessionState::Starting && working_.phase == SessionPhase::Enable &&
         !group_.active() && !group_.status().enable_pending)) {
        stop(driverFailure(now));
        return;
    }
    if (engaged && (!input.enabled || !source_fresh ||
        (working_.state == SessionState::Running && input.source.time_us <= prepared_us_))) {
        stop(!input.enabled && source_fresh ? SessionCause{} : cause(SessionOperation::Source, -ESTALE, RecoveryAction::NewInput, now));
        return;
    }
    if (!engaged) {
        if (new_ack && source_fresh && !input.enabled) {
            const int error = group_.clearFault();
            if (error < 0) {
                working_.current_blocker = cause(SessionOperation::Acknowledge, error, RecoveryAction::Acknowledge, now);
                working_.can_start = false;
                return;
            }
        }
        working_.current_blocker = readiness(now);
        if (!source_fresh)
            working_.current_blocker = cause(SessionOperation::Source, -ESTALE, RecoveryAction::NewInput, now);
        working_.can_start = working_.current_blocker.error == 0;
        if (!new_start || !input.enabled || !source_fresh) return;
        // Start may wait for fresh/stable feedback, but is never replayed after a stop.
        working_.state = SessionState::Starting;
        working_.phase = SessionPhase::Prepare;
        working_.can_start = false;
        attempt_us_ = now;
    }
    if (working_.state == SessionState::Starting) {
        if (now < attempt_us_ || now - attempt_us_ > config_.prepare_timeout_us) {
            auto reason = working_.current_blocker;
            if (reason.error == 0) reason = cause(SessionOperation::Enable, -ETIMEDOUT, RecoveryAction::NewStart, now);
            reason.timed_out = true;
            stop(reason);
            return;
        }
        if (working_.phase == SessionPhase::Prepare) {
            working_.current_blocker = readiness(now);
            if (working_.current_blocker.error < 0) return;
            for (std::size_t i = 0; i < count_; ++i) {
                const auto &member = members_[i];
                int error = 0;
                if (member.reference == ReferencePolicy::CaptureOnExplicitStart)
                    error = member.motor->reseedPosition(0.0);
                else if (member.reference == ReferencePolicy::UseCalibratedAbsolute)
                    error = member.motor->reseedPosition(member.motor->snapshot().feedback.absolute_position_rad);
                if (error < 0) {
                    stop(cause(SessionOperation::Reference, error, RecoveryAction::SupplyReference, now, i));
                    return;
                }
                error = std::visit([](auto *axis) { return axis->reset(); }, member.controller);
                if (error < 0) {
                    auto reason = cause(SessionOperation::Reset, error, RecoveryAction::NewStart, now, i);
                    reason.control_issue = std::visit([](auto *axis) { return axis->preflight().issue; }, member.controller);
                    stop(reason);
                    return;
                }
            }
            prepared_us_ = now;
            working_.phase = SessionPhase::FreshInput;
            working_.current_blocker = cause(SessionOperation::Target, -EAGAIN, RecoveryAction::NewInput, now);
            return;
        }
        if (working_.phase == SessionPhase::FreshInput) {
            // Cached RC frames/targets from before reset cannot activate a new generation.
            if (input.source.time_us <= prepared_us_ || input.target_stamp.time_us <= prepared_us_ ||
                !core::fresh(input.target_stamp, now, config_.target_timeout_us)) return;
            if (input.targets.size() != count_) {
                stop(cause(SessionOperation::Target, -EINVAL, RecoveryAction::NewInput, now));
                return;
            }
            for (std::size_t i = 0; i < count_; ++i)
                if (!std::isfinite(input.targets[i])) {
                    stop(cause(SessionOperation::Target, -EINVAL, RecoveryAction::NewInput, now, i));
                    return;
                }
            const int error = group_.enable();
            if (error < 0) {
                auto reason = readiness(now);
                if (reason.error == 0) reason = cause(SessionOperation::Enable, error, RecoveryAction::NewStart, now);
                stop(reason);
                return;
            }
            enable_us_ = now;
            working_.phase = SessionPhase::Enable;
            working_.current_blocker = cause(SessionOperation::Enable, -EAGAIN, RecoveryAction::WaitFeedback, now);
            return;
        }
        if (!group_.active()) {
            if (now - enable_us_ > config_.enable_timeout_us)
                stop(cause(SessionOperation::Enable, -ETIMEDOUT, RecoveryAction::NewStart, now));
            return;
        }
        working_.state = SessionState::Running;
        working_.phase = SessionPhase::Idle;
        working_.started_us = now;
        working_.enable_generation = group_.status().enable_generation;
    }
    if (!dt || dt > config_.max_cycle_us) {
        stop(cause(SessionOperation::Period, -ERANGE, RecoveryAction::NewStart, now));
        return;
    }
    if (!core::fresh(input.target_stamp, now, config_.target_timeout_us) ||
        input.target_stamp.time_us <= prepared_us_ || input.targets.size() != count_) {
        stop(cause(SessionOperation::Target, input.targets.size() != count_ ? -EINVAL : -ESTALE, RecoveryAction::NewInput, now));
        return;
    }
    for (std::size_t i = 0; i < count_; ++i) {
        const auto &member = members_[i];
        if (!std::isfinite(input.targets[i]) || std::fabs(input.targets[i]) > std::numeric_limits<float>::max()) {
            stop(cause(SessionOperation::Target, -ERANGE, RecoveryAction::NewInput, now, i));
            return;
        }
        const int error = std::visit([&](auto *axis) { return axis->update(input.targets[i], float(dt) * 1e-6f); }, member.controller);
        if (error < 0) {
            // If the worker revoked authority during computation, preserve that cause.
            if (!group_.active()) { stop(driverFailure(now)); return; }
            auto reason = cause(SessionOperation::Control, error, RecoveryAction::NewStart, now, i);
            reason.control_issue = std::visit([](auto *axis) { return axis->telemetry().issue; }, member.controller);
            stop(reason);
            return;
        }
    }
    // All members computed successfully before any target is published. This is
    // not atomic cross-bus delivery. Worker generation checks still revoke output.
    for (std::size_t i = 0; i < bus_count_; ++i) {
        const auto result = buses_[i]->commit();
        if (result.error < 0) {
            if (!group_.active()) { stop(driverFailure(now)); return; }
            std::size_t member = 0;
            while (members_[member].bus != buses_[i]) ++member;
            stop(cause(SessionOperation::Publish, result.error, RecoveryAction::WaitBus, now, member));
            return;
        }
    }
    working_.current_blocker = {};
}

void MotorSession::publish() {
    for (std::size_t i = 0; i < count_; ++i)
        if (motors_[i]) working_.stops[i] = motors_[i]->snapshot().stop;
    const auto key = k_spin_lock(&status_lock_);
    published_ = working_;
    k_spin_unlock(&status_lock_, key);
}

void MotorSession::step(const Input &input, core::TimeUs now_us) {
    const auto previous_state = working_.state;
    const auto previous_blocker = working_.current_blocker;
    const auto previous_stop = working_.stop_event;
    advance(input, now_us);
    publish();
    const auto &blocker = working_.current_blocker;
    if (previous_state != working_.state || previous_blocker.operation != blocker.operation ||
        previous_blocker.control_issue != blocker.control_issue || previous_blocker.error != blocker.error ||
        previous_blocker.member != blocker.member) {
        LOG_INF("SESSION state=%s phase=%u can_start=%d blocker=%s member=%s bus=%s op=%s err=%d next=%s",
                sessionStateName(working_.state), unsigned(working_.phase), working_.can_start,
                sessionCauseName(blocker), blocker.member, blocker.bus, sessionOperationName(blocker.operation),
                blocker.error, blocker.error == 0 && working_.state != SessionState::Running
                    ? "new_start" : recoveryActionName(blocker.action));
    }
    if (working_.stop_event != previous_stop) {
        const auto &reason = working_.last_stop;
        LOG_ERR("SESSION STOP event=%llu member=%s bus=%s op=%s reason=%s err=%d ms=%llu timeout=%d next=%s",
                static_cast<unsigned long long>(working_.stop_event), reason.member, reason.bus,
                sessionOperationName(reason.operation), sessionCauseName(reason), reason.error,
                static_cast<unsigned long long>(reason.occurred_us / 1000), reason.timed_out, recoveryActionName(reason.action));
        if (reason.has_recovery) {
            const auto &r = reason.recovery;
            LOG_ERR("SESSION CAN evidence count=%llu hw=%u TEC=%u REC=%u query_err=%d tx_valid=%d id=0x%x purpose=%u seq=%llu tx_err=%d tx_ms=%llu",
                    static_cast<unsigned long long>(r.count), unsigned(r.controller_state), unsigned(r.error_counts.tx_err_cnt),
                    unsigned(r.error_counts.rx_err_cnt), r.query_error, r.last_tx.valid, unsigned(r.last_tx.can_id),
                    unsigned(r.last_tx.purpose), static_cast<unsigned long long>(r.last_tx.sequence), r.last_tx.error,
                    static_cast<unsigned long long>(r.last_tx.completed_ms));
        }
    }
}

SessionStatus MotorSession::status() const {
    const auto key = k_spin_lock(&status_lock_);
    const auto result = published_;
    k_spin_unlock(&status_lock_, key);
    return result;
}

const char *sessionStateName(SessionState state) {
    switch (state) {
    case SessionState::Stopped: return "stopped";
    case SessionState::Starting: return "starting";
    case SessionState::Running: return "running";
    case SessionState::Blocked: return "blocked";
    }
    return "unknown";
}
const char *sessionOperationName(SessionOperation op) {
    switch (op) {
    case SessionOperation::None: return "none";
    case SessionOperation::Configure: return "configure";
    case SessionOperation::Source: return "source";
    case SessionOperation::Target: return "target";
    case SessionOperation::Period: return "period";
    case SessionOperation::Feedback: return "feedback";
    case SessionOperation::Reference: return "reference";
    case SessionOperation::Reset: return "reset";
    case SessionOperation::Enable: return "enable";
    case SessionOperation::Control: return "control";
    case SessionOperation::Publish: return "publish";
    case SessionOperation::Transport: return "transport";
    case SessionOperation::DriverFault: return "driver_fault";
    case SessionOperation::Acknowledge: return "acknowledge";
    }
    return "unknown";
}
const char *recoveryActionName(RecoveryAction action) {
    switch (action) {
    case RecoveryAction::None: return "none";
    case RecoveryAction::WaitFeedback: return "wait_feedback_or_safe_measurement_then_start_if_blocked";
    case RecoveryAction::WaitBus: return "wait_bus_then_new_start";
    case RecoveryAction::NewInput: return "fresh_input_then_new_start";
    case RecoveryAction::NewStart: return "new_start";
    case RecoveryAction::Acknowledge: return "stop_and_acknowledge";
    case RecoveryAction::SupplyReference: return "supply_reference_then_new_start";
    case RecoveryAction::FixConfiguration: return "fix_configuration";
    }
    return "unknown";
}
const char *sessionCauseName(const SessionCause &cause) {
    switch (cause.control_issue) {
    case ControlIssue::NotConfigured: return "not_configured";
    case ControlIssue::NotActive: return "output_not_permitted";
    case ControlIssue::FeedbackStale: return "feedback_expired";
    case ControlIssue::MissingFeedback: return "missing_feedback_fields";
    case ControlIssue::ReferenceLost: return "position_reference_lost";
    case ControlIssue::InvalidMeasurement: return "invalid_measurement";
    case ControlIssue::SpeedLimit: return "speed_limit";
    case ControlIssue::TemperatureLimit: return "temperature_limit";
    case ControlIssue::InvalidTarget: return "invalid_target";
    case ControlIssue::InvalidPeriod: return "invalid_control_period";
    case ControlIssue::OutputRejected: return "output_rejected";
    case ControlIssue::None: break;
    }
    switch (cause.driver_reason) {
    case motor::FaultReason::FeedbackExpired: return "feedback_expired";
    case motor::FaultReason::CommandExpired: return "command_expired";
    case motor::FaultReason::DriveFault: return "drive_fault";
    case motor::FaultReason::UnexpectedDisabled: return "drive_disabled";
    case motor::FaultReason::InvalidCommand: return "invalid_command";
    case motor::FaultReason::ControlRejected: return "control_rejected";
    case motor::FaultReason::EnableTimeout: return "enable_timeout";
    case motor::FaultReason::RxOverflow: return "rx_overflow";
    case motor::FaultReason::TransportError:
        return cause.error == -ENETUNREACH ? "can_bus_off" : "can_transport_error";
    case motor::FaultReason::None: break;
    }
    return cause.error == 0 ? "none" : sessionOperationName(cause.operation);
}

} // namespace skywalker::control
