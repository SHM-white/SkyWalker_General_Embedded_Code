#pragma once

#include <array>
#include <span>
#include <variant>
#include <control/position_motor.hpp>
#include <control/velocity_motor.hpp>
#include <core/measurement.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>

namespace skywalker::control {

enum class SessionState { Stopped, Starting, Running, Blocked };
enum class SessionPhase { Idle, Prepare, FreshInput, Enable };
enum class ReferencePolicy {
    NotRequired, CaptureOnExplicitStart, UseCalibratedAbsolute, RequireExternalReference,
};
enum class SessionOperation {
    None, Configure, Source, Target, Period, Feedback, Reference, Reset,
    Enable, Control, Publish, Transport, DriverFault, Acknowledge,
};
enum class RecoveryAction { None, WaitFeedback, WaitBus, NewInput, NewStart, Acknowledge, SupplyReference, FixConfiguration };

struct SessionCause {
    const char *member = "session";
    const char *bus = "none";
    SessionOperation operation = SessionOperation::None;
    ControlIssue control_issue = ControlIssue::None;
    motor::FaultReason driver_reason = motor::FaultReason::None;
    RecoveryAction action = RecoveryAction::None;
    int error = 0;
    bool timed_out = false;
    core::TimeUs occurred_us = 0;
    // Recovery evidence is retained with the stop, even after the bus recovers.
    bool has_recovery = false;
    motor::BusRecoverySnapshot recovery{};
};

struct SessionStatus {
    SessionState state = SessionState::Stopped;
    SessionPhase phase = SessionPhase::Idle;
    bool can_start = false;
    SessionCause current_blocker{};
    SessionCause last_stop{};
    std::uint64_t stop_event = 0;
    std::uint64_t enable_generation = 0;
    core::TimeUs started_us = 0;
    std::size_t member_count = 0;
    // TX completion / drive confirmation is not mechanical standstill.
    std::array<motor::StopReport, motor::kMaxGroupMembers> stops{};
};

class MotorSession {
public:
    struct Member {
        const char *name;
        motor::Motor *motor;
        motor::CanBus *bus;
        std::variant<VelocityMotor *, PositionMotor *> controller;
        ReferencePolicy reference = ReferencePolicy::NotRequired;
    };
    struct Config {
        core::TimeUs source_timeout_us = 100000;
        core::TimeUs target_timeout_us = 100000;
        core::TimeUs max_cycle_us = 20000;
        core::TimeUs prepare_timeout_us = 3000000;
        core::TimeUs enable_timeout_us = 1000000;
    };
    struct Input {
        bool enabled = false;
        // Strictly increasing event counters; holding Start never retries a stop.
        std::uint64_t start_sequence = 0;
        std::uint64_t acknowledge_sequence = 0;
        core::Stamp source{};
        core::Stamp target_stamp{};
        // In member order: rad/s for VelocityMotor, rad for PositionMotor.
        std::span<const double> targets{};
    };

    explicit MotorSession(std::span<const Member> members);
    MotorSession(std::span<const Member> members, Config config);
    MotorSession(const MotorSession &) = delete;
    MotorSession &operator=(const MotorSession &) = delete;
    // Session, controllers, motors and buses must outlive their I/O workers.
    // Setup once, then one application thread calls step(). No thread is created.
    [[nodiscard]] int configure();
    void step(const Input &input, core::TimeUs now_us);
    SessionStatus status() const;

private:
    static auto collectMotors(std::span<const Member> members);
    SessionCause cause(SessionOperation op, int error, RecoveryAction action,
                       core::TimeUs now, std::size_t index = motor::kMaxGroupMembers) const;
    SessionCause readiness(core::TimeUs now) const;
    SessionCause driverFailure(core::TimeUs now) const;
    void stop(const SessionCause &reason);
    void publish();
    void advance(const Input &input, core::TimeUs now);

    std::array<Member, motor::kMaxGroupMembers> members_{};
    std::array<motor::Motor *, motor::kMaxGroupMembers> motors_{};
    std::size_t count_ = 0;
    motor::Group group_;
    Config config_;
    std::array<motor::CanBus *, motor::kMaxGroupMembers> buses_{};
    std::size_t bus_count_ = 0;
    bool configure_attempted_ = false;
    int configuration_error_ = 0;
    bool configured_ = false;
    SessionStatus working_{};
    mutable k_spinlock status_lock_{};
    SessionStatus published_{};
    std::uint64_t start_seen_ = 0, acknowledge_seen_ = 0;
    core::TimeUs attempt_us_ = 0, prepared_us_ = 0, enable_us_ = 0, previous_us_ = 0;
    bool have_previous_ = false;
};

const char *sessionStateName(SessionState state);
const char *sessionOperationName(SessionOperation operation);
const char *recoveryActionName(RecoveryAction action);
const char *sessionCauseName(const SessionCause &cause);

} // namespace skywalker::control
