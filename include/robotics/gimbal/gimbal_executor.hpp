#pragma once

#include <core/measurement.hpp>
#include <drivers/motor/group.hpp>
#include <robotics/execution/recovery_gate.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>

namespace skywalker::robotics {

struct GimbalExecutionInputs {
    GimbalCommand command{};
    // Source production time, preserved independently of arbitration time.
    core::Stamp source_stamp{};
    OutputPermission permission{};
    bool require_permission = false;
    bool transport_ready = false;
    bool emergency_stop = false;
    bool clear_fault = false;
};

// One execution-thread owner. The application owns topology, attach/start and
// exactly one commit per physical bus after every mechanism stages its output.
// Both motors and their dedicated two-axis fault Group must outlive this object.
// This executor never owns, starts or commits a CAN controller.
class GimbalExecutor {
public:
    struct Config {
        RecoveryGate::Config recovery{};
        std::uint32_t permission_timeout_ms = 300;
        core::TimeUs max_cycle_us = 20000;
        std::uint32_t fault_retry_ms = 100;
    };

    GimbalExecutor(motor::Motor &yaw_drive, motor::Motor &pitch_drive, motor::Group &group,
                   const control::PositionMotor::Config &yaw_motor, const GimbalAxisConfig &yaw_axis,
                   const control::PositionMotor::Config &pitch_motor, const GimbalAxisConfig &pitch_axis,
                   const Config &config);
    // After every application-owned bus starts; configures axes without enabling.
    [[nodiscard]] int begin();
    RunStatus update(const GimbalExecutionInputs &inputs, core::TimeUs now_us);
    // Immediate withdrawal after the application's commit reports failure.
    RunStatus suspend(core::TimeUs now_us, WaitReason reason, int error = 0, bool blocked = false);
    double yawTargetRad() const { return yaw_.targetAngleRad(); }
    double pitchTargetRad() const { return pitch_.targetAngleRad(); }
    const RunStatus &status() const { return status_; } // Execution-thread only.

private:
    RunStatus publish(core::TimeUs now_us, RunState state, WaitReason reason, int error = 0);
    void withdraw(WaitReason reason, int error = 0, bool blocked = false);
    motor::Motor &yaw_drive_, &pitch_drive_;
    motor::Group &group_;
    GimbalAxis yaw_, pitch_;
    const Config config_;
    RecoveryGate recovery_;
    RunStatus status_{};
    core::TimeUs previous_us_ = 0;
    std::uint64_t retry_ms_ = 0, yaw_reference_ = 0, pitch_reference_ = 0;
    std::uint32_t production_sequence_ = 0;
    int config_error_ = 0, hard_error_ = 0;
    WaitReason hard_reason_ = WaitReason::Drive;
    bool configured_ = false, begin_attempted_ = false, have_time_ = false;
    bool have_reference_ = false, emergency_latched_ = false, explicit_clear_pending_ = false;
};

} // namespace skywalker::robotics
