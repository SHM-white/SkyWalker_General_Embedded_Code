#pragma once

#include <core/measurement.hpp>
#include <drivers/motor/group.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/command/command_source.hpp>
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
    bool yaw_output_valid = true, pitch_output_valid = true;
    bool emergency_stop = false;
    bool clear_estop = false;
};

// One execution-thread owner. The application owns topology, attach/start and
// exactly one commit per physical bus after every mechanism stages its output.
// Both motors and their two-axis batch Group must outlive this object.
// This executor never owns, starts or commits a CAN controller.
class GimbalExecutor {
public:
    struct Config {
        std::uint32_t command_timeout_ms = 100;
        core::TimeUs source_timeout_us = 100000;
        std::uint32_t permission_timeout_ms = 300;
        core::TimeUs max_cycle_us = 20000;
    };

    GimbalExecutor(motor::Motor &yaw_drive, motor::Motor &pitch_drive, motor::Group &group,
                   const control::PositionMotor::Config &yaw_motor, const GimbalAxisConfig &yaw_axis,
                   const control::PositionMotor::Config &pitch_motor, const GimbalAxisConfig &pitch_axis,
                   const Config &config);
    // After every application-owned bus starts; configures axes without enabling.
    [[nodiscard]] int begin();
    RunStatus update(const GimbalExecutionInputs &inputs, core::TimeUs now_us);
    // Explicit input withdrawal (stop, emergency, expired input).
    RunStatus suspend(core::TimeUs now_us, WaitReason reason, int error = 0, bool blocked = false);
    double yawTargetRad() const {
        return yaw_.targetAngleRad();
    }
    double pitchTargetRad() const {
        return pitch_.targetAngleRad();
    }
    control::PositionMotor::Telemetry yawTelemetry() const {
        return yaw_.telemetry();
    }
    control::PositionMotor::Telemetry pitchTelemetry() const {
        return pitch_.telemetry();
    }
    const GimbalAxis::TargetStatus &yawTargetDiagnostics() const {
        return yaw_.targetStatus();
    }
    const GimbalAxis::TargetStatus &pitchTargetDiagnostics() const {
        return pitch_.targetStatus();
    } // Execution-thread only.
    const RunStatus &status() const {
        return status_;
    } // Execution-thread only.

private:
    RunStatus publish(core::TimeUs now_us, RunState state, WaitReason reason, int error = 0);
    void withdraw(WaitReason reason, int error = 0, bool blocked = false);
    motor::Motor &yaw_drive_, &pitch_drive_;
    motor::Group &group_;
    GimbalAxis yaw_, pitch_;
    const Config config_;
    RunStatus status_{};
    core::TimeUs previous_us_ = 0;
    std::uint32_t production_sequence_ = 0;
    int config_error_ = 0;
    bool configured_ = false, begin_attempted_ = false, have_time_ = false;
};

} // namespace skywalker::robotics
