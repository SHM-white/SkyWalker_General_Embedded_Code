#pragma once
#include <control/velocity_motor.hpp>
#include <core/measurement.hpp>
#include <robotics/execution/recovery_gate.hpp>
#include <robotics/messages/interboard.hpp>

namespace skywalker::robotics {
struct BigYawExecutionInputs {
    BigYawRequest request{};
    std::uint64_t local_boot_id = 0;
    bool peer_online = false;
    bool transport_ready = false;
    bool emergency_stop = false;
    bool clear_fault = false;
};
// A dedicated velocity inner loop and recovery context. No absolute mechanical
// zero and no chassis/wheel recovery authority are required. The application
// attaches/starts the drive and commits its physical CAN once per cycle.
class BigYawExecutor {
public:
    struct Config {
        RecoveryGate::Config recovery{};
        std::uint32_t permission_timeout_ms = 300, fault_retry_ms = 100;
        core::TimeUs max_cycle_us = 20000;
        float rate_abs_max_rad_s = 1;
        float motor_to_joint_ratio = 1, direction = 1;
    };
    BigYawExecutor(motor::Motor &, const control::VelocityMotor::Config &, const Config &);
    [[nodiscard]] int begin();
    RunStatus update(const BigYawExecutionInputs &, core::TimeUs now_us);
    RunStatus suspend(core::TimeUs now_us, WaitReason reason, int error = 0, bool blocked = false);
    BigYawFeedback feedback() const; // Execution-thread owner; publish copies.
    const RunStatus &status() const { return status_; }
private:
    RunStatus publish(core::TimeUs, RunState, WaitReason, int error = 0);
    void withdraw(WaitReason, int, bool);
    motor::Motor &drive_;
    control::VelocityMotor axis_;
    const Config config_;
    RecoveryGate recovery_;
    RunStatus status_{};
    core::TimeUs previous_us_ = 0;
    std::uint64_t retry_ms_ = 0;
    std::uint32_t production_sequence_ = 0;
    int config_error_ = 0, latched_error_ = 0;
    bool configured_ = false, begin_attempted_ = false, emergency_latched_ = false;
};
} // namespace skywalker::robotics
