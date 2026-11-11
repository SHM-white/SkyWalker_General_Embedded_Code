#pragma once
#include <control/velocity_motor.hpp>
#include <robotics/execution/imu_heading_lock.hpp>
#include <core/measurement.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/command/command_source.hpp>
#include <robotics/messages/interboard.hpp>

namespace skywalker::robotics {
struct BigYawExecutionInputs {
    BigYawRequest request{};
    GimbalImuYaw head{};
    std::uint64_t peer_boot_id = 0;
    std::uint64_t local_boot_id = 0;
    bool peer_online = false;
    bool transport_ready = false;
    bool emergency_stop = false;
    bool clear_estop = false;
};
// A dedicated velocity inner loop and recovery context. No absolute mechanical
// zero and no chassis/wheel recovery authority are required. The application
// attaches/starts the drive and commits its physical CAN once per cycle.
class BigYawExecutor {
public:
    struct Config {
        std::uint32_t command_timeout_ms = 100;
        core::TimeUs source_timeout_us = 100000;
        core::TimeUs max_cycle_us = 20000;
        // Fixed at startup. No automatic fallback between world/relative feedback.
        bool heading_lock_enabled = false, mit_mode_confirmed = false, heading_reference_confirmed = false;
        // Opt-in only: the J4310 must be independently confirmed in native
        // velocity mode (hardware CTRL_MODE=3), not MIT mode.
        bool native_velocity_mode_confirmed = false;
        // Maximum change of the *motor-shaft* velocity request per second.
        float native_velocity_acceleration_rad_s2 = 4.0f;
        ImuHeadingLock::Config heading{};
        float rate_abs_max_rad_s = 1;
        float motor_to_joint_ratio = 1, direction = 1;
    };
    BigYawExecutor(motor::Motor &, const control::VelocityMotor::Config &, const Config &);
    [[nodiscard]] int begin();
    RunStatus update(const BigYawExecutionInputs &, core::TimeUs now_us);
    RunStatus suspend(core::TimeUs now_us, WaitReason reason, int error = 0, bool blocked = false);
    BigYawFeedback feedback() const; // Execution-thread owner; publish copies.
    const RunStatus &status() const {
        return status_;
    }
    // Last successfully staged native velocity, converted to physical joint units.
    // This is not a CAN acknowledgement; callers must also inspect motor/commit status.
    bool nativeVelocityStaged() const {
        return config_.native_velocity_mode_confirmed && native_enable_generation_ && status_.ready;
    }
    float stagedJointRateRadS() const {
        return native_last_rate_rad_s_ * config_.direction / config_.motor_to_joint_ratio;
    }

private:
    RunStatus publish(core::TimeUs, RunState, WaitReason, int error = 0);
    void withdraw(WaitReason, int, bool);
    motor::Motor &drive_;
    control::VelocityMotor axis_;
    const Config config_;
    ImuHeadingLock heading_;
    std::uint64_t heading_enable_generation_ = 0;
    std::uint64_t native_enable_generation_ = 0;
    float native_last_rate_rad_s_ = 0;
    RunStatus status_{};
    core::TimeUs previous_us_ = 0;
    std::uint32_t production_sequence_ = 0;
    int config_error_ = 0;
    bool configured_ = false, begin_attempted_ = false;
};
} // namespace skywalker::robotics
