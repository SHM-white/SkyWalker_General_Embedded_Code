#pragma once

#include <drivers/imu/imu_types.hpp>
#include <drivers/motor/motor.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>

namespace skywalker::robotics {

struct InertialGimbalInputs {
    // Preserve acquisition stamps/reference; reading a snapshot is not production.
    imu::Snapshot head{};
    motor::MotorSnapshot yaw{}, pitch{};
    GimbalCommand command{};
    core::Stamp source_stamp{};
    bool prerequisites_ready = false;
};

struct InertialGimbalOutput {
    GimbalCommand command{}; // Joint-space Rate command for GimbalExecutor.
    core::Stamp source_stamp{};
    RunStatus status{};
    float head_yaw_rad = 0, head_pitch_rad = 0;
    float yaw_error_rad = 0, pitch_error_rad = 0;
    bool stabilization_valid = false;
};

// One execution-thread owner. Adapts a head inertial goal into joint commands;
// mechanical GimbalAxis remains responsible for motor loops and hard limits.
// A changed IMU/motor reference creates a new session and rejects pre-session
// inputs. This object never enables a motor or submits a physical CAN bus.
class InertialGimbalAdapter {
public:
    struct Config {
        GimbalAxisConfig yaw{}, pitch{};
        core::TimeUs imu_timeout_us = 20000, source_timeout_us = 100000;
        std::uint32_t command_timeout_ms = 100, mechanical_timeout_ms = 50;
        core::TimeUs max_cycle_us = 20000;
        float yaw_direction = 1, pitch_direction = 1;
        float yaw_kp = 3, pitch_kp = 3, gyro_damping = 0.1f;
        float max_inertial_pitch_rad = 0.9f;
        float stable_yaw_error_rad = 0.05f, stable_pitch_error_rad = 0.05f;
        bool pitch_locked = true;
        bool allow_unknown_quality = false;
    };

    explicit InertialGimbalAdapter(const Config &config) : config_(config) {}
    InertialGimbalOutput update(const InertialGimbalInputs &, core::TimeUs now_us);
    InertialGimbalOutput suspend(core::TimeUs now_us, WaitReason reason, int error = 0);
    std::uint32_t generation() const { return generation_; }

private:
    InertialGimbalOutput publish(core::TimeUs, WaitReason, int, bool active = false);
    void withdraw();
    Config config_;
    InertialGimbalOutput output_{};
    core::OrientationReference reference_{};
    std::uint64_t yaw_reference_ = 0, pitch_reference_ = 0;
    core::TimeUs boundary_us_ = 0, previous_us_ = 0;
    std::uint32_t generation_ = 0, sequence_ = 0;
    float yaw_goal_rad_ = 0, pitch_goal_rad_ = 0;
    ControlSource goal_source_ = ControlSource::None;
    GimbalMode goal_mode_ = GimbalMode::Disabled;
    bool prepared_ = false, target_valid_ = false;
};

} // namespace skywalker::robotics
