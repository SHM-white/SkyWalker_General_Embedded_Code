#pragma once

#include <control/position_motor.hpp>

namespace skywalker::robotics {

struct HeadImuVisionInputs {
    float head_yaw_rad = 0.0f;
    float head_pitch_rad = 0.0f;
    float head_yaw_rate_rad_s = 0.0f;
    float head_pitch_rate_rad_s = 0.0f;
    bool head_valid = false;

    float yaw_target_rad = 0.0f;
    float yaw_target_rate_rad_s = 0.0f;
    float yaw_target_acc_rad_s2 = 0.0f;
    bool yaw_target_valid = false;

    float pitch_target_rad = 0.0f;
    float pitch_target_rate_rad_s = 0.0f;
    float pitch_target_acc_rad_s2 = 0.0f;
    bool pitch_target_valid = false;

    float yaw_joint_rad = 0.0f;
    bool yaw_joint_valid = false;
    float pitch_joint_rad = 0.0f;
    bool pitch_joint_valid = false;

    float carrier_rate_rad_s = 0.0f;
    bool carrier_valid = false;
    float yaw_outward_budget_pos_rad_s = 0.0f;
    float yaw_outward_budget_neg_rad_s = 0.0f;
    bool yaw_budget_valid = false;
};

struct HeadImuVisionOutput {
    bool yaw_valid = false;
    bool pitch_valid = false;
    bool yaw_limit_clamped = false;
    bool pitch_limit_clamped = false;
    control::PositionMotor::ExternalVelocityInput yaw_input{};
    control::PositionMotor::ExternalVelocityInput pitch_input{};
};

// Outer world-attitude loop for vision mode using the Head CAN IMU directly,
// producing motor-sign velocity commands + soft-limit braking protection for the
// Head-IMU gyro inner velocity loop inside PositionMotor::updateExternalVelocity.
class HeadImuVisionController {
public:
    struct Config {
        float yaw_direction = 1.0f;
        float pitch_direction = 1.0f;
        float yaw_min_rad = 0.0f;
        float yaw_max_rad = 0.0f;
        float pitch_min_rad = 0.0f;
        float pitch_max_rad = 0.0f;
        float max_inertial_pitch_rad = 0.9f;

        float yaw_angle_kp = 15.0f;
        float yaw_angle_deadband_rad = 0.0015f;
        float yaw_rate_ff_gain = 1.0f;
        float yaw_max_rate_rad_s = 5.0f;

        float pitch_angle_kp = 14.0f;
        float pitch_angle_deadband_rad = 0.0015f;
        float pitch_rate_ff_gain = 1.0f;
        float pitch_max_rate_rad_s = 4.0f;

        float yaw_brake_decel_rad_s2 = 12.0f;
        float yaw_limit_margin_rad = 0.03f;
        float pitch_brake_decel_rad_s2 = 20.0f;
        float pitch_limit_margin_rad = 0.02f;
        float limit_return_kp = 6.0f;
    };

    explicit HeadImuVisionController(const Config &config) : config_(config) {
    }

    int validate() const;
    HeadImuVisionOutput update(const HeadImuVisionInputs &in) const;
    const Config &config() const {
        return config_;
    }

private:
    Config config_{};
};

} // namespace skywalker::robotics
