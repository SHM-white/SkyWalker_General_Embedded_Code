#pragma once
#include <algorithm>
#include <cmath>
#include <robotics/messages/command.hpp>

namespace skywalker::robotics {
// Reference v4 ChassisControl carries +X forward/+Y left in the *gimbal* frame.
// A calibrated big-Yaw encoder zero defines its orientation relative to the body;
// no IMU startup heading is needed for ordinary translation.
inline ChassisCommand referenceV4ChassisToBody(ChassisCommand input, bool encoder_valid, float encoder_rad,
                                               float forward_zero_rad, float joint_direction, float gear_ratio) {
    if (input.mode == ChassisMode::Disabled)
        return input;
    if (!encoder_valid || !std::isfinite(encoder_rad) || !std::isfinite(forward_zero_rad) ||
        !std::isfinite(joint_direction) || !std::isfinite(gear_ratio) || gear_ratio <= 0 ||
        (joint_direction != -1.0f && joint_direction != 1.0f)) {
        input.vx_m_s = input.vy_m_s = 0;
        return input; // A valid spin command can continue without a translation reference.
    }
    const float vx = input.vx_m_s, vy = input.vy_m_s;
    const bool spinning = std::fabs(input.wz_rad_s) > 1e-4f;
    const float scale = spinning ? (std::fabs(vy) > 1e-4f ? .4f : std::fabs(vx) > 1e-4f ? .3f : 1.0f) : 1.0f;
    const float theta = std::remainder(joint_direction * (encoder_rad - forward_zero_rad) / gear_ratio,
                                       6.2831853071795864769f);
    const float c = std::cos(theta), sn = std::sin(theta);
    input.vx_m_s = (c * vx - sn * vy) * scale;
    input.vy_m_s = (sn * vx + c * vy) * scale;
    return input;
}

// The wire request's mode=1 rate is the desired head/world rate, whereas the
// legacy local velocity controller expects joint-relative rad/s.
// Use body gyro feedforward and head gyro feedback; keep the local motor limit.
inline float referenceV4WorldToJointRate(float desired_world_rate, float body_rate, float head_rate) {
    if (!std::isfinite(desired_world_rate) || !std::isfinite(body_rate) || !std::isfinite(head_rate))
        return 0;
    return std::clamp(desired_world_rate - body_rate + .25f * (desired_world_rate - head_rate), -10.0f, 10.0f);
}
} // namespace skywalker::robotics
