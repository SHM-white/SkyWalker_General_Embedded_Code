#pragma once
#include <algorithm>
#include <cmath>
#include <robotics/messages/command.hpp>

namespace skywalker::robotics {
// Reference v4 ChassisControl carries +X forward/+Y left in the *gimbal* frame.
// A calibrated big-Yaw encoder zero defines its orientation relative to the body;
// no IMU startup heading is needed for ordinary translation.
//
// encoder_rad must be the motor's continuous native coordinate
// (MotorSnapshot::native_unwrapped_position_rad), not the raw ±PMAX angle:
// one J4310 protocol span is 25 rad, so reducing the raw value after a wrap
// would rotate the translation by |25 - 4*2*pi| = 0.133 rad on every crossing.
inline ChassisCommand referenceV4ChassisToBody(ChassisCommand input, bool encoder_valid, double encoder_rad,
                                               double forward_zero_rad, float joint_direction, float gear_ratio,
                                               float spin_forward_scale = .3f, float spin_lateral_scale = .4f) {
    constexpr double pi = 3.14159265358979323846;
    if (input.mode == ChassisMode::Disabled)
        return input;
    if (!encoder_valid || !std::isfinite(encoder_rad) || !std::isfinite(forward_zero_rad) ||
        !std::isfinite(joint_direction) || !std::isfinite(gear_ratio) || gear_ratio <= 0 ||
        !std::isfinite(spin_forward_scale) || spin_forward_scale < 0 || spin_forward_scale > 1 ||
        !std::isfinite(spin_lateral_scale) || spin_lateral_scale < 0 || spin_lateral_scale > 1 ||
        (joint_direction != -1.0f && joint_direction != 1.0f)) {
        input.vx_m_s = input.vy_m_s = 0;
        return input; // A valid spin command can continue without a translation reference.
    }
    const double vx = input.vx_m_s, vy = input.vy_m_s;
    const bool spinning = std::fabs(input.wz_rad_s) > 1e-4f;
    const double scale = spinning ? (std::fabs(vy) > 1e-4   ? static_cast<double>(spin_lateral_scale)
                                     : std::fabs(vx) > 1e-4 ? static_cast<double>(spin_forward_scale)
                                                            : 1.0)
                                  : 1.0;
    // encoder_rad already extends the protocol wraps; only the physical joint
    // angle is reduced modulo 2*pi, so direction stays continuous for any turn count.
    const double mechanical_angle = static_cast<double>(joint_direction) * (encoder_rad - forward_zero_rad) /
                                    static_cast<double>(gear_ratio);
    const double theta = std::remainder(mechanical_angle, 2.0 * pi);
    const double c = std::cos(theta), sn = std::sin(theta);
    input.vx_m_s = static_cast<float>((c * vx - sn * vy) * scale);
    input.vy_m_s = static_cast<float>((sn * vx + c * vy) * scale);
    return input;
}
} // namespace skywalker::robotics
