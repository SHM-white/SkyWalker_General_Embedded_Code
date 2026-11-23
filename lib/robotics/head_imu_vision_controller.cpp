#include <robotics/gimbal/head_imu_vision_controller.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>

namespace skywalker::robotics {
namespace {
constexpr float kTwoPi = 6.28318530717958647692f;

float softDeadband(float error, float deadband) {
    return std::copysign(std::max(std::fabs(error) - deadband, 0.0f), error);
}

float brakeBudget(float distance, float decel, float max_rate) {
    if (!(distance > 0.0f))
        return 0.0f;
    return std::min(std::sqrt(2.0f * decel * distance), max_rate);
}
} // namespace

int HeadImuVisionController::validate() const {
    const auto &c = config_;
    for (float x : {c.yaw_direction, c.pitch_direction, c.yaw_min_rad, c.yaw_max_rad, c.pitch_min_rad,
                    c.pitch_max_rad, c.max_inertial_pitch_rad, c.yaw_angle_kp, c.yaw_angle_deadband_rad,
                    c.yaw_rate_ff_gain, c.yaw_max_rate_rad_s, c.pitch_angle_kp, c.pitch_angle_deadband_rad,
                    c.pitch_rate_ff_gain, c.pitch_max_rate_rad_s, c.yaw_brake_decel_rad_s2, c.yaw_limit_margin_rad,
                    c.pitch_brake_decel_rad_s2, c.pitch_limit_margin_rad, c.limit_return_kp}) {
        if (!std::isfinite(x))
            return -EINVAL;
    }
    if (std::fabs(c.yaw_direction) != 1.0f || std::fabs(c.pitch_direction) != 1.0f)
        return -EINVAL;
    if (c.yaw_min_rad >= c.yaw_max_rad || c.pitch_min_rad >= c.pitch_max_rad)
        return -EINVAL;
    if (c.max_inertial_pitch_rad <= 0.0f || c.max_inertial_pitch_rad >= 1.4f)
        return -EINVAL;
    if (c.yaw_angle_kp <= 0.0f || c.yaw_angle_deadband_rad < 0.0f || c.yaw_rate_ff_gain < 0.0f ||
        c.yaw_max_rate_rad_s <= 0.0f)
        return -EINVAL;
    if (c.pitch_angle_kp <= 0.0f || c.pitch_angle_deadband_rad < 0.0f || c.pitch_rate_ff_gain < 0.0f ||
        c.pitch_max_rate_rad_s <= 0.0f)
        return -EINVAL;
    if (c.yaw_brake_decel_rad_s2 <= 0.0f || c.yaw_limit_margin_rad < 0.0f || c.pitch_brake_decel_rad_s2 <= 0.0f ||
        c.pitch_limit_margin_rad < 0.0f || c.limit_return_kp < 0.0f)
        return -EINVAL;
    if (c.yaw_min_rad + c.yaw_limit_margin_rad >= c.yaw_max_rad - c.yaw_limit_margin_rad ||
        c.pitch_min_rad + c.pitch_limit_margin_rad >= c.pitch_max_rad - c.pitch_limit_margin_rad)
        return -EINVAL;
    return 0;
}

HeadImuVisionOutput HeadImuVisionController::update(const HeadImuVisionInputs &in) const {
    HeadImuVisionOutput out{};
    if (validate() < 0 || !in.head_valid || !std::isfinite(in.head_yaw_rad) || !std::isfinite(in.head_pitch_rad) ||
        !std::isfinite(in.head_yaw_rate_rad_s) || !std::isfinite(in.head_pitch_rate_rad_s))
        return out;

    const auto &c = config_;
    if (std::fabs(in.head_pitch_rad) >= c.max_inertial_pitch_rad)
        return out;

    if (in.yaw_target_valid && in.yaw_joint_valid && std::isfinite(in.yaw_target_rad) &&
        std::isfinite(in.yaw_target_rate_rad_s) && std::isfinite(in.yaw_target_acc_rad_s2) &&
        std::isfinite(in.yaw_joint_rad)) {
        const float e_yaw = std::remainder(in.yaw_target_rad - in.head_yaw_rad, kTwoPi);
        const float outer_raw = c.yaw_angle_kp * softDeadband(e_yaw, c.yaw_angle_deadband_rad);
        const float outer = std::clamp(outer_raw, -c.yaw_max_rate_rad_s, c.yaw_max_rate_rad_s);
        const float w_raw = outer + c.yaw_rate_ff_gain * in.yaw_target_rate_rad_s;
        const float w_cmd = std::clamp(w_raw, -c.yaw_max_rate_rad_s, c.yaw_max_rate_rad_s);

        float b_pos = in.yaw_budget_valid && std::isfinite(in.yaw_outward_budget_pos_rad_s)
                          ? std::max(in.yaw_outward_budget_pos_rad_s, 0.0f)
                          : brakeBudget((c.yaw_max_rad - c.yaw_limit_margin_rad) - in.yaw_joint_rad,
                                        c.yaw_brake_decel_rad_s2, c.yaw_max_rate_rad_s);
        float b_neg = in.yaw_budget_valid && std::isfinite(in.yaw_outward_budget_neg_rad_s)
                          ? std::max(in.yaw_outward_budget_neg_rad_s, 0.0f)
                          : brakeBudget(in.yaw_joint_rad - (c.yaw_min_rad + c.yaw_limit_margin_rad),
                                        c.yaw_brake_decel_rad_s2, c.yaw_max_rate_rad_s);
        if (in.yaw_joint_rad >= c.yaw_max_rad)
            b_pos = 0.0f;
        if (in.yaw_joint_rad <= c.yaw_min_rad)
            b_neg = 0.0f;

        const float omega_c = in.carrier_valid && std::isfinite(in.carrier_rate_rad_s) ? in.carrier_rate_rad_s : 0.0f;
        const float qdot_req = c.yaw_direction * (w_cmd - omega_c);
        float qdot_safe = std::clamp(qdot_req, -b_neg, b_pos);
        if (in.yaw_joint_rad < c.yaw_min_rad)
            qdot_safe = std::max(qdot_safe, std::min(c.limit_return_kp * (c.yaw_min_rad - in.yaw_joint_rad),
                                                     c.yaw_max_rate_rad_s));
        if (in.yaw_joint_rad > c.yaw_max_rad)
            qdot_safe = std::min(qdot_safe, -std::min(c.limit_return_kp * (in.yaw_joint_rad - c.yaw_max_rad),
                                                      c.yaw_max_rate_rad_s));

        const bool limit_clamped = qdot_safe != qdot_req;
        const float w_safe = limit_clamped ? (omega_c + c.yaw_direction * qdot_safe) : w_cmd;

        out.yaw_valid = true;
        out.yaw_limit_clamped = limit_clamped;
        out.yaw_input.velocity_request_rad_s = c.yaw_direction * w_safe;
        out.yaw_input.measured_velocity_rad_s = c.yaw_direction * in.head_yaw_rate_rad_s;
        out.yaw_input.position_reference_rad = in.head_yaw_rad;
        out.yaw_input.reference_velocity_rad_s = c.yaw_direction * in.yaw_target_rate_rad_s;
        out.yaw_input.acceleration_ff_rad_s2 = limit_clamped ? 0.0f : (c.yaw_direction * in.yaw_target_acc_rad_s2);
        out.yaw_input.outer_position_error_rad = e_yaw;
        out.yaw_input.outer_output_rad_s = c.yaw_direction * outer;
        out.yaw_input.outer_saturated = outer != outer_raw;
        out.yaw_input.velocity_request_limited = w_cmd != w_raw;
    }

    if (in.pitch_target_valid && in.pitch_joint_valid && std::isfinite(in.pitch_target_rad) &&
        std::isfinite(in.pitch_target_rate_rad_s) && std::isfinite(in.pitch_target_acc_rad_s2) &&
        std::isfinite(in.pitch_joint_rad)) {
        const float p_goal = std::clamp(in.pitch_target_rad, -c.max_inertial_pitch_rad, c.max_inertial_pitch_rad);
        const float e_pitch = p_goal - in.head_pitch_rad;
        const float outer_raw = c.pitch_angle_kp * softDeadband(e_pitch, c.pitch_angle_deadband_rad);
        const float outer = std::clamp(outer_raw, -c.pitch_max_rate_rad_s, c.pitch_max_rate_rad_s);
        const float w_raw = outer + c.pitch_rate_ff_gain * in.pitch_target_rate_rad_s;
        const float w_cmd = std::clamp(w_raw, -c.pitch_max_rate_rad_s, c.pitch_max_rate_rad_s);

        float b_pos = brakeBudget((c.pitch_max_rad - c.pitch_limit_margin_rad) - in.pitch_joint_rad,
                                  c.pitch_brake_decel_rad_s2, c.pitch_max_rate_rad_s);
        float b_neg = brakeBudget(in.pitch_joint_rad - (c.pitch_min_rad + c.pitch_limit_margin_rad),
                                  c.pitch_brake_decel_rad_s2, c.pitch_max_rate_rad_s);
        if (in.pitch_joint_rad >= c.pitch_max_rad)
            b_pos = 0.0f;
        if (in.pitch_joint_rad <= c.pitch_min_rad)
            b_neg = 0.0f;

        const float qdot_req = c.pitch_direction * w_cmd;
        float qdot_safe = std::clamp(qdot_req, -b_neg, b_pos);
        if (in.pitch_joint_rad < c.pitch_min_rad)
            qdot_safe = std::max(qdot_safe, std::min(c.limit_return_kp * (c.pitch_min_rad - in.pitch_joint_rad),
                                                     c.pitch_max_rate_rad_s));
        if (in.pitch_joint_rad > c.pitch_max_rad)
            qdot_safe = std::min(qdot_safe, -std::min(c.limit_return_kp * (in.pitch_joint_rad - c.pitch_max_rad),
                                                      c.pitch_max_rate_rad_s));

        const bool limit_clamped = qdot_safe != qdot_req;

        out.pitch_valid = true;
        out.pitch_limit_clamped = limit_clamped;
        out.pitch_input.velocity_request_rad_s = qdot_safe;
        out.pitch_input.measured_velocity_rad_s = c.pitch_direction * in.head_pitch_rate_rad_s;
        out.pitch_input.position_reference_rad = in.head_pitch_rad;
        out.pitch_input.reference_velocity_rad_s = c.pitch_direction * in.pitch_target_rate_rad_s;
        out.pitch_input.acceleration_ff_rad_s2 = limit_clamped ? 0.0f : (c.pitch_direction * in.pitch_target_acc_rad_s2);
        out.pitch_input.outer_position_error_rad = e_pitch;
        out.pitch_input.outer_output_rad_s = c.pitch_direction * outer;
        out.pitch_input.outer_saturated = outer != outer_raw;
        out.pitch_input.velocity_request_limited = w_cmd != w_raw;
    }

    return out;
}

} // namespace skywalker::robotics
