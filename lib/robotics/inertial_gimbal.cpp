#include <robotics/gimbal/inertial_gimbal.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <core/attitude.hpp>

namespace skywalker::robotics {
namespace {
float wrap(float angle) { return std::remainder(angle, 6.2831853071795864769f); }
bool axisValid(const GimbalAxisConfig &axis) {
    return std::isfinite(axis.min_angle_rad) && std::isfinite(axis.max_angle_rad) &&
           std::isfinite(axis.max_rate_rad_s) && axis.max_rate_rad_s > 0 &&
           (axis.topology == AxisTopology::Continuous || axis.min_angle_rad < axis.max_angle_rad);
}
bool jointHealthy(const motor::MotorSnapshot &m, const GimbalAxisConfig &axis,
                  std::uint64_t now_ms, std::uint32_t timeout_ms) {
    const auto &f = m.feedback;
    const bool continuous = axis.topology == AxisTopology::Continuous;
    const auto required = motor::FeedbackVelocity | (continuous ? motor::FeedbackAbsolutePosition : motor::FeedbackPosition);
    return m.feedback_fresh && (continuous || m.position_reference_valid) && (f.valid & required) == required &&
           std::isfinite(continuous ? f.absolute_position_rad : f.position_rad) && std::isfinite(f.velocity_rad_s) &&
           now_ms >= f.timestamp_ms && now_ms - f.timestamp_ms <= timeout_ms &&
           (axis.topology == AxisTopology::Continuous ||
            (f.position_rad >= axis.min_angle_rad && f.position_rad <= axis.max_angle_rad));
}
float stopAtLimits(float rate, float position, const GimbalAxisConfig &axis) {
    rate = std::clamp(rate, -axis.max_rate_rad_s, axis.max_rate_rad_s);
    if (axis.topology == AxisTopology::Limited &&
        ((position <= axis.min_angle_rad && rate < 0) || (position >= axis.max_angle_rad && rate > 0)))
        return 0;
    return rate;
}
} // namespace

void InertialGimbalAdapter::withdraw() {
    prepared_ = target_valid_ = false;
    goal_source_ = ControlSource::None;
    goal_mode_ = GimbalMode::Disabled;
    output_.command = {};
    output_.source_stamp = {};
    output_.stabilization_valid = false;
    output_.yaw_output_valid = output_.pitch_output_valid = false;
}

InertialGimbalOutput InertialGimbalAdapter::publish(core::TimeUs now, WaitReason reason, int error,
                                                  bool active) {
    output_.status.state = active ? RunState::Active : RunState::Recovering;
    output_.status.reason = reason;
    output_.status.error = error;
    output_.status.ready = prepared_;
    output_.status.requested = target_valid_;
    output_.status.member_count = 2;
    output_.status.active_count = unsigned(output_.yaw_output_valid) + unsigned(output_.pitch_output_valid);
    output_.status.waiting_count = 2 - output_.status.active_count;
    output_.status.stamp = {now / 1000, ++sequence_, true};
    return output_;
}

InertialGimbalOutput InertialGimbalAdapter::suspend(core::TimeUs now, WaitReason reason, int error) {
    withdraw();
    return publish(now, reason, error);
}

InertialGimbalOutput InertialGimbalAdapter::update(const InertialGimbalInputs &in, core::TimeUs now) {
    const auto previous = previous_us_;
    previous_us_ = now;
    const auto &c = config_;
    const bool cycle = previous && now > previous && now - previous <= c.max_cycle_us;
    if (!axisValid(c.yaw) || !axisValid(c.pitch) || !c.imu_timeout_us || !c.source_timeout_us ||
        !c.command_timeout_ms || !c.mechanical_timeout_ms || !c.max_cycle_us ||
        !std::isfinite(c.yaw_kp) || c.yaw_kp < 0 || !std::isfinite(c.pitch_kp) || c.pitch_kp < 0 ||
        !std::isfinite(c.gyro_damping) || c.gyro_damping < 0 ||
        std::fabs(c.yaw_direction) != 1 || std::fabs(c.pitch_direction) != 1 ||
        !std::isfinite(c.max_inertial_pitch_rad) || c.max_inertial_pitch_rad <= 0 ||
        c.max_inertial_pitch_rad >= 1.4f || !std::isfinite(c.stable_yaw_error_rad) ||
        !std::isfinite(c.stable_pitch_error_rad) || c.stable_yaw_error_rad < 0 || c.stable_pitch_error_rad < 0)
        return suspend(now, WaitReason::Configuration, -EINVAL);
    output_.yaw_output_valid = output_.pitch_output_valid = false;
    output_.stabilization_valid = false;
    output_.command = in.command;
    output_.command.mode = GimbalMode::Rate;
    output_.command.yaw_rate_rad_s = output_.command.pitch_rate_rad_s = 0;
    output_.source_stamp = in.source_stamp;
    if (in.command.mode == GimbalMode::Disabled) {
        auto out = suspend(now, WaitReason::Command);
        out.status.state = output_.status.state = RunState::Disabled;
        return out;
    }
    if (in.command.mode > GimbalMode::AbsoluteAngle || !std::isfinite(in.command.yaw_target_rad) ||
        !std::isfinite(in.command.pitch_target_rad) || !std::isfinite(in.command.yaw_rate_rad_s) ||
        !std::isfinite(in.command.pitch_rate_rad_s)) return suspend(now, WaitReason::Command, -EINVAL);
    if (!isFresh(in.command.stamp, now / 1000, c.command_timeout_ms) ||
        !core::fresh(in.source_stamp, now, c.source_timeout_us)) return suspend(now, WaitReason::Command, -ESTALE);
    if (!in.prerequisites_ready) return publish(now, WaitReason::Transport, 0);
    const auto &head = in.head.sample;
    const bool quality = head.attitude_quality == imu::AttitudeQuality::Tracking ||
        (c.allow_unknown_quality && head.attitude_quality == imu::AttitudeQuality::Unknown);
    const auto fields = imu::Orientation | imu::Gyro;
    auto quaternion = head.orientation.value;
    if (in.head.state != imu::State::Running || (in.head.fresh_mask & fields) != fields || !quality ||
        !core::fresh(head.orientation.stamp, now, c.imu_timeout_us) ||
        !core::fresh(head.gyro_rad_s.stamp, now, c.imu_timeout_us) ||
        !core::finite(head.gyro_rad_s.value) || !core::normalize(quaternion) ||
        !head.reference.frame_id || !head.reference.epoch)
        return publish(now, WaitReason::Reference, 0);
    const bool yaw_available = jointHealthy(in.yaw, c.yaw, now / 1000, c.mechanical_timeout_ms);
    const bool pitch_available = jointHealthy(in.pitch, c.pitch, now / 1000, c.mechanical_timeout_ms);
    const auto e = core::euler(quaternion);
    output_.head_yaw_rad = e.yaw;
    output_.head_pitch_rad = e.pitch;
    if (std::fabs(e.pitch) >= c.max_inertial_pitch_rad)
        return suspend(now, WaitReason::Reference, -ERANGE);
    if (!prepared_ || reference_ != head.reference) {
        reference_ = head.reference;
        prepared_ = true;
        target_valid_ = false;
    }
    const float dt = cycle ? float(now - previous) * 1e-6f : 0;
    if (!target_valid_ || goal_source_ != in.command.source || goal_mode_ != in.command.mode) {
        yaw_goal_rad_ = e.yaw; pitch_goal_rad_ = e.pitch;
        goal_source_ = in.command.source; goal_mode_ = in.command.mode; target_valid_ = true;
    }
    float desired_yaw_rate = 0, desired_pitch_rate = 0;
    if (in.command.mode == GimbalMode::AbsoluteAngle) {
        yaw_goal_rad_ = wrap(in.command.yaw_target_rad);
        pitch_goal_rad_ = std::clamp(in.command.pitch_target_rad,
                                     -c.max_inertial_pitch_rad, c.max_inertial_pitch_rad);
    }
    else if (in.command.mode == GimbalMode::Rate) {
        desired_yaw_rate = std::clamp(in.command.yaw_rate_rad_s, -c.yaw.max_rate_rad_s, c.yaw.max_rate_rad_s);
        desired_pitch_rate = c.pitch_locked ? 0 : std::clamp(in.command.pitch_rate_rad_s,
                                                            -c.pitch.max_rate_rad_s, c.pitch.max_rate_rad_s);
        yaw_goal_rad_ = wrap(yaw_goal_rad_ + desired_yaw_rate * dt);
        pitch_goal_rad_ = std::clamp(pitch_goal_rad_ + desired_pitch_rate * dt,
                                     -c.max_inertial_pitch_rad, c.max_inertial_pitch_rad);
    }
    output_.yaw_error_rad = wrap(yaw_goal_rad_ - e.yaw);
    output_.pitch_error_rad = pitch_goal_rad_ - e.pitch;
    const auto gyro = head.gyro_rad_s.value;
    const float yaw_rate = (std::sin(e.roll) * gyro.y + std::cos(e.roll) * gyro.z) / std::cos(e.pitch);
    const float pitch_rate = std::cos(e.roll) * gyro.y - std::sin(e.roll) * gyro.z;
    // Estimate carrier motion from head rate minus measured joint contribution.
    // TODO(calibration): validate axis projection and tune damping on the mounted
    // head before opening the full Pitch envelope or vehicle-motion tests.
    const float carrier_yaw_rate = yaw_available ? yaw_rate - c.yaw_direction * in.yaw.feedback.velocity_rad_s : 0;
    const float carrier_pitch_rate = pitch_available ? pitch_rate - c.pitch_direction * in.pitch.feedback.velocity_rad_s : 0;
    output_.command = in.command;
    output_.command.mode = GimbalMode::Rate;
    output_.command.yaw_rate_rad_s = yaw_available && cycle ? stopAtLimits(c.yaw_direction *
        (desired_yaw_rate + c.yaw_kp * output_.yaw_error_rad - carrier_yaw_rate -
         c.gyro_damping * (yaw_rate - desired_yaw_rate)), in.yaw.feedback.position_rad, c.yaw) : 0;
    output_.command.pitch_rate_rad_s = c.pitch_locked || !pitch_available || !cycle ? 0 : stopAtLimits(c.pitch_direction *
        (desired_pitch_rate + c.pitch_kp * output_.pitch_error_rad - carrier_pitch_rate -
         c.gyro_damping * (pitch_rate - desired_pitch_rate)), in.pitch.feedback.position_rad, c.pitch);
    output_.source_stamp = in.source_stamp;
    output_.status.last_command_sequence = in.command.stamp.sequence;
    output_.yaw_output_valid = yaw_available && cycle;
    output_.pitch_output_valid = pitch_available && cycle;
    output_.stabilization_valid = yaw_available && (c.pitch_locked || pitch_available) && std::fabs(output_.yaw_error_rad) <= c.stable_yaw_error_rad &&
        (c.pitch_locked || std::fabs(output_.pitch_error_rad) <= c.stable_pitch_error_rad);
    return publish(now, !cycle ? WaitReason::Cycle : !yaw_available || !pitch_available ? WaitReason::Feedback : WaitReason::None,
        0, output_.yaw_output_valid || output_.pitch_output_valid);
}

} // namespace skywalker::robotics
