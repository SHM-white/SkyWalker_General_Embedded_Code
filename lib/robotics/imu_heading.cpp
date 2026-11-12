#include <robotics/chassis/imu_heading.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
namespace skywalker::robotics {
namespace {
constexpr float pi = 3.14159265358979323846f;
float wrap(float a) {
    return std::remainder(a, 2 * pi);
}
}
LocalHeading planarYawHeading(const imu::Snapshot &s, core::TimeUs now) {
    LocalHeading out{};
    const auto &p = s.sample.planar_yaw;
    out.generation = p.generation;
    if (s.state != imu::State::Running)
        out.reason = HeadingWaitReason::SourceNotRunning;
    else if (!(s.capabilities & imu::PlanarYaw))
        out.reason = HeadingWaitReason::ReferenceInvalid;
    else if (p.state != imu::PlanarYawState::Ready) {
        switch (p.state) {
        case imu::PlanarYawState::GyroInvalid:
            out.reason = HeadingWaitReason::GyroInvalid;
            break;
        case imu::PlanarYawState::SampleGap:
            out.reason = HeadingWaitReason::SampleGap;
            break;
        case imu::PlanarYawState::TimeReset:
            out.reason = HeadingWaitReason::ReferenceReset;
            break;
        case imu::PlanarYawState::RateInvalid:
            out.reason = HeadingWaitReason::RateInvalid;
            break;
        default:
            out.reason = HeadingWaitReason::Initializing;
            break;
        }
    }
    else if (!core::fresh(p.yaw_rad.stamp, now, 20000) || !core::fresh(s.sample.gyro_rad_s.stamp, now, 20000))
        out.reason = HeadingWaitReason::GyroStale;
    else if (!p.generation)
        out.reason = HeadingWaitReason::ReferenceInvalid;
    else if (!std::isfinite(p.yaw_rad.value) || !std::isfinite(p.rate_rad_s) || std::fabs(p.rate_rad_s) > 35)
        out.reason = HeadingWaitReason::RateInvalid;
    else {
        out.yaw_rad = p.yaw_rad.value;
        out.rate_rad_s = p.rate_rad_s;
        out.stamp = p.yaw_rad.stamp;
        out.valid = true;
        out.reason = HeadingWaitReason::None;
    }
    return out;
}
LocalHeading ImuHeadingReference::update(const imu::Snapshot &s, bool /*disarmed*/, core::TimeUs now) {
    LocalHeading out{};
    out.generation = initialized_ ? 1 : 0;
    const auto reject = [&](HeadingWaitReason reason) {
        out.reason = reason;
        return out;
    };
    if (blocked_) {
        out.generation = 2;
        return reject(blocked_reason_);
    }
    if (initialized_ && last_sample_us_ && now > last_sample_us_ && now - last_sample_us_ > 500000) {
        blocked_ = true;
        blocked_reason_ = HeadingWaitReason::SampleGap;
        out.generation = 2;
        return reject(blocked_reason_);
    }
    const auto &v = s.sample;
    auto q = v.orientation.value;
    HeadingWaitReason input_reason = HeadingWaitReason::None;
    if (s.state != imu::State::Running)
        input_reason = HeadingWaitReason::SourceNotRunning;
    else if (!core::fresh(v.orientation.stamp, now, 50000))
        input_reason = HeadingWaitReason::OrientationStale;
    else if (!core::fresh(v.gyro_rad_s.stamp, now, 50000))
        input_reason = HeadingWaitReason::GyroStale;
    else if (!v.reference.frame_id || !v.reference.epoch)
        input_reason = HeadingWaitReason::ReferenceInvalid;
    else if (!core::normalize(q))
        input_reason = HeadingWaitReason::OrientationInvalid;
    else if (!core::finite(v.gyro_rad_s.value))
        input_reason = HeadingWaitReason::GyroInvalid;
    else if (v.attitude_quality == imu::AttitudeQuality::Degraded) {
        // A valid gyro prediction is a new observation, not permission to
        // repeat an old heading. Bound it by the last actual Tracking sample,
        // and never bridge initialization or a reference change.
        const bool prediction_allowed = initialized_ && have_reference_ && reference_ == v.reference &&
                                        config_.max_degraded_prediction_us &&
                                        core::fresh(last_tracking_stamp_, now, config_.max_degraded_prediction_us);
        if (!prediction_allowed)
            input_reason = HeadingWaitReason::Degraded;
    }
    else if (v.attitude_quality != imu::AttitudeQuality::Tracking &&
             v.attitude_quality != imu::AttitudeQuality::Unknown)
        input_reason = HeadingWaitReason::QualityUnavailable;
    if (input_reason != HeadingWaitReason::None) {
        stationary_samples_ = 0;
        return reject(input_reason);
    }
    if (have_reference_ && reference_ != v.reference) {
        if (initialized_) {
            blocked_ = true;
            blocked_reason_ = HeadingWaitReason::ReferenceReset;
            out.generation = 2;
            return reject(blocked_reason_);
        }
        stationary_samples_ = 0;
    }
    reference_ = v.reference;
    have_reference_ = true;
    const auto e = core::euler(q);
    if (std::fabs(std::cos(e.pitch)) < .2f)
        return reject(HeadingWaitReason::TiltSingular);
    const auto g = v.gyro_rad_s.value;
    const float rate = (std::sin(e.roll) * g.y + std::cos(e.roll) * g.z) / std::cos(e.pitch);
    if (!std::isfinite(rate) || std::fabs(rate) > 100)
        return reject(HeadingWaitReason::RateInvalid);
    if (v.attitude_quality == imu::AttitudeQuality::Tracking) {
        last_tracking_stamp_ = v.orientation.stamp;
        last_tracking_stamp_.time_us = std::min(v.orientation.stamp.time_us, v.gyro_rad_s.stamp.time_us);
    }
    const bool distinct = v.orientation.stamp.sequence != last_sequence_ &&
                          v.orientation.stamp.time_us > last_sample_us_;
    if (distinct) {
        last_sequence_ = v.orientation.stamp.sequence;
        last_sample_us_ = v.orientation.stamp.time_us;
        if (!initialized_) {
            // RC arming is independent of establishing a stationary IMU zero.
            if (now >= 1000000 && core::norm(g) < .05f)
                ++stationary_samples_;
            else
                stationary_samples_ = 0;
            if (stationary_samples_ >= 20) {
                zero_rad_ = e.yaw;
                initialized_ = true;
            }
        }
    }
    if (!initialized_)
        return reject(HeadingWaitReason::Initializing);
    out.yaw_rad = wrap(e.yaw - zero_rad_);
    out.rate_rad_s = rate;
    out.generation = 1;
    out.valid = true;
    out.reason = HeadingWaitReason::None;
    // The oldest constituent sample bounds the predicted heading's freshness.
    out.stamp = v.orientation.stamp;
    out.stamp.time_us = std::min(v.orientation.stamp.time_us, v.gyro_rad_s.stamp.time_us);
    return out;
}
bool startupSessionsMatch(const BoardHeartbeat &peer, std::uint64_t now, std::uint32_t skew) {
    if (!skew || !peer.sender_boot_id || !isFresh(peer.stamp, now, 100))
        return false;
    const std::uint64_t peer_uptime = std::uint64_t(peer.sender_uptime_ms) + (now - peer.stamp.timestamp_ms);
    return (now > peer_uptime ? now - peer_uptime : peer_uptime - now) <= skew;
}
int ChassisMotionAdapter::validate() const {
    return config_.frame > Frame::Gimbal || !config_.startup_max_skew_ms ||
                   !std::isfinite(config_.heading_prediction_s) || config_.heading_prediction_s < 0 ||
                   config_.heading_prediction_s > .05f || !std::isfinite(config_.spin_forward_scale) ||
                   config_.spin_forward_scale < 0 || config_.spin_forward_scale > 1 ||
                   !std::isfinite(config_.spin_lateral_scale) || config_.spin_lateral_scale < 0 ||
                   config_.spin_lateral_scale > 1 || !std::isfinite(config_.yaw_rate_feedback_kp) ||
                   config_.yaw_rate_feedback_kp < 0 || config_.yaw_rate_feedback_kp > 2 ||
                   !std::isfinite(config_.max_wz_rad_s) || config_.max_wz_rad_s <= 0
               ? -EINVAL
               : 0;
}
ChassisCommand ChassisMotionAdapter::update(const ChassisCommand &cmd, const LocalHeading &body,
                                            const GimbalImuYaw &head, std::uint64_t local_boot, bool disarmed,
                                            std::uint64_t now, const BoardHeartbeat &peer) {
    auto out = cmd;
    translation_ready_ = false;
    if (validate() < 0 || !std::isfinite(cmd.vx_m_s) || !std::isfinite(cmd.vy_m_s) || !std::isfinite(cmd.wz_rad_s)) {
        out.mode = ChassisMode::Disabled;
        out.vx_m_s = out.vy_m_s = out.wz_rad_s = 0;
        return out;
    }
    const bool bf = body.valid && body.generation && core::fresh(body.stamp, now * 1000, 50000) &&
                    std::isfinite(body.yaw_rad) && std::isfinite(body.rate_rad_s);
    const bool hf = head.valid && head.sender_boot_id && local_boot && head.receiver_boot_id == local_boot &&
                    head.zero_generation && forwardedFresh(head.stamp, head.source_age_ms, now, 50) &&
                    std::isfinite(head.yaw_rad) && std::isfinite(head.yaw_rate_rad_s);
    if (paired_ && ((head.sender_boot_id && head.sender_boot_id != peer_boot_) || local_boot != local_boot_ ||
                    (head.zero_generation && head.zero_generation != head_generation_) ||
                    (body.generation && body.generation != body_generation_)))
        blocked_ = true;
    if (!paired_ && !blocked_ && disarmed && bf && hf && config_.startup_alignment_confirmed &&
        peer.sender_boot_id == head.sender_boot_id && startupSessionsMatch(peer, now, config_.startup_max_skew_ms)) {
        paired_ = true;
        peer_boot_ = head.sender_boot_id;
        local_boot_ = local_boot;
        head_generation_ = head.zero_generation;
        body_generation_ = body.generation;
    }
    const bool rotating = std::fabs(cmd.wz_rad_s) > 1e-4f;
    if (config_.frame != Frame::Body) {
        translation_ready_ = bf && paired_ && !blocked_ && config_.startup_alignment_confirmed &&
                             (config_.frame == Frame::World || hf);
        if (!translation_ready_)
            out.vx_m_s = out.vy_m_s = 0; // Rotation remains a separate, bounded command.
        else {
            const float age = float(now - head.stamp.timestamp_ms + head.source_age_ms) * .001f;
            const float g = config_.frame == Frame::Gimbal ? wrap(head.yaw_rad + head.yaw_rate_rad_s * age) : 0;
            const float body_age = float(now * 1000 - body.stamp.time_us) * 1e-6f;
            const float b = wrap(body.yaw_rad + body.rate_rad_s * (body_age + (rotating && config_.compensation_enabled
                                                                                   ? config_.heading_prediction_s
                                                                                   : 0)));
            const float angle = wrap(g - b), cs = std::cos(angle), sn = std::sin(angle);
            out.vx_m_s = cs * cmd.vx_m_s - sn * cmd.vy_m_s;
            out.vy_m_s = sn * cmd.vx_m_s + cs * cmd.vy_m_s;
        }
    }
    else
        translation_ready_ = true;
    if (rotating && config_.compensation_enabled) {
        // Reference-project policy: choose one scalar from the original input
        // so the world translation direction is preserved by the transform.
        const float scale = std::fabs(cmd.vy_m_s) > 1e-4f   ? config_.spin_lateral_scale
                            : std::fabs(cmd.vx_m_s) > 1e-4f ? config_.spin_forward_scale
                                                            : 1;
        out.vx_m_s *= scale;
        out.vy_m_s *= scale;
        if (bf)
            out.wz_rad_s += config_.yaw_rate_feedback_kp * (cmd.wz_rad_s - body.rate_rad_s);
    }
    out.wz_rad_s = std::clamp(out.wz_rad_s, -config_.max_wz_rad_s, config_.max_wz_rad_s);
    return out;
}
}
