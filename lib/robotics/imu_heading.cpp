#include <robotics/chassis/imu_heading.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
namespace skywalker::robotics {
namespace {
constexpr float pi = 3.14159265358979323846f;
float wrap(float a) { return std::remainder(a,2*pi); }
}
LocalHeading ImuHeadingReference::update(const imu::Snapshot &s, bool disarmed, core::TimeUs now) {
    LocalHeading out{};
    out.generation = initialized_ ? 1 : 0;
    if (blocked_) { out.generation=2; return out; }
    if (initialized_ && last_sample_us_ && now>last_sample_us_ && now-last_sample_us_>500000) {
        blocked_=true; out.generation=2; return out;
    }
    const auto &v = s.sample;
    auto q = v.orientation.value;
    const bool fresh = s.state == imu::State::Running &&
        core::fresh(v.orientation.stamp,now,50000) && core::fresh(v.gyro_rad_s.stamp,now,50000) &&
        v.reference.frame_id && v.reference.epoch && core::normalize(q) && core::finite(v.gyro_rad_s.value) &&
        (v.attitude_quality == imu::AttitudeQuality::Tracking || v.attitude_quality == imu::AttitudeQuality::Unknown);
    if (!fresh) { stationary_samples_=0; return out; }
    if (have_reference_ && reference_ != v.reference) {
        if (initialized_) { blocked_=true; out.generation=2; return out; }
        stationary_samples_=0;
    }
    reference_=v.reference; have_reference_=true;
    const auto e=core::euler(q);
    if (std::fabs(std::cos(e.pitch)) < .2f) return out;
    const auto g=v.gyro_rad_s.value;
    const float rate=(std::sin(e.roll)*g.y+std::cos(e.roll)*g.z)/std::cos(e.pitch);
    if (!std::isfinite(rate) || std::fabs(rate)>100) return out;
    const bool distinct = v.orientation.stamp.sequence != last_sequence_ && v.orientation.stamp.time_us > last_sample_us_;
    if (distinct) {
        last_sequence_=v.orientation.stamp.sequence;
        last_sample_us_=v.orientation.stamp.time_us;
        if (!initialized_) {
            if (disarmed && now>=1000000 && core::norm(g)<.05f) ++stationary_samples_;
            else stationary_samples_=0;
            if (stationary_samples_>=20) { zero_rad_=e.yaw; initialized_=true; }
        }
    }
    if (!initialized_) return out;
    out.yaw_rad=wrap(e.yaw-zero_rad_); out.rate_rad_s=rate; out.generation=1; out.valid=true;
    // The oldest constituent sample bounds the predicted heading's freshness.
    out.stamp=v.orientation.stamp;
    out.stamp.time_us=std::min(v.orientation.stamp.time_us,v.gyro_rad_s.stamp.time_us);
    return out;
}
int ChassisMotionAdapter::validate() const {
    return config_.frame > Frame::Gimbal || !std::isfinite(config_.heading_prediction_s) ||
        config_.heading_prediction_s < 0 || config_.heading_prediction_s > .05f ||
        !std::isfinite(config_.spin_forward_scale) || config_.spin_forward_scale<0 || config_.spin_forward_scale>1 ||
        !std::isfinite(config_.spin_lateral_scale) || config_.spin_lateral_scale<0 || config_.spin_lateral_scale>1 ||
        !std::isfinite(config_.yaw_rate_feedback_kp) || config_.yaw_rate_feedback_kp<0 || config_.yaw_rate_feedback_kp>2 ||
        !std::isfinite(config_.max_wz_rad_s) || config_.max_wz_rad_s<=0 ? -EINVAL : 0;
}
ChassisCommand ChassisMotionAdapter::update(const ChassisCommand &cmd, const LocalHeading &body,
    const GimbalImuYaw &head, std::uint64_t local_boot, bool disarmed, std::uint64_t now) {
    auto out=cmd;
    translation_ready_=false;
    if (validate()<0 || !std::isfinite(cmd.vx_m_s) || !std::isfinite(cmd.vy_m_s) || !std::isfinite(cmd.wz_rad_s)) {
        out.mode=ChassisMode::Disabled; out.vx_m_s=out.vy_m_s=out.wz_rad_s=0; return out;
    }
    const bool bf=body.valid && body.generation && core::fresh(body.stamp,now*1000,50000) &&
                  std::isfinite(body.yaw_rad) && std::isfinite(body.rate_rad_s);
    const bool hf=head.valid && head.sender_boot_id && local_boot && head.receiver_boot_id==local_boot &&
        head.zero_generation && forwardedFresh(head.stamp,head.source_age_ms,now,50) &&
        std::isfinite(head.yaw_rad) && std::isfinite(head.yaw_rate_rad_s);
    if (paired_ && ((head.sender_boot_id && head.sender_boot_id!=peer_boot_) || local_boot!=local_boot_ ||
        (head.zero_generation && head.zero_generation!=head_generation_) ||
        (body.generation && body.generation!=body_generation_))) blocked_=true;
    if (!paired_ && !blocked_ && disarmed && bf && hf && config_.startup_alignment_confirmed) {
        paired_=true; peer_boot_=head.sender_boot_id; local_boot_=local_boot;
        head_generation_=head.zero_generation; body_generation_=body.generation;
    }
    const bool rotating=std::fabs(cmd.wz_rad_s)>1e-4f;
    if (config_.frame != Frame::Body) {
        translation_ready_=bf && !blocked_ && config_.startup_alignment_confirmed &&
            (config_.frame==Frame::World || (paired_ && hf));
        if (!translation_ready_) out.vx_m_s=out.vy_m_s=0; // Rotation remains a separate, bounded command.
        else {
            const float age=float(now-head.stamp.timestamp_ms+head.source_age_ms)*.001f;
            const float g=config_.frame==Frame::Gimbal ? wrap(head.yaw_rad+head.yaw_rate_rad_s*age) : 0;
            const float body_age=float(now*1000-body.stamp.time_us)*1e-6f;
            const float b=wrap(body.yaw_rad+body.rate_rad_s*(body_age+
                (rotating && config_.compensation_enabled ? config_.heading_prediction_s : 0)));
            const float angle=wrap(g-b), cs=std::cos(angle), sn=std::sin(angle);
            out.vx_m_s=cs*cmd.vx_m_s-sn*cmd.vy_m_s;
            out.vy_m_s=sn*cmd.vx_m_s+cs*cmd.vy_m_s;
        }
    } else translation_ready_=true;
    if (rotating && config_.compensation_enabled) {
        // Reference-project policy: choose one scalar from the original input
        // so the world translation direction is preserved by the transform.
        const float scale=std::fabs(cmd.vy_m_s)>1e-4f ? config_.spin_lateral_scale :
                          std::fabs(cmd.vx_m_s)>1e-4f ? config_.spin_forward_scale : 1;
        out.vx_m_s*=scale; out.vy_m_s*=scale;
        if (bf) out.wz_rad_s+=config_.yaw_rate_feedback_kp*(cmd.wz_rad_s-body.rate_rad_s);
    }
    out.wz_rad_s=std::clamp(out.wz_rad_s,-config_.max_wz_rad_s,config_.max_wz_rad_s);
    return out;
}
}
