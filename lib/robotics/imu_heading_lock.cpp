#include <robotics/execution/imu_heading_lock.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
namespace skywalker::robotics {
namespace { constexpr float pi=3.14159265358979323846f; float wrap(float a) { return std::remainder(a,2*pi); } }
int ImuHeadingLock::validate() const {
    for (float x : {config_.heading_kp,config_.correction_rate_max_rad_s,config_.error_max_rad,
                   config_.rate_kp_nm_s,config_.rate_ki_nm,config_.torque_max_nm})
        if (!std::isfinite(x) || x<0) return -EINVAL;
    return config_.error_max_rad>0 && config_.error_max_rad<=pi && config_.torque_max_nm>0 ? 0 : -EINVAL;
}
void ImuHeadingLock::reset() { initialized_=false; target_=integral_=previous_rate_=0; boot_=0; generation_=0; }
int ImuHeadingLock::step(const GimbalImuYaw &s,float rate,float dt,std::uint64_t now,float &torque) {
    torque=0;
    if (!s.valid || !s.sender_boot_id || !s.zero_generation || !forwardedFresh(s.stamp,s.source_age_ms,now,50) ||
        !std::isfinite(s.yaw_rad) || !std::isfinite(s.yaw_rate_rad_s) || !std::isfinite(rate) ||
        !std::isfinite(dt) || dt<.001f || dt>.02f || validate()<0) { reset(); return -ESTALE; }
    const float age=float(now-s.stamp.timestamp_ms+s.source_age_ms)*.001f;
    const float heading=wrap(s.yaw_rad+s.yaw_rate_rad_s*age);
    if (!initialized_ || boot_!=s.sender_boot_id || generation_!=s.zero_generation) {
        reset(); initialized_=true; boot_=s.sender_boot_id; generation_=s.zero_generation; target_=heading;
        return 0; // Seed reference without a recovery current step.
    }
    if (rate==0 && previous_rate_!=0) target_=heading;
    else target_=wrap(target_+rate*dt);
    previous_rate_=rate;
    const float error=std::clamp(wrap(target_-heading),-config_.error_max_rad,config_.error_max_rad);
    target_=wrap(heading+error);
    const float desired=rate+std::clamp(config_.heading_kp*error,-config_.correction_rate_max_rad_s,config_.correction_rate_max_rad_s);
    const float rate_error=desired-s.yaw_rate_rad_s;
    const float p=config_.rate_kp_nm_s*rate_error;
    const float candidate=std::clamp(integral_+config_.rate_ki_nm*rate_error*dt,-config_.torque_max_nm,config_.torque_max_nm);
    const float u=p+candidate;
    if (std::fabs(u)<=config_.torque_max_nm || (u>config_.torque_max_nm && rate_error<0) ||
        (u < -config_.torque_max_nm && rate_error>0)) integral_=candidate;
    torque=std::clamp(p+integral_,-config_.torque_max_nm,config_.torque_max_nm);
    return 0;
}
}
