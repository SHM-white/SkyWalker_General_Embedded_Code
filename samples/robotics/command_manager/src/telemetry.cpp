#include "telemetry.hpp"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(command_telemetry, LOG_LEVEL_INF);
namespace bench {
using namespace skywalker;
namespace {
const char *source(robotics::ControlSource s) {
    switch (s) {
    case robotics::ControlSource::None:
        return "None";
    case robotics::ControlSource::Remote:
        return "Remote";
    case robotics::ControlSource::KeyboardMouse:
        return "KeyboardMouse";
    case robotics::ControlSource::Vision:
        return "Vision";
    case robotics::ControlSource::Autonomous:
        return "Autonomous";
    }
    return "Unknown";
}
const char *mode(robotics::OperatorMode m) {
    switch (m) {
    case robotics::OperatorMode::Safe:
        return "Safe";
    case robotics::OperatorMode::Manual:
        return "Manual";
    case robotics::OperatorMode::Auto:
        return "Auto";
    }
    return "Unknown";
}
long long age(const robotics::MessageStamp &s, std::uint64_t now) {
    return s.valid && now >= s.timestamp_ms ? static_cast<long long>(now - s.timestamp_ms) : -1;
}
}
int Telemetry::start(const device *uart) {
#ifdef CONFIG_COMMAND_MANAGER_VOFA
    const int ret = vofa_init(&vofa_, uart);
    vofa_ready_ = ret == 0;
    return ret;
#else
    (void)uart;
    return 0;
#endif
}
void Telemetry::emit(const robotics::CommandSnapshot &f) {
    const auto &d = f.decision;
    const auto now = static_cast<std::uint64_t>(k_uptime_get());
    const auto &c = d.command;
    if (now >= next_log_ms_) {
        next_log_ms_ = now + 100;
        LOG_INF("seq=%u mode=%s override=%u quiet=%u error=%d reasons(c/g/s)=%08x/%08x/%08x", c.stamp.sequence,
                mode(d.operator_mode), unsigned(d.manual_override), unsigned(d.override_quiet), d.error,
                d.chassis_reasons, d.gimbal_reasons, d.shooter_reasons);
        LOG_INF(
            "chassis[%u %s] v=%.2f/%.2f/%.2f gimbal[%u %s] q=%.2f/%.2f rate=%.2f/%.2f shooter[%u %s] hz=%.2f speed=%.2f",
            unsigned(c.chassis.mode), source(c.chassis.source), double(c.chassis.vx_m_s), double(c.chassis.vy_m_s),
            double(c.chassis.wz_rad_s), unsigned(c.gimbal.mode), source(c.gimbal.source),
            double(c.gimbal.yaw_target_rad), double(c.gimbal.pitch_target_rad), double(c.gimbal.yaw_rate_rad_s),
            double(c.gimbal.pitch_rate_rad_s), unsigned(c.shooter.mode), source(c.shooter.source),
            double(c.shooter.fire_rate_hz), double(c.shooter.requested_bullet_speed_m_s));
        const auto &r = f.observed.referee.robot;
        const auto &v = f.observed.vision.stamp;
        const long long va = v.valid && now >= v.time_us / 1000 ? static_cast<long long>(now - v.time_us / 1000) : -1;
        LOG_INF(
            "age_ms(-1=NA) rc=%lld vision=%lld permit(g/c/s)=%lld/%lld/%lld state(rc/v)=%u/%u uart=%d/%d/%d drop=%u/%u/%lld sample=%d/%d/%d vofa=%u/%d",
            age(f.observed.remote.stamp, now), va, age(r.gimbal_output.stamp, now), age(r.chassis_output.stamp, now),
            age(r.shooter_output.stamp, now), unsigned(f.remote.state), unsigned(f.vision.state), f.remote.error,
            f.vision.error, f.permission.error, f.remote.dropped, f.vision.dropped,
            f.permission.dropped_available ? static_cast<long long>(f.permission.dropped) : -1LL, f.remote.sample_error,
            f.vision.sample_error, f.permission.sample_error, vofa_rejected_, last_vofa_error_);
        if (d.selected_vision.stamp.valid) {
            const auto &v = d.selected_vision.value;
            LOG_INF("vision_seq=%llu reference=%u/%u acceleration=%.2f/%.2f",
                    static_cast<unsigned long long>(d.selected_vision.stamp.sequence), v.reference.frame_id,
                    v.reference.epoch, double(v.yaw.acceleration_rad_s2), double(v.pitch.acceleration_rad_s2));
        }
    }
#ifdef CONFIG_COMMAND_MANAGER_VOFA
    if (vofa_ready_ && now >= next_vofa_ms_) {
        next_vofa_ms_ = now + 20;
        const auto reasons = d.reasons();
        const float values[VOFA_MAX_FLOATS] = {float(d.operator_mode),   float(c.gimbal.source),
                                               float(c.chassis.mode),    float(c.gimbal.mode),
                                               float(c.shooter.mode),    c.chassis.vx_m_s,
                                               c.chassis.vy_m_s,         c.chassis.wz_rad_s,
                                               c.gimbal.yaw_target_rad,  c.gimbal.pitch_target_rad,
                                               c.gimbal.yaw_rate_rad_s,  c.gimbal.pitch_rate_rad_s,
                                               c.shooter.fire_rate_hz,   c.shooter.requested_bullet_speed_m_s,
                                               float(reasons & 0xffffu), float(reasons >> 16)};
        last_vofa_error_ = vofa_send(&vofa_, values, VOFA_MAX_FLOATS);
        if (last_vofa_error_ < 0)
            ++vofa_rejected_;
    }
#endif
}
}
