#include <communication/vision/vision_link.hpp>
#include <core/attitude.hpp>
#include <core/clock.hpp>
#include <cerrno>
namespace skywalker::communication::vision {
int VisionLink::init() {
    if (atomic_get(&initialized_))
        return -EALREADY;
    if (!config_.aim_timeout_us || !config_.orientation_timeout_us || !config_.gyro_timeout_us ||
        !config_.bullet_speed_timeout_us || !config_.bullet_count_timeout_us || !config_.max_feedback_skew_us)
        return -EINVAL;
    const int r = protocol_.validateConfig();
    if (r < 0)
        return r;
    protocol_.reset();
    auto k = k_spin_lock(&lock_);
    value_ = {};
    feedback_ = {};
    k_spin_unlock(&lock_, k);
    atomic_set(&initialized_, 1);
    return 0;
}
int VisionLink::accept(const AimCommand &input, core::TimeUs rx) {
    auto command = input;
    auto axisValid = [](const AxisTarget &a) {
        return std::isfinite(a.angle_rad) && std::isfinite(a.rate_rad_s) && std::isfinite(a.acceleration_rad_s2);
    };
    int error = 0;
    if (rx > core::monotonicTimeUs())
        error = -EINVAL;
    else if (command.control_requested) {
        if (!command.reference.frame_id || !command.reference.epoch || !axisValid(command.yaw) ||
            !axisValid(command.pitch))
            error = -EINVAL;
    }
    else
        command = {}; // A stop always replaces the previous active target.
    auto k = k_spin_lock(&lock_);
    if (!error && value_.aim.stamp.valid && rx < value_.aim.stamp.time_us)
        error = -ESTALE;
    if (!error)
        value_.aim = {command, {rx, value_.aim.stamp.sequence + 1, true}};
    else
        ++value_.rejected_commands;
    k_spin_unlock(&lock_, k);
    return error;
}
int VisionLink::processRxBytes(const std::uint8_t *p, std::size_t n, core::TimeUs rx) {
    if (!atomic_get(&initialized_))
        return -EACCES;
    if (!p && n)
        return -EINVAL;
    const int r = protocol_.consume(p, n, rx, *this);
    const auto stats = protocol_.statistics();
    auto k = k_spin_lock(&lock_);
    value_.protocol = stats;
    k_spin_unlock(&lock_, k);
    return r;
}
void VisionLink::discardPartial() {
    protocol_.discardPartial();
}
Snapshot VisionLink::snapshot() const {
    auto k = k_spin_lock(&lock_);
    auto s = value_;
    k_spin_unlock(&lock_, k);
    s.aim_fresh = core::fresh(s.aim.stamp, core::monotonicTimeUs(), config_.aim_timeout_us);
    return s;
}
int VisionLink::setFeedback(const Feedback &input) {
    if (!atomic_get(&initialized_))
        return -EACCES;
    auto f = input;
    const auto now = core::monotonicTimeUs();
    const auto timeOk = [now](const auto &m) { return !m.stamp.valid || m.stamp.time_us <= now; };
    if (static_cast<unsigned>(f.mode) > 3 || !timeOk(f.orientation) || !timeOk(f.gyro_rad_s) ||
        !timeOk(f.bullet_speed_m_s) || !timeOk(f.bullet_count) ||
        (f.orientation.stamp.valid &&
         (!f.reference.frame_id || !f.reference.epoch || !core::normalize(f.orientation.value))) ||
        (f.gyro_rad_s.stamp.valid && !core::finite(f.gyro_rad_s.value)) ||
        (f.bullet_speed_m_s.stamp.valid && (!std::isfinite(f.bullet_speed_m_s.value) || f.bullet_speed_m_s.value < 0)))
        return -EINVAL;
    auto k = k_spin_lock(&lock_);
    feedback_ = f;
    k_spin_unlock(&lock_, k);
    return 0;
}
int VisionLink::encodeFeedback(std::uint8_t *out, std::size_t capacity) {
    if (!atomic_get(&initialized_))
        return -EACCES;
    if (!out)
        return -EINVAL;
    auto k = k_spin_lock(&lock_);
    auto f = feedback_;
    k_spin_unlock(&lock_, k);
    const auto now = core::monotonicTimeUs();
    f.orientation.stamp.valid = core::fresh(f.orientation.stamp, now, config_.orientation_timeout_us);
    f.gyro_rad_s.stamp.valid = core::fresh(f.gyro_rad_s.stamp, now, config_.gyro_timeout_us);
    f.bullet_speed_m_s.stamp.valid = core::fresh(f.bullet_speed_m_s.stamp, now, config_.bullet_speed_timeout_us);
    f.bullet_count.stamp.valid = core::fresh(f.bullet_count.stamp, now, config_.bullet_count_timeout_us);
    const auto qt = f.orientation.stamp.time_us, gt = f.gyro_rad_s.stamp.time_us;
    if (f.orientation.stamp.valid && f.gyro_rad_s.stamp.valid &&
        (qt > gt ? qt - gt : gt - qt) > config_.max_feedback_skew_us)
        return -ESTALE;
    return protocol_.encodeFeedback(f, now, out, capacity);
}
}
