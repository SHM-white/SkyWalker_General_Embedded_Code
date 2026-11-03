#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace skywalker::robotics {

// Generates a *world-frame* big-Yaw rate, for peers that compensate their own
// chassis gyro. Never send this rate to a native relative-rate BigYawExecutor.
// The carrier heading and joint offset must use the same physical positive
// direction; adding them estimates the head's world heading without aligning
// the zeroes of two separate IMUs.
class WorldYawWindowFollower {
public:
    struct Config {
        std::uint64_t window_us = 2500000;
        std::uint64_t sample_period_us = 20000;
        std::uint64_t max_update_gap_us = 100000;
        float kp_rad_s_per_rad = 0.8f;
        float max_rate_rad_s = 0.35f;
        float acceleration_rad_s2 = 0.6f;
        float deadband_rad = 0.005f;
    };
    struct Output {
        bool valid = false;
        float target_world_yaw_rad = 0;
        float head_world_yaw_rad = 0;
        float error_rad = 0;
        float world_rate_rad_s = 0;
        std::size_t samples = 0;
    };

    WorldYawWindowFollower() = default;
    explicit WorldYawWindowFollower(const Config &config) : config_(config) {}

    void reset() {
        oldest_ = count_ = 0;
        sum_sin_ = sum_cos_ = rate_ = 0;
        previous_us_ = last_sample_us_ = 0;
    }

    Output update(float carrier_world_rad, float joint_offset_rad,
                  std::uint64_t now_us, bool allowed) {
        Output result{};
        if (!allowed || !std::isfinite(carrier_world_rad) || !std::isfinite(joint_offset_rad) ||
            config_.window_us == 0 || config_.sample_period_us == 0 ||
            !(config_.kp_rad_s_per_rad > 0) || !(config_.max_rate_rad_s > 0) ||
            !(config_.acceleration_rad_s2 > 0) ||
            (previous_us_ && (now_us <= previous_us_ ||
                              now_us - previous_us_ > config_.max_update_gap_us))) {
            reset();
            return result;
        }
        const float dt = previous_us_ ? float(now_us - previous_us_) * 0.000001f : 0;
        previous_us_ = now_us;
        const float head_world_rad = carrier_world_rad + joint_offset_rad;
        if (!count_ || now_us - last_sample_us_ >= config_.sample_period_us) {
            if (count_ == samples_.size()) removeOldest();
            const auto tail = (oldest_ + count_) % samples_.size();
            const float s = std::sin(head_world_rad), c = std::cos(head_world_rad);
            samples_[tail] = {now_us, s, c};
            sum_sin_ += s;
            sum_cos_ += c;
            ++count_;
            last_sample_us_ = now_us;
        }
        while (count_ && now_us - samples_[oldest_].at_us > config_.window_us)
            removeOldest();
        // An average over opposite headings has no meaningful direction.
        if (!count_ || std::hypot(sum_sin_, sum_cos_) < 0.15f * float(count_)) {
            reset();
            return result;
        }
        const float target = std::atan2(sum_sin_, sum_cos_);
        const float error = std::remainder(target - carrier_world_rad, 6.2831853071795864769f);
        const float wanted = std::fabs(error) <= config_.deadband_rad
                                 ? 0.0f
                                 : std::clamp(config_.kp_rad_s_per_rad * error,
                                              -config_.max_rate_rad_s, config_.max_rate_rad_s);
        const float step = config_.acceleration_rad_s2 * dt;
        rate_ += std::clamp(wanted - rate_, -step, step);
        result.valid = true;
        result.target_world_yaw_rad = target;
        result.head_world_yaw_rad = std::remainder(head_world_rad, 6.2831853071795864769f);
        result.error_rad = error;
        result.world_rate_rad_s = rate_;
        result.samples = count_;
        return result;
    }

private:
    struct Sample {
        std::uint64_t at_us = 0;
        float sine = 0, cosine = 0;
    };
    void removeOldest() {
        sum_sin_ -= samples_[oldest_].sine;
        sum_cos_ -= samples_[oldest_].cosine;
        oldest_ = (oldest_ + 1) % samples_.size();
        --count_;
    }
    Config config_;
    std::array<Sample, 192> samples_{};
    std::size_t oldest_ = 0, count_ = 0;
    std::uint64_t previous_us_ = 0, last_sample_us_ = 0;
    float sum_sin_ = 0, sum_cos_ = 0, rate_ = 0;
};

} // namespace skywalker::robotics
