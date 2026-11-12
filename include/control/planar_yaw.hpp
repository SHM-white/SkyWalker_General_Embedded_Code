#pragma once
#include <core/attitude.hpp>
#include <cmath>

namespace skywalker::control {
// Near-horizontal carrier only. Feed each raw, mounting-corrected gyro sample
// from its acquisition thread, before any EKF-dependent bias correction.
class PlanarYawIntegrator {
public:
    enum class State { Calibrating, Ready, GyroInvalid, SampleGap, TimeReset, RateInvalid };
    struct Output {
        core::Measurement<float> yaw_rad{};
        float rate_rad_s = 0, bias_rad_s = 0;
        std::uint32_t generation = 0;
        State state = State::Calibrating;
    };
    // No runtime bias adaptation: slow real turns must not become learned bias.
    const Output &update(const core::Measurement<core::Vec3> &gyro, const core::Measurement<core::Vec3> &accel) {
        if (blocked_)
            return output_;
        if (!gyro.stamp.valid || !gyro.stamp.time_us || !core::finite(gyro.value))
            return reject(State::GyroInvalid);
        const auto time = gyro.stamp.time_us;
        if (previous_us_ && time < previous_us_)
            return block(State::TimeReset);
        if (previous_us_ && time == previous_us_)
            return output_; // Never integrate or refresh a cached sample twice.
        if (!std::isfinite(core::norm(gyro.value)) || std::fabs(gyro.value.z) > max_rate_rad_s)
            return reject(State::RateInvalid);
        const auto interval = previous_us_ ? time - previous_us_ : 0;
        if (interval > max_gap_us) {
            if (calibrated_)
                return block(State::SampleGap); // Unobserved rotation cannot be reconstructed.
            clearCalibration();
        }
        previous_us_ = time;
        if (!first_us_)
            first_us_ = time;
        if (!calibrated_) {
            const auto skew = time >= accel.stamp.time_us ? time - accel.stamp.time_us : accel.stamp.time_us - time;
            const bool stationary = time - first_us_ >= settling_us && accel.stamp.valid && skew <= max_gap_us &&
                                    core::finite(accel.value) &&
                                    std::fabs(core::norm(accel.value) - 9.80665f) <= 1.0f &&
                                    core::norm(gyro.value) < .05f;
            if (!stationary) {
                clearCalibration();
                return reject(State::Calibrating);
            }
            if (!calibration_start_us_)
                calibration_start_us_ = time;
            ++calibration_samples_;
            const double delta = double(gyro.value.z) - mean_z_;
            mean_z_ += delta / calibration_samples_;
            variance_sum_ += delta * (double(gyro.value.z) - mean_z_);
            if (time - calibration_start_us_ < calibration_us || calibration_samples_ < 800)
                return reject(State::Calibrating);
            if (variance_sum_ / (calibration_samples_ - 1) > .01 * .01) {
                clearCalibration();
                return reject(State::Calibrating);
            }
            output_.bias_rad_s = static_cast<float>(mean_z_);
            calibrated_ = true;
            output_.generation = 1;
            yaw_ = 0;
            previous_rate_ = gyro.value.z - output_.bias_rad_s;
        }
        else {
            const float rate = gyro.value.z - output_.bias_rad_s;
            if (!std::isfinite(rate) || std::fabs(rate) > max_rate_rad_s)
                return block(State::RateInvalid);
            yaw_ = std::remainder(yaw_ + .5 * (double(previous_rate_) + double(rate)) * double(interval) * 1e-6,
                                  6.2831853071795864769);
            previous_rate_ = rate;
        }
        output_.yaw_rad = {static_cast<float>(yaw_), {time, ++sequence_, true}};
        output_.rate_rad_s = previous_rate_;
        output_.state = State::Ready;
        return output_;
    }

private:
    // These limits are independent of EKF quality and of application polling.
    static constexpr core::TimeUs settling_us = 1000000, calibration_us = 1000000, max_gap_us = 20000;
    static constexpr float max_rate_rad_s = 35; // Existing interboard yaw-rate range.
    void clearCalibration() {
        calibration_start_us_ = 0;
        calibration_samples_ = 0;
        mean_z_ = variance_sum_ = 0;
    }
    const Output &reject(State reason) {
        output_.state = reason;
        output_.yaw_rad.stamp.valid = false;
        if (!calibrated_ && reason != State::Calibrating)
            clearCalibration();
        return output_;
    }
    const Output &block(State reason) {
        blocked_ = true;
        if (calibrated_)
            output_.generation = 2;
        return reject(reason);
    }
    Output output_{};
    core::TimeUs first_us_ = 0, previous_us_ = 0, calibration_start_us_ = 0;
    std::uint64_t sequence_ = 0;
    unsigned calibration_samples_ = 0;
    double mean_z_ = 0, variance_sum_ = 0, yaw_ = 0;
    float previous_rate_ = 0;
    bool calibrated_ = false, blocked_ = false;
};
} // namespace skywalker::control
