#pragma once
#include <core/measurement.hpp>
namespace skywalker::control {
// Allocation-free quaternion EKF. All covariance, bias and history are per instance.
// q_WS maps sensor vectors to a local gravity-aligned world (yaw is unobservable).
class QuaternionEkf {
public:
    struct Config {
        float dt_min_s = 0.0002f, dt_max_s = 0.02f;
        float gravity_m_s2 = 9.80665f, accel_gate_m_s2 = 1.0f;
        float process_noise = 0.002f, measurement_noise = 0.01f;
        float innovation_gate = 16.27f, accel_tau_s = 0.02f;
        float stationary_gyro_rad_s = 0.15f, bias_tau_s = 20.0f;
        unsigned initialization_samples = 100;
    };
    enum class Quality { Initializing, Tracking, Degraded };
    struct Input {
        core::Vec3 accel_m_s2{}, gyro_rad_s{};
        core::TimeUs time_us = 0;
    };
    explicit QuaternionEkf(const Config &c) : config_(c) {
    }
    int init();
    void reset();
    int update(const Input &); // 0 valid output; -EAGAIN initializing; negative errno otherwise.
    core::Quaternion attitude() const {
        return q_;
    }
    Quality quality() const {
        return quality_;
    }
    std::uint32_t generation() const {
        return generation_;
    }

private:
    Config config_;
    core::Quaternion q_{};
    core::Vec3 bias_{}, filtered_{}, accel_sum_{}, gyro_sum_{};
    float p_[4][4]{};
    core::TimeUs previous_us_ = 0;
    unsigned initial_count_ = 0;
    std::uint32_t generation_ = 0;
    Quality quality_ = Quality::Initializing;
    bool configured_ = false, initialized_ = false, have_time_ = false;
};
}
