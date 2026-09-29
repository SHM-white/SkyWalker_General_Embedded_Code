#pragma once
#include <core/measurement.hpp>
#include <control/pid.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/spinlock.h>
namespace skywalker::imu {
class ImuHeater {
public:
    struct Config {
        pwm_dt_spec pwm{};
        float target_c = 50, maximum_c = 65;
        core::TimeUs temperature_timeout_us = 200000, control_period_us = 100000;
        control_pid_config pid{.kp = 0.3f,
                               .ki = 0.01f,
                               .kd = 0,
                               .derivative_tau_s = 0,
                               .integral_min = 0,
                               .integral_max = 0.5f,
                               .output_min = 0,
                               .output_max = 1,
                               .deadband = 0,
                               .dt_min_s = 0.02f,
                               .dt_max_s = 0.5f};
    };
    struct Snapshot {
        float duty = 0;
        int last_error = 0, disable_error = 0;
    };
    explicit ImuHeater(const Config &c) : config_(c) {
    }
    ImuHeater(const ImuHeater &) = delete;
    ImuHeater &operator=(const ImuHeater &) = delete;
    int init(); // Explicitly writes zero duty.
    int update(const core::Measurement<float> &temperature, core::TimeUs now);
    int disable();
    Snapshot snapshot() const;

private:
    int fail(int error);
    void record(float duty, int error, int disable_error = 0);
    Config config_;
    control_pid_state pid_{};
    core::TimeUs previous_us_ = 0;
    bool initialized_ = false, have_time_ = false;
    mutable k_spinlock lock_{};
    Snapshot status_{};
};
}
