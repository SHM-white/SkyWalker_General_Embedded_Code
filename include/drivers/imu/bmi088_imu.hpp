#pragma once
#include <drivers/imu/imu.hpp>
#include <drivers/imu/imu_state.hpp>
#include <control/attitude_ekf.hpp>
#include <zephyr/device.h>
namespace skywalker::imu {
class Bmi088Imu final : public ImuSource {
public:
    struct Config {
        core::TimeUs sample_period_us = 1250, temperature_period_us = 10000, max_input_skew_us = 2000;
        core::OrientationReference reference{1, 1};
        core::Quaternion sensor_to_body{};
        Freshness freshness{};
    };
    Bmi088Imu(const device *accel, const device *gyro, const Config &c, control::QuaternionEkf *estimator = nullptr)
        : accel_(accel), gyro_(gyro), config_(c), estimator_(estimator), state_(c.freshness) {
    }
    int init() override;
    int service() override;
    Snapshot snapshot() const override {
        return state_.snapshot();
    }

private:
    const device *accel_, *gyro_;
    Config config_;
    control::QuaternionEkf *estimator_;
    ImuState state_;
    core::TimeUs next_sample_ = 0, next_temperature_ = 0;
    bool initialized_ = false;
};
}
