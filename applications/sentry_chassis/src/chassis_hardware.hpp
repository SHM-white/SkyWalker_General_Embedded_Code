#include <drivers/motor/motor.hpp>
#pragma once
#include <array>
#include <drivers/motor/dji_bus.hpp>
#include <robotics/swerve/swerve_types.hpp>
class DjiChassisHardware {
public:
    explicit DjiChassisHardware(const std::array<const device *, 8> &motors) : motors_(motors) {
    }
    int init();
    int read(skywalker::robotics::ChassisFeedback &out);
    int suspend();
    int pollRecovery(std::uint64_t now_ms);
    int arm();
    int apply(const skywalker::robotics::ChassisOutput &output, float effort_scale);
    bool ready() const {
        return ready_;
    }
    bool armed() const {
        return armed_;
    }
    float estimatedPowerW() const {
        return estimated_power_w_;
    }

private:
    std::array<const device *, 8> motors_;
    std::array<skywalker::motor::dji::Descriptor, 8> descriptors_{};
    std::array<std::uint64_t, 8> stamps_{}, first_stamps_{};
    skywalker::motor::dji::Bus bus_{};
    skywalker::motor::dji::FlushReport report_{};
    bool initialized_ = false, ready_ = false, armed_ = false, stable_ = false, stopped_ = true;
    std::uint64_t stable_since_ms_ = 0, next_retry_ms_ = 0;
    float estimated_power_w_ = 0;
};
