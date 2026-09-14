#include <drivers/motor/motor.hpp>
#pragma once
#include <array>
#include <drivers/motor/dji_bus.hpp>
#include <robotics/swerve/swerve_types.hpp>
class DjiChassisHardware {
public:
    static constexpr std::size_t kMaxBuses = 2;
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
    std::size_t busCount() const {
        return bus_count_;
    }
    float estimatedPowerW() const {
        return estimated_power_w_;
    }

private:
    int findOrCreateBus(const device *can);
    int stopAllBuses();
    std::array<const device *, 8> motors_;
    std::array<skywalker::motor::dji::Descriptor, 8> descriptors_{};
    std::array<std::uint64_t, 8> stamps_{}, first_stamps_{};
    std::array<const device *, kMaxBuses> can_devices_{};
    std::array<std::uint8_t, 8> motor_bus_index_{};
    std::array<skywalker::motor::dji::Bus, kMaxBuses> buses_{};
    std::array<skywalker::motor::dji::FlushReport, kMaxBuses> reports_{};
    std::size_t bus_count_ = 0;
    bool initialized_ = false, ready_ = false, armed_ = false, stable_ = false, stopped_ = true;
    std::uint64_t stable_since_ms_ = 0, next_retry_ms_ = 0;
    float estimated_power_w_ = 0;
};
