#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/group.hpp>
#include <robotics/swerve/swerve_types.hpp>

struct ChassisMotorConnection {
    const device *can = nullptr;
    skywalker::motor::dji::Config config{};
};

class DjiChassisHardware {
public:
    static constexpr std::size_t kMaxBuses = 2;

    explicit DjiChassisHardware(const std::array<ChassisMotorConnection, 8> &connections);

    int init();
    int read(skywalker::robotics::ChassisFeedback &out);
    int suspend();
    int pollRecovery(std::uint64_t now_ms);
    int arm();
    int apply(const skywalker::robotics::ChassisOutput &output, float effort_scale);
    int clearFault();

    bool ready() const {
        return ready_;
    }
    bool armed() const {
        return initialized_ && group_.active();
    }
    bool enabling() const {
        return initialized_ && group_.status().enable_pending;
    }
    std::size_t busCount() const {
        return bus_count_;
    }
    float estimatedPowerW() const {
        return estimated_power_w_;
    }

private:
    static const device *secondaryCan(const std::array<ChassisMotorConnection, 8> &connections);
    int validateFeedback(std::size_t index, const skywalker::motor::MotorSnapshot &snapshot,
                         bool require_reference) const;

    std::array<ChassisMotorConnection, 8> connections_;
    std::array<skywalker::motor::Motor, 8> motors_;
    skywalker::motor::Group group_;
    skywalker::motor::CanBus first_bus_;
    skywalker::motor::CanBus second_bus_;
    std::array<skywalker::motor::CanBus *, kMaxBuses> buses_;
    std::array<const device *, kMaxBuses> can_devices_{};
    std::array<skywalker::motor::dji::Descriptor, 8> descriptors_{};
    std::array<std::uint64_t, 8> stamps_{}, first_stamps_{}, references_{};
    std::array<std::uint8_t, 8> motor_bus_index_{};
    std::size_t bus_count_ = 0;
    bool initialized_ = false, ready_ = false, stable_ = false, stopped_ = true;
    std::uint64_t stable_since_ms_ = 0, next_retry_ms_ = 0;
    float estimated_power_w_ = 0;
};
