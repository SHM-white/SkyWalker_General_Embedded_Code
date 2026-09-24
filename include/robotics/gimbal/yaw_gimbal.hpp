#pragma once
#include <control/position_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <robotics/messages/command.hpp>
namespace skywalker::robotics {
enum class YawTopology : std::uint8_t { Continuous, Limited };
struct YawGimbalConfig {
    YawTopology topology = YawTopology::Continuous;
    float min_angle_rad = -3.14159265f, max_angle_rad = 3.14159265f, max_rate_rad_s = 3;
    bool hold_on_zero_rate = true;
};
class YawGimbal {
public:
    YawGimbal(motor::Motor &drive, control::PositionMotor &axis, const YawGimbalConfig &config)
        : drive_(drive), axis_(axis), config_(config) {
    }
    int validate() const;
    int begin(); // Configure the controller after CanBus::start().
    int reset(); // Seed a new target from fresh feedback; never enables the drive.
    int update(const GimbalCommand &, SafetyAction, float dt_s);
    double targetAngleRad() const {
        return target_angle_rad_;
    }

private:
    int seed(const motor::MotorSnapshot &snapshot);
    motor::Motor &drive_;
    control::PositionMotor &axis_;
    YawGimbalConfig config_;
    double target_angle_rad_ = 0;
    SafetyAction previous_action_ = SafetyAction::Disable;
    GimbalMode previous_mode_ = GimbalMode::Disabled;
    bool initialized_ = false;
    std::uint64_t generation_ = 0;
};
}
