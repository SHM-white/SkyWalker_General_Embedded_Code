#pragma once
#include <control/position_motor.hpp>
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
    YawGimbal(control::PositionMotor &motor, const YawGimbalConfig &config) : motor_(motor), config_(config) {
    }
    int validate() const;
    int begin(); // Configuration only, does not wait for power or arm.
    int poll(std::uint64_t now_ms);
    int update(const GimbalCommand &, SafetyAction, float dt_s);
    int suspend(PauseReason reason);
    int clearEmergencyStop(bool released) {
        return motor_.clearEmergencyStop(released);
    }
    int stop();
    double targetAngleRad() const {
        return target_angle_rad_;
    }

private:
    void seed();
    control::PositionMotor &motor_;
    YawGimbalConfig config_;
    double target_angle_rad_ = 0;
    SafetyAction previous_action_ = SafetyAction::Disable;
    GimbalMode previous_mode_ = GimbalMode::Disabled;
    bool initialized_ = false;
    std::uint32_t generation_ = 0;
};
}
