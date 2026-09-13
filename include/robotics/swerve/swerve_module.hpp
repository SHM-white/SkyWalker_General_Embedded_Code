#pragma once
#include <control/motor_position.h>
#include <robotics/swerve/swerve_types.hpp>
namespace skywalker::robotics {
class SwerveModule {
public:
    struct Config {
        float wheel_radius_m = 0;
        control_motor_position_config steer{};
        control_motor_velocity_config drive{};
    };
    explicit SwerveModule(const Config &config) : config_(config) {
    }
    int validate() const;
    int reset(const ModuleFeedback &);
    int step(const ModuleTarget &, const ModuleFeedback &, float dt_s, ModuleOutput &out);

private:
    Config config_;
    control_motor_position_state steer_{};
    control_motor_velocity_state drive_{};
    float origin_rad_ = 0;
    bool initialized_ = false;
};
}
