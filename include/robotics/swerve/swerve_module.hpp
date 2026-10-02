#pragma once
#include <control/motor_position.h>
#include <robotics/swerve/swerve_types.hpp>
namespace skywalker::robotics {
class SwerveModule {
public:
    enum class IdleBehavior { Hold, Coast };
    struct Config {
        float wheel_radius_m = 0;
        control_motor_position_config steer{};
        control_motor_velocity_config drive{};
        float steer_target_rate_rad_s = 4;
        float drive_enable_error_rad = 0.10f, drive_disable_error_rad = 0.40f;
        float flip_enter_error_rad = 1.6580628f, flip_exit_error_rad = 1.4835299f; // 95 / 85 degrees
        IdleBehavior idle_behavior = IdleBehavior::Hold;
    };
    explicit SwerveModule(const Config &config) : config_(config) {}
    int validate() const;
    int reset(const ModuleFeedback &);
    // Single execution writer. Failure leaves both state and output unchanged.
    int step(const ModuleTarget &, const ModuleFeedback &, float dt_s, ModuleOutput &out);

private:
    friend class SwerveChassis;
    // Called only on a transactional copy. Plan every steering loop before
    // allowing any drive loop to advance its integrator or reference ramp.
    int plan(const ModuleTarget &, const ModuleFeedback &, float dt_s, ModuleOutput &out);
    int drive(const ModuleFeedback &, float dt_s, bool permitted, ModuleOutput &out);
    Config config_;
    control_motor_position_state steer_{};
    control_motor_velocity_state drive_{};
    float origin_rad_ = 0, steer_reference_rad_ = 0;
    bool initialized_ = false, flipped_ = false, drive_ready_ = false;
};
}
