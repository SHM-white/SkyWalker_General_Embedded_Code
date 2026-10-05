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
        float flip_enter_error_rad = 1.6580628f, flip_exit_error_rad = 1.4835299f;
        IdleBehavior idle_behavior = IdleBehavior::Hold;
    };
    explicit SwerveModule(const Config &config) : config_(config) {
    }
    int validate() const;
    int reset(const ModuleFeedback &);
    // One writer. Feedback loss invalidates only the affected axis's output.
    int step(const ModuleTarget &, const ModuleFeedback &, float dt_s, ModuleOutput &out);

private:
    int steer(const ModuleFeedback &, float dt_s, ModuleOutput &out);
    int drive(const ModuleFeedback &, float dt_s, ModuleOutput &out);
    Config config_;
    ModuleTarget latest_target_{};
    control_motor_position_state steer_{};
    control_motor_velocity_state drive_{};
    double steer_local_position_rad_ = 0, steer_origin_rad_ = 0;
    float previous_absolute_rad_ = 0, steer_reference_rad_ = 0;
    std::uint64_t previous_steer_stamp_ms_ = 0;
    std::uint64_t observed_steer_enable_generation_ = 0, observed_steer_reference_generation_ = 0;
    std::uint64_t observed_drive_enable_generation_ = 0;
    bool steer_history_valid_ = false, drive_history_valid_ = false, flipped_ = false;
};
}
