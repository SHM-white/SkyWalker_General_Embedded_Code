#pragma once
#include <array>
#include <cstdint>
namespace skywalker::robotics {
// Array order: FL, FR, RL, RR. +x forward, +y left, +wz counterclockwise.
struct ModuleLocation { float x_m = 0, y_m = 0; };
struct ModuleTarget { float angle_rad = 0, wheel_velocity_m_s = 0; };
using ModuleTargets = std::array<ModuleTarget, 4>;
struct ModuleFeedback {
    float steer_absolute_rad = 0, steer_velocity_rad_s = 0, drive_velocity_rad_s = 0;
    bool steer_valid = false, drive_valid = false;
    std::uint64_t steer_feedback_stamp_ms = 0;
    std::uint64_t steer_enable_generation = 0, steer_reference_generation = 0;
    std::uint64_t drive_enable_generation = 0;
};
struct ModuleOutput {
    float optimized_angle_rad = 0, optimized_wheel_velocity_m_s = 0;
    float steer_continuous_target_rad = 0, drive_target_rad_s = 0, steer_effort = 0, drive_effort = 0;
    float steer_reference_rad = 0, alignment_error_rad = 0;
    bool steer_output_valid = false, drive_output_valid = false;
    bool flipped = false, coasting = false;
    std::uint64_t steer_enable_generation = 0, drive_enable_generation = 0;
};
struct ChassisFeedback { std::array<ModuleFeedback, 4> module{}; };
struct ChassisOutput { ModuleTargets target{}; std::array<ModuleOutput, 4> module{}; };
}
