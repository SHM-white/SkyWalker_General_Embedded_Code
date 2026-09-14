#pragma once
#include <robotics/messages/command.hpp>
#include <robotics/swerve/swerve_types.hpp>
namespace skywalker::robotics {
class SwerveKinematics {
public:
    struct Config {
        std::array<ModuleLocation, 4> locations{};
        float max_wheel_velocity_m_s = 0, stationary_epsilon_m_s = 0.01f;
    };
    explicit SwerveKinematics(const Config &config) : config_(config) {
    }
    int validate() const;
    int reset(const ModuleTargets &current);
    int solve(const ChassisCommand &, ModuleTargets &out);

private:
    Config config_;
    std::array<float, 4> last_angle_rad_{};
    bool initialized_ = false;
};
}
