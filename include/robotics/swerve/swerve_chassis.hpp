#pragma once
#include <robotics/swerve/swerve_kinematics.hpp>
#include <robotics/swerve/swerve_module.hpp>
namespace skywalker::robotics {
class SwerveChassis {
public:
    struct Config {
        SwerveKinematics::Config kinematics{};
        std::array<SwerveModule::Config, 4> modules{};
        bool require_all_modules_aligned = true;
    };
    explicit SwerveChassis(const Config &c)
        : kinematics_(c.kinematics), modules_{SwerveModule(c.modules[0]), SwerveModule(c.modules[1]),
                                              SwerveModule(c.modules[2]), SwerveModule(c.modules[3])},
          require_all_modules_aligned_(c.require_all_modules_aligned) {
    }
    int validate() const;
    int reset(const ChassisFeedback &);
    int step(const ChassisCommand &, const ChassisFeedback &, float dt_s, ChassisOutput &out);

private:
    SwerveKinematics kinematics_;
    std::array<SwerveModule, 4> modules_;
    bool require_all_modules_aligned_;
};
}
