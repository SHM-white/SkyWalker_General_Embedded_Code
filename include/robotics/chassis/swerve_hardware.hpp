#pragma once

#include <array>
#include <drivers/motor/group.hpp>
#include <robotics/swerve/swerve_types.hpp>

namespace skywalker::robotics {

// Application-created motors: steer FL/FR/RL/RR, drive FL/FR/RL/RR. One
// application execution thread writes targets and commits each physical bus.
// The batch group and every motor must outlive this adapter.
class SwerveHardware {
public:
    struct Config {
        bool hardware_confirmed = false;
        std::array<float, 8> directions{{1, 1, 1, 1, 1, 1, 1, 1}};
    };
    SwerveHardware(const std::array<motor::Motor *, 8> &motors, motor::Group &group, const Config &config)
        : motors_(motors), group_(group), config_(config) {}
    int begin();
    int read(ChassisFeedback &out);
    void suspend();
    int stage(const ChassisOutput &output, float steer_scale, float drive_scale);
    int stage(const ChassisOutput &output, float effort_scale) { return stage(output, effort_scale, effort_scale); }
    motor::Group &group() { return group_; }
    motor::Motor &motorAt(std::size_t index) { return *motors_[index]; }
    float absoluteCurrentSumA() const { return absolute_current_sum_a_; }
    std::uint64_t oldestFeedbackMs() const { return oldest_feedback_ms_; }
private:
    std::array<motor::Motor *, 8> motors_;
    motor::Group &group_;
    Config config_;
    std::uint64_t oldest_feedback_ms_ = 0;
    float absolute_current_sum_a_ = 0;
    bool configured_ = false;
};

} // namespace skywalker::robotics
