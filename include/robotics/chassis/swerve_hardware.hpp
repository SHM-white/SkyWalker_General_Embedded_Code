#pragma once

#include <array>
#include <drivers/motor/group.hpp>
#include <robotics/swerve/swerve_types.hpp>

namespace skywalker::robotics {

// Application-created motors: steer FL/FR/RL/RR, drive FL/FR/RL/RR. One
// application execution thread writes targets and commits each physical bus.
// The eight-motor fault group and every motor must outlive this adapter.
class SwerveHardware {
public:
    struct Config {
        bool hardware_confirmed = false;
        std::array<float, 8> directions{{1, 1, 1, 1, 1, 1, 1, 1}};
        std::uint32_t feedback_stable_ms = 30;
        float velocity_safety_rad_s = 40, temperature_limit_c = 70;
    };
    SwerveHardware(const std::array<motor::Motor *, 8> &motors, motor::Group &group, const Config &config)
        : motors_(motors), group_(group), config_(config) {}
    int begin();
    int read(ChassisFeedback &out);
    // Seeds references while disabled, then requires new feedback from all 8
    // members before the controller can reset and establish a recovery boundary.
    int prepare(std::uint64_t now_ms, ChassisFeedback &out);
    void suspend();
    int stage(const ChassisOutput &output, float steer_scale, float drive_scale);
    int stage(const ChassisOutput &output, float effort_scale) { return stage(output, effort_scale, effort_scale); }
    motor::Group &group() { return group_; }
    motor::Motor &motorAt(std::size_t index) { return *motors_[index]; }
    bool ready() const { return ready_; }
    float absoluteCurrentSumA() const { return absolute_current_sum_a_; }
    std::uint64_t oldestFeedbackMs() const { return oldest_feedback_ms_; }
private:
    int validateFeedback(std::size_t index, const motor::MotorSnapshot &view, bool reference) const;
    std::array<motor::Motor *, 8> motors_;
    motor::Group &group_;
    Config config_;
    std::array<std::uint64_t, 8> references_{}, baseline_stamps_{};
    std::uint64_t stable_since_ms_ = 0, oldest_feedback_ms_ = 0;
    float absolute_current_sum_a_ = 0;
    bool configured_ = false, seeded_ = false, ready_ = false;
};

} // namespace skywalker::robotics
