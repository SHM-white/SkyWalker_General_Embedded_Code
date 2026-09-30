#pragma once
#include <robotics/command/command_inputs.hpp>
#include <robotics/command/manual_command_mapper.hpp>
#include <robotics/messages/remote.hpp>
#include <robotics/messages/safety.hpp>
namespace skywalker::robotics {
class CommandManager {
public:
    struct Config {
        float max_chassis_vx_m_s = 3, max_chassis_vy_m_s = 3, max_chassis_wz_rad_s = 6;
        float max_gimbal_yaw_rate_rad_s = 3, max_gimbal_pitch_rate_rad_s = 2;
        std::uint32_t input_timeout_ms = 100;
        ManualCommandMapper::Config mapper{};
        std::uint32_t permission_timeout_ms = 300;
        core::TimeUs vision_timeout_us = 100000;
        core::OrientationReference expected_vision_reference{1, 1};
        float max_vision_yaw_acceleration_rad_s2 = 30, max_vision_pitch_acceleration_rad_s2 = 20;
        float requested_fire_rate_hz = 5;
        float override_enter_norm = 0.15f, override_exit_norm = 0.05f;
        core::TimeUs override_release_us = 200000;
    };
    explicit CommandManager(const Config &config) : config_(config) {
    }
    // One owner thread; do not alternate overloads on the same instance.
    [[nodiscard]] int validate() const;
    // now_us is microseconds; output robotics stamps remain milliseconds.
    [[nodiscard]] int step(const CommandInputs &, core::TimeUs now_us, CommandDecision &out);
    int reset(std::uint64_t now_ms);
    int step(const OperatorIntent &, const GlobalSafetyDecision &, std::uint64_t now_ms, RobotCommand &out);

private:
    Config config_;
    std::uint32_t sequence_ = 0;
    OperatorMode previous_mode_ = OperatorMode::Safe;
    core::TimeUs auto_entry_us_ = 0, last_step_us_ = 0, override_quiet_since_us_ = 0;
    std::uint64_t auto_baseline_sequence_ = 0, last_seen_vision_sequence_ = 0;
    bool have_vision_sequence_ = false, have_step_time_ = false;
    bool manual_override_ = false, have_override_quiet_time_ = false;
    void updateAutoOverride(OperatorIntent &, const CommandInputs &, core::TimeUs);
};
}
