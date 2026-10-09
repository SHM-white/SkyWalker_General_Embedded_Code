#pragma once
#include <robotics/command/command_inputs.hpp>
#include <robotics/command/manual_command_mapper.hpp>
#include <robotics/messages/remote.hpp>
namespace skywalker::robotics {
class CommandArbiter {
public:
    struct Config {
        float max_chassis_vx_m_s = 3, max_chassis_vy_m_s = 3, max_chassis_wz_rad_s = 6;
        float max_gimbal_yaw_rate_rad_s = 3, max_gimbal_pitch_rate_rad_s = 2;
        std::uint32_t input_timeout_ms = 100;
        ManualCommandMapper::Config mapper{};
        std::uint32_t permission_timeout_ms = 300;
        bool require_referee_for_motion = true;
        bool allow_auto = true;
        core::TimeUs vision_timeout_us = 100000;
        core::OrientationReference expected_vision_reference{1, 1};
        float max_vision_yaw_acceleration_rad_s2 = 30, max_vision_pitch_acceleration_rad_s2 = 20;
        // 调参：RC、鼠标连点和长按共用此射频；应按摩擦轮/拨盘/热量能力标定。
        float requested_fire_rate_hz = 5;
        // 调参：250 ms 长按为建议初值，500 ms 连点窗来自用户规则。
        MouseShooterGesture::Config mouse_gesture{};
        // 台架由执行线程显式授权；开启后 CommandManager 初始门控为关闭。
        bool require_external_run_gate = false;
        float override_enter_norm = 0.15f, override_exit_norm = 0.05f;
        core::TimeUs override_release_us = 200000;
    };
    explicit CommandArbiter(const Config &config);
    // One owner thread; inputs include the arbitration time. Results are owned values.
    [[nodiscard]] int configError() const {
        return config_error_;
    }
    [[nodiscard]] CommandDecision update(const CommandInputs &);
    void reset();
    // 唯一管理线程按原始队列顺序调用；update 再处理快照和退出期限。
    int observeRemoteFrame(const RemoteState &, bool run_allowed, std::uint64_t now_ms);
    void withdrawMouseShooter();

private:
    Config config_;
    const int config_error_;
    MouseShooterGesture mouse_gesture_;
    MouseFireIntent mouse_fire_{};
    int validateConfig() const;
    std::uint32_t sequence_ = 0;
    OperatorMode previous_mode_ = OperatorMode::Safe;
    core::TimeUs auto_entry_us_ = 0, last_step_us_ = 0, override_quiet_since_us_ = 0;
    std::uint64_t auto_baseline_sequence_ = 0, last_seen_vision_sequence_ = 0;
    bool have_vision_sequence_ = false, have_step_time_ = false;
    bool manual_override_ = false, have_override_quiet_time_ = false;
    void updateAutoOverride(OperatorIntent &, const CommandInputs &, core::TimeUs);
};
}
