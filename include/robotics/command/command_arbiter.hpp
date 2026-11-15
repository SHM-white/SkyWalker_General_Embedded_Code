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
        // Referee restrictions are action-local; no global motion veto.
        bool require_referee_for_chassis = true;
        bool require_referee_for_gimbal = true;
        bool require_referee_for_shooter = true;
        bool enable_shooter = true;
        bool allow_auto = true;
        bool allow_navigation = false;
        core::TimeUs navigation_timeout_us = 100000;
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
        // 本地巡航扫描：无有效视觉目标时由仲裁器生成头部世界速率命令。
        // 与 allow_navigation（导航接收器能力）互相独立，可单独启用。
        bool enable_cruise = false;
        // 调参：扫描头部世界偏航角速度 rad/s；方向沿用头部世界坐标约定。
        float cruise_yaw_rate_rad_s = 0.3f;
        // 调参：巡航扫描的俯仰关节速率 rad/s 与相对入口角的单边幅度 rad。
        // 速率为 0 时保持入口角度（Pitch 不摆头）。
        float cruise_pitch_rate_rad_s = 0;
        float cruise_pitch_span_rad = 0;
        // 扫描允许的 Pitch 软限位（编码器坐标），与“入口角±幅度”取交集。
        float cruise_pitch_min_rad = -3.14159265f, cruise_pitch_max_rad = 3.14159265f;
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
    ManualCommandMapper::Config mapperConfig() const;
    std::uint32_t sequence_ = 0;
    OperatorMode previous_mode_ = OperatorMode::Safe;
    core::TimeUs auto_entry_us_ = 0, last_step_us_ = 0, override_quiet_since_us_ = 0;
    std::uint64_t auto_baseline_sequence_ = 0, last_seen_vision_sequence_ = 0;
    bool have_vision_sequence_ = false, have_step_time_ = false;
    bool manual_override_ = false, have_override_quiet_time_ = false;
    bool navigation_active_ = false;
    core::TimeUs navigation_entry_us_ = 0;
    std::uint64_t navigation_baseline_sequence_ = 0;
    // 最近处理的解锁会话；变化时清理入口基线，避免跨会话复用目标。
    std::uint32_t operator_session_ = 0;
    // 巡航 Pitch 摆头：进入扫描时用实测关节角播种，之后由仲裁器按周期积分并在
    // 入口角±幅度与软限位的交集处折返。
    float cruise_pitch_planned_rad_ = 0, cruise_pitch_low_rad_ = 0, cruise_pitch_high_rad_ = 0;
    core::TimeUs cruise_pitch_last_us_ = 0;
    std::int8_t cruise_pitch_direction_ = 1;
    bool cruise_pitch_seeded_ = false;
    void updateAutoOverride(OperatorIntent &, const CommandInputs &, core::TimeUs);
    void clearEntryBaselines();
    void resetCruisePitch();
    float cruisePitchRate(const CommandInputs &, core::TimeUs);
};
// 两个调用方（仲裁器与上电启动动作判定）共用的有效映射配置，
// 避免快捷解锁使用一份缺少巡航/导航/键鼠语义的默认 mapper。
inline ManualCommandMapper::Config effectiveMapperConfig(const CommandArbiter::Config &config) {
    auto c = config.mapper;
    // 视觉执行样例保留左 Up=自动；巡航启用后左 Up 必须仍是手动档。
    c.auto_on_up = c.auto_on_up || (config.allow_auto && !config.allow_navigation && !config.enable_cruise);
    c.navigation_on_middle = c.navigation_on_middle || config.allow_navigation || config.enable_cruise;
    c.cruise_on_middle = c.cruise_on_middle || config.enable_cruise;
    return c;
}
}
