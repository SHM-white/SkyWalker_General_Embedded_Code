#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <robotics/messages/remote.hpp>
#if defined(CONFIG_SKYWALKER_REMOTE_RECEIVER)
#include <communication/remote/remote_receiver.hpp>
#endif
#include <robotics/messages/remote.hpp>

namespace skywalker::samples::control {

#if defined(CONFIG_SKYWALKER_REMOTE_RECEIVER)
inline communication::RemoteReceiver::Config receiverConfig() {
    communication::RemoteReceiver::Config config{};
    config.decoder.decode_wheel = true;
    config.remote.offline_timeout_ms = 100; // 输入失联期限，ms；连点窗口不能延长此期限。
    return config;
}
#endif

// 统一键位：左 Down=安全、Middle=导航、Up=手动；右 Down=小陀螺、
// Middle=不请求机构（键鼠样例选择键鼠）、Up=摩擦轮与拨弹。
// 右侧波轮（弹簧回中）不映射到云台/平移：在 sentry_gimbal 中作为小陀螺的
// 模拟强度输入（符号决定转向，幅度缩放速度曲线），回中时退回开关的满速曲线。
enum class RcMode : std::uint8_t { Safe, Navigation, Manual };

struct RcControlState {
    robotics::RemoteState remote{};
    bool fresh = false;
    RcMode mode = RcMode::Safe;
    bool armed = false;             // 安全档归中基线后由左 Down→Middle 建立
    bool navigation_active = false; // 已解锁且左 Middle；无导航源的样例保持停机
    bool manual_active = false;     // 已解锁且左 Up
    bool run_allowed = false;       // 手动运动授权，与 manual_active 同步
    // 左 Middle 导航档失联后保留解锁和导航授权；重新上线后无需重新拨档。
    bool navigation_hold = false;
    bool spin_requested = false; // 右 Down：小陀螺（仅手动档）
    // 左 Middle 已授权的自主子模式；掉线保持时保留最后接受的值。
    robotics::AutonomousMode autonomous = robotics::AutonomousMode::None;
    std::uint64_t start_event_id = 0;
    // Dormant compatibility fields for bench examples; no RC clear gesture is produced.
    bool clear_estop = false;
    std::uint32_t clear_event_id = 0;
    robotics::MessageStamp clear_stamp{};
    bool keyboard_mouse_selected = false;
    bool friction_requested = false;
    bool continuous_requested = false;
};

// One application thread owns this state machine. Other threads read a copy
// through SnapshotCache. Reading a cached RC frame never renews its stamp.
class RcControlAdapter {
public:
    struct Config {
        // 视觉执行样例保留原三档：左 Middle=手动、左 Up=自动；其余样例左 Up=手动。
        bool allow_auto = false;
        bool keyboard_mouse_selectable = false;  // 右 Middle 在手动档选择键鼠。
        bool require_right_middle_to_arm = true; // 仅前后行走入口不使用右拨杆。
        std::uint32_t input_timeout_ms = 100;    // ms，须与接收和命令层失联期限协调。
        // 左 Middle 导航档失联时保持导航解锁；左 Up 手动档失联仍立即撤权锁定。
        bool hold_navigation_on_link_loss = true;
        // 正式应用显式打开：左 Middle 时右拨杆选择巡航子模式。
        bool enable_cruise_switches = false;
        // 正式应用显式打开：上电快捷解锁改看“解锁后是否会产生动作”。
        // 其他样例维持“归中 + 右 Middle”的旧快捷条件。
        bool semantic_startup_unlock = false;
    };
    RcControlAdapter() = default;
    explicit RcControlAdapter(Config config) : config_(config) {
    }

    // Read-only arming diagnostics; use the received frame's time, not wall time.
    bool controlsCentered() const {
        return centered(state_.remote);
    }
    bool armReady() const {
        return neutral_ready_;
    }
    // Kept for diagnostics compatibility. The 500 ms timed baseline is gone.
    std::uint64_t neutralHeldMs() const {
        return 0;
    }

    static float normalize(std::int16_t raw) {
        const float value = std::clamp(float(raw) / 660.0f, -1.0f, 1.0f);
        return std::fabs(value) <= .03f ? 0.0f : std::copysign((std::fabs(value) - .03f) / .97f, value);
    }

    // Withdraw only for user stop/emergency or an expired real input/link.
    // Motor and CAN execution states never withdraw this input intent.
    void withdraw() {
        stopMotion();
        neutral_ready_ = false;
        startup_neutral_available_ = false;
        if (have_input_time_)
            startup_consumed_ = true;
    }

    // `startup_no_action` 只描述当前候选帧的启动语义：由调用方用正式配置判定
    // “解锁后不会主动产生动作”。内部先完成原有在线/时间戳/序号/开关合法性判定，
    // 只有真正接受这一帧后才使用它。返回值仍为内部状态的只读引用，
    // 调用方复制为局部变量，不跨线程保存这个引用。
    const RcControlState &update(const robotics::RemoteState &remote, std::uint64_t now_ms,
                                 bool startup_no_action = false) {
        using robotics::RcSwitch;
        // The accepted frame, not an untrusted candidate, owns the input lease.
        // An offline receiver is a real link-loss event; an old/bad candidate is not.
        if (!remote.online) {
            state_.fresh = false;
            // 保留已经接受的拨杆、序号和原始戳：掉线保持必须依据最后有效输入，
            // 不能被失联快照中的 Unknown 或旧帧改写自主选择。
            state_.remote.online = false;
            if (!holdNavigationOnLinkLoss(state_.remote.left_switch)) {
                withdraw();
                previous_left_ = previous_right_ = RcSwitch::Unknown;
            }
            return state_;
        }
        const bool newer = remote.stamp.valid &&
                           (!have_sequence_ || robotics::sequenceAfter(remote.stamp.sequence, last_sequence_)) &&
                           (!have_input_time_ || remote.stamp.timestamp_ms >= last_input_ms_);
        const bool candidate_valid = newer && robotics::isFresh(remote.stamp, now_ms, config_.input_timeout_ms) &&
                                     remote.left_switch != RcSwitch::Unknown &&
                                     remote.right_switch != RcSwitch::Unknown;
        if (!candidate_valid) {
            if (!have_input_time_ || !robotics::isFresh(state_.remote.stamp, now_ms, config_.input_timeout_ms)) {
                state_.fresh = false;
                if (!holdNavigationOnLinkLoss(state_.remote.left_switch)) {
                    withdraw();
                    previous_left_ = previous_right_ = RcSwitch::Unknown;
                }
            }
            return state_;
        }
        const auto input_ms = remote.stamp.timestamp_ms;
        const bool first_frame = !have_input_time_;
        state_.remote = remote;
        state_.fresh = true;
        state_.navigation_hold = false;
        have_sequence_ = true;
        last_sequence_ = remote.stamp.sequence;
        have_input_time_ = true;
        last_input_ms_ = input_ms;
        const bool neutral = centered(remote);
        const auto left = remote.left_switch, right = remote.right_switch;
        const bool arm_switch_ready = !config_.require_right_middle_to_arm || right == RcSwitch::Middle;
        // 普通解锁继续要求归中 + 右 Middle；语义快捷解锁改看“解锁后是否会产生动作”。
        // 两者语义不同：raw=25 满足 ±33 归中，但映射后可能已是非零运动意图。
        const bool startup_eligible = config_.semantic_startup_unlock ? startup_no_action
                                                                      : (neutral && arm_switch_ready);

        // One-shot startup shortcut: the first accepted frame may arm from a
        // centred neutral pose without a 500 ms hold. It is never rebuilt after
        // a later link loss or a non-neutral frame.
        if (first_frame && !startup_consumed_) {
            if (startup_eligible)
                startup_neutral_available_ = true;
            else
                startup_consumed_ = true;
        }
        if (startup_neutral_available_ && !startup_eligible) {
            startup_neutral_available_ = false;
            startup_consumed_ = true;
        }

        if (left == RcSwitch::Down) {
            if (previous_left_ != RcSwitch::Down || state_.run_allowed)
                stopMotion();
            state_.mode = RcMode::Safe;
            state_.run_allowed = false;
            state_.manual_active = state_.navigation_active = false;
            state_.autonomous = robotics::AutonomousMode::None;
            state_.spin_requested = state_.friction_requested = state_.continuous_requested = false;
            state_.keyboard_mouse_selected = false;
            // Down still stops immediately. It does not destroy the startup
            // shortcut while the pose remains centered and the right switch is Middle.
            neutral_ready_ = neutral && arm_switch_ready;
            // 语义模式由 startup_eligible 管理的一次性资格决定，不再被右拨杆取消。
            if (!config_.semantic_startup_unlock && !neutral_ready_)
                startup_neutral_available_ = false;
        }
        else {
            // The startup shortcut arms on the first non-Down frame. After that,
            // the normal explicit Down->Middle edge remains the only re-arm path.
            if (startup_neutral_available_) {
                if (state_.start_event_id != std::numeric_limits<std::uint64_t>::max())
                    ++state_.start_event_id;
                armed_ = true;
                startup_neutral_available_ = false;
                startup_consumed_ = true;
            }
            if (left == RcSwitch::Middle && previous_left_ == RcSwitch::Down && neutral_ready_ && neutral &&
                arm_switch_ready && !armed_ && state_.start_event_id != std::numeric_limits<std::uint64_t>::max()) {
                ++state_.start_event_id;
                armed_ = true;
            }
            neutral_ready_ = false;
            // 视觉执行样例保留 Middle=手动、Up=Auto；其余样例只有左 Up 是手动档。
            const bool manual = armed_ && (config_.allow_auto ? left != RcSwitch::Down : left == RcSwitch::Up);
            state_.manual_active = manual;
            state_.navigation_active = armed_ && !config_.allow_auto && left == RcSwitch::Middle;
            state_.run_allowed = manual;
            state_.mode = !armed_ ? RcMode::Safe : manual ? RcMode::Manual : RcMode::Navigation;
            // An already-selected Down/Up must never start spinning/firing on unlock.
            // Require a neutral frame *in manual mode* before accepting right-switch actions.
            if (!manual) {
                right_neutral_seen_ = false;
                mouse_release_seen_ = false;
            }
            else if (right == RcSwitch::Middle) {
                right_neutral_seen_ = true;
            }
            state_.keyboard_mouse_selected = manual && config_.keyboard_mouse_selectable && right == RcSwitch::Middle;
            if (state_.keyboard_mouse_selected && !remote.mouse.left && !remote.mouse.right)
                mouse_release_seen_ = true;
            if (!state_.keyboard_mouse_selected)
                mouse_release_seen_ = false;
            state_.spin_requested = manual && right_neutral_seen_ && right == RcSwitch::Down;
            state_.friction_requested = manual &&
                                        (state_.keyboard_mouse_selected ? mouse_release_seen_ && remote.mouse.right
                                                                        : right_neutral_seen_ && right == RcSwitch::Up);
            state_.continuous_requested = manual && !state_.keyboard_mouse_selected && right_neutral_seen_ &&
                                          right == RcSwitch::Up;
        }
        // 自主子模式只在已解锁的左 Middle 有效；手动档右拨杆不改变它。
        state_.autonomous = state_.navigation_active
                                ? robotics::selectAutonomousMode(remote, config_.enable_cruise_switches)
                                : robotics::AutonomousMode::None;
        state_.armed = armed_;
        previous_left_ = left;
        previous_right_ = right;
        return state_;
    }

private:
    // 只有已解锁且处于左 Middle 导航档时才保留导航会话；手动/安全档失联仍撤权。
    bool holdNavigationOnLinkLoss(robotics::RcSwitch last_left) {
        if (!config_.hold_navigation_on_link_loss || !armed_ || state_.mode != RcMode::Navigation ||
            last_left != robotics::RcSwitch::Middle)
            return false;
        state_.fresh = false;
        state_.navigation_hold = true;
        state_.navigation_active = true;
        state_.manual_active = false;
        state_.run_allowed = false;
        state_.mode = RcMode::Navigation;
        // 巡航小陀螺只看 state_.autonomous，不依赖手动字段，故不被误停。
        state_.spin_requested = state_.friction_requested = state_.continuous_requested = false;
        state_.keyboard_mouse_selected = false;
        right_neutral_seen_ = mouse_release_seen_ = false;
        previous_left_ = previous_right_ = robotics::RcSwitch::Unknown;
        return true;
    }
    void stopMotion() {
        state_.armed = state_.navigation_active = state_.manual_active = false;
        state_.run_allowed = false;
        state_.navigation_hold = false;
        state_.mode = RcMode::Safe;
        state_.autonomous = robotics::AutonomousMode::None;
        state_.spin_requested = state_.friction_requested = state_.continuous_requested = false;
        state_.keyboard_mouse_selected = false;
        armed_ = false;
        right_neutral_seen_ = false;
        mouse_release_seen_ = false;
    }
    static bool centered(const robotics::RemoteState &remote) {
        const auto centered_axis = [](std::int16_t raw) { return std::abs(int(raw)) <= 33; };
        return centered_axis(remote.analog.left_x) && centered_axis(remote.analog.left_y) &&
               centered_axis(remote.analog.right_x) && centered_axis(remote.analog.right_y) &&
               centered_axis(remote.analog.wheel);
    }
    Config config_{};
    RcControlState state_{};
    robotics::RcSwitch previous_left_ = robotics::RcSwitch::Unknown;
    robotics::RcSwitch previous_right_ = robotics::RcSwitch::Unknown;
    std::uint32_t last_sequence_ = 0;
    std::uint64_t last_input_ms_ = 0;
    bool have_sequence_ = false, neutral_ready_ = false;
    bool have_input_time_ = false, armed_ = false;
    bool startup_neutral_available_ = false, startup_consumed_ = false;
    bool right_neutral_seen_ = false, mouse_release_seen_ = false;
};

} // namespace skywalker::samples::control
