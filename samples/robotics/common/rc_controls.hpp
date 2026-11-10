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
enum class RcMode : std::uint8_t { Safe, Navigation, Manual };

struct RcControlState {
    robotics::RemoteState remote{};
    bool fresh = false;
    RcMode mode = RcMode::Safe;
    bool armed = false;             // 安全档归中基线后由左 Down→Middle 建立
    bool navigation_active = false; // 已解锁且左 Middle；无导航源的样例保持停机
    bool manual_active = false;     // 已解锁且左 Up
    bool run_allowed = false;       // 手动运动授权，与 manual_active 同步
    bool spin_requested = false;    // 右 Down：小陀螺（仅手动档）
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

    const RcControlState &update(const robotics::RemoteState &remote, std::uint64_t now_ms) {
        using robotics::RcSwitch;
        const bool fresh = remote.online && robotics::isFresh(remote.stamp, now_ms, config_.input_timeout_ms) &&
                           remote.left_switch != RcSwitch::Unknown && remote.right_switch != RcSwitch::Unknown;
        if (!fresh) {
            state_.remote = remote;
            state_.fresh = false;
            withdraw();
            previous_left_ = previous_right_ = RcSwitch::Unknown;
            return state_;
        }
        // Old sequence or time-regressed candidates are dropped without
        // overwriting the last accepted frame or withdrawing its state.
        if (have_sequence_ && !robotics::sequenceAfter(remote.stamp.sequence, last_sequence_))
            return state_;
        const auto input_ms = remote.stamp.timestamp_ms;
        if (have_input_time_ && input_ms < last_input_ms_)
            return state_;
        const bool first_frame = !have_input_time_;
        state_.remote = remote;
        state_.fresh = true;
        have_sequence_ = true;
        last_sequence_ = remote.stamp.sequence;
        have_input_time_ = true;
        last_input_ms_ = input_ms;
        const bool neutral = centered(remote);
        const auto left = remote.left_switch, right = remote.right_switch;
        const bool arm_switch_ready = !config_.require_right_middle_to_arm || right == RcSwitch::Middle;

        // One-shot startup shortcut: the first accepted frame may arm from a
        // centered neutral pose without a 500 ms hold. It is never rebuilt after
        // a later link loss or a non-neutral frame.
        if (first_frame && !startup_consumed_) {
            if (neutral && arm_switch_ready)
                startup_neutral_available_ = true;
            else
                startup_consumed_ = true;
        }
        if (startup_neutral_available_ && (!neutral || !arm_switch_ready)) {
            startup_neutral_available_ = false;
            startup_consumed_ = true;
        }

        if (left == RcSwitch::Down) {
            if (previous_left_ != RcSwitch::Down || state_.run_allowed)
                stopMotion();
            state_.mode = RcMode::Safe;
            state_.run_allowed = false;
            state_.manual_active = state_.navigation_active = false;
            state_.spin_requested = state_.friction_requested = state_.continuous_requested = false;
            state_.keyboard_mouse_selected = false;
            // Down still stops immediately. It does not destroy the startup
            // shortcut while the pose remains centered and the right switch is Middle.
            neutral_ready_ = neutral && arm_switch_ready;
            if (!neutral_ready_)
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
                arm_switch_ready && !armed_ &&
                state_.start_event_id != std::numeric_limits<std::uint64_t>::max()) {
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
        state_.armed = armed_;
        previous_left_ = left;
        previous_right_ = right;
        return state_;
    }

private:
    void stopMotion() {
        state_.armed = state_.navigation_active = state_.manual_active = false;
        state_.run_allowed = false;
        state_.mode = RcMode::Safe;
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
