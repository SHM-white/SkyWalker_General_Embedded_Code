#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <communication/remote/remote_receiver.hpp>
#include <robotics/messages/remote.hpp>

namespace skywalker::samples::control {

inline communication::RemoteReceiver::Config receiverConfig() {
    communication::RemoteReceiver::Config config{};
    config.decoder.decode_wheel = true;
    config.remote.offline_timeout_ms = 100; // 输入失联期限，ms；连点窗口不能延长此期限。
    return config;
}

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
        bool keyboard_mouse_selectable = false; // 右 Middle 在手动档选择键鼠。
        std::uint32_t input_timeout_ms = 100; // ms，须与接收和命令层失联期限协调。
        std::uint32_t neutral_arm_ms = 500; // ms，安全档归中保持时间，防止上电直接运行。
        std::uint32_t clear_hold_ms = 1000; // ms，安全档清故障手势保持时间。
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
    std::uint64_t neutralHeldMs() const {
        return neutral_timing_ && state_.remote.stamp.timestamp_ms >= neutral_since_
                   ? state_.remote.stamp.timestamp_ms - neutral_since_
                   : 0;
    }

    static float normalize(std::int16_t raw) {
        const float value = std::clamp(float(raw) / 660.0f, -1.0f, 1.0f);
        return std::fabs(value) <= .03f ? 0.0f : std::copysign((std::fabs(value) - .03f) / .97f, value);
    }

    // Withdraw only for user stop/emergency or an expired real input/link.
    // Motor and CAN execution states never withdraw this input intent.
    void withdraw() {
        state_.armed = state_.navigation_active = state_.manual_active = false;
        state_.run_allowed = false;
        state_.mode = RcMode::Safe;
        state_.spin_requested = state_.friction_requested = state_.continuous_requested = false;
        state_.keyboard_mouse_selected = false;
        neutral_timing_ = neutral_ready_ = safe_baseline_ = false;
        clear_timing_ = false;
    }

    const RcControlState &update(const robotics::RemoteState &remote, std::uint64_t now_ms) {
        using robotics::RcSwitch;
        state_.remote = remote;
        state_.clear_estop = false;
        state_.fresh = remote.online && robotics::isFresh(remote.stamp, now_ms, config_.input_timeout_ms) &&
                       remote.left_switch != RcSwitch::Unknown && remote.right_switch != RcSwitch::Unknown;
        if (!state_.fresh) {
            withdraw();
            state_.clear_stamp = {};
            previous_left_ = previous_right_ = RcSwitch::Unknown;
            return state_;
        }
        if (have_sequence_ && !robotics::sequenceAfter(remote.stamp.sequence, last_sequence_))
            return state_;
        have_sequence_ = true;
        last_sequence_ = remote.stamp.sequence;
        const auto input_ms = remote.stamp.timestamp_ms;
        if (have_input_time_ && input_ms < last_input_ms_) {
            withdraw();
            state_.fresh = false;
            previous_left_ = previous_right_ = RcSwitch::Unknown;
            last_input_ms_ = input_ms;
            return state_;
        }
        have_input_time_ = true;
        last_input_ms_ = input_ms;
        const bool neutral = centered(remote);
        const auto left = remote.left_switch, right = remote.right_switch;

        if (left == RcSwitch::Down) {
            if (previous_left_ != RcSwitch::Down || state_.run_allowed)
                withdraw();
            state_.mode = RcMode::Safe;
            state_.run_allowed = false;
            state_.manual_active = state_.navigation_active = false;
            state_.spin_requested = state_.friction_requested = state_.continuous_requested = false;
            state_.keyboard_mouse_selected = false;
            if (neutral && right == RcSwitch::Down) {
                if (!neutral_timing_) {
                    neutral_since_ = input_ms;
                    neutral_timing_ = true;
                }
                if (input_ms >= neutral_since_ && input_ms - neutral_since_ >= config_.neutral_arm_ms)
                    neutral_ready_ = safe_baseline_ = true;
            }
            else {
                neutral_timing_ = neutral_ready_ = false;
            }
            if (neutral && right == RcSwitch::Up && safe_baseline_) {
                if (!clear_timing_) {
                    clear_since_ = input_ms;
                    clear_timing_ = true;
                }
                if (input_ms >= clear_since_ && input_ms - clear_since_ >= config_.clear_hold_ms &&
                    state_.clear_event_id != std::numeric_limits<std::uint32_t>::max()) {
                    ++state_.clear_event_id;
                    state_.clear_stamp = remote.stamp;
                    state_.clear_estop = true;
                    safe_baseline_ = neutral_ready_ = neutral_timing_ = clear_timing_ = false;
                }
            }
            else
                clear_timing_ = false;
        }
        else {
            // 安全档归中基线成立后，左 Down→Middle 只建立解锁；手动档在左 Up。
            if (left == RcSwitch::Middle && previous_left_ == RcSwitch::Down && neutral_ready_ && neutral &&
                right == RcSwitch::Down && state_.start_event_id != std::numeric_limits<std::uint64_t>::max()) {
                ++state_.start_event_id;
                armed_ = true;
            }
            neutral_timing_ = neutral_ready_ = safe_baseline_ = clear_timing_ = false;
            // 视觉执行样例保留 Middle=手动、Up=Auto；其余样例只有左 Up 是手动档。
            const bool manual = armed_ && (config_.allow_auto ? left != RcSwitch::Down : left == RcSwitch::Up);
            state_.manual_active = manual;
            state_.navigation_active = armed_ && !config_.allow_auto && left == RcSwitch::Middle;
            state_.run_allowed = manual;
            state_.mode = !armed_ ? RcMode::Safe : manual ? RcMode::Manual : RcMode::Navigation;
            state_.keyboard_mouse_selected = manual && config_.keyboard_mouse_selectable &&
                                             right == RcSwitch::Middle;
            state_.spin_requested = manual && right == RcSwitch::Down;
            state_.friction_requested = manual &&
                                        (state_.keyboard_mouse_selected ? remote.mouse.right
                                                                        : right == RcSwitch::Up);
            // RC 发射拨杆直接请求同一射频的连发，不生产与键鼠编号冲突的单发事件。
            // 键鼠连发手势只由命令线程管理，不在此处再次判断左键或点击间隔。
            state_.continuous_requested = manual && !state_.keyboard_mouse_selected &&
                                          right == RcSwitch::Up;
        }
        state_.armed = armed_;
        previous_left_ = left;
        previous_right_ = right;
        return state_;
    }

private:
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
    std::uint64_t neutral_since_ = 0, clear_since_ = 0, last_input_ms_ = 0;
    bool have_sequence_ = false, neutral_timing_ = false, neutral_ready_ = false;
    bool safe_baseline_ = false, clear_timing_ = false, have_input_time_ = false;
    bool armed_ = false;
};

} // namespace skywalker::samples::control
