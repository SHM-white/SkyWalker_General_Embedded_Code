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
    config.remote.offline_timeout_ms = 100;
    return config;
}

struct RcControlState {
    robotics::RemoteState remote{};
    bool fresh = false;
    bool run_allowed = false;
    std::uint64_t start_event_id = 0;
    bool clear_estop = false;
    std::uint32_t clear_event_id = 0;
    robotics::MessageStamp clear_stamp{};
    std::uint32_t shot_event_id = 0;
    robotics::MessageStamp shot_stamp{};
    bool friction_requested = false;
    bool continuous_requested = false;
};

// One application thread owns this state machine. Other threads read a copy
// through SnapshotCache. Reading a cached RC frame never renews its stamp.
class RcControlAdapter {
public:
    struct Config {
        bool allow_auto = false;
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
        state_.run_allowed = false;
        state_.friction_requested = state_.continuous_requested = false;
        state_.shot_stamp = {};
        neutral_timing_ = neutral_ready_ = safe_baseline_ = false;
        clear_timing_ = firing_timing_ = false;
    }

    const RcControlState &update(const robotics::RemoteState &remote, std::uint64_t now_ms) {
        using robotics::RcSwitch;
        state_.remote = remote;
        state_.clear_estop = false;
        state_.fresh = remote.online && robotics::isFresh(remote.stamp, now_ms, 100) &&
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
        const bool neutral = centered(remote);
        const auto left = remote.left_switch, right = remote.right_switch;

        if (left == RcSwitch::Down) {
            if (previous_left_ != RcSwitch::Down || state_.run_allowed)
                withdraw();
            state_.run_allowed = false;
            state_.friction_requested = state_.continuous_requested = false;
            state_.shot_stamp = {};
            firing_timing_ = false;
            if (neutral && right == RcSwitch::Down) {
                if (!neutral_timing_) {
                    neutral_since_ = input_ms;
                    neutral_timing_ = true;
                }
                if (input_ms >= neutral_since_ && input_ms - neutral_since_ >= 500)
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
                if (input_ms >= clear_since_ && input_ms - clear_since_ >= 1000 &&
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
            if (left == RcSwitch::Middle && previous_left_ == RcSwitch::Down && neutral_ready_ && neutral &&
                right == RcSwitch::Down && state_.start_event_id != std::numeric_limits<std::uint64_t>::max()) {
                ++state_.start_event_id;
                state_.run_allowed = true;
            }
            neutral_timing_ = neutral_ready_ = safe_baseline_ = clear_timing_ = false;
            if (left == RcSwitch::Up && !config_.allow_auto)
                withdraw();
            state_.friction_requested = state_.run_allowed && right != RcSwitch::Down;
            if (state_.run_allowed && right == RcSwitch::Up) {
                if (previous_right_ == RcSwitch::Middle) {
                    if (state_.shot_event_id != std::numeric_limits<std::uint32_t>::max()) {
                        ++state_.shot_event_id;
                        state_.shot_stamp = remote.stamp;
                    }
                    firing_since_ = input_ms;
                    firing_timing_ = true;
                }
                state_.continuous_requested = firing_timing_ && input_ms >= firing_since_ &&
                                              input_ms - firing_since_ >= 500;
            }
            else {
                state_.continuous_requested = false;
                state_.shot_stamp = {};
                firing_timing_ = false;
            }
        }
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
    std::uint64_t neutral_since_ = 0, clear_since_ = 0, firing_since_ = 0;
    bool have_sequence_ = false, neutral_timing_ = false, neutral_ready_ = false;
    bool safe_baseline_ = false, clear_timing_ = false, firing_timing_ = false;
};

} // namespace skywalker::samples::control
