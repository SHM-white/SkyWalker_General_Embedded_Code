#pragma once

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <core/measurement.hpp>
#include <robotics/vehicle/calibration.hpp>

namespace skywalker::robotics {

// Variable-speed top spin (小陀螺) shaper.
//
// The operator switch says "spin"; the optional analog dial (DR16 wheel) says
// "how fast and which way". The actual body yaw-rate target comes from the
// tunable curve `vehicle::spinRateCurveRadS(elapsed_s)`, which is a separate
// pure function so the shape can be edited without touching this class. This
// class only owns:
//   * the elapsed time since the spin was enabled,
//   * the down-ramp when the operator releases the switch,
//   * the immediate stop when the run permission is withdrawn.
//
// Units: rad/s for the rate, rad/s^2 for the limits, s for the curve argument.
// Single execution thread; fixed state, no allocation.
class SpinSpeedProfile {
public:
    struct Config {
        float max_rate_rad_s = 6.0f;
        // Limit while the target magnitude grows. Zero follows the curve exactly
        // (the curve itself must then be continuous).
        float acceleration_limit_rad_s2 = 0;
        // Limit while the target returns to zero after a release.
        float deceleration_rad_s2 = 6.0f;
        // Rate below which a released profile snaps to exactly zero.
        float stop_epsilon_rad_s = 0.01f;
        // Analog dial (DR16 wheel) dead band in normalized units. Below it the
        // dial counts as centred and the switch owns the request.
        float dial_deadband = 0.05f;
        // Largest accepted control interval; longer intervals stage no change.
        core::TimeUs max_cycle_us = 50000;
    };
    struct Output {
        bool active = false;
        float target_rad_s = 0, applied_rad_s = 0, elapsed_s = 0;
    };

    explicit SpinSpeedProfile(const Config &config) : config_(config) {
    }
    int validate() const {
        return std::isfinite(config_.max_rate_rad_s) && config_.max_rate_rad_s > 0 &&
                       std::isfinite(config_.acceleration_limit_rad_s2) && config_.acceleration_limit_rad_s2 >= 0 &&
                       std::isfinite(config_.deceleration_rad_s2) && config_.deceleration_rad_s2 > 0 &&
                       std::isfinite(config_.stop_epsilon_rad_s) && config_.stop_epsilon_rad_s >= 0 &&
                       std::isfinite(config_.dial_deadband) && config_.dial_deadband >= 0 &&
                       config_.dial_deadband < 1 && config_.max_cycle_us
                   ? 0
                   : -EINVAL;
    }
    void reset() {
        previous_us_ = 0;
        elapsed_s_ = 0;
        applied_rad_s_ = 0;
        active_ = false;
    }
    // `requested` is the operator switch and `allowed` the run permission.
    SpinSpeedProfile::Output update(bool requested, bool allowed, core::TimeUs now_us) {
        return update(requested, allowed, 0.0f, now_us);
    }
    // Same, with the optional analog dial in [-1, 1]. A deflected dial owns the
    // rate: its sign is the spin direction and its magnitude scales the curve.
    // A centred dial falls back to the switch's full curve.
    SpinSpeedProfile::Output update(bool requested, bool allowed, float intensity, core::TimeUs now_us) {
        Output out{};
        const auto previous = previous_us_;
        previous_us_ = now_us;
        if (validate() < 0) {
            reset();
            return out;
        }
        const float dt = previous && now_us > previous && now_us - previous <= config_.max_cycle_us
                             ? static_cast<float>(now_us - previous) * 1e-6f
                             : 0.0f;
        if (!allowed) {
            // A withdrawn permission never ramps; the chassis must stop asking
            // for spin in the same cycle.
            elapsed_s_ = 0;
            applied_rad_s_ = 0;
            active_ = false;
            return out;
        }
        const float dial = std::isfinite(intensity) ? std::clamp(intensity, -1.0f, 1.0f) : 0.0f;
        const bool dial_active = std::fabs(dial) >= config_.dial_deadband;
        const bool spinning = requested || dial_active;
        if (spinning) {
            active_ = true;
            elapsed_s_ += dt;
            const float curve = vehicle::spinRateCurveRadS(elapsed_s_);
            const float target = dial_active ? curve * dial : curve;
            out.target_rad_s = std::clamp(target, -config_.max_rate_rad_s, config_.max_rate_rad_s);
        }
        else {
            // Release: keep the current rate and let the deceleration limit take
            // it to zero along the same path.
            elapsed_s_ = 0;
            out.target_rad_s = 0;
        }
        if (dt > 0) {
            const bool growing = std::fabs(out.target_rad_s) > std::fabs(applied_rad_s_);
            const float limit = growing ? config_.acceleration_limit_rad_s2 : config_.deceleration_rad_s2;
            const float max_change = limit > 0 ? limit * dt : std::fabs(out.target_rad_s - applied_rad_s_);
            applied_rad_s_ += std::clamp(out.target_rad_s - applied_rad_s_, -max_change, max_change);
        }
        if (!spinning && std::fabs(applied_rad_s_) <= config_.stop_epsilon_rad_s) {
            applied_rad_s_ = 0;
            active_ = false;
        }
        out.applied_rad_s = applied_rad_s_;
        out.elapsed_s = elapsed_s_;
        out.active = active_;
        return out;
    }

private:
    Config config_{};
    core::TimeUs previous_us_ = 0;
    float elapsed_s_ = 0, applied_rad_s_ = 0;
    bool active_ = false;
};

} // namespace skywalker::robotics
