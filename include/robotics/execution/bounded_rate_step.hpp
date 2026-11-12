#pragma once
#include <algorithm>
#include <cmath>

namespace skywalker::robotics {

// One scalar rate plan shared by the cloud-board allocator and the
// chassis-board world-rate shaper. Units: rad/s for the rate, rad/s^2 for the
// acceleration and rad/s^3 for the jerk.
struct BoundedRatePlan {
    float rate_rad_s = 0, acceleration_rad_s2 = 0;
    // Last step status: a bound clipped the request, or no feasible acceleration
    // window existed and the plan was braked towards standstill.
    bool limited = false, conflict = false;
};

struct BoundedRateLimits {
    float min_rate_rad_s = 0, max_rate_rad_s = 0;
    // Asymmetric acceleration bounds: the caller may tighten one side only,
    // e.g. when the carrier already accelerates beyond the joint capability.
    float min_acceleration_rad_s2 = 0, max_acceleration_rad_s2 = 0;
    // Zero disables jerk shaping. The acceleration is then clamped per cycle
    // only, which is the first-stage bench configuration.
    float max_jerk_rad_s3 = 0;
};

// Advance one control cycle towards `target_acceleration_rad_s2`.
//
// With jerk shaping the feasible next-acceleration window is
// [max(a0-J*dt, a_min), min(a0+J*dt, a_max)]. Inside that window the rate is
// integrated as w1 = w0 + (a0+a1)*dt/2, and both the end-of-cycle rate and the
// interior extremum of w(t) = w0 + a0*t + j*t^2/2 must stay inside the rate
// bounds. A velocity reserve of a1^2/(2*J) is kept so the acceleration can still
// be brought back to zero before the bound is reached.
//
// Both feasibility predicates are monotone in a1 (the cycle maximum grows with
// a1, the cycle minimum shrinks), so the admissible window is bracketed by two
// bisections. `limited` reports a clipped request; `conflict` reports an empty
// window, in which case the plan is braked towards standstill and may exceed the
// comfort jerk bound for this cycle. Returns false only when the interval or the
// inputs are unusable, leaving the plan unchanged.
inline bool stepBoundedRate(BoundedRatePlan &plan, float target_acceleration_rad_s2, float dt_s,
                            const BoundedRateLimits &limits, bool &limited, bool &conflict) {
    limited = false;
    conflict = false;
    if (!(dt_s > 0) || !std::isfinite(dt_s) || !std::isfinite(target_acceleration_rad_s2) ||
        !std::isfinite(plan.rate_rad_s) || !std::isfinite(plan.acceleration_rad_s2) ||
        !std::isfinite(limits.min_acceleration_rad_s2) || !std::isfinite(limits.max_acceleration_rad_s2) ||
        limits.min_acceleration_rad_s2 > limits.max_acceleration_rad_s2 ||
        limits.max_rate_rad_s <= limits.min_rate_rad_s)
        return false;
    constexpr int bisection_steps = 24;
    plan.acceleration_rad_s2 = std::clamp(plan.acceleration_rad_s2, limits.min_acceleration_rad_s2,
                                          limits.max_acceleration_rad_s2);
    plan.rate_rad_s = std::clamp(plan.rate_rad_s, limits.min_rate_rad_s, limits.max_rate_rad_s);
    if (!(limits.max_jerk_rad_s3 > 0)) {
        const float acceleration = std::clamp(target_acceleration_rad_s2, limits.min_acceleration_rad_s2,
                                              limits.max_acceleration_rad_s2);
        const float requested_rate = plan.rate_rad_s + acceleration * dt_s;
        const float rate = std::clamp(requested_rate, limits.min_rate_rad_s, limits.max_rate_rad_s);
        limited = acceleration != target_acceleration_rad_s2 || rate != requested_rate;
        plan.acceleration_rad_s2 = (rate - plan.rate_rad_s) / dt_s;
        plan.rate_rad_s = rate;
        plan.limited = limited;
        plan.conflict = conflict;
        return true;
    }
    const float jerk = limits.max_jerk_rad_s3;
    const float acceleration_min = std::max(limits.min_acceleration_rad_s2, plan.acceleration_rad_s2 - jerk * dt_s);
    const float acceleration_max = std::min(limits.max_acceleration_rad_s2, plan.acceleration_rad_s2 + jerk * dt_s);
    const float rate0 = plan.rate_rad_s, acceleration0 = plan.acceleration_rad_s2;
    const auto probe = [&](float acceleration1, float &rate1, float &rate_min, float &rate_max) {
        const float slope = (acceleration1 - acceleration0) / dt_s;
        rate1 = rate0 + (acceleration0 + acceleration1) * 0.5f * dt_s;
        rate_min = std::min(rate0, rate1);
        rate_max = std::max(rate0, rate1);
        if (slope != 0) {
            // Interior extremum where the acceleration crosses zero.
            const float extremum_t = -acceleration0 / slope;
            if (extremum_t > 0 && extremum_t < dt_s) {
                const float rate = rate0 + acceleration0 * extremum_t + 0.5f * slope * extremum_t * extremum_t;
                rate_min = std::min(rate_min, rate);
                rate_max = std::max(rate_max, rate);
            }
        }
    };
    const auto upper_ok = [&](float acceleration1) {
        float rate1 = 0, rate_min = 0, rate_max = 0;
        probe(acceleration1, rate1, rate_min, rate_max);
        const float reserve = acceleration1 > 0 ? acceleration1 * acceleration1 / (2 * jerk) : 0.0f;
        return rate_max <= limits.max_rate_rad_s && rate1 + reserve <= limits.max_rate_rad_s;
    };
    const auto lower_ok = [&](float acceleration1) {
        float rate1 = 0, rate_min = 0, rate_max = 0;
        probe(acceleration1, rate1, rate_min, rate_max);
        const float reserve = acceleration1 < 0 ? acceleration1 * acceleration1 / (2 * jerk) : 0.0f;
        return rate_min >= limits.min_rate_rad_s && rate1 - reserve >= limits.min_rate_rad_s;
    };
    float acceleration_lower = acceleration_max;
    if (!lower_ok(acceleration_max))
        conflict = true;
    else {
        float low = acceleration_min, high = acceleration_max;
        for (int i = 0; i < bisection_steps; ++i) {
            const float middle = 0.5f * (low + high);
            if (lower_ok(middle))
                high = middle;
            else
                low = middle;
        }
        acceleration_lower = high;
    }
    float acceleration_upper = acceleration_min;
    if (!upper_ok(acceleration_min))
        conflict = true;
    else {
        float low = acceleration_min, high = acceleration_max;
        for (int i = 0; i < bisection_steps; ++i) {
            const float middle = 0.5f * (low + high);
            if (upper_ok(middle))
                low = middle;
            else
                high = middle;
        }
        acceleration_upper = low;
    }
    float acceleration;
    if (conflict || acceleration_lower > acceleration_upper) {
        conflict = true;
        // Shrunk bounds or an already out-of-range state: brake the rate first
        // and let the next cycles restore the comfort bounds.
        acceleration = std::clamp(-rate0 / dt_s, acceleration_min, acceleration_max);
    }
    else {
        acceleration = std::clamp(target_acceleration_rad_s2, acceleration_lower, acceleration_upper);
        limited = acceleration != target_acceleration_rad_s2;
    }
    const float integrated = rate0 + (acceleration0 + acceleration) * 0.5f * dt_s;
    const float rate = std::clamp(integrated, limits.min_rate_rad_s, limits.max_rate_rad_s);
    if (rate != integrated) {
        // The rate bound clipped the integrated trajectory. Report the conflict
        // and fall back to the mean acceleration that was actually achieved so
        // the next cycle does not restart from an acceleration the joint never
        // produced.
        plan.acceleration_rad_s2 = (rate - rate0) / dt_s;
        conflict = true;
        limited = true;
    }
    else
        // Constant jerk over the cycle: the end-of-cycle acceleration, not the
        // mean, is the correct state for the next step.
        plan.acceleration_rad_s2 = acceleration;
    plan.rate_rad_s = rate;
    plan.limited = limited;
    plan.conflict = conflict;
    return true;
}

} // namespace skywalker::robotics
