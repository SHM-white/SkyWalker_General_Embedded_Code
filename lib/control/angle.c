#include <errno.h>
#include <math.h>
#include <stddef.h>

#include <control/angle.h>

static const float control_pi = 3.14159265358979323846f;
static const float control_two_pi = 6.28318530717958647692f;

static float wrap_to_minus_pi_inclusive(float angle_rad)
{
    float wrapped = remainderf(angle_rad, control_two_pi);

    /* The canonical interval is [-pi, pi), so map exact +pi to -pi. */
    if (wrapped >= control_pi) {
        wrapped -= control_two_pi;
    }
    return wrapped;
}

int control_angle_unwrap_reset(control_angle_unwrapper *state,
                               float wrapped_rad)
{
    if (state == NULL || !isfinite(wrapped_rad)) {
        return -EINVAL;
    }

    const float normalized =
        wrap_to_minus_pi_inclusive(wrapped_rad);
    if (!isfinite(normalized)) {
        return -ERANGE;
    }

    const control_angle_unwrapper next = {
        .last_wrapped_rad = normalized,
        .continuous_rad = normalized,
        .initialized = true,
    };
    *state = next;
    return 0;
}

int control_angle_unwrap_step(control_angle_unwrapper *state,
                              float wrapped_rad,
                              float *continuous_rad)
{
    if (state == NULL || continuous_rad == NULL) {
        return -EINVAL;
    }
    if (!isfinite(wrapped_rad)) {
        return -EINVAL;
    }
    if (!state->initialized) {
        return -EACCES;
    }
    if (!isfinite(state->last_wrapped_rad) ||
        !isfinite(state->continuous_rad)) {
        return -EINVAL;
    }

    const float normalized =
        wrap_to_minus_pi_inclusive(wrapped_rad);
    const float delta = wrap_to_minus_pi_inclusive(
        normalized - state->last_wrapped_rad);
    const float next_continuous = state->continuous_rad + delta;
    if (!isfinite(normalized) ||
        !isfinite(delta) ||
        !isfinite(next_continuous)) {
        return -ERANGE;
    }

    control_angle_unwrapper next = *state;
    next.last_wrapped_rad = normalized;
    next.continuous_rad = next_continuous;

    *state = next;
    *continuous_rad = next_continuous;
    return 0;
}

int control_shortest_angle_error(float target_rad,
                                 float measurement_rad,
                                 float *error_rad)
{
    if (error_rad == NULL ||
        !isfinite(target_rad) ||
        !isfinite(measurement_rad)) {
        return -EINVAL;
    }

    const float difference = target_rad - measurement_rad;
    if (!isfinite(difference)) {
        return -ERANGE;
    }

    const float local_error =
        wrap_to_minus_pi_inclusive(difference);
    if (!isfinite(local_error)) {
        return -ERANGE;
    }

    *error_rad = local_error;
    return 0;
}
