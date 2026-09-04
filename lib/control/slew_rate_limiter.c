#include <errno.h>
#include <math.h>
#include <stddef.h>

#include <control/slew_rate_limiter.h>

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

int control_slew_rate_validate(
    const control_slew_rate_config *config)
{
    if (config == NULL) {
        return -EINVAL;
    }
    if (!isfinite(config->rising_rate_per_s) ||
        !isfinite(config->falling_rate_per_s) ||
        config->rising_rate_per_s < 0.0f ||
        config->falling_rate_per_s < 0.0f) {
        return -EINVAL;
    }
    return 0;
}

int control_slew_rate_reset(control_slew_rate_state *state,
                            float current_value)
{
    if (state == NULL || !isfinite(current_value)) {
        return -EINVAL;
    }

    const control_slew_rate_state next = {
        .value = current_value,
        .initialized = true,
    };
    *state = next;
    return 0;
}

int control_slew_rate_step(control_slew_rate_state *state,
                           const control_slew_rate_config *config,
                           float requested_value,
                           float dt_s,
                           float *limited_value,
                           float *limited_rate)
{
    if (state == NULL ||
        limited_value == NULL ||
        limited_rate == NULL) {
        return -EINVAL;
    }

    int ret = control_slew_rate_validate(config);
    if (ret < 0) {
        return ret;
    }

    if (!isfinite(requested_value) || !isfinite(dt_s)) {
        return -EINVAL;
    }
    if (dt_s <= 0.0f) {
        return -ERANGE;
    }
    if (!state->initialized) {
        return -EACCES;
    }
    if (!isfinite(state->value)) {
        return -EINVAL;
    }

    const float delta = requested_value - state->value;
    if (!isfinite(delta)) {
        return -ERANGE;
    }

    const float rate_limit = delta >= 0.0f
                           ? config->rising_rate_per_s
                           : config->falling_rate_per_s;
    const float maximum_delta = rate_limit * dt_s;
    if (!isfinite(maximum_delta)) {
        return -ERANGE;
    }

    const float applied_delta =
        clampf(delta, -maximum_delta, maximum_delta);
    const float next_value = state->value + applied_delta;
    const float next_rate = applied_delta / dt_s;
    if (!isfinite(next_value) || !isfinite(next_rate)) {
        return -ERANGE;
    }

    control_slew_rate_state next = *state;
    next.value = next_value;

    *state = next;
    *limited_value = next_value;
    *limited_rate = next_rate;
    return 0;
}
