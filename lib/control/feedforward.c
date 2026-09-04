#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include <control/feedforward.h>

static bool feedforward_fields_are_finite(
    const control_feedforward_config *config)
{
    return isfinite(config->k_bias) &&
           isfinite(config->k_static) &&
           isfinite(config->k_velocity) &&
           isfinite(config->k_acceleration) &&
           isfinite(config->k_gravity) &&
           isfinite(config->velocity_epsilon) &&
           isfinite(config->acceleration_epsilon);
}

static float direction_from_reference(
    const control_feedforward_config *config,
    const control_feedforward_reference *reference)
{
    if (fabsf(reference->velocity_ref) >
        config->velocity_epsilon) {
        return reference->velocity_ref > 0.0f ? 1.0f : -1.0f;
    }

    if (fabsf(reference->acceleration_ref) >
        config->acceleration_epsilon) {
        return reference->acceleration_ref > 0.0f ? 1.0f : -1.0f;
    }

    return 0.0f;
}

int control_feedforward_validate(
    const control_feedforward_config *config)
{
    if (config == NULL) {
        return -EINVAL;
    }
    if (!feedforward_fields_are_finite(config)) {
        return -EINVAL;
    }
    if (config->velocity_epsilon < 0.0f ||
        config->acceleration_epsilon < 0.0f) {
        return -EINVAL;
    }
    if (config->gravity_model != CONTROL_GRAVITY_NONE &&
        config->gravity_model != CONTROL_GRAVITY_SIN &&
        config->gravity_model != CONTROL_GRAVITY_COS) {
        return -EINVAL;
    }
    return 0;
}

int control_feedforward_calculate(
    const control_feedforward_config *config,
    const control_feedforward_reference *reference,
    float *output)
{
    if (reference == NULL || output == NULL) {
        return -EINVAL;
    }

    int ret = control_feedforward_validate(config);
    if (ret < 0) {
        return ret;
    }

    if (!isfinite(reference->position_ref_rad) ||
        !isfinite(reference->velocity_ref) ||
        !isfinite(reference->acceleration_ref)) {
        return -EINVAL;
    }

    const float direction =
        direction_from_reference(config, reference);

    float gravity = 0.0f;
    switch (config->gravity_model) {
    case CONTROL_GRAVITY_NONE:
        gravity = 0.0f;
        break;
    case CONTROL_GRAVITY_SIN:
        gravity = config->k_gravity *
                  sinf(reference->position_ref_rad);
        break;
    case CONTROL_GRAVITY_COS:
        gravity = config->k_gravity *
                  cosf(reference->position_ref_rad);
        break;
    default:
        return -EINVAL;
    }

    const float local_output =
        config->k_bias +
        config->k_static * direction +
        config->k_velocity * reference->velocity_ref +
        config->k_acceleration * reference->acceleration_ref +
        gravity;

    if (!isfinite(gravity) || !isfinite(local_output)) {
        return -ERANGE;
    }

    *output = local_output;
    return 0;
}
