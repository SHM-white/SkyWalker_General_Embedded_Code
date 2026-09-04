#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include <control/pid.h>

#include "control_internal.h"

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

static bool pid_config_fields_are_finite(
    const control_pid_config *config)
{
    return isfinite(config->kp) &&
           isfinite(config->ki) &&
           isfinite(config->kd) &&
           isfinite(config->derivative_tau_s) &&
           isfinite(config->integral_min) &&
           isfinite(config->integral_max) &&
           isfinite(config->output_min) &&
           isfinite(config->output_max) &&
           isfinite(config->deadband) &&
           isfinite(config->dt_min_s) &&
           isfinite(config->dt_max_s);
}

int control_pid_validate(const control_pid_config *config)
{
    if (config == NULL) {
        return -EINVAL;
    }
    if (!pid_config_fields_are_finite(config)) {
        return -EINVAL;
    }
    if (config->kp < 0.0f ||
        config->ki < 0.0f ||
        config->kd < 0.0f ||
        config->derivative_tau_s < 0.0f ||
        config->deadband < 0.0f) {
        return -EINVAL;
    }
    if (config->integral_min > config->integral_max ||
        config->output_min > config->output_max) {
        return -EINVAL;
    }
    if (config->dt_min_s <= 0.0f ||
        config->dt_max_s < config->dt_min_s) {
        return -EINVAL;
    }
    return 0;
}

int control_pid_reset(control_pid_state *state,
                      float current_measurement)
{
    if (state == NULL || !isfinite(current_measurement)) {
        return -EINVAL;
    }

    const control_pid_state next = {
        .integral_output = 0.0f,
        .previous_measurement = current_measurement,
        .filtered_measurement_rate = 0.0f,
        .initialized = true,
    };

    *state = next;
    return 0;
}

int control_pid_step_core(control_pid_state *state,
                          const control_pid_config *config,
                          const control_pid_input *input,
                          float additive_output,
                          control_pid_result *result)
{
    if (state == NULL || input == NULL || result == NULL) {
        return -EINVAL;
    }

    int ret = control_pid_validate(config);
    if (ret < 0) {
        return ret;
    }

    if (!isfinite(input->setpoint) ||
        !isfinite(input->measurement) ||
        !isfinite(input->dt_s) ||
        !isfinite(additive_output)) {
        return -EINVAL;
    }
    if (input->dt_s < config->dt_min_s ||
        input->dt_s > config->dt_max_s) {
        return -ERANGE;
    }

    if (state->initialized &&
        (!isfinite(state->integral_output) ||
         !isfinite(state->previous_measurement) ||
         !isfinite(state->filtered_measurement_rate))) {
        return -EINVAL;
    }
    if (state->initialized &&
        (state->integral_output < config->integral_min ||
         state->integral_output > config->integral_max)) {
        return -ERANGE;
    }

    control_pid_state next = *state;
    control_pid_result local = {0};
    const bool first_step = !next.initialized;

    if (first_step) {
        next.integral_output = 0.0f;
        next.previous_measurement = input->measurement;
        next.filtered_measurement_rate = 0.0f;
        next.initialized = true;
    }

    const float error = input->setpoint - input->measurement;
    if (!isfinite(error)) {
        return -ERANGE;
    }

    const float effective_error =
        fabsf(error) <= config->deadband ? 0.0f : error;

    const float p = config->kp * effective_error;
    if (!isfinite(p)) {
        return -ERANGE;
    }

    float measurement_rate = 0.0f;
    if (!first_step) {
        measurement_rate =
            (input->measurement - next.previous_measurement) /
            input->dt_s;
        if (!isfinite(measurement_rate)) {
            return -ERANGE;
        }
    }

    float filtered_rate = measurement_rate;
    if (config->derivative_tau_s > 0.0f) {
        const float denominator =
            config->derivative_tau_s + input->dt_s;
        const float alpha = input->dt_s / denominator;
        filtered_rate = next.filtered_measurement_rate +
                        alpha * (measurement_rate -
                                 next.filtered_measurement_rate);
    }
    if (!isfinite(filtered_rate)) {
        return -ERANGE;
    }

    const float d = -config->kd * filtered_rate;
    if (!isfinite(d)) {
        return -ERANGE;
    }

    const float integral_delta =
        config->ki * effective_error * input->dt_s;
    const float integral_unclamped =
        next.integral_output + integral_delta;
    if (!isfinite(integral_delta) ||
        !isfinite(integral_unclamped)) {
        return -ERANGE;
    }

    const float integral_candidate =
        clampf(integral_unclamped,
               config->integral_min,
               config->integral_max);

    const float candidate_feedback =
        p + integral_candidate + d;
    const float candidate_total =
        candidate_feedback + additive_output;
    if (!isfinite(candidate_feedback) ||
        !isfinite(candidate_total)) {
        return -ERANGE;
    }

    const bool pushes_high_saturation =
        candidate_total > config->output_max &&
        effective_error > 0.0f;
    const bool pushes_low_saturation =
        candidate_total < config->output_min &&
        effective_error < 0.0f;

    if (!input->freeze_integrator &&
        !pushes_high_saturation &&
        !pushes_low_saturation) {
        next.integral_output = integral_candidate;
    }

    next.previous_measurement = input->measurement;
    next.filtered_measurement_rate = filtered_rate;

    local.error = error;
    local.effective_error = effective_error;
    local.p = p;
    local.i = next.integral_output;
    local.d = d;
    local.feedback_unsaturated =
        local.p + local.i + local.d;
    local.total_unsaturated =
        local.feedback_unsaturated + additive_output;

    if (!isfinite(local.feedback_unsaturated) ||
        !isfinite(local.total_unsaturated)) {
        return -ERANGE;
    }

    local.output = clampf(local.total_unsaturated,
                          config->output_min,
                          config->output_max);
    local.saturated =
        local.total_unsaturated < config->output_min ||
        local.total_unsaturated > config->output_max;

    *state = next;
    *result = local;
    return 0;
}

int control_pid_step(control_pid_state *state,
                     const control_pid_config *config,
                     const control_pid_input *input,
                     control_pid_result *result)
{
    return control_pid_step_core(state,
                                 config,
                                 input,
                                 0.0f,
                                 result);
}
