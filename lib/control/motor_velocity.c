#include <errno.h>
#include <math.h>
#include <stddef.h>

#include <control/motor_velocity.h>

static float clampf(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float apply_soft_deadband(float value, float deadband) {
    const float magnitude = fabsf(value);
    if (magnitude <= deadband) {
        return 0.0f;
    }
    return copysignf(magnitude - deadband, value);
}

int control_motor_velocity_validate(const control_motor_velocity_config *config) {
    if (config == NULL) {
        return -EINVAL;
    }

    int ret = control_feedforward_pid_validate(&config->regulator);
    if (ret < 0) {
        return ret;
    }
    ret = control_slew_rate_validate(&config->reference_slew);
    if (ret < 0) {
        return ret;
    }

    if (!isfinite(config->measurement_filter_tau_s) || !isfinite(config->soft_deadband_rad_s) || !isfinite(config->requested_velocity_abs_max_rad_s) || !isfinite(config->effort_abs_max)) {
        return -EINVAL;
    }
    if (config->measurement_filter_tau_s < 0.0f || config->soft_deadband_rad_s < 0.0f || config->requested_velocity_abs_max_rad_s <= 0.0f || config->effort_abs_max < 0.0f) {
        return -EINVAL;
    }
    return 0;
}

int control_motor_velocity_reset(control_motor_velocity_state *state, float measured_velocity_rad_s, float initial_reference_rad_s) {
    if (state == NULL || !isfinite(measured_velocity_rad_s) || !isfinite(initial_reference_rad_s)) {
        return -EINVAL;
    }

    control_motor_velocity_state next = {0};
    int ret = control_feedforward_pid_reset(&next.regulator, measured_velocity_rad_s);
    if (ret < 0) {
        return ret;
    }
    ret = control_slew_rate_reset(&next.reference_slew, initial_reference_rad_s);
    if (ret < 0) {
        return ret;
    }

    next.filtered_velocity_rad_s = measured_velocity_rad_s;
    next.filter_initialized = true;
    *state = next;
    return 0;
}

int control_motor_velocity_step(control_motor_velocity_state *state, const control_motor_velocity_config *config, const control_motor_velocity_input *input, control_motor_velocity_output *output) {
    if (state == NULL || input == NULL || output == NULL) {
        return -EINVAL;
    }

    int ret = control_motor_velocity_validate(config);
    if (ret < 0) {
        return ret;
    }
    if (!isfinite(input->requested_velocity_rad_s) || !isfinite(input->measured_velocity_rad_s) || !isfinite(input->position_reference_rad) || !isfinite(input->dt_s)) {
        return -EINVAL;
    }
    if (fabsf(input->requested_velocity_rad_s) > config->requested_velocity_abs_max_rad_s) {
        return -ERANGE;
    }
    if (!state->filter_initialized) {
        return -EACCES;
    }
    if (!isfinite(state->filtered_velocity_rad_s)) {
        return -EINVAL;
    }

    control_motor_velocity_state next = *state;
    control_motor_velocity_output local = {0};

    const float filter_denominator = config->measurement_filter_tau_s + input->dt_s;
    if (!isfinite(filter_denominator) || filter_denominator <= 0.0f) {
        return -ERANGE;
    }
    const float alpha = input->dt_s / filter_denominator;
    const float filtered_velocity = next.filtered_velocity_rad_s + alpha * (input->measured_velocity_rad_s - next.filtered_velocity_rad_s);
    if (!isfinite(alpha) || !isfinite(filtered_velocity)) {
        return -ERANGE;
    }
    local.filtered_velocity_rad_s = filtered_velocity;

    ret = control_slew_rate_step(&next.reference_slew, &config->reference_slew, input->requested_velocity_rad_s, input->dt_s, &local.velocity_reference_rad_s, &local.acceleration_reference_rad_s2);
    if (ret < 0) {
        return ret;
    }

    local.velocity_error_rad_s = local.velocity_reference_rad_s - filtered_velocity;
    const float effective_error = apply_soft_deadband(local.velocity_error_rad_s, config->soft_deadband_rad_s);
    const float regulator_measurement = local.velocity_reference_rad_s - effective_error;
    if (!isfinite(local.velocity_error_rad_s) || !isfinite(effective_error) || !isfinite(regulator_measurement)) {
        return -ERANGE;
    }

    const control_feedforward_pid_input regulator_input = {
        .feedback =
            {
                .setpoint = local.velocity_reference_rad_s,
                .measurement = regulator_measurement,
                .dt_s = input->dt_s,
                .freeze_integrator = input->freeze_integrator,
            },
        .reference =
            {
                .position_ref_rad = input->position_reference_rad,
                .velocity_ref = local.velocity_reference_rad_s,
                .acceleration_ref = local.acceleration_reference_rad_s2,
            },
    };

    ret = control_feedforward_pid_step(&next.regulator, &config->regulator, &regulator_input, &local.regulator);
    if (ret < 0) {
        return ret;
    }

    local.effort_command = clampf(local.regulator.output, -config->effort_abs_max, config->effort_abs_max);
    if (!isfinite(local.effort_command)) {
        return -ERANGE;
    }

    next.filtered_velocity_rad_s = filtered_velocity;
    next.filter_initialized = true;
    *state = next;
    *output = local;
    return 0;
}
