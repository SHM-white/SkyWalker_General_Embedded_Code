#include <errno.h>
#include <math.h>
#include <stddef.h>

#include <control/motor_position.h>

int control_motor_position_validate(const control_motor_position_config *config) {
    if (config == NULL) {
        return -EINVAL;
    }

    int ret = control_pid_validate(&config->position);
    if (ret < 0) {
        return ret;
    }
    ret = control_motor_velocity_validate(&config->velocity);
    if (ret < 0) {
        return ret;
    }

    if (!isfinite(config->velocity_feedforward_gain) || config->velocity_feedforward_gain < 0.0f ||
        !isfinite(config->acceleration_effort_gain) || config->acceleration_effort_gain < 0.0f) {
        return -EINVAL;
    }

    if (config->position.output_min < -config->velocity.requested_velocity_abs_max_rad_s ||
        config->position.output_max > config->velocity.requested_velocity_abs_max_rad_s) {
        return -ERANGE;
    }
    return 0;
}

int control_motor_position_reset(control_motor_position_state *state, const control_motor_position_config *config,
                                 float measured_position_rad, float measured_velocity_rad_s) {
    if (state == NULL || !isfinite(measured_position_rad) || !isfinite(measured_velocity_rad_s)) {
        return -EINVAL;
    }

    int ret = control_motor_position_validate(config);
    if (ret < 0) {
        return ret;
    }

    control_motor_position_state next = {0};
    ret = control_pid_reset(&next.position, measured_position_rad);
    if (ret < 0) {
        return ret;
    }
    ret = control_motor_velocity_reset(&next.velocity, measured_velocity_rad_s, 0.0f);
    if (ret < 0) {
        return ret;
    }

    *state = next;
    return 0;
}

int control_motor_position_step(control_motor_position_state *state, const control_motor_position_config *config,
                                const control_motor_position_input *input, control_motor_position_output *output) {
    if (state == NULL || input == NULL || output == NULL) {
        return -EINVAL;
    }

    int ret = control_motor_position_validate(config);
    if (ret < 0) {
        return ret;
    }
    if (!isfinite(input->continuous_target_rad) || !isfinite(input->continuous_position_rad) ||
        !isfinite(input->measured_velocity_rad_s) || !isfinite(input->dt_s) ||
        (input->has_position_reference && !isfinite(input->position_reference_rad)) ||
        (input->has_reference_velocity && !isfinite(input->reference_velocity_rad_s)) ||
        (input->has_reference_acceleration && !isfinite(input->reference_acceleration_rad_s2))) {
        return -EINVAL;
    }

    control_motor_position_state next = *state;
    control_motor_position_output local = {0};

    const control_pid_input position_input = {
        .setpoint = input->continuous_target_rad,
        .measurement = input->continuous_position_rad,
        .dt_s = input->dt_s,
        /* Preserve the existing sample: I is configured but frozen. */
        .freeze_integrator = true,
    };
    ret = control_pid_step(&next.position, &config->position, &position_input, &local.position);
    if (ret < 0) {
        return ret;
    }

    const control_motor_velocity_input velocity_input = {
        .requested_velocity_rad_s = local.position.output +
                                    config->velocity_feedforward_gain *
                                        (input->has_reference_velocity ? input->reference_velocity_rad_s : 0.0f),
        .measured_velocity_rad_s = input->measured_velocity_rad_s,
        .position_reference_rad = input->has_position_reference ? input->position_reference_rad
                                                                : input->continuous_target_rad,
        .dt_s = input->dt_s,
        .freeze_integrator = false,
    };
    if (!isfinite(velocity_input.requested_velocity_rad_s)) {
        return -ERANGE;
    }
    const control_motor_velocity_state velocity_state_before = next.velocity;
    ret = control_motor_velocity_step(&next.velocity, &config->velocity, &velocity_input, &local.velocity);
    if (ret < 0) {
        return ret;
    }

    /* Acceleration effort feedforward is added outside the regulator, so the
     * regulator has to be told when the combined effort saturates. Re-running
     * one step with a frozen integrator uses the existing anti-windup input
     * instead of duplicating the regulator state in this layer. */
    const float effort_feedforward = config->acceleration_effort_gain *
                                     (input->has_reference_acceleration ? input->reference_acceleration_rad_s2 : 0.0f);
    if (!isfinite(effort_feedforward)) {
        return -ERANGE;
    }
    float effort_total = local.velocity.effort_command + effort_feedforward;
    if (fabsf(effort_total) > config->velocity.effort_abs_max) {
        control_motor_velocity_input retry = velocity_input;
        retry.freeze_integrator = true;
        next.velocity = velocity_state_before;
        ret = control_motor_velocity_step(&next.velocity, &config->velocity, &retry, &local.velocity);
        if (ret < 0) {
            return ret;
        }
        effort_total = local.velocity.effort_command + effort_feedforward;
    }
    if (effort_total > config->velocity.effort_abs_max) {
        effort_total = config->velocity.effort_abs_max;
    }
    else if (effort_total < -config->velocity.effort_abs_max) {
        effort_total = -config->velocity.effort_abs_max;
    }

    local.effort_command = effort_total;
    if (!isfinite(local.effort_command)) {
        return -ERANGE;
    }

    *state = next;
    *output = local;
    return 0;
}
