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

    if (config->position.output_min < -config->velocity.requested_velocity_abs_max_rad_s || config->position.output_max > config->velocity.requested_velocity_abs_max_rad_s) {
        return -ERANGE;
    }
    return 0;
}

int control_motor_position_reset(control_motor_position_state *state, const control_motor_position_config *config, float measured_position_rad, float measured_velocity_rad_s) {
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

int control_motor_position_step(control_motor_position_state *state, const control_motor_position_config *config, const control_motor_position_input *input, control_motor_position_output *output) {
    if (state == NULL || input == NULL || output == NULL) {
        return -EINVAL;
    }

    int ret = control_motor_position_validate(config);
    if (ret < 0) {
        return ret;
    }
    if (!isfinite(input->continuous_target_rad) || !isfinite(input->continuous_position_rad) || !isfinite(input->measured_velocity_rad_s) || !isfinite(input->dt_s)) {
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
        .requested_velocity_rad_s = local.position.output,
        .measured_velocity_rad_s = input->measured_velocity_rad_s,
        .position_reference_rad = input->continuous_target_rad,
        .dt_s = input->dt_s,
        .freeze_integrator = false,
    };
    ret = control_motor_velocity_step(&next.velocity, &config->velocity, &velocity_input, &local.velocity);
    if (ret < 0) {
        return ret;
    }

    local.effort_command = local.velocity.effort_command;
    if (!isfinite(local.effort_command)) {
        return -ERANGE;
    }

    *state = next;
    *output = local;
    return 0;
}
