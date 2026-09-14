#ifndef SKYWALKER_CONTROL_MOTOR_VELOCITY_H
#define SKYWALKER_CONTROL_MOTOR_VELOCITY_H

#include <stdbool.h>

#include <control/feedforward_pid.h>
#include <control/slew_rate_limiter.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    control_feedforward_pid_config regulator;
    control_slew_rate_config reference_slew;

    float measurement_filter_tau_s;
    float soft_deadband_rad_s;
    float requested_velocity_abs_max_rad_s;
    /* Actuator command limit: amperes for DJI, N*m for DM MIT mode. */
    float effort_abs_max;
} control_motor_velocity_config;

typedef struct {
    control_feedforward_pid_state regulator;
    control_slew_rate_state reference_slew;

    float filtered_velocity_rad_s;
    bool filter_initialized;
} control_motor_velocity_state;

typedef struct {
    float requested_velocity_rad_s;
    float measured_velocity_rad_s;
    float position_reference_rad;
    float dt_s;
    bool freeze_integrator;
} control_motor_velocity_input;

typedef struct {
    float velocity_reference_rad_s;
    float acceleration_reference_rad_s2;
    float filtered_velocity_rad_s;
    float velocity_error_rad_s;
    control_feedforward_pid_result regulator;
    /* Actuator effort: amperes for DJI, N*m for DM MIT mode. */
    float effort_command;
} control_motor_velocity_output;

int control_motor_velocity_validate(const control_motor_velocity_config *config);

int control_motor_velocity_reset(control_motor_velocity_state *state, float measured_velocity_rad_s,
                                 float initial_reference_rad_s);

/*
 * On failure neither state nor output is modified. reset() must be called
 * before the first step().
 */
int control_motor_velocity_step(control_motor_velocity_state *state, const control_motor_velocity_config *config,
                                const control_motor_velocity_input *input, control_motor_velocity_output *output);

#ifdef __cplusplus
}
#endif

#endif
