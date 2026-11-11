#ifndef SKYWALKER_CONTROL_MOTOR_POSITION_H
#define SKYWALKER_CONTROL_MOTOR_POSITION_H

#include <control/motor_velocity.h>
#include <control/pid.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    control_pid_config position;
    control_motor_velocity_config velocity;
    /*
     * Joint reference feedforward: velocity_feedforward_gain * reference velocity
     * is added to the position-loop velocity request. Zero keeps the legacy
     * cascade where the position loop alone owns the velocity reference.
     */
    float velocity_feedforward_gain;
    /*
     * Actuator effort per rad/s^2 (amperes for DJI, N*m for DM MIT mode). Zero
     * disables acceleration effort feedforward. Enable only after a bench has
     * identified the direction and the available current margin.
     */
    float acceleration_effort_gain;
} control_motor_position_config;

typedef struct {
    control_pid_state position;
    control_motor_velocity_state velocity;
} control_motor_position_state;

typedef struct {
    float continuous_target_rad;
    float continuous_position_rad;
    float measured_velocity_rad_s;
    float dt_s;
    /* Optional physical target for feedforward when PID coordinates are rebased. */
    float position_reference_rad;
    bool has_position_reference;
    /* Optional joint reference from the same feasible trajectory as the target. */
    float reference_velocity_rad_s;
    bool has_reference_velocity;
    float reference_acceleration_rad_s2;
    bool has_reference_acceleration;
} control_motor_position_input;

typedef struct {
    control_pid_result position;
    control_motor_velocity_output velocity;
    /* Actuator effort: amperes for DJI, N*m for DM MIT mode. */
    float effort_command;
} control_motor_position_output;

int control_motor_position_validate(const control_motor_position_config *config);

int control_motor_position_reset(control_motor_position_state *state, const control_motor_position_config *config,
                                 float measured_position_rad, float measured_velocity_rad_s);

/*
 * On failure neither the outer position state, the nested velocity state,
 * nor output is modified. reset() must be called before the first step().
 */
int control_motor_position_step(control_motor_position_state *state, const control_motor_position_config *config,
                                const control_motor_position_input *input, control_motor_position_output *output);

#ifdef __cplusplus
}
#endif

#endif
