#ifndef SKYWALKER_CONTROL_PID_H
#define SKYWALKER_CONTROL_PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;

    /* D is a first-order low-pass filtered measurement rate. Zero disables filtering. */
    float derivative_tau_s;

    /* These limits apply to the I-term output, after multiplication by Ki. */
    float integral_min;
    float integral_max;

    /* Final combined output limits; asymmetric ranges are supported. */
    float output_min;
    float output_max;

    float deadband;
    float dt_min_s;
    float dt_max_s;
} control_pid_config;

typedef struct {
    float integral_output;
    float previous_measurement;
    float filtered_measurement_rate;
    bool initialized;
} control_pid_state;

typedef struct {
    float setpoint;
    float measurement;
    float dt_s;
    bool freeze_integrator;
} control_pid_input;

typedef struct {
    float error;
    float effective_error;
    float p;
    float i;
    float d;
    float feedback_unsaturated;
    float total_unsaturated;
    float output;
    bool saturated;
} control_pid_result;

int control_pid_validate(const control_pid_config *config);

int control_pid_reset(control_pid_state *state,
                      float current_measurement);

int control_pid_step(control_pid_state *state,
                     const control_pid_config *config,
                     const control_pid_input *input,
                     control_pid_result *result);

#ifdef __cplusplus
}
#endif

#endif
