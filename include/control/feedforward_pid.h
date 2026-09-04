#ifndef SKYWALKER_CONTROL_FEEDFORWARD_PID_H
#define SKYWALKER_CONTROL_FEEDFORWARD_PID_H

#include <control/feedforward.h>
#include <control/pid.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    control_pid_config feedback;
    control_feedforward_config feedforward;
} control_feedforward_pid_config;

typedef struct {
    control_pid_state feedback;
} control_feedforward_pid_state;

typedef struct {
    control_pid_input feedback;
    control_feedforward_reference reference;
} control_feedforward_pid_input;

typedef struct {
    control_pid_result feedback;
    float feedforward;
    float output;
} control_feedforward_pid_result;

int control_feedforward_pid_validate(
    const control_feedforward_pid_config *config);

int control_feedforward_pid_reset(
    control_feedforward_pid_state *state,
    float current_measurement);

int control_feedforward_pid_step(
    control_feedforward_pid_state *state,
    const control_feedforward_pid_config *config,
    const control_feedforward_pid_input *input,
    control_feedforward_pid_result *result);

#ifdef __cplusplus
}
#endif

#endif
