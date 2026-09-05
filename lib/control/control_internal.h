#ifndef SKYWALKER_CONTROL_INTERNAL_H
#define SKYWALKER_CONTROL_INTERNAL_H

#include <control/pid.h>

int control_pid_step_core(control_pid_state *state,
                          const control_pid_config *config,
                          const control_pid_input *input,
                          float additive_output,
                          control_pid_result *result);

#endif
