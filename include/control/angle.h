#ifndef SKYWALKER_CONTROL_ANGLE_H
#define SKYWALKER_CONTROL_ANGLE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float last_wrapped_rad;
    float continuous_rad;
    bool initialized;
} control_angle_unwrapper;

int control_angle_unwrap_reset(control_angle_unwrapper *state,
                               float wrapped_rad);

int control_angle_unwrap_step(control_angle_unwrapper *state,
                              float wrapped_rad,
                              float *continuous_rad);

int control_shortest_angle_error(float target_rad,
                                 float measurement_rad,
                                 float *error_rad);

#ifdef __cplusplus
}
#endif

#endif
