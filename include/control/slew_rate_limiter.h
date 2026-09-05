#ifndef SKYWALKER_CONTROL_SLEW_RATE_LIMITER_H
#define SKYWALKER_CONTROL_SLEW_RATE_LIMITER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float value;
    bool initialized;
} control_slew_rate_state;

typedef struct {
    float rising_rate_per_s;
    float falling_rate_per_s;
} control_slew_rate_config;

int control_slew_rate_validate(
    const control_slew_rate_config *config);

int control_slew_rate_reset(control_slew_rate_state *state,
                            float current_value);

int control_slew_rate_step(control_slew_rate_state *state,
                           const control_slew_rate_config *config,
                           float requested_value,
                           float dt_s,
                           float *limited_value,
                           float *limited_rate);

#ifdef __cplusplus
}
#endif

#endif
