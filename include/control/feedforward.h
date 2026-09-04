#ifndef SKYWALKER_CONTROL_FEEDFORWARD_H
#define SKYWALKER_CONTROL_FEEDFORWARD_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONTROL_GRAVITY_NONE = 0,
    CONTROL_GRAVITY_SIN,
    CONTROL_GRAVITY_COS,
} control_gravity_model;

typedef struct {
    float k_bias;
    float k_static;
    float k_velocity;
    float k_acceleration;
    float k_gravity;
    float velocity_epsilon;
    float acceleration_epsilon;
    control_gravity_model gravity_model;
} control_feedforward_config;

typedef struct {
    float position_ref_rad;
    float velocity_ref;
    float acceleration_ref;
} control_feedforward_reference;

int control_feedforward_validate(
    const control_feedforward_config *config);

int control_feedforward_calculate(
    const control_feedforward_config *config,
    const control_feedforward_reference *reference,
    float *output);

#ifdef __cplusplus
}
#endif

#endif
