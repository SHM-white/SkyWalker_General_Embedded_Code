#include <errno.h>
#include <stddef.h>

#include <control/feedforward_pid.h>

#include "control_internal.h"

int control_feedforward_pid_validate(
    const control_feedforward_pid_config *config)
{
    if (config == NULL) {
        return -EINVAL;
    }

    int ret = control_pid_validate(&config->feedback);
    if (ret < 0) {
        return ret;
    }

    return control_feedforward_validate(&config->feedforward);
}

int control_feedforward_pid_reset(
    control_feedforward_pid_state *state,
    float current_measurement)
{
    if (state == NULL) {
        return -EINVAL;
    }

    return control_pid_reset(&state->feedback,
                             current_measurement);
}

int control_feedforward_pid_step(
    control_feedforward_pid_state *state,
    const control_feedforward_pid_config *config,
    const control_feedforward_pid_input *input,
    control_feedforward_pid_result *result)
{
    if (state == NULL || input == NULL || result == NULL) {
        return -EINVAL;
    }

    int ret = control_feedforward_pid_validate(config);
    if (ret < 0) {
        return ret;
    }

    float feedforward = 0.0f;
    ret = control_feedforward_calculate(&config->feedforward,
                                        &input->reference,
                                        &feedforward);
    if (ret < 0) {
        return ret;
    }

    control_feedforward_pid_state next = *state;
    control_feedforward_pid_result local = {0};

    ret = control_pid_step_core(&next.feedback,
                                &config->feedback,
                                &input->feedback,
                                feedforward,
                                &local.feedback);
    if (ret < 0) {
        return ret;
    }

    local.feedforward = feedforward;
    local.output = local.feedback.output;

    *state = next;
    *result = local;
    return 0;
}
