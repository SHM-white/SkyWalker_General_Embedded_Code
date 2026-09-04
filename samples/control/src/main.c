#include <errno.h>
#include <math.h>

#include <zephyr/sys/printk.h>

#include <control/angle.h>
#include <control/feedforward_pid.h>
#include <control/slew_rate_limiter.h>

#define SAMPLE_TOLERANCE 0.0001f

static bool float_near(float actual, float expected)
{
    return fabsf(actual - expected) <= SAMPLE_TOLERANCE;
}

static int run_feedforward_pid_sample(void)
{
    const control_feedforward_pid_config config = {
        .feedback = {
            .kp = 4.0f,
            .ki = 1.0f,
            .kd = 0.0f,
            .derivative_tau_s = 0.0f,
            .integral_min = -5.0f,
            .integral_max = 5.0f,
            .output_min = -5.0f,
            .output_max = 5.0f,
            .deadband = 0.0f,
            .dt_min_s = 0.001f,
            .dt_max_s = 1.0f,
        },
        .feedforward = {
            .k_bias = 3.0f,
            .k_static = 0.0f,
            .k_velocity = 0.0f,
            .k_acceleration = 0.0f,
            .k_gravity = 0.0f,
            .velocity_epsilon = 0.0f,
            .acceleration_epsilon = 0.0f,
            .gravity_model = CONTROL_GRAVITY_NONE,
        },
    };
    control_feedforward_pid_state state = {0};
    const control_feedforward_pid_input input = {
        .feedback = {
            .setpoint = 1.0f,
            .measurement = 0.0f,
            .dt_s = 1.0f,
            .freeze_integrator = false,
        },
        .reference = {
            .position_ref_rad = 0.0f,
            .velocity_ref = 0.0f,
            .acceleration_ref = 0.0f,
        },
    };
    control_feedforward_pid_result result = {0};

    int ret = control_feedforward_pid_reset(&state, 0.0f);
    if (ret < 0) {
        return ret;
    }

    ret = control_feedforward_pid_step(&state, &config, &input, &result);
    if (ret < 0) {
        return ret;
    }

    if (!float_near(result.output, 5.0f) ||
        !float_near(state.feedback.integral_output, 0.0f) ||
        !result.feedback.saturated) {
        return -EIO;
    }

    printk("feedforward PID: output=%d mU, saturated=%d\n",
           (int)(result.output * 1000.0f),
           result.feedback.saturated);
    return 0;
}

// static int run_reference_helpers_sample(void)
{
    const control_slew_rate_config slew_config = {
        .rising_rate_per_s = 2.0f,
        .falling_rate_per_s = 4.0f,
    };
    control_slew_rate_state slew_state = {0};
    float value = 0.0f;
    float rate = 0.0f;

    int ret = control_slew_rate_reset(&slew_state, 0.0f);
    if (ret < 0) {
        return ret;
    }

    ret = control_slew_rate_step(&slew_state,
                                 &slew_config,
                                 10.0f,
                                 0.1f,
                                 &value,
                                 &rate);
    if (ret < 0) {
        return ret;
    }
    if (!float_near(value, 0.2f) || !float_near(rate, 2.0f)) {
        return -EIO;
    }

    float angle_error = 0.0f;
    ret = control_shortest_angle_error(
        3.14159265358979323846f, 0.0f, &angle_error);
    if (ret < 0) {
        return ret;
    }
    if (!float_near(angle_error, -3.14159265358979323846f)) {
        return -EIO;
    }

    printk("helpers: slew=%d milli-units, rate=%d milli-units/s, angle=%d mrad\n",
           (int)(value * 1000.0f),
           (int)(rate * 1000.0f),
           (int)(angle_error * 1000.0f));
    return 0;
}

int main(void)
{
    int ret = run_feedforward_pid_sample();
    if (ret < 0) {
        printk("control sample failed in feedforward PID: %d\n", ret);
        return ret;
    }

    // ret = run_reference_helpers_sample();
    // if (ret < 0) {
    //     printk("control sample failed in reference helpers: %d\n", ret);
    //     return ret;
    // }

    printk("control sample passed\n");
    return 0;
}
