#pragma once

#include <cerrno>

#include <control/motor_velocity.h>

namespace skywalker::control {

class VelocityController {
public:
    using Config = control_motor_velocity_config;
    using State = control_motor_velocity_state;
    using Input = control_motor_velocity_input;
    using Output = control_motor_velocity_output;

    explicit VelocityController(const Config &config) : config_(config) {
    }

    int validate() const {
        return control_motor_velocity_validate(&config_);
    }

    int reset(float measured_velocity_rad_s, float initial_reference_rad_s = 0.0f) {
        const int ret = control_motor_velocity_reset(&state_, measured_velocity_rad_s, initial_reference_rad_s);
        if (ret == 0) {
            initialized_ = true;
            last_output_ = {};
        }
        return ret;
    }

    int step(const Input &input, Output &output) {
        if (!initialized_) {
            return -EACCES;
        }

        const int ret = control_motor_velocity_step(&state_, &config_, &input, &output);
        if (ret == 0) {
            last_output_ = output;
        }
        return ret;
    }

    int step(float requested_velocity_rad_s, float measured_velocity_rad_s, float dt_s, Output &output) {
        const Input input = {
            .requested_velocity_rad_s = requested_velocity_rad_s,
            .measured_velocity_rad_s = measured_velocity_rad_s,
            .position_reference_rad = 0.0f,
            .dt_s = dt_s,
            .freeze_integrator = false,
        };
        return step(input, output);
    }

    const Config &config() const {
        return config_;
    }

    const State &state() const {
        return state_;
    }

    const Output &lastOutput() const {
        return last_output_;
    }

    bool initialized() const {
        return initialized_;
    }

private:
    Config config_{};
    State state_{};
    Output last_output_{};
    bool initialized_ = false;
};

} // namespace skywalker::control
