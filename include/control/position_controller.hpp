#pragma once

#include <cerrno>

#include <control/motor_position.h>

namespace skywalker::control {

class PositionController {
public:
    using Config = control_motor_position_config;
    using State = control_motor_position_state;
    using Input = control_motor_position_input;
    using Output = control_motor_position_output;

    explicit PositionController(const Config &config) : config_(config) {
    }

    int validate() const {
        return control_motor_position_validate(&config_);
    }

    int reset(float measured_position_rad, float measured_velocity_rad_s) {
        const int ret = control_motor_position_reset(&state_, &config_, measured_position_rad, measured_velocity_rad_s);
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

        const int ret = control_motor_position_step(&state_, &config_, &input, &output);
        if (ret == 0) {
            last_output_ = output;
        }
        return ret;
    }

    int step(float target_position_rad, float measured_position_rad, float measured_velocity_rad_s, float dt_s,
             Output &output) {
        const Input input = {
            .continuous_target_rad = target_position_rad,
            .continuous_position_rad = measured_position_rad,
            .measured_velocity_rad_s = measured_velocity_rad_s,
            .dt_s = dt_s,
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
