#pragma once

#include <control/motor_backend.hpp>
#include <control/motor_velocity.h>

namespace skywalker::control {

class VelocityMotor {
public:
    struct Config {
        control_motor_velocity_config loop{};
        EffortUnit effort_unit = EffortUnit::Unspecified;
        MotorSafety safety{};
    };
    struct Telemetry {
        MotorMeasurement measurement{};
        control_motor_velocity_output output{};
        float target_rad_s = 0.0f;
        float dt_s = 0.0f;
        bool valid = false;
    };

    VelocityMotor(MotorBackend &backend, const Config &config);
    // One begin attempt per object. May block and enable motor output.
    int begin();
    // Call periodically after begin, in rad/s. Computes real dt, reads/checks
    // feedback, steps the C controller and sends. Any failure stops output.
    int update(float target_velocity_rad_s);
    int stop();
    std::int64_t elapsedMs() const { return runtime_.elapsedMs(); }
    const Telemetry &telemetry() const { return telemetry_; }
    const MotorStatus &status() const { return runtime_.status(); }

private:
    Config config_;
    MotorRuntime runtime_;
    control_motor_velocity_state state_{};
    Telemetry telemetry_{};
    bool begin_attempted_ = false;
};

} // namespace skywalker::control
