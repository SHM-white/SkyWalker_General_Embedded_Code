#pragma once

#include <control/motor_backend.hpp>
#include <control/motor_position.h>

namespace skywalker::control {

enum class PositionReference {
    StartupRelative, // Continuous rad, zero at begin's first valid measurement.
    DriverContinuous, // Continuous driver coordinates (DM saved zero / DJI RX zero).
    AbsoluteNearest, // Single-turn target, nearest path; requires capability.
};

class PositionMotor {
public:
    struct Config {
        control_motor_position_config loop{};
        EffortUnit effort_unit = EffortUnit::Unspecified;
        MotorSafety safety{};
        PositionReference reference = PositionReference::StartupRelative;
    };
    struct Telemetry {
        MotorMeasurement measurement{};
        control_motor_position_output output{};
        double requested_position_rad = 0.0;
        double target_position_rad = 0.0; // Resolved in driver coordinates.
        double position_rad = 0.0; // Relative to begin only in StartupRelative.
        float dt_s = 0.0f;
        bool valid = false;
    };

    PositionMotor(MotorBackend &backend, const Config &config);
    int begin();
    int update(double target_position_rad);
    int stop();
    std::int64_t elapsedMs() const { return runtime_.elapsedMs(); }
    const Telemetry &telemetry() const { return telemetry_; }
    const MotorStatus &status() const { return runtime_.status(); }

private:
    Config config_;
    MotorRuntime runtime_;
    control_motor_position_state state_{};
    Telemetry telemetry_{};
    double initial_position_rad_ = 0.0;
    double coordinate_origin_rad_ = 0.0;
    bool begin_attempted_ = false;
};

} // namespace skywalker::control
