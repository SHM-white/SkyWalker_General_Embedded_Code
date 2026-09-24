#pragma once

#include <cstdint>

#include <control/motor_common.hpp>
#include <control/motor_position.h>
#include <drivers/motor/motor.hpp>
#include <zephyr/spinlock.h>

namespace skywalker::control {

enum class PositionReference {
    StartupRelative,
    DriverContinuous,
    AbsoluteNearest,
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
        motor::MotorSnapshot motor{};
        control_motor_position_output output{};
        double requested_position_rad = 0.0;
        double target_position_rad = 0.0;
        double position_rad = 0.0;
        float dt_s = 0.0f;
        float effort_command = 0.0f;
        EffortUnit effort_unit = EffortUnit::Unspecified;
        bool valid = false;
        int error = 0;
    };

    PositionMotor(motor::Motor &motor, const Config &config);
    PositionMotor(const PositionMotor &) = delete;
    PositionMotor &operator=(const PositionMotor &) = delete;

    [[nodiscard]] int configure();
    [[nodiscard]] int update(double target_position_rad, float dt_s);
    [[nodiscard]] int reset();
    Telemetry telemetry() const;
    PositionReference reference() const { return config_.reference; }

private:
    int resetFrom(const motor::MotorSnapshot &snapshot, bool explicit_reset);
    int fail(int error, const motor::MotorSnapshot &snapshot, bool active);
    void publish(const Telemetry &next);

    motor::Motor &motor_;
    Config config_{};
    control_motor_position_state state_{};
    mutable struct k_spinlock telemetry_lock_{};
    Telemetry telemetry_{};
    double initial_position_rad_ = 0.0;
    double coordinate_origin_rad_ = 0.0;
    std::uint64_t anchor_reference_generation_ = 0;
    std::uint64_t observed_enable_generation_ = 0;
    std::uint64_t observed_reference_generation_ = 0;
    bool configured_ = false;
    bool history_valid_ = false;
    bool explicit_reset_anchor_valid_ = false;
};

} // namespace skywalker::control
