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
    struct ExternalVelocityInput {
        float velocity_request_rad_s = 0.0f;   // Motor-sign desired velocity (rad/s)
        float measured_velocity_rad_s = 0.0f;  // Motor-sign measured velocity from external IMU gyro (rad/s)
        float position_reference_rad = 0.0f;   // Physical angle for gravity feedforward (rad)
        float reference_velocity_rad_s = 0.0f; // Feedforward velocity for diagnostics (rad/s)
        float acceleration_ff_rad_s2 = 0.0f;   // Optional motor-sign acceleration feedforward (rad/s^2)
        float outer_position_error_rad = 0.0f; // Outer attitude error for telemetry (rad)
        float outer_output_rad_s = 0.0f;       // Outer attitude output for telemetry (rad/s)
        bool outer_saturated = false;
        bool velocity_request_limited = false;
    };

    struct Config {
        control_motor_position_config loop{};
        EffortUnit effort_unit = EffortUnit::Unspecified;
        PositionReference reference = PositionReference::StartupRelative;
        bool has_external_velocity = false;
        control_motor_velocity_config external_velocity{};
        float external_acceleration_effort_gain = 0.0f;
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
        bool target_valid = false;
        std::uint64_t target_sequence = 0;
        bool output_valid = false;
        int error = 0;
        ControlIssue issue = ControlIssue::None;
    };

    // Controllers used by one execution owner may share its output identity.
    // That owner must select exactly one controller to write each cycle.
    PositionMotor(motor::Motor &motor, const Config &config, const void *execution_owner = nullptr);
    PositionMotor(const PositionMotor &) = delete;
    PositionMotor &operator=(const PositionMotor &) = delete;

    [[nodiscard]] int configure();
    motor::Motor &motor() const {
        return motor_;
    }
    [[nodiscard]] int update(double target_position_rad, float dt_s);
    // Same cascade with the joint reference that produced the target. The
    // feedforward gains live in `Config::loop`; zero keeps the legacy behavior
    // bit-identical. `reference_*` are consumed only when this overload is used.
    [[nodiscard]] int update(double target_position_rad, float reference_velocity_rad_s,
                             float reference_acceleration_rad_s2, float dt_s);
    [[nodiscard]] int updateExternalVelocity(const ExternalVelocityInput &input, float dt_s);
    [[nodiscard]] int reset();
    Telemetry telemetry() const;
    PositionReference reference() const {
        return config_.reference;
    }

private:
    int updateImpl(double target_position_rad, float reference_velocity_rad_s, float reference_acceleration_rad_s2,
                   bool use_reference, float dt_s);
    int resetFrom(const motor::MotorSnapshot &snapshot, double position);
    int resolveMeasurement(const motor::MotorSnapshot &snapshot, double &position);
    int fail(int error, const motor::MotorSnapshot &snapshot, ControlIssue issue = ControlIssue::OutputRejected);
    int wait(const motor::MotorSnapshot &snapshot, ControlIssue issue);
    int stageEffort(float effort, std::uint64_t generation);
    void publish(const Telemetry &next);

    enum class ActiveLoop : std::uint8_t { None, EncoderPosition, ExternalVelocity };
    motor::Motor &motor_;
    const void *producer_;
    Config config_{};
    double latest_target_rad_ = 0.0;
    float latest_dt_s_ = 0.0f;
    control_motor_position_state state_{};
    control_motor_velocity_state external_state_{};
    mutable struct k_spinlock telemetry_lock_{};
    Telemetry telemetry_{};
    double initial_position_rad_ = 0.0;
    double coordinate_origin_rad_ = 0.0;
    std::uint64_t observed_enable_generation_ = 0;
    std::uint64_t observed_reference_generation_ = 0;
    bool configured_ = false;
    bool history_valid_ = false;
    bool external_history_valid_ = false;
    ActiveLoop active_loop_ = ActiveLoop::None;
    std::uint64_t target_sequence_ = 0;
    bool startup_origin_valid_ = false;
    bool absolute_local_valid_ = false;
    double absolute_local_position_rad_ = 0.0;
    float previous_absolute_rad_ = 0.0f;
    std::uint64_t previous_absolute_stamp_ms_ = 0;
};

} // namespace skywalker::control
