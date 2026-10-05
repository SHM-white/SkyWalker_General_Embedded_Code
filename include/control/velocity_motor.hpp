#pragma once

#include <cstdint>

#include <control/motor_common.hpp>
#include <control/motor_velocity.h>
#include <drivers/motor/motor.hpp>
#include <zephyr/spinlock.h>

namespace skywalker::control {

class VelocityMotor {
public:
    struct Config {
        control_motor_velocity_config loop{};
        EffortUnit effort_unit = EffortUnit::Unspecified;
    };

    struct Telemetry {
        motor::MotorSnapshot motor{};
        control_motor_velocity_output output{};
        float target_rad_s = 0.0f;
        float dt_s = 0.0f;
        float effort_command = 0.0f;
        EffortUnit effort_unit = EffortUnit::Unspecified;
        bool target_valid = false;
        std::uint64_t target_sequence = 0;
        bool output_valid = false;
        int error = 0;
        ControlIssue issue = ControlIssue::None;
    };

    VelocityMotor(motor::Motor &motor, const Config &config);
    VelocityMotor(const VelocityMotor &) = delete;
    VelocityMotor &operator=(const VelocityMotor &) = delete;

    [[nodiscard]] int configure();
    motor::Motor &motor() const { return motor_; }
    [[nodiscard]] int update(float target_rad_s, float dt_s);
    [[nodiscard]] int reset();
    Telemetry telemetry() const;

private:
    int resetFrom(const motor::MotorSnapshot &snapshot);
    int fail(int error, const motor::MotorSnapshot &snapshot, ControlIssue issue = ControlIssue::OutputRejected);
    int wait(const motor::MotorSnapshot &snapshot, ControlIssue issue);
    int stageEffort(float effort, std::uint64_t generation);
    void publish(const Telemetry &next);

    motor::Motor &motor_;
    Config config_{};
    float latest_target_rad_s_ = 0.0f;
    float latest_dt_s_ = 0.0f;
    control_motor_velocity_state state_{};
    mutable struct k_spinlock telemetry_lock_{};
    Telemetry telemetry_{};
    std::uint64_t observed_enable_generation_ = 0;
    bool configured_ = false;
    bool history_valid_ = false;
    std::uint64_t target_sequence_ = 0;
};

} // namespace skywalker::control
