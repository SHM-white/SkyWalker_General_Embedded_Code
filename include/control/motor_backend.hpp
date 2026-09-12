#pragma once

#include <cstdint>
#include <drivers/motor/motor.hpp>

namespace skywalker::control {

enum class EffortUnit { Unspecified, Ampere, NewtonMeter };

struct MotorInfo {
    EffortUnit effort_unit = EffortUnit::Unspecified;
    float effort_limit = 0.0f;
    std::uint32_t capabilities = 0;
    std::uint32_t feedback_timeout_ms = 0;
};

struct MotorMeasurement {
    motor::Feedback feedback{};
    // Continuous driver coordinates. DM unwrapping assumes +/-PMAX wrap.
    double position_rad = 0.0;
    float driver_temperature_c = 0.0f;
    bool driver_temperature_valid = false;
};

// Single-motor, exclusive Bus context. Keep backends alive for the application
// lifetime: the drivers retain CAN callbacks/ownership after stop(). All calls
// belong to one thread, never an ISR. No allocation or automatic recovery.
class MotorBackend {
public:
    MotorBackend() = default;
    MotorBackend(const MotorBackend &) = delete;
    MotorBackend &operator=(const MotorBackend &) = delete;
    virtual ~MotorBackend() = default;

    virtual int describe(MotorInfo &info) = 0;
    virtual int prepare() = 0;
    virtual int read(MotorMeasurement &measurement) = 0;
    virtual int arm() = 0;
    virtual int write(float effort) = 0;
    virtual int flush() = 0;
    virtual int stop() = 0;

private:
    friend class MotorRuntime;
    bool claimed_ = false;
};

struct MotorSafety {
    // Mandatory positive cutoff, rad/s. Independent of the requested limit.
    float velocity_abs_max_rad_s = 0.0f;
    // 0 disables temperature protection; >0 requires temperature capability.
    float temperature_max_c = 0.0f;
};

enum class MotorRunState { Idle, Starting, Running, Stopped, Fault };

struct MotorStatus {
    MotorRunState state = MotorRunState::Idle;
    int error = 0;
    int stop_error = 0;
};

// Shared execution mechanics; applications use VelocityMotor/PositionMotor.
class MotorRuntime {
public:
    MotorRuntime(MotorBackend &backend, EffortUnit unit, MotorSafety safety);
    MotorRuntime(const MotorRuntime &) = delete;
    MotorRuntime &operator=(const MotorRuntime &) = delete;
    int prepare(float effort_limit, std::uint32_t required, MotorMeasurement &first);
    int arm();
    int cycle(float dt_min_s, float dt_max_s, MotorMeasurement &measurement, float &dt_s);
    int send(float effort);
    int fail(int error);
    int stop();
    std::int64_t elapsedMs() const;
    const MotorStatus &status() const { return status_; }

private:
    int read(MotorMeasurement &measurement);
    MotorBackend &backend_;
    EffortUnit unit_;
    MotorSafety safety_;
    MotorInfo info_{};
    MotorStatus status_{};
    std::uint32_t required_ = 0;
    bool owns_backend_ = false;
    bool prepare_attempted_ = false;
    std::int64_t started_ms_ = 0;
    std::int64_t previous_ms_ = 0;
};

} // namespace skywalker::control
