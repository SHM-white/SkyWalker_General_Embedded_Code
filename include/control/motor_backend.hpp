#pragma once

#include <cstdint>
#include <control/motor_lifecycle.hpp>
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
// belong to one thread, never an ISR. No allocation. configure/poll/resume support bounded recovery.
class MotorBackend {
public:
    MotorBackend() = default;
    MotorBackend(const MotorBackend &) = delete;
    MotorBackend &operator=(const MotorBackend &) = delete;
    virtual ~MotorBackend() = default;

    virtual int describe(MotorInfo &info) = 0;
    virtual int prepare() = 0; // Legacy entry; new backends use the split operations below.
    virtual int configure() { return 0; }
    virtual int pollPrepare(std::uint64_t) { return prepare(); }
    virtual int resetMeasurementReference() { return 0; }
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
    std::uint32_t recovery_stable_ms = 30;
    std::uint32_t recovery_retry_ms = 100;
    std::uint32_t recovery_poll_ms = 5; // Awaiting an in-progress handshake, not retrying a failed TX.
};

enum class MotorRunState { Idle, Starting, Running, Stopped, Fault, Waiting, Recovering, Ready, EStopLatched, ConfigBlocked };

struct MotorStatus {
    MotorRunState state = MotorRunState::Idle;
    int error = 0;
    int stop_error = 0;
    PauseReason reason = PauseReason::OperatorDisabled;
    std::uint32_t resume_generation = 0, recovery_attempts = 0;
    int last_recovery_error = 0;
};

// Shared execution mechanics; applications use VelocityMotor/PositionMotor.
class MotorRuntime {
public:
    MotorRuntime(MotorBackend &backend, EffortUnit unit, MotorSafety safety);
    MotorRuntime(const MotorRuntime &) = delete;
    MotorRuntime &operator=(const MotorRuntime &) = delete;
    int prepare(float effort_limit, std::uint32_t required, MotorMeasurement &first);
    int configure(float effort_limit, std::uint32_t required);
    int poll(std::uint64_t now_ms, MotorMeasurement &measurement);
    int suspend(PauseReason reason);
    int clearEmergencyStop(bool released);
    int block(int error);
    ExecutionState state() const;
    int arm();
    int cycle(float dt_min_s, float dt_max_s, MotorMeasurement &measurement, float &dt_s);
    int send(float effort);
    int fail(int error, PauseReason reason = PauseReason::TransportTemporary);
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
    std::uint64_t next_retry_ms_ = 0, stable_since_ms_ = 0, last_feedback_ms_ = 0;
    bool reference_reset_ = false, stable_started_ = false;
    std::int64_t started_ms_ = 0;
    std::int64_t previous_ms_ = 0;
};

} // namespace skywalker::control
