#pragma once

#include <cstdint>
#include <variant>
#include <zephyr/spinlock.h>

#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dm_motor.hpp>
#include <drivers/motor/motor_types.hpp>

namespace skywalker::control {
class PositionMotor;
class VelocityMotor;
}

namespace skywalker::motor {

enum Capability : std::uint32_t {
    CommandCurrent = 1u << 0,
    CommandTorque = 1u << 1,
    CommandMit = 1u << 2,
    CommandVelocity = 1u << 3,
    CommandPositionVelocity = 1u << 4,
    FeedbackPosition = 1u << 8,
    FeedbackVelocity = 1u << 9,
    FeedbackCurrent = 1u << 10,
    FeedbackTorque = 1u << 11,
    FeedbackTemperature = 1u << 12,
    FeedbackAbsolutePosition = 1u << 13,
};

struct Feedback {
    /* Continuous output-shaft position; the first received frame is 0 rad. */
    float position_rad = 0.0f;
    /*
     * Fixed-zero, single-turn output-shaft position in [-pi, pi). Valid only
     * when FeedbackAbsolutePosition is set in both capabilities and valid.
     */
    float absolute_position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float current_a = 0.0f;
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;
    std::uint32_t valid = 0;
    std::uint64_t timestamp_ms = 0;
};

class CanBus;
class Group;
class Motor;

enum class MotorState : std::uint8_t { Offline, Disabled, Enabling, Active, Fault };
enum class StopProgress : std::uint8_t { None, Pending, TxComplete, DriveConfirmed, Unreachable };
enum class FaultReason : std::uint8_t {
    None,
    FeedbackExpired,
    CommandExpired,
    DriveFault,
    UnexpectedDisabled,
    InvalidCommand,
    ControlRejected,
    EnableTimeout,
    TransportError,
    RxOverflow,
};

struct FaultInfo {
    FaultReason reason = FaultReason::None;
    int error = 0;
    const Motor *source_motor = nullptr;
    std::uint64_t occurred_ms = 0;
};

struct MotorInfo {
    std::uint32_t capabilities = 0;
    float current_limit_a = 0.0f;
    float torque_limit_nm = 0.0f;
    Timing timing{};
};

struct StopReport {
    StopProgress progress = StopProgress::None;
    std::uint64_t request_generation = 0;
    int tx_error = 0;
};

struct MotorSnapshot {
    Feedback feedback{};
    dji::RawFeedback native_dji_feedback{};
    bool native_dji_feedback_valid = false;
    float native_mos_temperature_c = 0.0f;
    float native_rotor_temperature_c = 0.0f;
    bool native_temperatures_valid = false;
    float native_position_rad = 0.0f;
    bool native_position_valid = false;
    MotorState state = MotorState::Offline;
    bool feedback_fresh = false;
    bool output_permitted = false;
    bool position_reference_valid = false;
    std::uint64_t enable_generation = 0;
    std::uint64_t reference_generation = 0;
    FaultInfo last_fault{};
    StopReport stop{};
    std::uint32_t native_drive_status = 0;
    bool native_drive_status_valid = false;
};

enum class CommandKind : std::uint8_t { None, Current, Torque, Mit, Velocity, PositionVelocity };

struct Command {
    CommandKind kind = CommandKind::None;
    float primary = 0.0f;
    float secondary = 0.0f;
    dm::MitCommand mit{};
};

struct StagedCommand {
    Command command{};
    bool valid = false;
    std::uint64_t written_ms = 0;
    std::uint64_t revision = 0;
    std::uint64_t enable_generation = 0;
};

class Motor {
public:
    explicit Motor(dji::Config config);
    explicit Motor(dm::Config config);
    Motor(const Motor &) = delete;
    Motor &operator=(const Motor &) = delete;
    Motor(Motor &&) = delete;
    Motor &operator=(Motor &&) = delete;

    MotorInfo info() const;
    MotorSnapshot snapshot() const;
    bool ready() const;
    bool active() const;

    [[nodiscard]] int enable();
    [[nodiscard]] int disable();
    [[nodiscard]] int clearFault();
    [[nodiscard]] int setCurrent(float ampere);
    [[nodiscard]] int setTorque(float newton_meter);
    [[nodiscard]] int setMit(const dm::MitCommand &command);
    [[nodiscard]] int setVelocity(float rad_s);
    [[nodiscard]] int setPositionVelocity(float rad, float max_rad_s);
    [[nodiscard]] int reseedPosition(double known_position_rad);

private:
    friend class CanBus;
    friend class Group;
    friend class skywalker::control::PositionMotor;
    friend class skywalker::control::VelocityMotor;

    int validateConfig() const;
    int stage(const Command &command, const void *producer = nullptr);
    int bindProducer(const void *producer, float velocity_abs_max_rad_s, float temperature_max_c,
                     std::uint32_t required_feedback, bool require_position_reference);
    int checkProducerSafetyLocked() const;
    bool readyLocked(std::uint64_t now_ms) const;
    int setCurrentFrom(const void *producer, float ampere);
    int setTorqueFrom(const void *producer, float newton_meter);
    void rejectControl(int error);
    StagedCommand copyStaged() const;
    int requestEnable(std::uint64_t generation);
    void requestDisable();
    int requestClearFault();
    void grantGroupActive(std::uint64_t generation);
    void markStarted();
    bool busStarted() const;
    void markSafePrepared(std::uint64_t expected_stop_generation = 0);
    void markEnableTxComplete(std::uint64_t generation, std::uint64_t completed_ms, std::uint64_t completed_order);
    void markClearTxComplete(std::uint64_t completed_ms, std::uint64_t completed_order);
    void markPrepared(std::uint64_t generation);
    void markStopped(StopProgress progress, int tx_error, std::uint64_t request_generation,
                     std::uint64_t completed_ms = 0, std::uint64_t completed_order = 0);
    void markFaultCleared();
    int acceptDjiFeedback(const dji::RawFeedback &raw, std::uint64_t received_ms);
    int acceptDmFeedback(const dm::DecodedFeedback &decoded, std::uint64_t received_ms, std::uint64_t callback_order);
    void raiseFault(const FaultInfo &fault);
    void wakeBus();
    bool feedbackFresh(std::uint64_t now_ms) const;

    struct DjiRuntime {
        std::uint16_t last_encoder = 0;
        std::int64_t total_encoder_ticks = 0;
        bool has_encoder = false;
    };
    struct DmRuntime {
        double position_offset_rad = 0.0;
        double accumulated_native_rad = 0.0;
        float last_native_position_rad = 0.0f;
        bool has_native_position = false;
    };

    std::variant<dji::Config, dm::Config> config_;
    std::variant<DjiRuntime, DmRuntime> protocol_state_;
    CanBus *bus_ = nullptr;
    Group *group_ = nullptr;
    bool group_conflict_ = false;
    mutable struct k_spinlock lock_{};
    MotorSnapshot snapshot_{};
    StagedCommand staged_{};
    const void *producer_ = nullptr;
    struct ProducerSafety {
        float velocity_abs_max_rad_s = 0.0f;
        float temperature_max_c = 0.0f;
        std::uint32_t required_feedback = 0;
        bool require_position_reference = false;
    } producer_safety_{};
    bool started_ = false;
    bool safe_prepared_ = false;
    bool safe_pending_ = false;
    bool clear_pending_ = false;
    bool enable_pending_ = false;
    bool enable_tx_done_ = false;
    bool safe_tx_done_ = false;
    bool clear_tx_done_ = false;
    std::uint64_t enable_requested_at_ms_ = 0;
    std::uint64_t activated_ms_ = 0;
    std::uint64_t enable_tx_completed_ms_ = 0;
    std::uint64_t enable_tx_completed_order_ = 0;
    std::uint64_t safe_tx_completed_ms_ = 0;
    std::uint64_t safe_tx_completed_order_ = 0;
    std::uint64_t safe_first_tx_completed_ms_ = 0;
    std::uint64_t clear_tx_completed_ms_ = 0;
    std::uint64_t clear_tx_completed_order_ = 0;
    std::uint64_t feedback_event_order_ = 0;
    std::uint64_t feedback_stable_since_ms_ = 0;
};

} // namespace skywalker::motor
