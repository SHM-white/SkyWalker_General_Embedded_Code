#pragma once
#include <array>
#include <control/position_motor.hpp>
#include <control/velocity_motor.hpp>
#include <drivers/motor/group.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/command/command_source.hpp>

namespace skywalker::robotics {
struct ShooterHeatState {
    float heat = 0, limit = 0, cooling_per_s = 0;
    MessageStamp stamp{};
};
class IShooterHeatSource {
public:
    virtual ~IShooterHeatSource() = default;
    // Original measurement stamp, -EAGAIN means cached data retains its age.
    virtual int sample(ShooterHeatState &) = 0;
};
struct ShooterExecutionInputs {
    ShooterCommand command{};
    core::Stamp source_stamp{};
    OutputPermission permission{};
    ShooterHeatState heat{};
    core::Measurement<double> dial_home_reference{};
    RunStatus gimbal{};
    bool transport_ready = false, allow_feed = false;
    bool require_permission = true, require_heat = true, require_gimbal = true;
    bool emergency_stop = false, clear_estop = false;
};
struct ShooterStatus {
    RunStatus friction{}, feed{};
    bool friction_ready = false, dial_busy = false, jammed = false;
    std::uint32_t last_event_id = 0, shots = 0;
    double dial_target_rad = 0;
    float reserved_heat = 0;
    MessageStamp stamp{};
};

// Single execution-thread owner; topology and physical CAN publication belong
// to the application. Friction pair and dial have separate batch groups.
class ShooterExecutor {
public:
    struct Config {
        std::uint32_t command_timeout_ms = 100;
        core::TimeUs source_timeout_us = 100000;
        std::uint32_t permission_timeout_ms = 300, heat_timeout_ms = 300, gimbal_timeout_ms = 100;
        std::uint32_t friction_dwell_ms = 200, dial_settle_ms = 30, jam_timeout_ms = 1500;
        core::TimeUs max_cycle_us = 20000;
        float friction_speed_rad_s = 40, friction_tolerance_rad_s = 2;
        std::array<float, 2> friction_direction{1, -1};
        float dial_direction = 1, dial_step_rad = 0.7853982f;
        float dial_tolerance_rad = 0.03f, dial_settle_velocity_rad_s = 0.2f;
        float heat_per_round = 10, max_fire_rate_hz = 5;
        bool allow_relative_dial_reseed = false; // Empty indexing bench only.
    };
    ShooterExecutor(motor::Motor &left, motor::Motor &right, motor::Motor &dial,
                    motor::Group &friction_group, motor::Group &dial_group,
                    const control::VelocityMotor::Config &friction_control,
                    const control::PositionMotor::Config &dial_control, const Config &);
    int begin(); // After application attach/start; no enable or commit.
    ShooterStatus update(const ShooterExecutionInputs &, core::TimeUs now_us);
    ShooterStatus suspend(core::TimeUs now_us, WaitReason, int error = 0);
    ShooterStatus status() const { return status_; } // Execution owner only.
private:
    void stopFriction(WaitReason, int);
    void stopFeed(WaitReason, int);
    void discardEvent(const ShooterCommand &);
    void publish(core::TimeUs);
    motor::Motor &left_, &right_, &dial_;
    motor::Group &friction_group_, &dial_group_;
    control::VelocityMotor left_control_, right_control_;
    control::PositionMotor dial_control_;
    Config config_;
    ShooterStatus status_{};
    core::TimeUs previous_us_ = 0;
    std::uint64_t friction_good_since_ms_ = 0, step_started_ms_ = 0, dial_good_since_ms_ = 0;
    std::uint64_t next_shot_ms_ = 0, last_shot_ms_ = 0, dial_reference_ = 0;
    std::uint32_t heat_sequence_ = 0, production_sequence_ = 0;
    int configuration_error_ = 0;
    bool configured_ = false, begin_attempted_ = false, have_time_ = false;
    bool have_dial_reference_ = false;
};
} // namespace skywalker::robotics
