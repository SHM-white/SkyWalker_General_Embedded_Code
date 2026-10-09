#pragma once

#include <control/position_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <robotics/messages/command.hpp>

namespace skywalker::robotics {

enum class AxisTopology : std::uint8_t { Continuous, Limited };
// CalibratedFeedback requires absolute/native feedback in the configured limit coordinates.
enum class AxisReferenceInit : std::uint8_t { Preserve, CalibratedFeedback };

struct GimbalAxisConfig {
    AxisTopology topology = AxisTopology::Continuous;
    float min_angle_rad = -3.14159265f, max_angle_rad = 3.14159265f, max_rate_rad_s = 3;
    bool hold_on_zero_rate = true;
    AxisReferenceInit reference_init = AxisReferenceInit::Preserve;
    // Limited Rate targets only, in calibrated joint coordinates. Zero disables
    // the guard. Tune against measured tracking error and acceptable stopping tail;
    // input beyond this lead is discarded rather than replayed after saturation.
    float max_lead_rad = 0;
};

struct AxisCommand {
    GimbalMode mode = GimbalMode::Disabled;
    float target_rad = 0;
    float rate_rad_s = 0;
};

// Call control methods from one execution thread. Motor must outlive this object.
// Enable/disable, command freshness and CAN publication remain the caller's responsibility.
class GimbalAxis {
public:
    struct Status {
        bool feedback_healthy = false;
        int error = 0; // Current feedback/reference preparation error, not a latched drive fault.
    };
    struct TargetStatus {
        double target_rad = 0;
        double measured_rad = 0;
        float requested_rate_rad_s = 0;
        float limited_rate_rad_s = 0;
        float last_dt_s = 0;
        float integration_dt_s = 0;
        bool target_valid = false;
        bool feedback_healthy = false;
        bool rate_limited = false;
        bool mechanical_limited = false;
        bool lead_limited = false;
    };

    GimbalAxis(motor::Motor &drive, const control::PositionMotor::Config &position_config,
               const GimbalAxisConfig &config)
        : drive_(drive), position_(drive, position_config), config_(config) {
    }
    GimbalAxis(const GimbalAxis &) = delete;
    GimbalAxis &operator=(const GimbalAxis &) = delete;
    GimbalAxis(GimbalAxis &&) = delete;
    GimbalAxis &operator=(GimbalAxis &&) = delete;

    int validate() const;
    int begin();                       // Configure after CanBus::start(); never enables the drive.
    Status poll(std::uint64_t now_ms); // May reseed a disabled Limited axis when explicitly configured.
    int reset(); // Reset control history and target from fresh feedback; never reseeds driver coordinates.
    // Explicit withdrawal: discard targets/control history and staged effort.
    // The next Rate/Hold session seeds from fresh feedback, with no deferred motion.
    void withdraw();
    int update(const AxisCommand &, SafetyAction, float dt_s);
    int updateRate(float rate_rad_s, float dt_s); // Convenience for an already authorized Active path.
    double targetAngleRad() const {
        return target_angle_rad_;
    }
    control::PositionMotor::Telemetry telemetry() const {
        return position_.telemetry();
    }
    const TargetStatus &targetStatus() const {
        return target_status_;
    } // Execution-thread only; targets and measurements share the joint reference.

private:
    int feedbackError(const motor::MotorSnapshot &) const;
    int prepareReference(const motor::MotorSnapshot &);
    int seed(const motor::MotorSnapshot &);
    motor::Motor &drive_;
    control::PositionMotor position_;
    GimbalAxisConfig config_;
    Status status_{};
    TargetStatus target_status_{};
    double target_angle_rad_ = 0;
    std::uint64_t observed_enable_generation_ = 0, observed_reference_generation_ = 0;
    SafetyAction previous_action_ = SafetyAction::Disable;
    GimbalMode previous_mode_ = GimbalMode::Disabled;
    bool configured_ = false;
    bool reference_seeded_ = false;
    bool initialized_ = false;
};

} // namespace skywalker::robotics
