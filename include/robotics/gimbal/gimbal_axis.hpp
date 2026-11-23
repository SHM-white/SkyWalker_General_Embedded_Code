#pragma once

#include <control/position_motor.hpp>
#include <core/measurement.hpp>
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
    // 请求把本轴实际积分出来的关节参考速度交给位置环做速度前馈。只有
    // Active + Rate、已完成播种且目标未被软限位/超前角投影的周期才可能生效。
    // 该字段只是“本周期请求”，不是前馈已生效的证据，见 TargetStatus。
    bool use_rate_reference = false;
    // 开始新的 Rate 会话：丢弃累计目标和控制历史，用本周期新鲜反馈播种，
    // 本周期不积分也不前馈。由拥有者按参考来源/会话变化显式置位，
    // 不能用逐帧递增的序号代替。
    bool restart_rate_session = false;
};

// Rate 目标被重建（而不是沿旧目标继续积分）的原因。
enum class AxisRestartReason : std::uint8_t { None, CommandSession, RequestEdge, UnconsumedCycle, ExplicitReseed };

// Validated joint trajectory for the small Yaw, in encoder coordinates.
// The allocator produces position/rate/acceleration from one feasible plan; the
// existing position loop enforces the position, while the rate and acceleration
// are carried for diagnostics and for the stage-C velocity/effort feedforward.
struct GimbalJointTrajectory {
    bool valid = false;
    float position_rad = 0, rate_rad_s = 0, acceleration_rad_s2 = 0;
    core::Stamp stamp{};
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
        // Rate 参考桥接诊断：requested 是本周期请求，accepted 表示位置环真的
        // 收到了与本次积分目标同源、有限的参考速度。accepted=false 时
        // reference_velocity_rad_s 为 0，而不是残留值。
        bool reference_requested = false;
        bool reference_accepted = false;
        float reference_velocity_rad_s = 0;
        AxisRestartReason restart_reason = AxisRestartReason::None;
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
    // Absolute joint target from a validated trajectory. Soft limits and the
    // optional lead guard are applied to the target; no rate is integrated, so a
    // rejected cycle can never leave a deferred position target behind.
    int updateTrajectory(const GimbalJointTrajectory &, SafetyAction, float dt_s);
    int updateExternalVelocity(const control::PositionMotor::ExternalVelocityInput &input, bool limit_clamped,
                               SafetyAction action, float dt_s);
    // Drop the accumulated Rate/trajectory lead and reseed the joint target from
    // the current feedback. Used when the travel envelope rejects an outward
    // command so the position loop cannot keep pulling toward the soft end.
    int reseedTargetFromFeedback();
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
    // Bridge session state: a request edge marks a new reference owner, and a
    // motion cycle whose output never reached the drive must reseed instead of
    // replaying its target debt on the next executed cycle.
    bool reference_request_previous_ = false;
    bool rate_restart_pending_ = false;
    bool explicit_reseed_ = false;
};

} // namespace skywalker::robotics
