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
enum class DialControlMode : std::uint8_t {
    Disabled,
    SinglePosition,
    ContinuousVelocity,
    BrakingVelocity,
    HoldPosition,
};
struct ShooterStatus {
    RunStatus friction{}, feed{};
    // 单发 busy 直到机械位置、速度和稳定时长全部满足；连发/减速时同样为 true。
    bool friction_ready = false, dial_busy = false, jammed = false;
    DialControlMode dial_mode = DialControlMode::Disabled;
    std::uint32_t last_event_id = 0;
    std::uint32_t last_accepted_event_id = 0, last_rejected_event_id = 0, active_single_event_id = 0;
    std::uint32_t accepted_single_events = 0, rejected_single_events = 0, completed_single_events = 0;
    std::uint32_t shots = 0; // 供弹/热量预留次数，包含减速预留，绝不表示实际发射弹数。
    double dial_target_rad = 0;
    double dial_position_rad = 0, dial_position_error_rad = 0;
    float dial_requested_velocity_rad_s = 0, dial_actual_velocity_rad_s = 0; // 输出轴，rad/s。
    bool dial_feedback_valid = false;
    float reserved_heat = 0;
    MessageStamp stamp{};
};

// Single execution-thread owner; topology and physical CAN publication belong
// to the application. Friction pair and dial have separate batch groups.
class ShooterExecutor {
public:
    struct Config {
        // 调参：命令有效期 ms、原始输入有效期 us；不得靠重复读取缓存延长其年龄。
        std::uint32_t command_timeout_ms = 100;
        // 调参：原始单击事件有效期，ms；延长会增加迟到点击的可接受时间，不由命令重发续期。
        std::uint32_t event_timeout_ms = 100;
        core::TimeUs source_timeout_us = 100000;
        // 调参：裁判输出许可、热量、云台执行状态有效期，ms；失效立即撤销供弹。
        std::uint32_t permission_timeout_ms = 300, heat_timeout_ms = 300, gimbal_timeout_ms = 100;
        // 调参：摩擦速度进入容差后连续保持时间，ms；过短可能在加速中放行供弹。
        std::uint32_t friction_dwell_ms = 200;
        // 调参：到位需连续稳定的时长/单发及连发无进展超时，ms；按机械响应设置，保留堵转保护。
        std::uint32_t dial_settle_ms = 30, jam_timeout_ms = 1500;
        // 调参：正常停连发后的最大受控减速时间，ms；超时走停机，不能无限保持速度环。
        std::uint32_t dial_brake_timeout_ms = 1500;
        // 调参：最大允许控制间隔，us；不得超过 20000，超时停机且不追补错过的运动。
        core::TimeUs max_cycle_us = 20000;
        // 调参：摩擦目标/就绪容差，输出轴 rad/s；容差与 dwell 共同防止未就绪供弹。
        float friction_speed_rad_s = 40, friction_tolerance_rad_s = 2;
        // 调参：摩擦轮方向只能 ±1；按机械安装确认，不能用负 PID 修正方向。
        std::array<float, 2> friction_direction{1, -1};
        // 调参：拨盘方向只能 ±1，每发机械分度为输出轴 rad；更换拨盘/减速比后重新标定。
        float dial_direction = 1, dial_step_rad = 0.7853982f;
        // 调参：拨盘机构速度上限，输出轴 rad/s；连发取射频×分度与此上限的较小值。
        float dial_speed_rad_s = 1.5707964f;
        // 调参：单发到位位置容差 rad/停稳速度 rad/s；过大可能把尚未到位误判为完成。
        float dial_tolerance_rad = 0.03f, dial_settle_velocity_rad_s = 0.2f;
        // 调参：停下后识别“已在一个分度”的相位容差，rad；须小于半个分度，实机标定。
        float dial_phase_tolerance_rad = 0.03f;
        // 调参：单轮热量/射频硬上限，裁判热量单位、Hz；不能通过放大上限绕过裁判许可。
        float heat_per_round = 10, max_fire_rate_hz = 50;
        // 仅无弹台架首次启动允许相对零点；参考丢失后必须提供真正的归零参考。
        bool allow_relative_dial_reseed = false;
    };
    ShooterExecutor(motor::Motor &left, motor::Motor &right, motor::Motor &dial, motor::Group &friction_group,
                    motor::Group &dial_group, const control::VelocityMotor::Config &friction_control,
                    const control::PositionMotor::Config &dial_control, const Config &);
    int begin(); // After application attach/start; no enable or commit.
    ShooterStatus update(const ShooterExecutionInputs &, core::TimeUs now_us);
    ShooterStatus suspend(core::TimeUs now_us, WaitReason, int error = 0);
    ShooterStatus status() const {
        return status_;
    } // Execution owner only.
    // Read-only controller diagnostics; does not enable or stage any output.
    control::PositionMotor::Telemetry dialTelemetry() const {
        return dial_control_.telemetry();
    }
    control::VelocityMotor::Telemetry dialVelocityTelemetry() const {
        return dial_velocity_control_.telemetry();
    }
private:
    void stopFriction(WaitReason, int);
    void stopFeed(WaitReason, int);
    bool newEvent(const ShooterCommand &) const;
    void consumeEvent(const ShooterCommand &, bool accepted = false);
    void setDialMode(DialControlMode);
    double nextIndex(double measured_position) const;
    void publish(core::TimeUs);
    motor::Motor &left_, &right_, &dial_;
    motor::Group &friction_group_, &dial_group_;
    control::VelocityMotor left_control_, right_control_;
    control::PositionMotor dial_control_;
    control::VelocityMotor dial_velocity_control_;
    Config config_;
    ShooterStatus status_{};
    core::TimeUs previous_us_ = 0;
    std::uint64_t friction_good_since_ms_ = 0;
    core::TimeUs single_started_us_ = 0, dial_settled_since_us_ = 0, brake_started_us_ = 0;
    core::TimeUs progress_since_us_ = 0, heat_updated_us_ = 0;
    std::uint64_t dial_reference_ = 0;
    double dial_phase_origin_rad_ = 0, continuous_budget_end_rad_ = 0, progress_position_rad_ = 0;
    float dial_velocity_limit_rad_s_ = 0, dial_deceleration_rad_s2_ = 0, heat_model_ = 0;
    std::uint32_t heat_sequence_ = 0, production_sequence_ = 0;
    int configuration_error_ = 0;
    bool configured_ = false, begin_attempted_ = false, have_time_ = false;
    bool have_dial_reference_ = false, have_heat_model_ = false;
    WaitReason braking_reason_ = WaitReason::Command;
};
} // namespace skywalker::robotics
