#pragma once

#include <array>
#include <cfloat>
#include <cstdint>
#include <core/measurement.hpp>

namespace skywalker::robotics::vehicle {
// One calibration source for the staged samples and the two vehicle roles.
// TODO(hardware): replace template values using the installed robot's records.
// Keep confirmation flags false until the corresponding bench has passed.
inline constexpr bool connections_confirmed = true;
inline constexpr bool imu_mounting_confirmed = true;
inline constexpr bool power_model_calibrated = false;
inline constexpr bool shooter_constraints_confirmed = true;

struct MotorCalibration {
    std::uint8_t id = 0;
    std::uint16_t master_id = 0;
    // 调参：gear_ratio 为电机轴/输出轴转速比；direction 只取 ±1，按实际安装逐轴确认方向。
    float gear_ratio = 1, direction = 1;
    // 调参：effort_limit 为 DJI 电流 A 或 DM 力矩 N·m；velocity_limit 为输出轴 rad/s。
    // 提高速率前先确认输出饱和与机械能力，不能以增加电流补偿输入单位错误。
    float effort_limit = 0.5f, velocity_limit_rad_s = 10;
    // 调参：编码器零点，ticks；修改后需同步机械端点，不能用零点替代独立机械中心。
    std::uint16_t encoder_zero_ticks = 0;
};

constexpr bool validDirection(float direction) {
    return direction == 1.0f || direction == -1.0f;
}
constexpr bool finiteConstant(float value) {
    return value == value && value >= -FLT_MAX && value <= FLT_MAX;
}
constexpr bool positive(float value) {
    return finiteConstant(value) && value > 0.0f;
}
template <std::size_t N> constexpr bool validMotorSet(const std::array<MotorCalibration, N> &motors) {
    for (std::size_t i = 0; i < N; ++i) {
        if (!motors[i].id || !validDirection(motors[i].direction) || !positive(motors[i].gear_ratio) ||
            !positive(motors[i].effort_limit) || !positive(motors[i].velocity_limit_rad_s) ||
            motors[i].encoder_zero_ticks >= 8192)
            return false;
        for (std::size_t j = 0; j < i; ++j)
            if (motors[i].id == motors[j].id || (motors[i].master_id && motors[i].master_id == motors[j].master_id))
                return false;
    }
    return true;
}

// TODO(calibration): ID/mode, current-mode firmware, output-shaft units and signs.
inline constexpr MotorCalibration small_yaw{7, 0, 1, 1, 3.0f, 10, 5670};
inline constexpr MotorCalibration pitch{1, 0x00, 1, 1, 10.0f, 10, 0};
inline constexpr MotorCalibration big_yaw{2, 0x12, 1, 1, 10.0f, 10, 0};
inline constexpr std::array<MotorCalibration, 2> friction{{
    {1, 0, 19, -1, 2, 60, 0},
    {2, 0, 19, 1, 2, 60, 0},
}};
inline constexpr MotorCalibration dial{4, 0, 36, 1, 10.0, 20, 0};
// Viewed from above with the robot's front at the top, motor IDs run
// counterclockwise: FL=1, RL=2, RR=3, FR=4 (both steer and drive buses).
// Controller arrays remain FL, FR, RL, RR, so their IDs are 1, 4, 2, 3.
// Candidate zeros below follow the original motor IDs, assuming IDs were not
// reassigned. After moving/reorienting modules, measure each forward-facing
// raw encoder again; installation order alone cannot determine a new zero.
// Encoder polarity is fixed for every motion mode.
inline constexpr float wheel_gear_ratio = 3591.0f / 187.0f;
// ID1=FL, ID2=RL, ID3=RR, ID4=FR; 0..8191 ticks, indexed by motor ID - 1.
// ID1 retains the old candidate from h7_framework-main-source.zip (2026-10-02).
// ID2 moved by 180deg: (2421 + 4096) % 8192 = 6517.
// IDs 3/4: clockwise 90deg from the old zeros, then another 180deg.
// Raw zero adjustments: 335+4096=4431, 1049+4096=5145.
inline constexpr std::array<std::uint16_t, 4> steer_zero_ticks_by_id{{3060, 6517, 4431, 5145}};
static_assert(steer_zero_ticks_by_id[0] < 8192 && steer_zero_ticks_by_id[1] < 8192 &&
                  steer_zero_ticks_by_id[2] < 8192 && steer_zero_ticks_by_id[3] < 8192,
              "steer encoder zero must be a 13-bit tick value");
// All four steering motors are mounted upside down: their native positive
// rotation opposes body-positive (CCW viewed from above). The hardware adapter
// applies this sign to angle/velocity feedback AND current output.
inline constexpr std::array<MotorCalibration, 4> legacy_steer{{
    {1, 0, 1, -1, 2.0f, 4, steer_zero_ticks_by_id[0]}, // FL
    {4, 0, 1, -1, 2.0f, 4, steer_zero_ticks_by_id[3]}, // FR
    {2, 0, 1, -1, 2.0f, 4, steer_zero_ticks_by_id[1]}, // RL
    {3, 0, 1, -1, 2.0f, 4, steer_zero_ticks_by_id[2]}, // RR
}};
// Preserve the original direction of each motor ID; reconfirm after remounting.
inline constexpr std::array<MotorCalibration, 4> legacy_wheel{{
    {1, 0, wheel_gear_ratio, -1, 20.0f, 40, 0}, // FL
    {4, 0, wheel_gear_ratio, 1, 20.0f, 40, 0},  // FR
    {2, 0, wheel_gear_ratio, 1, 20.0f, 40, 0},  // RL
    {3, 0, wheel_gear_ratio, -1, 20.0f, 40, 0}, // RR
}};
// Source: JJL0612/sentry_onlychassis 7c7e870 (2026-10-28), NOT a new measurement.
// Same FL/FR/RL/RR coordinates; hardware adapter owns both feedback/output signs.
enum class SwerveCalibrationSet { LegacyCandidate, Reference20261028Candidate };
inline constexpr SwerveCalibrationSet swerve_calibration_set = SwerveCalibrationSet::LegacyCandidate;
inline constexpr std::array<MotorCalibration, 4> reference_steer = [] {
    auto c = legacy_steer;
    constexpr std::uint16_t zeros[] = {3060, 986, 2383, 4390};
    for (std::size_t i = 0; i < 4; ++i) {
        c[i].encoder_zero_ticks = zeros[i];
        c[i].direction = -1;
    }
    return c;
}();
inline constexpr std::array<MotorCalibration, 4> reference_wheel = [] {
    auto c = legacy_wheel;
    for (auto &m : c)
        m.direction = -1;
    return c;
}();
inline constexpr auto steer = swerve_calibration_set == SwerveCalibrationSet::Reference20261028Candidate
                                  ? reference_steer
                                  : legacy_steer;
inline constexpr auto wheel = swerve_calibration_set == SwerveCalibrationSet::Reference20261028Candidate
                                  ? reference_wheel
                                  : legacy_wheel;
static_assert(validMotorSet(steer), "steer calibration must have unique IDs, +/-1 direction and valid limits");
static_assert(validMotorSet(wheel), "wheel calibration must have unique IDs, +/-1 direction and valid limits");
static_assert(validMotorSet(legacy_steer) && validMotorSet(reference_steer),
              "both steer calibration sets must be valid");
static_assert(validMotorSet(legacy_wheel) && validMotorSet(reference_wheel),
              "both wheel calibration sets must be valid");
static_assert(validMotorSet(friction), "friction calibration must have unique IDs, +/-1 direction and valid limits");
static_assert(validDirection(small_yaw.direction) && validDirection(pitch.direction) &&
                  validDirection(big_yaw.direction),
              "single-axis motor direction must be +/-1");
static_assert(small_yaw.encoder_zero_ticks < 8192 && pitch.encoder_zero_ticks < 8192 &&
                  big_yaw.encoder_zero_ticks < 8192,
              "DJI-compatible encoder zero must be a 13-bit tick value");
static_assert(positive(small_yaw.gear_ratio) && positive(pitch.gear_ratio) && positive(big_yaw.gear_ratio) &&
                  positive(small_yaw.effort_limit) && positive(pitch.effort_limit) && positive(big_yaw.effort_limit) &&
                  positive(small_yaw.velocity_limit_rad_s) && positive(pitch.velocity_limit_rad_s) &&
                  positive(big_yaw.velocity_limit_rad_s),
              "gear ratios, effort and velocity limits must be positive");
// J4310 bus IDs are also unverified; MIT backend remains unchanged.
// Joint speed includes body-rotation compensation above the world-follow target.
inline constexpr MotorCalibration reference_big_yaw{6, 0x11, 1, -1, 10.0f, 10.0f, 0};
static_assert(validDirection(reference_big_yaw.direction) && positive(reference_big_yaw.gear_ratio) &&
                  positive(reference_big_yaw.effort_limit) && positive(reference_big_yaw.velocity_limit_rad_s),
              "reference big-Yaw calibration must be valid");
// Calibrated encoder forward zero in the reference firmware, 2026-10-28.
// Confirm that the installed J4310 and CAN identity match before motor operation.
inline constexpr float reference_big_yaw_forward_zero_rad = -2.62f;
inline constexpr float steer_position_kp = 40, steer_velocity_kp = 0.20f;
// A/(motor-shaft rad/s); swerve_profile converts once to wheel-shaft units.
inline constexpr float drive_rotor_velocity_kp = 0.08f;
inline constexpr core::TimeUs chassis_period_us = 2000, bench_command_lease_us = 1000000;

// 调参：手动底盘/云台速度上限，单位 m/s 或 rad/s。
inline constexpr float manual_max_vx_m_s = 1.0f, manual_max_vy_m_s = 1.0f, manual_max_wz_rad_s = 6.0f;
inline constexpr float manual_max_gimbal_yaw_rate_rad_s = 3.0f, manual_max_gimbal_pitch_rate_rad_s = 3.0f;
// 调参：小陀螺边平移的参考倍率；0..1。
inline constexpr float chassis_spin_forward_scale = 1.0f, chassis_spin_lateral_scale = 1.0f;
// 调参：舵轮启停迟滞，单位 m/s；与轮轴角度/线速度无关。
inline constexpr float chassis_zero_speed_epsilon_m_s = 0.01f, chassis_resume_speed_m_s = 0.02f;
// 调参：底盘运动适配器的车体 wz 上限、舵向目标速率和驱动/舵向参考斜率。
inline constexpr float chassis_max_wz_rad_s = 10.0f, chassis_steer_speed_rad_s = 10.0f;
inline constexpr float chassis_drive_slew_rad_s2 = 30.0f, chassis_steer_slew_rad_s2 = 100.0f;
// 调参：大 Yaw 世界航向闭环（航向 P + 角速度 PI），单位 rad/s；仅作实车整定起点，新硬件上未验证。
// heading_kp 作用于航向误差，修正量上限 heading_rate_max_rad_s；
// rate_kp/rate_ki 作用于“目标世界角速度 - 实测角速度”，积分上限 integral_max_rad_s。
// release_epsilon_rad_s 是回中断的“零速”判断，只用于松手捕获航向，不是运行许可。
inline constexpr float big_yaw_heading_kp = 2.0f, big_yaw_heading_rate_max_rad_s = 6.0f;
inline constexpr float big_yaw_heading_rate_kp = 0.0f, big_yaw_heading_rate_ki = 0.0f;
inline constexpr float big_yaw_heading_integral_max_rad_s = 1.5f;
inline constexpr float big_yaw_heading_release_epsilon_rad_s = 1e-4f;
// 调参：大 Yaw 世界运动整形加速度/jerk（世界坐标 rad/s²、rad/s³）。
// 在原请求+航向修正叠加之后再整形，jerk=0 表示只限制加速度。
// 参考底盘阶段先从 4 rad/s² 起步；配套 Zephyr 底盘实测有余量后按 8→12→16 逐级试。
inline constexpr float big_yaw_world_acceleration_rad_s2 = 8.0f;
inline constexpr float big_yaw_world_jerk_rad_s3 = 0.0f;
// 调参：正常停车的航向捕获停留时间与进入/退出阈值（s、rad/s）。
// 停留 0 表示沿用旧的“请求穿越零速即捕获”；配套底盘先按 60 ms 试验。
inline constexpr float big_yaw_stop_capture_dwell_s = 0.06f;
inline constexpr float big_yaw_stop_capture_enter_rad_s = 0.05f, big_yaw_stop_capture_exit_rad_s = 0.10f;
// 调参：大 Yaw 原生速度模式的目标加速度，单位电机轴 rad/s²。
inline constexpr float big_yaw_native_acceleration_rad_s2 = 20.0f;
static_assert(positive(manual_max_vx_m_s) && positive(manual_max_vy_m_s) && positive(manual_max_wz_rad_s) &&
                  positive(manual_max_gimbal_yaw_rate_rad_s) && positive(manual_max_gimbal_pitch_rate_rad_s),
              "manual speed limits must be positive");
static_assert(chassis_spin_forward_scale >= 0 && chassis_spin_forward_scale <= 1 && chassis_spin_lateral_scale >= 0 &&
                  chassis_spin_lateral_scale <= 1,
              "spin translation scales must be within 0..1");
static_assert(chassis_zero_speed_epsilon_m_s >= 0 && chassis_resume_speed_m_s >= chassis_zero_speed_epsilon_m_s,
              "chassis speed hysteresis must be ordered");
static_assert(positive(chassis_max_wz_rad_s) && positive(chassis_steer_speed_rad_s) &&
                  positive(chassis_drive_slew_rad_s2) && positive(chassis_steer_slew_rad_s2),
              "chassis rate and slew limits must be positive");
static_assert(big_yaw_heading_kp >= 0 && big_yaw_heading_rate_max_rad_s > 0 && big_yaw_heading_rate_kp >= 0 &&
                  big_yaw_heading_rate_ki >= 0 && big_yaw_heading_integral_max_rad_s > 0 &&
                  big_yaw_heading_release_epsilon_rad_s >= 0 && positive(big_yaw_native_acceleration_rad_s2),
              "big-Yaw heading follower gains and acceleration must be valid");
static_assert(positive(big_yaw_world_acceleration_rad_s2) && big_yaw_world_jerk_rad_s3 >= 0 &&
                  big_yaw_stop_capture_dwell_s >= 0 &&
                  big_yaw_stop_capture_exit_rad_s >= big_yaw_stop_capture_enter_rad_s,
              "big-Yaw world shaping and stop capture must be valid");

inline constexpr float two_pi = 6.2831853071795864769f;
// 调参：小 Yaw 原始编码器上下限，ticks（8192 ticks/转）；当前为全量程，非实测机械安全端点。
inline constexpr std::uint16_t yaw_low_ticks = 5770, yaw_high_ticks = 7340;
inline constexpr float yaw_min_rad = (yaw_low_ticks - small_yaw.encoder_zero_ticks) * two_pi / 8192;
inline constexpr float yaw_max_rad = (yaw_high_ticks - small_yaw.encoder_zero_ticks) * two_pi / 8192;
// TODO(calibration): measure this independently; encoder zero is not joint center.
inline constexpr float yaw_center_rad = (yaw_min_rad + yaw_max_rad) * 0.5f;
// 调参：Pitch 驱动坐标的软件限位，rad；沿用现值，须与实测机械端点确认后再修改。
inline constexpr float pitch_min_rad = 2.60f, pitch_max_rad = 3.40f;
static_assert(yaw_min_rad < yaw_max_rad && yaw_center_rad > yaw_min_rad && yaw_center_rad < yaw_max_rad &&
                  pitch_min_rad < pitch_max_rad,
              "gimbal software limits must be ordered and contain their centers");
// TODO(measure): these are existing geometry placeholders, not validated by
// the reference's empirical radius=0.01 / reduction=1 speed conversion.
// wheel_gear_ratio must include any transmission between the rotor and wheel.
inline constexpr float wheel_radius_m = 0.055f, wheelbase_m = 0.4f, track_m = 0.4f;
static_assert(positive(wheel_radius_m) && positive(wheelbase_m) && positive(track_m),
              "wheel radius, wheelbase and track must be positive");

// TODO(IMU): measure sensor-to-mechanical-body rotations, then establish epochs
// on each boot/reference reset; equal frame IDs do not synchronize two clocks.
inline constexpr core::Quaternion chassis_sensor_to_body{}, carrier_sensor_to_body{}, head_sensor_to_body{};
inline constexpr core::OrientationReference chassis_reference{1, 1}, carrier_reference{2, 1}, head_reference{3, 1};
inline constexpr float power_idle_w = 5, power_per_abs_amp_w = 8;
// TODO(shooter): calibrate wheel speed -> projectile speed, dial angle per round,
// speed tolerance, dwell, jam current/time, heat per round and cooling data.
// 调参：摩擦轮输出轴速度大小，rad/s；方向由各 friction.direction 决定，先无弹确认两轮转向。
inline constexpr float friction_speed_rad_s = 20;
// 调参：拨盘每发对应的输出轴角度，rad；当前为九分度，按实际齿数/机构测量后标定。
inline constexpr float dial_step_rad = two_pi / 9;
// 调参：拨盘输出轴速度上限，rad/s；独立于位置环增益，连发还受 射频×每格角度 限制。
inline constexpr float dial_speed_rad_s = 20.0f * dial_step_rad;
// 调参：每发预留热量，裁判协议热量单位；loaded 必须采用对应裁判规则和真实热量反馈。
inline constexpr float heat_per_round = 10;
} // namespace skywalker::robotics::vehicle
