#pragma once

#include <cstdint>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <control/position_motor.hpp>
#include <drivers/imu/dm_imu_can.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dm_motor.hpp>
#if defined(CONFIG_SKYWALKER_COMMAND_SERVICE)
#include <robotics/command/command_manager.hpp>
#endif
#include <robotics/gimbal/dual_yaw_motion_allocator.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>
#include <robotics/gimbal/gimbal_executor.hpp>
#include <robotics/vehicle/calibration.hpp>

namespace skywalker::robotics::vehicle {
// Values and side-effect-free configuration factories only. Applications own
// their devices, receivers, motor instances, startup order and execution loops.
inline constexpr std::uint32_t gimbal_command_timeout_ms = 100;
inline constexpr core::TimeUs gimbal_period_us = 5000;
static_assert(small_yaw.gear_ratio == 1 && pitch.gear_ratio == 1,
              "Native joint profiles require an explicit external gearbox transform otherwise");
static_assert(yaw_low_ticks < yaw_high_ticks && yaw_high_ticks < 8192 && small_yaw.encoder_zero_ticks < 8192);

inline motor::dji::Config yawHardware() {
    return motor::dji::gm6020({.id = small_yaw.id,
                               .current_limit_a = small_yaw.effort_limit,
                               .encoder_zero_ticks = small_yaw.encoder_zero_ticks,
                               .current_mode_confirmed = connections_confirmed,
                               .timing = {.feedback_timeout_ms = 20,
                                          .command_timeout_ms = 20,
                                          .enable_timeout_ms = 100,
                                          .retry_interval_ms = 100}});
}
inline motor::dm::Config pitchHardware() {
    return motor::dm::j4310Mit({.id = pitch.id,
                                .master_id = pitch.master_id,
                                .position_max_rad = 12.5f,
                                .velocity_max_rad_s = 30.0f,
                                .torque_max_nm = 10.0f,
                                .torque_limit_nm = pitch.effort_limit,
                                .timing = {.feedback_timeout_ms = 50,
                                           .command_timeout_ms = 20,
                                           .enable_timeout_ms = 3000,
                                           .retry_interval_ms = 100}});
}
inline constexpr GimbalAxisConfig yawAxisConfig() {
    return {AxisTopology::Limited,          yaw_min_rad, yaw_max_rad,
            small_yaw.velocity_limit_rad_s, true,        AxisReferenceInit::CalibratedFeedback};
}
inline constexpr GimbalAxisConfig pitchAxisConfig() {
    return {AxisTopology::Limited,      pitch_min_rad, pitch_max_rad,
            pitch.velocity_limit_rad_s, true,          AxisReferenceInit::CalibratedFeedback};
}

inline skywalker::control::PositionMotor::Config yawMotorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = skywalker::control::EffortUnit::Ampere;
    c.reference = skywalker::control::PositionReference::DriverContinuous;
    // 调参：先确认速度内环，再改位置外环增益/积分限幅；保持原有初值，
    // 不通过提高 kp 掩盖速度、电流饱和。deadband 单位 rad，0.012约0.69°；
    // 根据静止噪声与机械间隙逐步缩小，不能直接清零来消除微调台阶。
    c.loop.position = {.kp = 30.0f,
                       .ki = 0.5f,
                       .kd = 1.0f,
                       .derivative_tau_s = 0.0f,
                       .integral_min = -0.6f,
                       .integral_max = 0.6f,
                       .output_min = -4.0f,
                       .output_max = 4.0f,
                       .deadband = 0.012f,
                       .dt_min_s = 0.001f,
                       .dt_max_s = 0.020f};
    // 调参：Yaw速度内环。误差单位 rad/s，输出/积分限幅单位 A；
    // 增益、微分滤波和限幅按电流/转速反馈分别调，勿同时改多组参数。
    c.loop.velocity.regulator.feedback = {.kp = 0.45f,
                                          .ki = 0.55f,
                                          .kd = 0.00005f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -1.2f,
                                          .integral_max = 1.2f,
                                          .output_min = -1.2f,
                                          .output_max = 1.2f,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    // 调参：速度参考升/降斜率（rad/s²），决定启动、停转和反向响应。
    c.loop.velocity.reference_slew = {20.0f, 20.0f};
    // 调参：速度反馈滤波时间常数（s），0直通；软死区单位rad/s。
    c.loop.velocity.measurement_filter_tau_s = 0.0f;
    c.loop.velocity.soft_deadband_rad_s = 0.0f;
    // 调参：内环速度/电流上限；与硬件能力协调，鼠标灵敏度独立设置。
    c.loop.velocity.requested_velocity_abs_max_rad_s = 8.0f;
    c.loop.velocity.effort_abs_max = 1.2f;
    // 调参：轨迹前馈。位置环输出作为速度修正，受限轨迹的参考速度直接加到
    // 速度请求（增益 1）；加速度电流前馈先为 0，台架确认同方向电流响应与
    // 限幅余量后再填 K_a（A/(rad/s²)）。
    c.loop.velocity_feedforward_gain = 1.0f;
    c.loop.acceleration_effort_gain = 0.0f;
    return c;
}
inline skywalker::control::PositionMotor::Config pitchMotorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = skywalker::control::EffortUnit::NewtonMeter;
    c.reference = skywalker::control::PositionReference::DriverContinuous;
    // 调参：Pitch位置外环，保持既有PID；deadband=0.01 rad约0.57°。
    // 先记录重力负载、静止噪声和跟踪误差，再逐项调整增益/死区/积分。
    c.loop.position = {.kp = 30.0f,
                       .ki = 0.6f,
                       .kd = 2.0f,
                       .derivative_tau_s = 0.0f,
                       .integral_min = -1.2f,
                       .integral_max = 1.2f,
                       .output_min = -5.0f,
                       .output_max = 5.0f,
                       .deadband = 0.01f,
                       .dt_min_s = 0.001f,
                       .dt_max_s = 0.020f};
    // 调参：Pitch速度内环输出单位N·m；先看速度跟踪与力矩饱和，
    // 再改增益，避免用外环增益补偿内环或机构不足。
    c.loop.velocity.regulator.feedback = {.kp = 0.6f,
                                          .ki = 0.8f,
                                          .kd = 0.005f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -2.0f,
                                          .integral_max = 2.0f,
                                          .output_min = -6.0f,
                                          .output_max = 6.0f,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    // 调参：Pitch升/降斜率（rad/s²）与Yaw独立，过小会增加停转拖尾。
    c.loop.velocity.reference_slew = {10.0f, 10.0f};
    // 调参：反馈滤波时间常数（s），此处0.02不是鼠标输入固定延迟；
    // 软死区（rad/s）影响小速度跟踪，须结合噪声分别调整。
    c.loop.velocity.measurement_filter_tau_s = 0.02f;
    c.loop.velocity.soft_deadband_rad_s = 0.02f;
    // 调参：机构速度/力矩上限，保留原值并按安全机械能力逐步确认。
    c.loop.velocity.requested_velocity_abs_max_rad_s = 6.0f;
    c.loop.velocity.effort_abs_max = 5.0f;
    return c;
}

#if defined(CONFIG_SKYWALKER_COMMAND_SERVICE)
inline constexpr CommandManager::Config gimbalCommandConfig() {
    CommandManager::Config c{};
    c.mapper.input_profile = RemoteInputProfile::PhysicalRemote;
    c.max_gimbal_yaw_rate_rad_s = manual_max_gimbal_yaw_rate_rad_s;
    c.max_gimbal_pitch_rate_rad_s = manual_max_gimbal_pitch_rate_rad_s;
    c.input_timeout_ms = gimbal_command_timeout_ms;
    c.require_referee_for_chassis = false;
    c.require_referee_for_gimbal = false;
    c.require_referee_for_shooter = false;
    // 发射机构先接线：手动档允许键鼠产生摩擦轮/发射请求，仲裁器据此产出
    // ShooterCommand（含单发事件戳）。摩擦轮/拨盘执行端与热量门控另行接入。
    c.enable_shooter = true;
    c.allow_auto = vision_execute_confirmed && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER) &&
                   DT_HAS_ALIAS(vision_uart);
    c.expected_vision_reference = head_reference;
    return c;
}
#endif
inline GimbalExecutor::Config gimbalExecutionConfig() {
    return {.command_timeout_ms = gimbal_command_timeout_ms,
            .source_timeout_us = gimbal_command_timeout_ms * 1000ULL,
            .permission_timeout_ms = 300,
            .max_cycle_us = 20000};
}
// 双 Yaw 协同分配：把枪头世界运动拆成大 Yaw 载体运动和小 Yaw 相对运动。
// 关节位置/速度使用小 Yaw 编码器坐标；枪头和大 Yaw 意图使用世界坐标，逆时针为正。
// 关系：枪头世界速度 = 载体世界速度 + joint_direction × 小 Yaw 关节速度。
// 这里配置的是运动预算与参考整形，电机 PID、电流/力矩限制和底盘旋转补偿另行配置。
inline constexpr DualYawAllocatorConfig dualYawAllocatorConfig() {
    DualYawAllocatorConfig c{};
    // 小 Yaw 机械中心，单位 rad；回中以此为目标，不一定等于编码器零位。
    c.joint_center_rad = yaw_center_rad;
    // c.joint_center_rad = 0.0f;
    // 小 Yaw 编码器坐标下的位置下限/上限，单位 rad；应来自实际机械行程标定。
    c.joint_min_rad = yaw_min_rad;
    c.joint_max_rad = yaw_max_rad;
    // 编码器正转到枪头相对载体正转的方向，只能为 +1 或 -1，在分配器内使用一次。
    c.joint_direction = small_yaw.direction;
    // 两端各预留的软边界，单位 rad；0.0175 约为 1°，位置计划限制在此边界内。
    c.joint_margin_rad = 0.0175f;
    // 估算向外剩余行程时额外扣除的位置误差余量，单位 rad，覆盖死区/标定误差。
    // 正向可用行程 = joint_max_rad - joint_margin_rad - joint_tolerance_rad - 实测位置。
    c.joint_tolerance_rad = 0.012f;
    // 小 Yaw 可执行的关节速度上限，单位 rad/s；制动包络还会按剩余行程收紧它。
    c.joint_rate_max_rad_s = small_yaw.velocity_limit_rad_s;

    // 枪头世界速度总上限，单位 rad/s；来自共享手动速度配置。
    // 必须满足 0 < head_steady_rate_max_rad_s <= head_rate_max_rad_s，否则分配被拒绝。
    c.head_rate_max_rad_s = manual_max_gimbal_yaw_rate_rad_s;
    // 枪头世界速度计划的加速度绝对值上限，单位 rad/s²；越大，起停/反向越快。
    // 实际加速度窗口还会结合载体实测加速度和小 Yaw 关节加速度能力进一步收紧。
    c.head_acceleration_max_rad_s2 = 30.0f;
    // 枪头加速度变化率（jerk）上限，单位 rad/s³；0 表示只限加速度，不限 jerk。
    c.head_jerk_max_rad_s3 = 0;
    // 枪头持续世界速度预算，单位 rad/s；限制手动目标和视觉速度前馈。
    // 大 Yaw 跟踪不足时还会乘 tracking_budget_scale_min 到 1 之间的预算比例。
    c.head_steady_rate_max_rad_s = 6.0f;
    // 外置头部 IMU 闭环的世界 Yaw 目标最大超前角，单位 rad；0.25 约为 14.3°。
    // 跟踪不足时丢弃超出部分，避免触限/恢复后追赶积累的目标；0 禁用此保护。
    c.head_angle_lead_max_rad = 0.25f;

    // 发给底盘的大 Yaw 世界速度意图上限，单位 rad/s，包含慢分量和回中修正。
    // 持续跟随能力需与枪头持续预算匹配；底盘会另行叠加车体补偿并限制关节速度。
    c.big_intent_max_rad_s = 6.0f;
    // 大 Yaw 意图的加速度绝对值上限，单位 rad/s²；底盘/电机侧还有独立整形限制。
    c.big_acceleration_max_rad_s2 = 20.0f;
    // 大 Yaw 意图的 jerk 上限，单位 rad/s³；0 表示不限制加速度变化率。
    c.big_jerk_max_rad_s3 = 0;
    // 大 Yaw 意图追踪“枪头慢分量 + 回中速度”的增益，单位 1/s。
    // 未触及加速度/jerk 限制时，近似时间常数为 1/增益；3 对应约 0.33 s。
    // 增大可减少意图滞后，但需结合回中增益观察超调和振荡。
    c.big_track_gain_s_inv = 5.0f;

    // 手动模式的一阶低通时间常数，单位 s；同时滤波主动枪头速度计划和小 Yaw 回中误差。
    // 主动计划已限速/限加速度，不含触限保护产生的被动随动，避免松杆后继续驱动大 Yaw。
    // 越大越平滑、滞后越明显；滤出的主动运动慢分量交由大 Yaw 承担。
    c.slow_tau_s = 0.10f;
    // 视觉模式使用的同类滤波时间常数，单位 s；仅视觉轨迹分配生效。
    c.vision_slow_tau_s = 0.24f;
    // 视觉目标角与枪头计划角之间的误差转速度增益，单位 1/s；仅视觉模式生效。
    // 该角度修正可使用 head_rate_max_rad_s 总预算，仍受行程和加速度限制。
    c.vision_angle_gain_s_inv = 8.0f;

    // 小 Yaw 正向/负向可达的关节加速度幅值，单位 rad/s²；两者均填写正数。
    // 载体加速度有效时，用它们收紧枪头加速度窗口；应根据机构实测能力设置。
    c.joint_acceleration_max_pos_rad_s2 = 20.0f;
    c.joint_acceleration_max_neg_rad_s2 = 20.0f;

    // 回中误差的连续软死区，单位 rad；0.0087 约为 0.5°。
    // 死区内回中修正为零，死区外只对超出部分计算修正，减少中心附近来回摆动。
    c.center_deadband_rad = 0.0087f;
    // 过滤后的回中偏角转大 Yaw 世界速度的增益，单位 (rad/s)/rad，即 1/s。
    // 增大可加快回中，也可能因滤波和跟随滞后产生振荡；它不改变手动速度上限。
    c.center_kp_rad_s_per_rad = 1.0f;
    // 回中修正速度的绝对值上限，单位 rad/s；不单独限制枪头慢分量。
    c.center_rate_max_rad_s = 5.0f;

    // 制动距离估算中的反应延迟，单位 s，覆盖采样/传输/执行建立时间。
    // 延迟期间按最坏向外加速计算，之后按可保证的减速度制动，得到向外速度预算。
    c.brake_delay_s = 0.02f;
    // 反应延迟期间可能出现的最坏向外加速度，单位 rad/s²；需覆盖实际可能值。
    c.brake_outward_acceleration_rad_s2 = 4.0f;
    // 机构可保证的制动减速度幅值，单位 rad/s²，必须大于零。
    // 填得越大，估计的制动距离越短、放行速度越高，应依据实测制动能力设置。
    c.brake_deceleration_rad_s2 = 10.0f;

    // 载体实测世界速度与大 Yaw 意图的偏差阈值，单位 rad/s；超出时降低持续预算。
    c.tracking_error_rad_s = 0.35f;
    // 跟踪不足时持续速度预算最多降到原值的此比例，范围 (0, 1]；0.4 表示 40%。
    c.tracking_budget_scale_min = 0.4f;
    // 持续预算比例每秒的下降/恢复速度，单位 1/s；0.5 表示每秒变化 0.5。
    c.tracking_budget_slew_per_s = 0.5f;

    // 停稳锁零的速度阈值，单位 rad/s；枪头目标/计划、慢分量、回中和大 Yaw 意图
    // 等速率同时足够小，且计划加速度满足下一项时，才将速度计划精确归零。
    c.still_rate_epsilon_rad_s = 0.01f;
    // 停稳锁零的计划加速度阈值，单位 rad/s²；配合速度阈值抑制残余微小运动。
    c.still_acceleration_epsilon_rad_s2 = 0.05f;
    return c;
}
static_assert(dualYawAllocatorConfig().joint_min_rad < dualYawAllocatorConfig().joint_center_rad &&
                  dualYawAllocatorConfig().joint_center_rad < dualYawAllocatorConfig().joint_max_rad,
              "dual-Yaw allocator geometry must contain the small-Yaw centre");
inline imu::DmImuCanSource::Config headSensorConfig() {
    imu::DmImuCanSource::Config c{};
#if DT_NODE_HAS_STATUS(DT_ALIAS(imu_can), okay)
    c.can = DEVICE_DT_GET(DT_ALIAS(imu_can));
#else
    c.can = DEVICE_DT_GET(DT_NODELABEL(can3));
#endif
    // The installed head IMU is a dedicated CAN3 quaternion reporter.
    // Changing transport/mode is a hardware-baseline change, not a stage option.
    c.feedback_id = 0x11;
    c.slave_id = 1;
    c.mode = imu::DmImuCanSource::Mode::ActiveReport;
    c.attitude = imu::DmImuCanSource::Attitude::Quaternion;
    c.reference = head_reference;
    c.sensor_to_body = head_sensor_to_body;
    return c;
}
} // namespace skywalker::robotics::vehicle
