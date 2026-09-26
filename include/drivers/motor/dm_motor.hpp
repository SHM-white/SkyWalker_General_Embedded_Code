#pragma once

#include <cstdint>

#include <drivers/motor/dm_protocol.hpp>
#include <drivers/motor/motor_types.hpp>

namespace skywalker::motor::dm {

enum class Model : std::uint8_t {
    J4310_2EC_V1_1 = 0,
};

struct J4310Options {
    // 电机 CAN ID，范围为 1–15。
    std::uint8_t id = 0;
    // 驱动器发送反馈使用的标准 CAN ID。
    std::uint16_t master_id = 0;
    // 驱动器实际配置的 PMAX，单位 rad。
    float position_max_rad = 0.0f;
    // 驱动器实际配置的 VMAX，单位 rad/s。
    float velocity_max_rad_s = 0.0f;
    // 驱动器实际配置的 TMAX，单位 N·m。
    float torque_max_nm = 0.0f;
    // 应用允许的最大命令力矩绝对值，单位 N·m，不得超过 TMAX。
    float torque_limit_nm = 0.0f;
    // 该电机的反馈、命令、恢复和使能超时设置。
    Timing timing{50, 20, 50, 3000};
};

struct Config {
    // 达妙电机型号，当前支持 J4310-2EC-V1.1。
    Model model = Model::J4310_2EC_V1_1;
    // 固件中持久化的控制模式，须与电机实际设置一致。
    ControlMode mode = ControlMode::Mit;
    // 电机 CAN ID，范围为 1–15。
    std::uint8_t id = 0;
    // 驱动器发送反馈使用的标准 CAN ID。
    std::uint16_t master_id = 0;
    // 驱动器实际配置的 PMAX、VMAX、TMAX，用于命令量化和反馈解码。
    Limits limits{};
    // 应用允许的最大命令力矩绝对值，单位 N·m，不得超过 TMAX。
    float torque_limit_nm = 0.0f;
    // 该电机的反馈、命令、恢复和使能超时设置。
    Timing timing{50, 20, 50, 3000};
};

Config j4310Mit(const J4310Options &options);
Config j4310Velocity(const J4310Options &options);
Config j4310PositionVelocity(const J4310Options &options);

struct Descriptor {
    Model model = Model::J4310_2EC_V1_1;
    ControlMode mode = ControlMode::Mit;
    std::uint16_t motor_id = 0;
    std::uint16_t master_id = 0;
    std::uint16_t control_id = 0;
    Limits limits{};
    float torque_limit_nm = 0.0f;
};

int describe(const Config &config, Descriptor &out);

} // namespace skywalker::motor::dm
