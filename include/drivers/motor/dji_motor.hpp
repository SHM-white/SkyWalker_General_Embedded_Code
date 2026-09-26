#pragma once

#include <cstdint>

#include <drivers/motor/dji_protocol.hpp>
#include <drivers/motor/motor_types.hpp>

namespace skywalker::motor::dji {

enum class Model : std::uint8_t {
    M3508C620 = 0,
    M2006C610,
    GM6020Current,
};

struct Gm6020Options {
    // 电机 CAN ID，范围为 1–7。
    std::uint8_t id = 0;
    // 软件允许的最大命令电流绝对值，单位 A，不得超过协议满幅 3 A。
    float current_limit_a = 0.0f;
    // 单圈编码器固定零点，范围为 0–8191 tick。
    std::uint16_t encoder_zero_ticks = 0;
    // 已确认电机固件工作在电流控制模式时设为 true。
    bool current_mode_confirmed = false;
    // 该电机的反馈、命令、恢复和使能超时设置。
    Timing timing{};
};

struct M3508Options {
    // 电机 CAN ID，范围为 1–8。
    std::uint8_t id = 0;
    // 软件允许的最大命令电流绝对值，单位 A，不得超过协议满幅 20 A。
    float current_limit_a = 0.0f;
    // 电机轴转数与输出轴转数之比，必须为正。
    float gear_ratio = 19.0f;
    // 该电机的反馈、命令、恢复和使能超时设置。
    Timing timing{};
};

struct M2006Options {
    // 电机 CAN ID，范围为 1–8。
    std::uint8_t id = 0;
    // 软件允许的最大命令电流绝对值，单位 A，不得超过协议满幅 10 A。
    float current_limit_a = 0.0f;
    // 电机轴转数与输出轴转数之比，必须为正。
    float gear_ratio = 36.0f;
    // 该电机的反馈、命令、恢复和使能超时设置。
    Timing timing{};
};

struct Config {
    // DJI 电机与驱动器型号，决定 CAN ID 映射和协议电流满幅。
    Model model = Model::M3508C620;
    // 电机 CAN ID；GM6020 为 1–7，其余型号为 1–8。
    std::uint8_t id = 0;
    // 软件允许的最大命令电流绝对值，单位 A。
    float current_limit_a = 0.0f;
    // 电机轴转数与输出轴转数之比；GM6020 使用 1:1。
    float gear_ratio = 1.0f;
    // GM6020 的单圈编码器固定零点，范围为 0–8191 tick。
    std::uint16_t encoder_zero_ticks = 0;
    // GM6020 固件的电流控制模式确认标志。
    bool current_mode_confirmed = false;
    // 该电机的反馈、命令、恢复和使能超时设置。
    Timing timing{};
};

Config gm6020(const Gm6020Options &options);
Config m3508(const M3508Options &options);
Config m2006(const M2006Options &options);

struct Descriptor {
    Model model = Model::M3508C620;
    std::uint8_t motor_id = 0;
    std::uint16_t feedback_id = 0;
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
    float protocol_current_max_a = 0.0f;
    float configured_current_limit_a = 0.0f;
    float gear_ratio = 1.0f;
    bool temperature_valid = false;
};

int describe(const Config &config, Descriptor &out);

} // namespace skywalker::motor::dji
