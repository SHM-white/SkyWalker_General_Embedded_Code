#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
struct RefereePowerState {
    // 裁判系统测得的底盘实时功率，单位 W。
    float chassis_power_w = 0;
    // 裁判系统给出的底盘功率上限，单位 W。
    float chassis_power_limit_w = 0;
    // 底盘缓冲能量，单位 J。
    float buffer_energy_j = 0;
    // 功率与缓冲能量的测量时间戳；机器人状态帧不会刷新它。
    MessageStamp stamp{};
    // 功率上限的更新时间戳。
    MessageStamp limit_stamp{};
};
struct RefereeRobotState {
    // 裁判系统分配的机器人编号。
    std::uint8_t robot_id = 0;
    // 底盘输出许可。
    OutputPermission chassis_output{};
    // 云台输出许可。
    OutputPermission gimbal_output{};
    // 发射机构输出许可。
    OutputPermission shooter_output{};
};
struct RefereeState {
    // 机器人编号与各机构输出许可。
    RefereeRobotState robot{};
    // 底盘功率与缓冲能量状态。
    RefereePowerState power{};
    // 最近一次裁判系统消息的时间戳与序号。
    MessageStamp stamp{};
    // 裁判系统数据当前是否在线且未超时。
    bool online = false;
};
}
