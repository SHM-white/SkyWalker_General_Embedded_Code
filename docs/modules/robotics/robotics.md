# 机器人决策与执行模块

机器人算法位于 include/robotics 与 lib/robotics。它把时间戳消息转换为带单位的命令，或把已授权目标转换为运动学和控制输入。板级设备、电机绑定和应用线程由 samples/ 与 applications/ 负责；协议恢复由 Motor/CAN 独立处理。

## 当前调用链

~~~text
通信接收器
  → RemoteSource / VisionSource / RefereePermissionSource
  → CommandManager worker
  → CommandArbiter
  → CommandSnapshot
  → 应用执行器
  → GimbalAxis / SwerveChassis / Motor / CanBus
~~~

CommandArbiter 是同步策略核心，接收 CommandInputs 并返回 CommandDecision。CommandManager 是注册来源和后台调度服务，不直接接收 CommandInputs，也没有 update()。旧的 GlobalSafetyManager、GimbalLocalSafety、ChassisLocalSafety 与专用输入/决策类型已移除；应用执行器检查原始输入年龄和硬件配置，持续提交目标；实际反馈只决定对应轴是否能够计算输出。

## 命令来源服务

启动时注册 Operator 来源，按策略可注册 Aim 来源，并绑定许可来源；随后调用 CommandManager::start()。后台线程按配置周期采样来源、调用 CommandArbiter、发布 CommandSnapshot。执行线程用 snapshot() 读取完整诊断帧，或用 current() 读取最终 RobotCommand。读快照不会消费结果，也不会延长旧输入的有效期。

完整注册、错误语义、静态生命周期和同步仲裁入口见[命令来源与后台服务](command-service.md)。三源运行入口见 samples/robotics/command_manager/src/main.cpp。

## GimbalAxis

持续保存机械目标，位置控制独立等待本轴反馈。Rate 时间轴不因驱动离线停止；实际反馈恢复后 PositionMotor 自动重置本轴 PID。Limited 轴保留必要机械范围和可信参考；没有全组 ready 或手动恢复准入。

## SwerveChassis 与功率限制

运动学持续解算，舵向与轮驱分别有效、分别初始化、分别计算。没有全模块对齐互锁、drive_ready 或 cos 缩速。轮驱速度控制不需要位置参考；舵向使用绝对角的本地短程展开，未知角度不参与翻转判断。

功率限制保留真实预算和测量有效期，删除与电机恢复时刻比较的授权边界。各轴输出携带计算时执行版本，统一物理 CAN 发布者提交。

更多接口、协议与实板验收见[实施指南](../../dev/电机持续指令与独立自动恢复重构实施指南.md)。
