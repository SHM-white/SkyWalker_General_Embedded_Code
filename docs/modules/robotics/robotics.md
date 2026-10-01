# 机器人决策与执行模块

机器人算法位于 include/robotics 与 lib/robotics。它把时间戳消息转换为带单位的命令，或把已授权目标转换为运动学和控制输入。板级设备、电机绑定、应用线程及本地恢复由 samples/ 与 applications/ 负责。

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

CommandArbiter 是同步策略核心，接收 CommandInputs 并返回 CommandDecision。CommandManager 是注册来源和后台调度服务，不直接接收 CommandInputs，也没有 update()。旧的 GlobalSafetyManager、GimbalLocalSafety、ChassisLocalSafety 与专用输入/决策类型已移除；应用执行器自己检查本地反馈、时间戳、恢复代次和硬件配置。

## 命令来源服务

启动时注册 Operator 来源，按策略可注册 Aim 来源，并绑定许可来源；随后调用 CommandManager::start()。后台线程按配置周期采样来源、调用 CommandArbiter、发布 CommandSnapshot。执行线程用 snapshot() 读取完整诊断帧，或用 current() 读取最终 RobotCommand。读快照不会消费结果，也不会延长旧输入的有效期。

完整注册、错误语义、静态生命周期和同步仲裁入口见[命令来源与后台服务](command-service.md)。三源运行入口见 samples/robotics/command_manager/src/main.cpp。

## GimbalAxis

GimbalAxis 封装单轴位置参考处理和 PositionMotor 调用，但不拥有底层 Motor / CanBus，不负责 enable、disable 或 CAN commit。应用完成 attach/start 和 begin 后，持续 poll 反馈；在输入新鲜、轴就绪并有新的运行授权时 reset，再显式使能。Active 周期调用 update 或 updateRate，成功后提交对应物理 CAN。

- Continuous 拓扑要求固定零点单圈反馈，并使用 PositionReference::AbsoluteNearest。
- Limited 拓扑要求已校准 DriverContinuous 坐标和机械角限位。
- ready_for_enable 在电机 Active 时为 false 是正常现象；Active 与 feedback_healthy 应分别判断。
- 错误时由调用者停对应 Motor 或 Group，并按新的有效输入和可信位置参考恢复。

详细周期代码见[封装模块调用示例](../call-examples.md#9-gimbalaxis)与 samples/robotics/gimbal_control/src/main.cpp。

## SwerveChassis 与功率限制

SwerveChassis 是不访问设备的算法封装。它按 FL、FR、RL、RR 顺序接收四轮反馈和底盘命令，完成运动学与各模块控制计算；它不会绑定八台电机或提交 CAN。applications/sentry_chassis 中的 ChassisExecutor 与 DjiChassisHardware 完成这些工作。

ChassisPowerLimiter 根据测量功率、功率限额和缓冲能量计算 effort_scale，是台架启发式，不等于竞赛功率合规认证。当前 sentry_chassis 只有在 power_model_calibrated 且预算新鲜时才使用 limiter；默认标定开关关闭。

完整调用示例见[封装模块调用示例](../call-examples.md#10-swervechassis-与-chassispowerlimiter)及 applications/sentry_chassis/src/chassis_executor.cpp。
