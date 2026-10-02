# 命令仲裁与执行恢复

当前命令层由同步 CommandArbiter 和后台 CommandManager 组成。CommandManager 注册输入来源并发布 CommandSnapshot；CommandArbiter 按操作模式、来源新鲜度、视觉 reference 和裁判权限生成最终命令。执行层另行处理电机反馈、位置参考、CAN 生命周期和本地恢复。

旧的 GlobalSafetyManager、GimbalLocalSafety、ChassisLocalSafety 与对应输入/决策结构已从当前源码移除。不要把开发方案中的这些名称当作现行接口。

## 服务和消息边界

- 必须注册一个 Operator 来源；Aim 来源可选，每个角色各一个。
- 裁判许可通过 IPermissionSource 单独绑定。require_referee_for_motion 为真时，缺少许可来源会使 start 失败；许可数据未到或过期时仲裁保持禁止运动。
- start() 启动来源并创建后台 worker；返回 0 不表示已有有效输入。start 失败后对象不可销毁重建或再次启动。
- snapshot() 读取 observed、decision、来源诊断的同一份拷贝；current() 只读 RobotCommand。两者都不刷新时间戳。
- 遥控或视觉采样出现 -EAGAIN 时保留缓存并继续由原始 stamp 判断过期；其他采样错误让对应输入失效。
- 应用执行器不得把旧命令复制到新时间，也不能因通信恢复就自动恢复电机输出。

## 应用侧恢复责任

applications 内的 GimbalExecutor 与 ChassisExecutor 是应用私有封装。它们处理配置检查、初始化重试、命令超时、本地反馈、可信参考、显式使能和 CAN 提交。RunStatus 汇报 Disabled、Recovering、Active、Blocked 及等待原因；stamp 必须标记执行状态的真实生产时间。

`include/robotics/gimbal/gimbal_executor.hpp` 另提供 `skywalker::robotics::GimbalExecutor` 双轴机械执行器，供 `command_gimbal` 与后续共享 CAN 组合复用。应用创建并 attach/start 两个 Motor、专属 Group 和物理 CanBus，然后调用 begin；执行器 update 只暂存电机输出，应用在周期末统一 commit。`GimbalExecutionInputs` 包含最终命令、原始 source_stamp、可选许可、总线有效性、急停和显式清除。commit 失败时应用调用 suspend，立即撤销双轴。

`RecoveryGate` 对每个机构维护独立恢复代次和准备边界。复位控制历史后调用 prepared；accept 必须同时检查最终命令和原始输入的生产时间严格晚于边界，并在 Enabling/Active 的每轮继续检查年龄。最终仲裁序号变新不表示原始输入已更新。暂态撤销清掉授权，准备完成后等待新的原始输入；急停和硬故障按显式清除边界处理。

`SnapshotCache<T>` 提供独立消费者的完整快照复制，保留原始生产时间。`execution_skeleton` 演练来源停产、单消费者停产、控制周期停顿和参考失效；不驱动电机。三个 IMU 安装位置的观测、后续惯性适配与双 Yaw 路线见[逐级整车指南](../../dev/项目优化与逐级整车验证样例实施指南.md)。

sentry_gimbal 目前只注册 RemoteSource 和 RefereePermissionSource，未注册 VisionSource；board_config 将 allow_auto 设为 false，裁判版本仍为 Unspecified，且 connections_configured=false。sentry_chassis 同样保持 connections_configured=false，power_model_calibrated=false。当前默认配置是防止未核对硬件输出的门禁，不代表接口缺失。

软件撤销输出不等于机械立即停止，也不能代替物理急停。反馈丢失导致连续位置参考不可信时，恢复前须重建参考并等待新的命令。

## 大 Yaw 独立执行与跨板恢复

`include/robotics/execution/big_yaw_executor.hpp` 提供 `BigYawExecutor`。应用注入独立的 DM Motor 与 VelocityMotor 配置，负责 attach/start 和周期末 CAN commit；执行器只执行速度内环、反馈检查、权限、启停与恢复，不提交物理总线。速度与反馈通过减速比、方向换算为关节 rad/s，连续旋转不依赖固定绝对机械零点。commit 失败后调用 `suspend` 撤销该轴。

`BigYawExecutionInputs` 包含 V2 请求、本板 boot_id、契约兼容状态、本地/板间链路状态、急停和显式清除。该轴的 `RecoveryGate`、生产时间、恢复代次和许可完全独立于四舵轮；V1 心跳的轮控 `resume_generation` 不会授权大 Yaw。

恢复按以下顺序推进：

1. 总线、反馈、控制周期或授权撤销时停止驱动输出，原目标不再有效。暂态协议故障满足条件后尝试清除；急停与硬故障要求显式清除。
2. 链路、反馈和周期恢复，速度控制历史 reset 完成后建立新的本地准备边界，独立恢复代次递增。
3. 底盘执行线程生产 `BigYawFeedback`，通信线程保留其生产时间并发送独立代次。云台端收到新鲜有效反馈后绑定目标板 boot_id 和大 Yaw 代次，记录当前原始输入序号为发送边界。
4. 云台原始输入在新边界之后继续生产，且头部惯性保持有效、小云台实际 Active，回中外环才产生有效速度请求。请求保留原始源序号、原始源年龄、仲裁命令年龄与输出许可。
5. 底盘执行器要求目标 boot_id/独立代次匹配、原始源和最终命令均晚于本地准备边界且新鲜、许可新鲜有效，才申请驱动 enable。Enabling 每轮继续监督这些条件，握手完成后执行速度内环。

单板重启、独立轴恢复代次变化、能力或反馈失效，会撤销旧上下文；旧目标与旧原始源不能仅凭通信恢复重新使能。禁用命令可立即撤销输出，不需要用旧授权继续运动。

V2 有独立 capabilities/request/feedback ID，双方显式开启该能力且契约版本相符后才开放新增轴；V1 保留原有布局与含义。版本失配、旧固件不声明能力或能力过期时，大 Yaw 保持禁用。协议字段与年龄处理见[板间通信](../communication/interboard-transports.md)。

`BigYawFeedback.stamp` 由执行线程真实生产；`setBigYawFeedback`、通信发送、对端读取快照不会刷新它。状态停止生产但心跳继续时，反馈生产年龄超过时限即撤销 ready/armed/valid；请求停止生产或原始输入停止更新同样过期。最终请求序号、板间帧序号变新都不能代替原始输入生产。

`big_yaw` 单轴台架先验证速度内环，`dual_yaw_centering` 的云台角色运行头部惯性适配和回中外环，底盘角色仅运行独立大 Yaw。机械中心使用中央 `vehicle::yaw_center_rad`，与编码器零点分开。默认接线/安装确认仍关闭；速度环、方向、回中参数、停止延迟和故障恢复留有 TODO，必须在实板关卡保存标定结果。

接口示例见[命令来源服务文档](command-service.md)，应用级时序见[模块联动](../../applications/module-integration.md)。
