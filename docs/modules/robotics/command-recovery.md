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

GimbalExecutor 与 ChassisExecutor 是应用私有封装，不是 lib/robotics 的通用接口。它们处理配置检查、初始化重试、命令超时、本地反馈、可信参考、显式使能和 CAN 提交。RunStatus 汇报 Disabled、Recovering、Active、Blocked 及等待原因。

sentry_gimbal 目前只注册 RemoteSource 和 RefereePermissionSource，未注册 VisionSource；board_config 将 allow_auto 设为 false，裁判版本仍为 Unspecified，且 connections_configured=false。sentry_chassis 同样保持 connections_configured=false，power_model_calibrated=false。当前默认配置是防止未核对硬件输出的门禁，不代表接口缺失。

软件撤销输出不等于机械立即停止，也不能代替物理急停。反馈丢失导致连续位置参考不可信时，恢复前须重建参考并等待新的命令。

接口示例见[命令来源服务文档](command-service.md)，应用级时序见[模块联动](../../applications/module-integration.md)。
