# 命令保险与局部恢复

命令层决定目标与拨杆保险；通信和执行封装各自恢复。GlobalSafetyManager、GimbalLocalSafety、ChassisLocalSafety 及其专用输入/决策消息已移除。

```cpp
const auto frame = sources.poll();
const auto decision = manager.update(frame.inputs);
telemetry.emit(frame, decision);
```

CommandInputs 包含 now_us，CommandDecision 按值返回并包含 error。构造后配置固定，用 configError() 读配置结果，reset() 清运行历史。旧 step、validate、带时间 reset 不再提供。

Down 禁用；有效工作档与新命令允许恢复。遥控丢失撤销依赖它的命令；视觉丢失时 Auto 云台 Hold，遥控底盘不受影响。裁判软件许可由 require_referee_for_motion 控制且仅在命令层裁剪。正式云台应用 allow_auto=false，尚未装配视觉。

应用主循环只交接命令。GimbalExecutor 和 ChassisExecutor 内部处理配置、初始化重试、命令超时、反馈与参考准备、控制器复位及使能。RunStatus 显示 Disabled/Recovering/Active/Blocked 与等待原因。普通掉线不全局锁存；未知驱动故障只阻止所属机构。UnexpectedDisabled/EnableTimeout 可由封装在反馈恢复后有间隔清除，组内存在其他硬故障时不自动清除。

RemoteReceiver 初次初始化失败会重试。RefereeReceiver 封装裁判 UART 恢复。InterBoardEndpoint 封装 UART、V1 编码、会话、线程间快照和源命令年龄；上下文改变后等待新 producer 命令，不给旧命令换代次。两板按同一版本成套更新。

电机与 CAN 的内部安全输出、期限、反馈稳定和 Group 机制保留。SafetyAction/SafetyState/ExecutionState 目前用于已有控制动作和 V1 适配，不再是应用层的安全管理器。

底盘本地 require_power_budget 保留原功率预算门控，需按实际标定配置；云台和底盘 connections_configured 的出厂阻止状态没有放开。旧低层电机样例仍可显式 enable/disable，其手动操作不代表正式应用需要人工恢复。

恢复依赖新鲜反馈与可信参考，丢失多圈位置时不能仅凭恢复通信续跑。软件撤销输出不等于机械立即停止，编译不能替代设备恢复时序和机械行为验证。
