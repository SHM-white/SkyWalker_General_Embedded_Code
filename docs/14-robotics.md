# 14 机器人算法与安全链路

实现位置：`include/robotics/`、`lib/robotics/`。这些模块不直接决定板卡引脚和电机 ID，而是消费 typed message、时间戳和执行状态。

## 1. 消息流

```text
RemoteState / RefereeState / InterBoard snapshots
                 │
                 ▼
ManualCommandMapper → OperatorIntent
                 │
                 ▼
GlobalSafetyManager → GlobalSafetyDecision
                 │
                 ▼
CommandManager → RobotCommand
                 │
                 ├── InterBoardCodec → chassis controller
                 └── local subsystem safety → motor / actuator wrapper
```

消息都带 `MessageStamp`：timestamp、sequence、valid。安全判断必须使用 `isFresh()`，不能仅看 `valid`。

## 2. 模块职责与一次命令计算

| 模块 | 输入 | 输出 | 运行位置 |
|---|---|---|---|
| `ManualCommandMapper` | `RemoteState` | `OperatorIntent` | 云台命令线程或无电机样例 |
| `GlobalSafetyManager` | 新鲜度、权限、急停和对端状态 | 分域 `SafetyAction` 与原因 | 云台命令线程 |
| `CommandManager` | 意图与全局安全决策 | 带物理单位的 `RobotCommand` | 云台命令线程 |
| `CommandRouter` | 命令与安全决策 | 本地云台/远端底盘值拷贝 | 云台命令线程 |
| `GimbalLocalSafety` / `ChassisLocalSafety` | 本地反馈、硬件和转发命令 | 本地执行许可 | 对应执行器线程 |
| `YawGimbal` / `SwerveChassis` | 已授权的物理目标 | 电机控制器目标 | 对应执行器线程 |

下面是 [command_safety 无电机样例](../samples/robotics/command_safety/src/main.cpp) 的核心顺序。`input` 还需填入急停、裁判权限和对端心跳/反馈；样例用模拟对端，整机应用使用真实板间快照。

```cpp
OperatorIntent intent{};
int ret = mapper.map(remote, intent);
GlobalSafetyInputs input{};
input.now_ms = now_ms;
input.command_source_fresh = remote.online;
input.operator_motion_enabled = remote.stamp.valid &&
    remote.left_switch == RcSwitch::Middle;
// 同一周期还要填入 input 的权限、急停和对端状态。
GlobalSafetyDecision decision{};
if (ret == 0) ret = safety.evaluate(input, decision);
RobotCommand command{};
if (ret == 0) ret = manager.step(intent, decision, now_ms, command);
if (ret < 0) {
    // 应用撤销输出许可，并记录原因；不要复用旧 command。
}
```

命令生成之后，云台板可经 `CommandRouter` 分发本地与底盘目标，但真正使能之前必须由本地安全层再次检查。完整板间闭环与执行调用见 [模块联动](module-integration.md#双主控命令与安全闭环)。

## 3. 指令层

`ManualCommandMapper` 把 `RemoteState` 转成 `OperatorIntent`：

- 左右拨杆决定 Safe / Manual / Auto 等操作模式。
- 摇杆归一化并施加 deadband。
- 鼠标可映射为云台 yaw/pitch rate。
- `friction_requested` / `fire_requested` 保留发射机构意图。

`CommandManager` 再把 intent 变成带物理单位的 `RobotCommand`，默认限制：

```text
chassis vx/vy 3 m/s，wz 6 rad/s
gimbal yaw 3 rad/s，pitch 2 rad/s
input timeout 100 ms
```

安全动作不是普通 mode：`Disable` 清零/关闭，`Hold` 保持，`Active` 才允许目标继续更新。

## 4. 全局与局部安全

### GlobalSafetyManager

输入包括命令源、操作开关、急停、裁判权限、底盘 heartbeat/feedback 和底盘执行状态。输出包括云台/底盘/发射机构各自的 `SafetyAction`、总体 `SafetyState` 和 `active_reasons`。

常见 reason：`CommandStale`、`PowerStale`、`FeedbackStale`、`TransportUnavailable`、`EmergencyStop`、`RecoveryBoundary`、`InvalidConfiguration`。

急停是 latch：只有输入释放且调用 `clearEmergencyStop(true)` 才能清除；清除后仍要重新经过 Ready/恢复流程，不会自动开始运动。

### ChassisLocalSafety / GimbalLocalSafety

局部安全在执行器所在板上再次验证：

- 命令是否来自当前 boot/generation。
- heartbeat、command、feedback 是否新鲜。
- 硬件是否 ready/armed。
- 是否经过稳定命令数量和恢复边界。

跨板转发不能只依赖远端的 global decision；本地必须有独立的 disable 路径。

## 5. 舵轮运动学

`SwerveKinematics` 的模块顺序固定为 FL、FR、RL、RR，坐标约定为 +x 前、+y 左、+wz 逆时针。给定底盘 `vx/vy/wz` 和四个模块位置，计算每个模块的角度与轮速；当速度接近 0 时保持上一次角度，避免舵向抖动。

`SwerveModule` 将目标接到位置外环和速度内环，并做轮半径换算。它不负责 CAN Bus 生命周期；应用需先读取和校验反馈，再决定何时 arm/flush。

当前 `samples/robotics/swerve` 只驱动一个物理模块做台架验证；`applications/sentry_chassis` 才是四模块硬件编排入口。

## 6. Yaw 云台

`YawGimbal` 支持：

- `Continuous`：要求 `PositionReference::AbsoluteNearest`，目标走固定零点的最短路径。
- `Limited`：要求 `PositionReference::DriverContinuous`，目标和实际都必须在机械限位内。
- `Hold`、`Rate`、`AbsoluteAngle` 三种命令模式。

`begin()` 只做配置，不等待电机供电或反馈；应用应循环 `poll()`，在权限和反馈都满足后才 update。发生 disable、非法 dt、超出机械限位或急停时应 suspend。

## 7. 功率限幅

`ChassisPowerLimiter` 是台架启发式，不是比赛功率合规证明。它根据实测功率、功率上限、buffer energy 和 reserve 计算 `effort_scale`，并以 `recovery_per_s` 限制恢复速度。

当前底盘应用只有在 `power_model_calibrated=true` 且裁判功率数据新鲜时才使用它；校准前使用固定 `bench_effort_scale`，生产应用必须替换为真实标定模型和保护策略。

## 8. 集成顺序

1. 先用通信样例确认 Remote/Referee/InterBoard 状态。
2. 用 `command_safety` 在无电机条件下观察 stale、急停和 reset。
3. 单独用 `yaw_gimbal` 或 `swerve` 验证执行器。
4. 再在 `sentry_*` 中打开真实连接，并保留 config blocked 默认值直到完成接线核对。
5. 最后才启用功率限幅、远端命令和多轴底盘。
