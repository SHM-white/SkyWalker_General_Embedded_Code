# 15 双主控应用骨架

> 命令与安全架构已迁移：[现行命令保险与局部恢复](19-command-recovery.md)。下文旧全局/局部安全链路属于历史说明。

当前有两个整机应用入口：

- `applications/sentry_gimbal/`：遥控/裁判/板间汇聚、全局安全、云台执行。
- `applications/sentry_chassis/`：板间接收、四轮舵底盘、本地安全、DJI 多总线硬件。

它们已经是可读的 Zephyr application 工程，但默认配置保守：连接未配置，云台和底盘不会因为编译成功就自动运动。

应用层的端到端调用与时序见 [模块联动：双主控命令与安全闭环](module-integration.md#双主控命令与安全闭环)。本页侧重两个工程的配置、线程和上机顺序。

## 1. 架构

```text
                 UART / Referee / DR16
                         │
                 sentry_gimbal
             ┌───────────┴───────────┐
             │ command/safety thread │
             │ interboard link       │──── heartbeat/control/constraint ────┐
             │ gimbal thread         │                                       │
             └───────────┬───────────┘                                       ▼
                       Yaw motor                                  sentry_chassis link thread
                                                                         │
                                                                    local safety
                                                                         │
                                                               4 steer + 4 drive DJI
```

板间链路使用 `InterBoardLink`；gimbal 向 chassis 发送 `ChassisControl` 和 `ChassisConstraint`，chassis 返回 heartbeat 和 `ChassisFeedback`。

## 2. `sentry_chassis`

源码：`applications/sentry_chassis/src/`。

- `linkTask`：异步 UART、解析板间帧、周期发送 heartbeat + chassis feedback。
- `chassisTask`：检查 peer boot、权限、功率、DJI 八个电机反馈，运行 `SwerveChassis`、`ChassisLocalSafety` 和 `DjiChassisHardware`。
- `chassis_hardware.*`：8 台 `Motor` 与一组 `Group` 先完成绑定，再启动所需 `CanBus`；控制目标按总线提交。
- 默认拓扑：4 个 GM6020 舵向 + 4 个 M3508 驱动；`board_config::connections_configured=false` 阻止未核对接线时使能。
- 底盘在锁存故障后可按 MC02 的 `sw0` 用户按钮请求清除；按下经过 30 ms 消抖后触发一次，仍须等待全组安全停机与稳定反馈，并收到新代次命令才能再次使能。

安全相关默认值：

```cpp
connections_configured = false;
require_referee_for_motion = true;
power_model_calibrated = false;
bench_effort_scale = 0.15f;
```

要启用真实底盘，至少需要修改：

1. `src/board_config.hpp` 的八台电机 CAN 绑定、ID、限流、GM6020 零点和 M3508 减速比。
2. 同一文件中的 `connections_configured`、方向、底盘几何、轮半径、PID 和权限策略。
3. 板间 UART 及其波特率，确认与云台应用相反端口接线。
4. 真实急停输入；复位入口默认使用 MC02 `sw0` 用户按钮，需确认现场可触达且接线正确。
5. 功率模型标定后才能把 `power_model_calibrated` 设为 true。

## 3. `sentry_gimbal`

源码：`applications/sentry_gimbal/src/`。

线程职责：

- `RemoteReceiver` 内部线程：DR16 UART → `AsyncUart` → `RemoteService` → `Snapshot`；由 main 调用 `remote_receiver.start()` 启动，应用不再定义 `remoteTask`。
- `refereeTask`：裁判 UART → `RefereeService` → `RefereeState`。
- `commandTask`：Remote/Referee/Peer feedback → mapper → global safety → `CommandManager` → `CommandRouter`。
- `linkTask`：板间接收并周期发送 heartbeat、约束和底盘控制。
- `gimbalTask`：构造一台 `Motor` 与 `CanBus`，运行私有拥有 `PositionMotor` 的 `GimbalAxis` 和 `GimbalLocalSafety`，显式管理使能与故障清除。

`commandTask` 保留自己的 `RemoteReceiver::Snapshot`，每轮读取全部输入快照后取当前时间。模块检查接收超时，命令层继续检查 `board_config::command_timeout_ms`。启动前或初始化失败时快照无效、离线，不能满足运动使能条件。

`start()` 返回 0 只代表安排线程，UART 结果见 `Remote UART init: ...` 日志和 `Snapshot::state/uart_error`。`prj.conf` 启用 `SKYWALKER_REMOTE_RECEIVER`；静态接收实例与 `__nocache` DMA 缓冲区存活整个固件周期。应用已移除本地重复的 `src/latest.hpp`，`CommandRouter` 使用公共 `include/latest.hpp`。

默认 `app.overlay` 中：

- interboard UART alias 已给出，但仅是建议端口。
- Yaw 电机的型号、ID 和限幅位于 `src/board_config.hpp`，默认 `connections_configured=false`。
- remote-uart 与 RX DMA 由板级 DTS 提供；referee-uart 仍需按真实 DMA 和电气链路配置，不能与板间 UART 冲突。
- `board_config::connections_configured=false`，所以命令层会进入 `ConfigBlocked`。

## 4. 构建

```bash
west build -p -b dm_mc02/stm32h723xx \
  -d build/sentry-gimbal \
  applications/sentry_gimbal

west build -p -b dm_mc02/stm32h723xx \
  -d build/sentry-chassis \
  applications/sentry_chassis
```

如果从 workspace 根构建，则路径使用 `skywalker_code/applications/...`。当前默认 overlay 适合检查工程和线程骨架；真实上机前必须完成上一节的配置。

## 5. 安全启动顺序

1. 先分别构建并检查 `zephyr.dts` / `.config`。
2. 只给两块 MCU 供电，观察两端 heartbeat/peer online。
3. 接入 DR16 和裁判串口，确认 command/permission timestamp 新鲜。
4. 单独接一台执行器，验证本地反馈和恢复。
5. 先保持 `bench_effort_scale` 很低，确认舵向/驱动方向。
6. 才启用四模块和远端 active 命令。

急停必须有硬件断电能力。软件的 `EmergencyStop`、`suspend()`、`stop()` 是控制层保护，不是机械制动。

## 6. 与样例的关系

应用骨架复用正式库，不直接复用其他 sample 的源文件：

```text
applications/sentry_*
  → lib/communication + lib/robotics + lib/control
  → drivers/motor
```

先用 `samples/communication/*`、`samples/robotics/command_safety`、`samples/robotics/yaw_gimbal` 和 `samples/robotics/swerve` 分别验证组件，再合入整机应用，故障定位会更清晰。
