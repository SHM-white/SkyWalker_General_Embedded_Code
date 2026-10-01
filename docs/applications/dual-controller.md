# 双主控应用骨架

applications/sentry_gimbal 与 applications/sentry_chassis 是整机编排入口，复用公开通信、机器人、控制和驱动模块。它们展示线程边界、消息交接和执行器恢复，不是无需按真实接线配置即可直接运行的整机固件。默认连接开关关闭，编译成功不会自动使能电机。

端到端数据流和关键调用见[模块联动](module-integration.md)；单个封装的 API 入口见[封装模块调用示例](../modules/call-examples.md)。

## 分层与线程

~~~text
sentry_gimbal
  RemoteReceiver worker
  referee thread: RefereeReceiver.poll()
  command thread: Remote + Referee + peer snapshot
                  → CommandManager → CommandRouter
  link thread: InterBoardEndpoint.poll()
  gimbal thread: GimbalExecutor
                 → GimbalAxis → PositionMotor → Motor / CanBus

sentry_chassis
  link thread: InterBoardEndpoint.poll()
  chassis thread: ChassisExecutor
                  → SwerveChassis + DjiChassisHardware
                  → Motor / Group / CanBus
~~~

公开库负责单项数据处理和算法；GimbalExecutor、ChassisExecutor、CommandRouter、DjiChassisHardware 属于应用本地编排代码，不是通用公开 API。

## sentry_gimbal

源码位于 applications/sentry_gimbal/src/。

- main.cpp 创建 RemoteReceiver、RefereeReceiver、InterBoardEndpoint，并启动命令、通信和执行线程。
- RemoteReceiver 内部线程独占 DR16 UART；业务线程保留自己的 RemoteReceiver::Snapshot，每轮复制后按消息 stamp 判断新鲜度。
- referee 线程调用 RefereeReceiver::poll()，把 RefereeState 同时交给命令线程和板间端点。
- command 线程构造 CommandInputs，调用 CommandManager::update()，再经 CommandRouter 把 GimbalCommand 放到本地快照、把 ChassisCommand 交给 InterBoardEndpoint::submit()。
- link 线程是 InterBoardEndpoint::poll() 的唯一所有者。
- GimbalExecutor 将单轴 Motor、CanBus、GimbalAxis 和恢复流程封装在一个执行线程内。

默认 board_config::connections_configured=false。打开真实硬件前应核对 CAN/UART 端口、波特率、DMA、急停输入、电机型号与 ID、方向、零点、限幅和参考坐标。云台应用当前 allow_auto=false，因为没有组装 VisionReceiver。

## sentry_chassis

源码位于 applications/sentry_chassis/src/。

- link 线程独占调用 InterBoardEndpoint::poll()；其他线程通过 submit/setStatus/snapshot 交换值副本。
- ChassisExecutor 在 chassis 线程读取端点快照，验证对端 boot、命令时间戳、恢复代次和功率约束。
- DjiChassisHardware 绑定八台 DJI Motor、Group 与所需 CanBus；SwerveChassis 计算四个舵向和驱动目标。
- ChassisPowerLimiter 仅在功率模型标定且预算数据新鲜时参与输出缩放。当前默认 power_model_calibrated=false。
- board_config::connections_configured=false 会把执行器保持在配置阻断状态。

真实应用至少要核对八台电机的总线、ID、电调模式、减速比、限流和舵向零点；四模块几何、轮半径、方向、PID、功率模型及物理急停路径也必须按机器人配置。

## 构建与上机顺序

从 Zephyr workspace 根目录执行：

~~~sh
west build -p -b dm_mc02/stm32h723xx -d build/sentry-gimbal skywalker_code/applications/sentry_gimbal
west build -p -b dm_mc02/stm32h723xx -d build/sentry-chassis skywalker_code/applications/sentry_chassis
~~~

若当前目录就是 skywalker_code，可将源路径改为 applications/sentry_gimbal 和 applications/sentry_chassis。

推荐先分别验证 samples/communication/*、samples/robotics/command_manager、command_safety、yaw_gimbal 和 swerve，再联调双板 heartbeat、时间戳与恢复代次，最后逐项接入真实执行器并从低限幅开始。软件撤销输出不能替代物理急停或确认机构已停止。
