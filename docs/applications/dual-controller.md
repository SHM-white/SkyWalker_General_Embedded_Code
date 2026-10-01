# 双主控应用骨架

applications/sentry_gimbal 与 applications/sentry_chassis 是整机编排入口，复用公开通信、命令仲裁、机器人、控制和驱动模块。它们展示线程边界与恢复流程；默认连接门禁关闭，不能视作完成真实接线后的成品固件。

端到端消息路线见[模块联动](module-integration.md)，命令服务 API 见[命令来源与后台服务](../modules/robotics/command-service.md)。

## 当前线程与数据流

~~~text
sentry_gimbal
  RemoteReceiver worker
  CommandManager worker
    RemoteSource + RefereePermissionSource
      → CommandArbiter → CommandSnapshot
  link thread: snapshot → InterBoardEndpoint.submit / poll
  gimbal thread: current → GimbalExecutor → GimbalAxis → Motor / CanBus

sentry_chassis
  link thread: InterBoardEndpoint.poll
  chassis thread: ChassisExecutor
    → SwerveChassis + DjiChassisHardware
    → Motor / Group / CanBus
~~~

云台 app 没有单独的裁判线程：RefereePermissionSource 在 CommandManager worker 中调用 RefereeReceiver::poll。当前 app 不注册 VisionSource，因此 board_config 将 allow_auto 设为 false。CommandManager 需要先在启动线程完成来源注册和许可绑定，然后调用 start；消费者读取的 CommandSnapshot 与 RobotCommand 是值副本。

## sentry_gimbal

源码位于 applications/sentry_gimbal/src/。

- main.cpp 静态构造 RemoteReceiver、RefereeReceiver、InterBoardEndpoint、RemoteSource、RefereePermissionSource 和 CommandManager。
- main() 依次注册遥控来源、绑定裁判许可、启动命令服务；默认 referee_version=Unspecified，需按实际裁判 profile 和 UART 完成板级配置后才会产生有效权限。
- linkTask 读取完整 CommandSnapshot，将最终底盘命令和裁判状态提交给 InterBoardEndpoint，再调用 poll 推进所选传输后端。两侧 `board_config.hpp` 的 `interboard_transport.kind` 可选择 UART、RS485 或 CAN；默认 UART，RS485 使用 USART2，CAN 使用独占 CAN3，详见[传输配置](../modules/communication/interboard-transports.md)。
- gimbalTask 读取 CommandManager::current()，交给应用私有 GimbalExecutor。
- GimbalExecutor 管理单轴 Motor、CanBus、GimbalAxis、恢复与提交。

board_config::connections_configured 默认 false。真实启动前核对 UART/CAN、DMA、电机模式与 ID、控制方向、零点、限幅、急停和位置参考。

## sentry_chassis

源码位于 applications/sentry_chassis/src/。

- linkTask 是 InterBoardEndpoint::poll() 的唯一调用方。
- chassisTask 持有 ChassisExecutor，由它读取端点快照并校验对端身份、命令年龄、恢复代次及功率约束。
- DjiChassisHardware 绑定八台 DJI Motor、Group 和最多两条物理 CanBus。
- SwerveChassis 计算四个舵向与驱动目标；ChassisPowerLimiter 仅在功率模型已标定且数据新鲜时参与输出缩放。
- board_config::connections_configured 与 power_model_calibrated 默认关闭。

## 构建与上机顺序

从 west workspace 根目录执行：

~~~sh
west build -p -b dm_mc02/stm32h723xx -d build/sentry-gimbal skywalker_code/applications/sentry_gimbal
west build -p -b dm_mc02/stm32h723xx -d build/sentry-chassis skywalker_code/applications/sentry_chassis
~~~

如当前目录就是 skywalker_code，可将工程路径改成 applications/sentry_gimbal 和 applications/sentry_chassis。先分别验证通信、命令管理、云台轴和舵轮样例，再联调双板。软件状态不代替实物方向、限幅、停机和物理急停验证。
