# 独立上机微型项目

这些 samples 是可单独编译、刷入达妙 MC02 的小固件。电机样例使用 `Motor`、`CanBus`，需要联动时使用 `Group`；闭环样例另使用 `PositionMotor` 或 `VelocityMotor`。上机前核对 CAN 接线、电机模式、ID、减速比和限幅。

| 项目 | 真实硬件/观察内容 | 操作 |
| --- | --- | --- |
| [DR16](communication/dr16/README.md) | 接遥控 UART，观察通道、拨杆、鼠标键盘、掉线 | 遥控器实际操作 |
| [裁判](communication/referee/README.md) | 接裁判 User 串口，观察 CRC、许可、额度、缓冲和过期 | 插拔真实串口/改变供电许可 |
| [板间](communication/interboard/README.md) | 两块板互接 UART，观察在线、boot/generation、命令超时 | 控制台 `p` 暂停命令生产，`g` 更改恢复上下文 |
| [命令与安全](robotics/command_safety/README.md) | 真实 DR16 -> 意图 -> 安全 -> 机器人命令；不接电机 | RC 拨杆；控制台 `!` 急停、`r` 复位 |
| [电机恢复](motor/recovery/README.md) | 单 DJI 或 DM MIT 电机，主控存活时单独断电恢复 | `e` 运行，空格暂停，`!` 急停，`r` 复位 |
| [多电机拓扑](motor/mixed_topology/README.md) | DJI 同帧独立组、DM 共用 Master ID、跨 CAN 联动及故障隔离 | 构建时选择四种拓扑之一；控制台显式使能 |
| [小 Yaw](robotics/yaw_gimbal/README.md) | GM6020 小云台，Hold/Rate/AbsoluteAngle | `e/a/d/h/0/1`，空格暂停，`!` 急停，`r` 复位 |
| [单舵轮](robotics/swerve/README.md) | 一套 GM6020 舵向 + M3508 驱动，共享 CAN | `w/s/a/d/q` 选择平移/旋转，`e` 使能 |
| [双轴云台](robotics/gimbal_control/README.md) | DJI + DM 双轴 Group，观察跨品牌联动 | 按样例 README 配线和使能 |

控制台样例沿用板定义的 USART10 / 115200 baud。`recovery` 和 `mixed_topology` 均要求显式使能；真实掉电后，反馈恢复不会重新授权运动，需再次按 `e`（多电机独立组按 `1`/`2`）。`r` 只清除可清故障，不使能。原有定时自动运行的单电机样例保留其既定目标和 VOFA 通道，驱动故障会撤销输出。

通用构建/刷写方式（替换 sample 路径与 build 目录）：

```sh
west build -b dm_mc02/stm32h723xx samples/communication/dr16 -d build/bench_dr16
west flash -d build/bench_dr16
```

直接驱动样例位于 `motor/dji_unified`、`motor/dm_mit_control`、`motor/dm_velocity_control`、`motor/dm_position_control`；闭环样例位于 `motor/dji_speed_control`、`motor/dji_position_control`、`motor/m2006_speed_control`、`motor/dm_mit_velocity_control`、`motor/dm_mit_position_control`。这些样例的电机配置已移入各自的 `main.cpp`，overlay 不再创建旧 motor 设备节点。DM 样例按原台架流程使能 XT30_1，并保留原有 PID、自动轨迹、限幅及 VOFA 通道；已迁移的示例仍需实机回归验证。
