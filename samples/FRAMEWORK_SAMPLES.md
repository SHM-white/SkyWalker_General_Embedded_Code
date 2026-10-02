# 独立上机微型项目

这些 samples 是可单独编译、刷入达妙 MC02 的小固件。电机样例使用 `Motor`、`CanBus`，需要联动时使用 `Group`；闭环样例另使用 `PositionMotor` 或 `VelocityMotor`。上机前核对 CAN 接线、电机模式、ID、减速比和限幅。

| 项目 | 真实硬件/观察内容 | 操作 |
| --- | --- | --- |
| [DR16](communication/dr16/README.md) | 接遥控 UART，观察通道、拨杆、鼠标键盘、掉线 | 遥控器实际操作 |
| [裁判](communication/referee/README.md) | 接裁判 User 串口，观察 CRC、许可、额度、缓冲和过期 | 插拔真实串口/改变供电许可 |
| [板间](communication/interboard/README.md) | 两块板互接 UART，观察在线、boot/generation、命令超时 | 唯一当前板间协议；独立诊断构建定时暂停生产或改变恢复上下文 |
| [命令与安全](robotics/command_safety/README.md) | 真实 DR16 -> 意图 -> 安全 -> 机器人命令；不接电机 | 安全档归中后遥控解锁，左 Down 停机 |
| [电机恢复](motor/recovery/README.md) | 单 DJI 或 DM MIT 电机，主控存活时单独断电恢复 | `e` 运行，空格暂停，`!` 急停，`r` 复位 |
| [多电机拓扑](motor/mixed_topology/README.md) | DJI 同帧独立组、DM 共用 Master ID、跨 CAN 联动及故障隔离 | 构建时选择拓扑；遥控解锁后用左纵杆、右纵杆、拨轮保持各组运行 |
| [小 Yaw](robotics/yaw_gimbal/README.md) | GM6020 小云台，Hold/Rate/AbsoluteAngle | 遥控解锁；右开关选择速率/Hold/绝对角，安全档复位 |
| [单舵轮](robotics/swerve/README.md) | 一套 GM6020 舵向 + M3508 驱动，双 CAN | 遥控解锁；左杆平移，拨轮旋转 |
| [双轴云台](robotics/gimbal_control/README.md) | DJI + DM 双轴 Group，观察跨品牌联动 | 按样例 README 配线和使能 |

机器人机构与多电机拓扑使用 [统一遥控操作](robotics/common/REMOTE_CONTROL.md)，console 与 VOFA 只用于观察。单电机模块测试保留原有方式：`recovery` 继续使用 `e`、空格、`!`、`r`；其余定时自动运行的单电机样例保留既定轨迹及 VOFA 通道。

通用构建/刷写方式（替换 sample 路径与 build 目录）：

```sh
west build -b dm_mc02/stm32h723xx samples/communication/dr16 -d build/bench_dr16
west flash -d build/bench_dr16
```

直接驱动样例位于 `motor/dji_unified`、`motor/dm_mit_control`、`motor/dm_velocity_control`、`motor/dm_position_control`；闭环样例位于 `motor/dji_speed_control`、`motor/dji_position_control`、`motor/m2006_speed_control`、`motor/dm_mit_velocity_control`、`motor/dm_mit_position_control`。这些样例的电机配置已移入各自的 `main.cpp`，overlay 不再创建旧 motor 设备节点。DM 样例按原台架流程使能 XT30_1，并保留原有 PID、自动轨迹、限幅及 VOFA 通道；已迁移的示例仍需实机回归验证。
