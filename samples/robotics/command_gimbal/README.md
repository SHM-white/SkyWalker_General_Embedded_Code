# 仲裁接入小 Yaw / Pitch 兼容样例

更新日期：2026-10-25。本目录保留仲裁、裁判和视觉宏组合教学用途；电机参数已共享，但物理overlay、USART1 VOFA和USART10 console仍沿用旧样例分配，尚未统一为installed/DAPLink诊断。实际机械上车优先选择 [小Yaw单轴](../../bringup/gimbal_yaw_encoder/README.md)、[Pitch单轴](../../bringup/gimbal_pitch_encoder/README.md)、[机械双轴](../../bringup/gimbal_joint_pair/README.md)；当前CAN头部惯性链路使用 [头部IMU观察](../../bringup/gimbal_head_imu/README.md)、[惯性Yaw](../../bringup/gimbal_inertial_yaw/README.md)、[惯性双轴](../../bringup/gimbal_inertial_pair/README.md)。完整入口见 [上车目录](../../bringup/README.md)。

`RemoteReceiver → RemoteSource → CommandManager` 持续生产目标，5ms执行线程消费快照，10ms仲裁服务独立运行。每轴独立等待反馈与恢复，电机状态不会撤销遥控授权。

| 资源 | 默认分配 |
| --- | --- |
| 小Yaw | GM6020电流模式 CAN1 / ID7 |
| Pitch | DM J4310 MIT CAN2 / ID1、Master0x00 |
| DR16 | MC02 remote-uart / UART5 |
| VOFA | telemetry-uart / USART1，115200 JustFloat |
| console | USART10日志 |

`src/board_config.hpp` 直接转接共享 [gimbal_profile.hpp](../../../include/robotics/vehicle/gimbal_profile.hpp)，硬件标定来自 [calibration.hpp](../../../include/robotics/vehicle/calibration.hpp)，不从其他sample复制PID。当前connections_confirmed为true，这是已有代码值，不表示本次实板已确认。当前Yaw驱动1.5A、控制±1.2A，Pitch驱动1.0N·m、控制±0.8N·m；Yaw零点5670tick、行程5770～7340tick，Pitch驱动坐标范围为[2.50,3.50]rad。修改共享参数后需要重建所有受影响入口；协议量程、机械限位和实际设备仍须一致。

应用attach/start并配置执行器后，每周期两个轴各自update，各物理CAN分别commit。总线Recovering、单轴Fault/Offline不会暂停上层目标生产。控制封装在本轴实际可计算时重置历史并执行最新目标；参考不足只等待本轴。停止、急停、输入超时以及明确启用的裁判权限仍有效。

| 配置 | 功能 |
| --- | --- |
| 默认 | 物理遥控与机械双轴，左Up禁用 |
| referee.conf | 裁判云台输出权限 |
| vision_observe.conf | 视觉观测，手动执行 |
| vision_execute.conf | 旧RS485头部IMU路径上的视觉与惯性坐标转换，尚未迁移到CAN实车基线 |
| diagnostic.conf | 暂停原始输入生产 |

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal_vision -- -DEXTRA_CONF_FILE=vision_execute.conf
```

本目录的 `vision_execute.conf` 仍选择 `CONFIG_SKYWALKER_IMU_DM_RS485`，入口创建 `DmImuRs485Source(board_config::head_uart,...)`，本地head_uart正确指向 `DT_ALIAS(rs485_2)`；它没有随inertial_gimbal的CAN迁移一起改成CAN Source。RS485-2与当前实车CAN3头部IMU是不同电气/驱动路径，不能把CAN设备直接塞进head_uart，也不能以此旧组合宣称CAN视觉执行已具备。该组合还启用USART1裁判与UART7视觉，分别与新实车基线的USART1/USART10板间、UART7诊断资源不同，启用前须按本目录overlay确认单一所有者。

惯性模式分别传递yaw_output_valid和pitch_output_valid，缺少一个轴的数学输入不禁止另一轴输出。头部参考与视觉参考必须匹配，旧输入原始年龄不会因仲裁、通信或重复读取变新。AB缺少远端epoch元数据时按明确参考约定处理，缺少真实弹速/弹数时反馈TX关闭。

本旧样例VOFA仍经USART1每50ms输出16通道：原始源序号/年龄、命令序号/年龄、RunState、WaitReason、实际Active轴数、状态年龄、最后命令序号、Yaw/Pitch实角、Yaw/Pitch目标、两CAN错误、周期超限。ready/armed为诊断。裁判占用USART1时不初始化VOFA。共享diagnostic_output变量不会切换本样例观测口；需要单DAPLink口的Log/VOFA模式时运行前述bringup入口。

诊断支持1输入、2执行、3状态、4头部、5视觉、6权限暂停；只启用已配置来源对应项。操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。电机掉电恢复无需新解锁，输入失联和用户停止仍停止输出。Pitch需要机械支撑；尚无本次实板验收记录。
