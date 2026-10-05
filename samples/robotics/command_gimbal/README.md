# 仲裁接入小 Yaw / Pitch 持续控制

`RemoteReceiver → RemoteSource → CommandManager` 持续生产目标，5ms执行线程消费快照，10ms仲裁服务独立运行。每轴独立等待反馈与恢复，电机状态不会撤销遥控授权。

| 资源 | 默认分配 |
| --- | --- |
| 小Yaw | GM6020电流模式 CAN1 / ID7 |
| Pitch | DM J4310 MIT CAN2 / ID1、Master0x11 |
| DR16 | MC02 remote-uart / UART5 |
| VOFA | telemetry-uart / USART1，115200 JustFloat |
| console | USART10日志 |

`src/board_config.hpp` 使用中央 `include/robotics/vehicle/calibration.hpp`。连接确认默认关闭。Yaw驱动1.5A、控制±1.2A，Pitch驱动1.0N·m、控制±0.5N·m；Yaw零点5670tick、行程5770～7340tick，Pitch±0.5rad仍需实测。协议量程与实际设备必须一致。

应用attach/start并配置执行器后，每周期两个轴各自update，各物理CAN分别commit。总线Recovering、单轴Fault/Offline不会暂停上层目标生产。控制封装在本轴实际可计算时重置历史并执行最新目标；参考不足只等待本轴。停止、急停、输入超时以及明确启用的裁判权限仍有效。

| 配置 | 功能 |
| --- | --- |
| 默认 | 物理遥控与机械双轴，左Up禁用 |
| referee.conf | 裁判云台输出权限 |
| vision_observe.conf | 视觉观测，手动执行 |
| vision_execute.conf | 视觉、真实头部IMU与惯性坐标转换 |
| diagnostic.conf | 暂停原始输入生产 |

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal_vision -- -DEXTRA_CONF_FILE=vision_execute.conf
```

惯性模式分别传递yaw_output_valid和pitch_output_valid，缺少一个轴的数学输入不禁止另一轴输出。头部参考与视觉参考必须匹配，旧输入原始年龄不会因仲裁、通信或重复读取变新。AB缺少远端epoch元数据时按明确参考约定处理，缺少真实弹速/弹数时反馈TX关闭。

VOFA每50ms输出16通道：原始源序号/年龄、命令序号/年龄、RunState、WaitReason、实际Active轴数、状态年龄、最后命令序号、Yaw/Pitch实角、Yaw/Pitch目标、两CAN错误、周期超限。ready/armed为诊断。裁判占用USART1时不初始化VOFA。

诊断支持1输入、2执行、3状态、4头部、5视觉、6权限暂停；只启用已配置来源对应项。操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。电机掉电恢复无需新解锁，输入失联和用户停止仍停止输出。Pitch需要机械支撑；尚无本次实板验收记录。
