# 仲裁接入小 Yaw / Pitch 实板样例

真实 `RemoteReceiver → RemoteSource → CommandManager` 生成机械角速度命令，5 ms 执行线程消费快照，10 ms 仲裁服务独立运行。仅使用物理摇杆、三档开关和拨轮；console 与 VOFA 为观察输出。完成安全档归中解锁后才允许执行，失联或机构失权后须重新解锁。

## 接线与标定

| 资源 | 默认分配 |
| --- | --- |
| 小 Yaw | GM6020 电流模式，CAN1 / ID 7 |
| Pitch | DM J4310 MIT，CAN2 / ID 1、Master 0x11 |
| DR16 | MC02 remote-uart / UART5 |
| VOFA | telemetry-uart / USART1，115200 JustFloat，仅输出 |
| console | USART10 日志 |

配置见 `src/board_config.hpp`，中央硬件与标定参数见 `include/robotics/vehicle/calibration.hpp`。连接确认默认关闭，未确认时只运行接收与观察，不启动电机 CAN。保持既有 Yaw 1.5 A 驱动 / ±1.2 A 控制器、Pitch 1.0 N·m 驱动 / ±0.5 N·m 控制器限幅。Yaw 零点 5670 tick，安全行程 5770～7340 tick；独立中心约 0.679 rad。Pitch ±0.5 rad 仍需实测，DM PMAX/VMAX/TMAX 与驱动器配置必须一致。

应用先 attach/start，再配置执行器；执行器暂存目标，执行线程每条物理 CAN 提交一次。同 CAN 与跨 CAN 的两轴仍是一个故障组。任何轴或提交失效都会撤销云台输出。恢复先重建参考、控制历史和独立恢复代次，再等待准备边界之后的原始输入。仲裁序号或时间戳不能代替真正的新 RC 帧。

## 配置与操作

| 配置 | 链路与行为 |
| --- | --- |
| 默认 | 物理遥控与机械双轴；左 Up 禁用 |
| referee.conf | 追加真实裁判云台权限 |
| vision_observe.conf | 接收视觉与裁判，仍保持手动执行 |
| vision_execute.conf | 视觉与真实头部 IMU 经 InertialGimbalAdapter 执行；解锁后可左 Up |
| diagnostic.conf | 输入暂停演练，普通构建不注入 |

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal_vision -- -DEXTRA_CONF_FILE=vision_execute.conf
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

[统一遥控操作](../common/REMOTE_CONTROL.md)规定解锁、Safe、100 ms 失联和安全档复位。物理急停仍优先。默认控制最大角速度为 Yaw 1 rad/s、Pitch 0.8 rad/s，机构断电或失能时须支撑 Pitch。

诊断支持 1 输入、2 执行、3 状态、4 头部、5 视觉、6 权限，后三级只用于已配置相应来源的构建。稳定 Active 3 秒后暂停指定链路 1.5 秒，原始数据和状态年龄不会刷新；遥控管理继续处理停止及取消。移除了 VOFA `key=value` 回调与文本控制缓冲。

VOFA 50 ms 一帧，16 通道依次为原始源序号/年龄、命令序号/年龄、RunState、WaitReason、恢复代次、状态年龄、最后执行命令序号、Yaw/Pitch 实际角、Yaw/Pitch 目标、两条 CAN 错误与周期超限。状态过期时有效 Ready/Armed 撤销。裁判占用 USART1 时不初始化 VOFA，console 继续输出观察值。

头部 IMU 使用 USART2 RX DMA 8，保留板载 SPI2 1/2、RC 6、视觉 5/7。AB 没有远端 epoch 元数据，仍使用明确的头部参考约定；IMU 参考变化使旧视觉会话失效。缺少真实弹速/弹数时 AB 反馈 TX 保持关闭。

实板记录应包含接线、ID/模式/方向/零点/限位、来源年龄、参考与机构恢复代次、停止延迟、控制耗时及组故障范围。编译不能替代这些记录。
