# 遥控双轴持续控制样例

一台 GM6020 电流模式 yaw 和一台 DM J4310 MIT pitch 使用两个独立控制器。Group 仅批量启停；一轴掉线不撤销另一轴目标。DR16 接收线程、应用控制线程与每条 CAN 的 I/O 线程分别运行。

型号、ID、限幅、零点、协议量程和总线绑定在 `src/board_config.hpp`：

| 轴 | 默认设备 | 驱动限幅 | 控制输出限幅 |
| --- | --- | --- | --- |
| yaw | GM6020 CAN1 / ID7 | 1.5 A | ±1.2 A |
| pitch | J4310 MIT CAN1 / ID1、Master 0x11 | 1.0 N·m | ±0.5 N·m |

yaw 零点 5670 tick，端点 5670～7440 tick，两端各留 100 tick，目标范围为 5770～7340 tick，对应约 0.0767～1.281 rad。yaw PID 位置环 (20,0.5,1.48)、速度环 (0.43,0.55,0.00005)；pitch 位置环 (0.8,0.1,0)、速度环 (0.03,0.1,0)。DM PMAX 12.5 rad、VMAX 30 rad/s、TMAX 10 N·m 需与驱动器一致。Pitch ±0.5 rad 仍为待实测配置。

`connections_configured` 默认 false。配置合法并启动 CAN 后即可提交运行意图，不等待物理电机响应。把 pitch_can 改为 CAN2 可演示跨 CAN；每周期两个轴分别 update，每个物理 CAN commit 一次，任何单轴错误都不会跳过另一轴。

保持双 Down、全部通道归中 500 ms，再拨左 Middle启动。右摇杆横/纵给 yaw/pitch 角速度，左 Down停止，左 Up禁用。电机不存在或掉线时目标继续被接收；恢复后自动执行最新值。遥控失联按原输入时间停止。物理急停锁存后，显式解除只清急停，仍需新的用户启动。原先温度、实测超速和全组反馈门控已删除，数值、机械范围与PID限幅保留。

Limited轴需要可信位置参考。掉线导致多圈参考失效时只等待本轴参考，不自动重新定义零点。协议可重建的已校准绝对位置由本轴自动恢复。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/gimbal_control -d build/gimbal_control
```

VOFA 使用 telemetry-uart / USART1，115200 JustFloat，每秒11通道：RC有效、左开关、运行请求、急停锁存、实际Active成员数、请求运行成员数、yaw状态、pitch状态、调用错误、yaw提交错误、pitch提交错误。同CAN时最后一项为0。遥控仍使用UART5，console使用USART10。

诊断支持1输入、2执行、3状态暂停。原始时间保留，电机故障不会要求重复解锁。操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。初次运行支撑Pitch并确认实际方向与限位；尚无本次实板验收记录。
