# 遥控双轴电机样例

本样例用一台 GM6020 电流模式 yaw 和一台 DM J4310 MIT pitch 展示跨品牌、同 CAN、一个联动 Group。DR16 接收由 `RemoteReceiver` 内部线程管理，控制使用独立应用线程；每条已启动的 CanBus 自有 I/O 线程。

## 配置与接线

电机型号、ID、限幅、零点、协议量程与总线绑定都在 `src/board_config.hpp`；`app.overlay` 不再声明电机设备节点。配置采用指定的 GM6020 小 yaw 与同一颗 DM-J4310 的调参结果：

| 轴 | 电机与 CAN | 驱动限幅 | 控制器输出限幅 | 安全阈值 |
| --- | --- | --- | --- | --- |
| yaw | GM6020 电流模式，CAN1 / ID 7 | 1.5 A | ±1.2 A | 4 rad/s、70 °C |
| pitch | J4310 MIT，CAN1 / ID 1、Master 0x11 | 1.0 N·m | ±0.5 N·m | 10 rad/s、60 °C |

yaw 编码器零点为 5670 tick，机械端点为 5670～7440 tick。两端各留 100 tick，实际目标与反馈允许范围为 5770～7340 tick，换算成零点相对角约为 0.0767～1.281 rad。yaw 位置环为 (20, 0.5, 1.48)，速度环为 (0.43, 0.55, 0.00005)；pitch 位置环为 (0.8, 0.1, 0)，速度环为 (0.03, 0.1, 0)。两个控制环均按 5 ms 更新，接受 1～20 ms 的实测周期。

DM 的 PMAX 12.5 rad、VMAX 30 rad/s、TMAX 10 N·m 必须与驱动器实际配置一致；沿用该电机内部已保存的零点，不写软件 tick 零点。pitch 的 -0.5～0.5 rad 机械范围仍是待实测的样例值，不能据此启用实机。MC02 的 DR16 UART5 接口来自板级 DTS。电机独立供电，样例没有应用层电源 GPIO；CAN 收发器由 CanBus.start 启动。

`connections_configured` 仍为 `false`，核对实物接线、两轴方向与机械限位、GM6020 电流模式和编码器零点、DM 的 Master ID / 量程 / 已保存零点及 pitch 机械支撑后才设为 `true`。改 `pitch_can` 为 CAN2 即展示跨 CAN Group：程序先 attach 两轴，再依次 start 两条 CAN，周期末分别 commit。改型号时同时改硬件工厂、控制输出单位与限幅，不能沿用另一品牌的单位。

## 调用与安全行为

应用先 attach/start，再 configure 两个 PositionMotor。遥控左拨杆需先到上或下位，收到新的有效帧后拨到中位，程序才调用一次 `Group.enable()`。右摇杆横向/纵向分别给 yaw/pitch 角速度；两个控制器只暂存安培和牛·米目标，周期末由 CanBus.commit 发布。任一轴反馈、限位、遥控或控制周期异常会撤销整个 Group；故障后需重新经过安全拨杆动作。Group.disable 会立刻关闭软件输出许可，安全帧由 I/O 线程发送。

Limited 轴在禁用且反馈稳定后，使用 GM6020 校准的单圈绝对角或 DM 原生保存零点的位置重建连续参考。切电机电源后重新建立参考，再允许位置控制。pitch 失能可能下坠，须支撑机构；首次测试先验证方向、限位和禁用动作。

`emergencyStopRequested()`、`takeEmergencyResetRequest()` 是板级预留接口，默认返回 false。接入物理复位输入后，释放急停并请求复位会禁用联动组、清除可清故障；仍需稳定反馈和新的一轮安全拨杆动作才会重新使能。

## 构建

```sh
west build -b dm_mc02 samples/robotics/gimbal_control -d ../build/gimbal_rc_test
```

运行状态每秒通过独立的 `telemetry-uart`（MC02 USART1）发送一帧 VOFA+ JustFloat；遥控接收仍使用 UART5，控制台日志仍使用 USART10。VOFA+ 串口设为 115200 baud、JustFloat 协议。按顺序为 11 个通道：遥控帧有效（0/1）、左拨杆位置、允许重新使能（0/1）、急停锁存（0/1）、联动组运行（0/1）、联动组等待使能（0/1）、yaw 电机状态、pitch 电机状态、联动组故障原因、yaw CAN 最近错误码、pitch CAN 最近错误码。同 CAN 配置下最后一个通道固定为 0。VOFA 初始化或发送失败时，错误写入控制台日志。

本次 VOFA 改动已在 MC02 编译链接通过；未刷写或实机验证。
