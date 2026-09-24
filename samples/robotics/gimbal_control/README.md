# 遥控双轴电机样例

本样例用一台 GM6020 电流模式 yaw 和一台 DM J4310 MIT pitch 展示跨品牌、同 CAN、一个联动 Group。DR16 接收和控制各有一个应用线程；每条已启动的 CanBus 自有 I/O 线程。

## 配置与接线

电机型号、ID、限幅、零点、协议量程与总线绑定都在 `src/board_config.hpp`；`app.overlay` 不再声明电机设备节点。默认值是供核对的模板：

| 轴 | 电机 | 总线与 ID | 命令上限 |
| --- | --- | --- | --- |
| yaw | GM6020 电流模式 | CAN1 / 7 | 1.5 A |
| pitch | J4310 MIT | CAN1 / 1、Master 0x00 | 1.0 N·m 前馈 |

DM 的 PMAX、VMAX、TMAX 必须与驱动器配置一致。两轴的方向、机械限位和位置环参数也在配置文件中；pitch 的参数只供空载低限幅起步。MC02 的 DR16 UART5 接口来自板级 DTS。电机独立供电，样例没有应用层电源 GPIO；CAN 收发器由 CanBus.start 启动。

`connections_configured` 默认为 `false`，核对实物接线、GM6020 电流模式、编码器零点、DM 量程和机械支撑后才设为 `true`。改 `pitch_can` 为 CAN2 即展示跨 CAN Group：程序先 attach 两轴，再依次 start 两条 CAN，周期末分别 commit。改型号时同时改硬件工厂、控制输出单位与限幅，不能沿用另一品牌的单位。

## 调用与安全行为

应用先 attach/start，再 configure 两个 PositionMotor。遥控左拨杆需先到上或下位，收到新的有效帧后拨到中位，程序才调用一次 `Group.enable()`。右摇杆横向/纵向分别给 yaw/pitch 角速度；两个控制器只暂存安培和牛·米目标，周期末由 CanBus.commit 发布。任一轴反馈、限位、遥控或控制周期异常会撤销整个 Group；故障后需重新经过安全拨杆动作。Group.disable 会立刻关闭软件输出许可，安全帧由 I/O 线程发送。

Limited 轴在禁用且反馈稳定后，使用 GM6020 校准的单圈绝对角或 DM 原生保存零点的位置重建连续参考。切电机电源后重新建立参考，再允许位置控制。pitch 失能可能下坠，须支撑机构；首次测试先验证方向、限位和禁用动作。

`emergencyStopRequested()`、`takeEmergencyResetRequest()` 是板级预留接口，默认返回 false。接入物理复位输入后，释放急停并请求复位会禁用联动组、清除可清故障；仍需稳定反馈和新的一轮安全拨杆动作才会重新使能。

## 构建

```sh
west build -b dm_mc02 samples/robotics/gimbal_control -d ../build/gimbal_rc_test
```

当前默认禁用配置已在 MC02 编译链接通过；未刷写或实机验证。日志打印遥控有效性、组状态、成员状态、故障原因与 CAN 错误。
