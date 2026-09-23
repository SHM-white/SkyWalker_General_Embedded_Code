# 遥控器小 yaw + pitch 双轴测试

只保留两个任务：`remoteTask` 接收 DR16；`gimbalTask` 读取遥控快照，控制小 yaw 和 pitch。没有裁判系统、板间通信、底盘、大 yaw、发射机构或键鼠控制。

## 使用前配置

当前接线只是**禁用的模板**，不能直接驱动实物：

| 轴 | 模板电机 | CAN / ID | 示例角度范围 |
|---|---|---|---|
| 小 yaw | GM6020 电流模式 | CAN1 / 1 | -1 ～ 1 rad |
| pitch | GM6020 电流模式 | CAN2 / 1 | -0.5 ～ 0.5 rad |

1. 在 `app.overlay` 按实物填写两轴电机型号、总线、ID、减速比、编码器零点和输出限制，确认后启用节点。MC02 的遥控接口沿用板级 `remote-uart`（UART5，PD2 RX，100000、8E1、RX DMA）；确认接收机接线与电平。
2. 在 `src/board_config.hpp` 设置每轴后端、方向、机械限位、速度和 PID。DM 电机还需更换设备树节点为对应 MIT 模式配置；输出单位为 N·m，DJI 为 A，不能直接套用参数。
3. 两轴默认 `Limited + DriverContinuous`。限位必须对应校准后的驱动坐标，不是上电位置的相对角度。示例 PID 不代表带载 pitch 已调好。
4. 确认后将 `connections_configured` 设为 `true`。默认 false 会让控制任务记录配置错误后退出，两轴不使能。

现有 `DjiMotorBackend` 独占一条 CAN 总线，所以两个 DJI 轴使用不同 CAN 控制器。若实物两个 DJI 电机共用一条 CAN，必须改成共用 DJI Bus 的组帧发送方案；不能仅把两个节点都改为 CAN1，否则第二个后端会配置失败，两轴均不会启动。

代码为每轴只构造所选的一种后端。当前 Kconfig 保留 DJI 和 DM 支持，以便选择实际电机。

## 控制行为

- 上电后，等待两个轴反馈正常，先将左拨杆置上位或下位，再拨到中位使能。
- 右摇杆横向控制小 yaw 角速度，纵向控制 pitch 角速度。默认最大速度均为 0.3 rad/s，方向可独立调整。
- 摇杆回中保持目标角度，左拨杆退出中位撤销两轴输出。右拨杆不切换输入源。
- 遥控超时、反馈异常、限位越界或控制周期异常时撤销两轴输出；恢复后需要再次退出中位再使能。
- `emergencyStopRequested()` 和 `takeEmergencyResetRequest()` 是预留接口，默认返回 false，尚未接入物理急停。接入后急停会锁存，释放并显式复位后才能重新使能。

pitch 在应用内复用 `YawGimbal` 作为单轴控制器，`Axis::update()` 将两轴各自的角速度放进其实际读取的 `yaw_rate_rad_s` 字段。每轴都有独立的位置环、电机实例和恢复状态；这不是 IMU 稳定控制。

失能会撤销力矩，pitch 应有机械支撑以免下坠。首次测试使用低速、小行程，确认方向和限位。

## 构建与运行

在仓库目录中运行：

```sh
west build -b dm_mc02 samples/robotics/gimbal_control -d ../build/gimbal_rc_test
west flash -d ../build/gimbal_rc_test
```

本次修改已通过 MC02 默认禁用模板的编译和链接，未刷写或实机验证。启动日志显示两轴配置返回值；配置成功后每秒输出遥控有效性、拨杆、重新使能许可、急停与两轴状态。`rc=0` 先检查接收机，配置错误先检查设备节点和 `connections_configured`；恢复错误参考对应电机反馈和 CAN 状态。
