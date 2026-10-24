# 哨兵云台应用

默认使用头部 IMU 模式（`CONFIG_VEHICLE_ENCODER_ONLY=n`），遥控解锁方式与双 Yaw 样例相同：左右拨杆 Down、摇杆和拨轮归中保持500 ms，再将左拨杆切到 Middle。右横杆控制小Yaw，右纵杆控制Pitch（`CONFIG_VEHICLE_LOCK_PITCH=n`），头部稳定后大Yaw根据关节中心偏差跟随。三轴不设置固定锁定，仍保留解锁、反馈有效性、关节限位和回中就绪条件。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d build/sentry_gimbal_can
```

当前 IMU 安装在 Pitch 头部，X前、Y左、Z上，使用中央标定中的单位四元数安装变换和安装确认参数。IMU 模式要求头部姿态与角速度反馈新鲜。若另行设置 `CONFIG_VEHICLE_ENCODER_ONLY=y`，则不启动头部IMU，不受安装确认参数阻挡，但不能保持对地惯性朝向；此时小Yaw用对端实际速度反向补偿，也不会把关节回中就绪伪装成惯性稳定来授权实弹供弹。默认仍关闭发射。若叠加 `referee.conf`，运动仍要求真实裁判许可。

本地执行循环直接读取DR16并运行解锁状态机，与双Yaw样例一致；板间线程转发带原始时间戳的运行状态。链路丢失/对端重启仍撤销运行授权，需重新解锁。关闭 `VEHICLE_SHOOTING` 时仅将Yaw/Pitch加入电机总线并初始化云台控制器，摩擦轮/拨盘配置不再阻止云台启动。日志中 `allowed` 表示遥控解锁结果，`centered/armReady/neutralMs` 显示归中条件，`topology/run/reason/error` 用于区分已解锁但执行被阻止的情况。

总云台继续发送v4底盘行走 `ChassisControl`、操作授权 `OperatorControl` 和独立大Yaw `BigYawRequest`。另一套底盘需返回有效心跳以建立链路。编码器模式即使没有有效 `BigYawFeedback`，也按回中算法正常发送目标速度，不因对端速度反馈失效而清零或重置斜坡。有效速度反馈仅用于小Yaw的实测反向补偿；缺少它时不做该补偿，因此不能保证双Yaw协调回中，小Yaw偏差持续存在时大Yaw可能持续收到转速请求。遥控停止、急停、输入/关节反馈超时、限速和链路上线条件仍生效。本地小Yaw不等待大Yaw上线。

| 用途 | 接口 | 接线/配置 |
| --- | --- | --- |
| 板间通信 | USART10 | TX PE3、RX PE2，双板交叉并共地，460800、8N1 |
| 日志 | UART7 | PE8 TX接适配器RX并共地；控制台输入使用PE7 RX，115200、8N1 |
| 遥控器 | UART5 / DBUS | 接云台板，保留板上DBUS电气通路 |
| 小Yaw / Pitch | CAN1 / CAN2 | 电机ID、限位沿用中央标定 |

当前按对端安装方向启用 `CONFIG_VEHICLE_BIG_YAW_REVERSE=y`，将 `0x0402` 的大Yaw目标速度反号；收到的 `0x0403` 实测速度使用同一符号转换后参与小Yaw补偿，要求对端反馈与命令采用相同的正方向。`request_mrad_s` 显示转换后的发送目标。大Yaw回中目标限幅为±0.3 rad/s，斜坡为0.4 rad/s²；底盘拨轮旋转 `0x0101.wz_rad_s` 独立限幅为±0.5 rad/s，左摇杆前后/横移分别为±0.3 m/s。对端自身的限幅由另一套底盘程序决定。

USART10的TX/RX DMA使用DMAMUX通道3/4，请求号119/118；UART7使用轮询日志，移除继承的DMA配置。串口覆盖仅作用于本应用。启动日志显示 `reference=encoders` 或 `reference=head IMU`；诊断含 `rcFresh/allowed`、`followReady/peerRateValid/centerReason` 及链路错误。

UART7当前专用于日志；共享设备树中的 `vision-uart` 别名仍指向UART7。默认配置未启动视觉接收器；启用 `vision_observe.conf` 或 `vision_execute.conf` 前，需将视觉串口及其DMA配置迁移到其他可用接口，避免与控制台争用。

接入IMU时，安装参考与变换仍取中央 `include/robotics/vehicle/calibration.hpp`。CAN1/CAN2 为现有电机总线，IMU 独占云台板 CAN3；底盘板CAN分配以对端程序为准。

## 当前主动模式接入

- 将 IMU CANH/CANL 接云台板 CAN3，按实际总线连接公共参考地与终端电阻。
- 厂家上位机选择 CAN、主动上报、1 Mbps，从机地址 `0x1`、主机地址 `0x11`。
- 开启 CAN 输出、四元数、角速度、加速度/温度（输出位图 `0x0D`），先设置 5ms 上报周期并保存。若启用温控，等待模块的温控输出门控解除。
- 非默认地址可修改 `CONFIG_SKYWALKER_IMU_DM_CAN_SLAVE_ID` 和 `CONFIG_SKYWALKER_IMU_DM_CAN_FEEDBACK_ID`；非默认速率修改设备树 `&can3` 的 `bitrate`。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d build/sentry_gimbal_can -- -DEXTRA_CONF_FILE=imu_can_active.conf
```

每秒 `dm_imu_can` 应显示 `mode=active slave=1 feedback=11`，`rx/updates` 增长、`tx=0`。快照全部新鲜时 `fresh_mask=15`。接收断流时字段到期失效，不保活旧姿态。

## 切换模式

模块先在上位机设置并保存被动应答，再编译：

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d build/sentry_gimbal_can_passive -- -DEXTRA_CONF_FILE=imu_can_passive.conf
```

Pitch 默认已解锁，无需额外叠加 `pitch_free.conf`。`imu_can_active.conf`、`imu_can_passive.conf` 和 `imu_rs485.conf` 均显式关闭编码器模式。切换配置不自动修改 IMU 内部模式。

恢复原 RS485 接线与驱动：

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d build/sentry_gimbal_rs485 -- -DEXTRA_CONF_FILE=imu_rs485.conf
```

`vision_execute.conf` 和 `shooting.conf` 也会关闭编码器模式，恢复真实IMU要求。CAN 两种模式的共用解码依据工作区 V1.3 手册；被动请求格式由厂家 MC02 示例补充。未进行硬件通信验证。惯性模式的安装标定、`imu_mounting_confirmed` 以及执行约束仍需按实际机构配置；通信接入不代表已完成惯性云台标定。

[CAN 协议、接口与诊断](../../docs/modules/drivers/imu.md#dm-imu-l1-can) · [无电机的双 IMU 观测样例](../../samples/imu/dual_imu/README.md#外置-imu-改用-can)
