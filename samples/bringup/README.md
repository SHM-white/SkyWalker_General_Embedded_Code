# 实车逐模块上车入口

本目录以独立工程选择启动对象，共享 `config/vehicle/installed/` 物理资源及 `include/robotics/vehicle/` 标定/PID。各main自行装配；不使用角色宏、额外conf或公共整车run。当前交付覆盖诊断、机械/惯性云台、板间、大Yaw、双Yaw、舵轮及手动整车链；视觉、实测功率和有载发射仍按原方案后续批次推进。

| 顺序 | 云台工程 | 配套底盘工程 | 本次目的 |
| --- | --- | --- | --- |
| 0 | [gimbal_io](gimbal_io/README.md) | [chassis_io](chassis_io/README.md) | 诊断通路，无执行器 |
| 1 | [gimbal_remote](gimbal_remote/README.md) | — | 真实RC输入观察 |
| 1 | [gimbal_head_imu](gimbal_head_imu/README.md) | — | CAN3头IMU，无电机 |
| 2 | [gimbal_yaw_encoder](gimbal_yaw_encoder/README.md) | — | CAN1小Yaw单轴 |
| 2 | [gimbal_pitch_encoder](gimbal_pitch_encoder/README.md) | — | CAN2 Pitch单轴，需支撑 |
| 3 | [gimbal_joint_pair](gimbal_joint_pair/README.md) | — | 双轴机械闭环 |
| 4 | [gimbal_inertial_yaw](gimbal_inertial_yaw/README.md) | — | 仅小Yaw惯性控制 |
| 4 | [gimbal_inertial_pair](gimbal_inertial_pair/README.md) | — | 双轴头部惯性控制 |
| 5 | [gimbal_link_observe](gimbal_link_observe/README.md) | [chassis_link_observe](chassis_link_observe/README.md) | 无输出协议观察 |
| 6 | [gimbal_big_yaw_command](gimbal_big_yaw_command/README.md) | [chassis_big_yaw](chassis_big_yaw/README.md) | 关节侧±0.1rad/s，轮控不启动 |
| 7 | [gimbal_dual_yaw](gimbal_dual_yaw/README.md) | [chassis_big_yaw](chassis_big_yaw/README.md) | 双Yaw回中，不平移 |
| 8 | [gimbal_drive_command](gimbal_drive_command/README.md) | [FL](chassis_module_fl/README.md)、[FR](chassis_module_fr/README.md)、[RL](chassis_module_rl/README.md)、[RR](chassis_module_rr/README.md) | 逐轮固定工程，仅注册本轮 |
| 9 | [gimbal_drive_command](gimbal_drive_command/README.md) | [chassis_four_modules](chassis_four_modules/README.md) | 四轮仅前后低速直行 |
| 9 | [gimbal_drive_command](gimbal_drive_command/README.md) | [chassis_drive_link](chassis_drive_link/README.md) | 完整低速车体运动 |
| 10 | [sentry_gimbal](../../applications/sentry_gimbal/README.md) | [sentry_chassis](../../applications/sentry_chassis/README.md) | 各板独立装配的手动整车 |

双Yaw阶段云台回中与底盘大Yaw入口的上限均为±0.1rad/s，超过接收限制的请求会失能。所有独立阶段都保留真实零点、方向、ID和PID，只减小动作幅度/电流。单Yaw控制不构造Pitch电机；Pitch单轴须机械支撑。

## 构建与操作

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_yaw_encoder -d ../build/bringup/gimbal_yaw_encoder
west flash -d ../build/bringup/gimbal_yaw_encoder
```

每工程一个固定build目录，不叠加EXTRA_CONF_FILE。先对照[共享资源](../../config/vehicle/installed/README.md)确认DAPLink实际串口：当前代码基线采用UART7/PE8-TX→DAPLink RX、共地，115200 8N1；云台板间USART10，底盘板间USART1，460800 8N1。该接线未经过本次现场核对。UART7是诊断唯一所有者，视觉需先分配其他资源。

输入授权：真实遥控双拨Down，摇杆/拨轮归中保持500ms，再把左拨切Middle；左Down停止。遥控过期或双板对端离线/重启后重新解锁。先观察、再小幅动作；不使用永久有效授权，不上电自动运行轨迹。正常小幅动作、撤销输入、断流、恢复，双板再加单板重启与执行状态停更，由现场操作并记录结果。

仅修改 `include/robotics/vehicle/diagnostics_config.hpp` 的 `diagnostic_output`，重编译烧录切换Log/Vofa。UART无console/shell/banner/默认串口后台；LOG和printk统一进自定义后台，Vofa固件丢弃文本，整次运行保持纯JustFloat。两模式用固定16包队列及低优先级输出线程，拥塞丢观测包并计数，控制线程不等串口。故障可能导致波形停流，主机保存时间及原始数据；切换固件不保留先前RAM事件。远端只提供协议已有字段，没有远端内部PID转发。

## 诊断布局

JustFloat版本1、50Hz上限、19个float、80字节/帧，约4kB/s。每个工程README说明其target/feedback/extra字段的物理意义。未使用字段为0；feedback无效时为NaN，age未知为-1。序号、时间、丢包计数拆为两个16位值：原值=low+65536×high，避免float丢低位；uptime_ms按32位回绕。

| 通道(从0开始) | 含义 |
| --- | --- |
| 0 | 布局版本=1 |
| 1 / 2 | 本地诊断生产序号低16/高16 |
| 3 / 4 | 本地uptime_ms低16/高16 |
| 5 | reason，编号见下表 |
| 6 / 7 | valid测量有效 / enabled本阶段输出状态 |
| 8 / 9 | target / feedback，物理量与单位见入口README |
| 10 | age_ms，所对应输入或测量见入口README |
| 11 / 12 | dt_us / effort，effort单位见入口README |
| 13 / 14 | detail状态码 / error错误码；启动第一处负错误保留到复位 |
| 15 / 16 | 共享队列丢包计数低16/高16 |
| 17 / 18 | extra0 / extra1，物理量与单位见入口README |

reason：0 Boot、1 Ready、2 InitFailed、3 RemoteStale、4 Disarmed、5 FeedbackStale、6 ImuStale、7 PeerStale、8 Configuration、9 Permission、10 CommandStale。日志每秒状态并记录原因变化；BOOT身份包含应用、板角色、源码dirty标识、硬件/标定候选版本和协议v4。reason给首个直接阻塞原因，detail补执行器/电机状态；不能把Ready单独理解成机构通过上车。

板间字节、物理量、boot/年龄/生产序号规则统一见[协议v4契约](../../docs/protocols/interboard-v4.md)。ChassisFeedback目前没有有效实测速度/功率；不能拿默认0作零速证明。

## 上车登记

2026-10-25：本目录21个工程、两个最终应用，以及受共享配置迁移影响的gimbal_control、command_gimbal、inertial_gimbal、dual_yaw_centering，共27个工程均通过 `dm_mc02/stm32h723xx` 编译与链接。构建产物位于 `../build/refactor-20261025/<目录路径以双下划线连接>/zephyr/`；每项保留独立bin/elf。编译日志和结果清单位于本机 `/tmp/skywalker-refactor-builds/`。只做编译检查，没有运行测试、烧录或实板验证。

所有工程入口已经具备；实板输入、单机构与组合全部待确认。中央connections/imu/shooter确认bool沿用现值，不能替代现场记录；功率标定仍未完成。原始机械参数和另一套云台/底盘兼容性均未在本次验证。

现场每次记录：日期/人员、应用身份、双方固件/配置/标定/协议版本、本次唯一变化、操作及允许误差/停止时限、目标/反馈/原因/时间、通过或失败、日志位置、下一步和可回退固件。通过后保存二进制与当时配置；标定变化后重做受影响阶段。视觉参考会话、真实功率输入、真实热量/发射许可/拨盘归位有独立输入缺口，本次未以开关或占位值代替。
