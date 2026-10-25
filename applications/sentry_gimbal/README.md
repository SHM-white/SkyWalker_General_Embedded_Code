# 哨兵云台应用

固定的手动双轴惯性云台 + 大 Yaw 回中发送 + 底盘行走发送入口。`src/main.cpp` 显式装配遥控、CAN3 头部 IMU、CAN1 小 Yaw、CAN2 Pitch、板间端点和控制器，不再调用样例公共 `vehicle::run()`。不启动摩擦轮、拨盘、视觉或裁判接收器。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
```

不叠加旧 `imu_*.conf`、`pitch_free.conf`、`referee.conf`、`shooting.conf`、`vision_*.conf` 等角色/场景配置。它们属于历史公共运行体的构建方式；机械单轴、IMU、板间观察应选择 [独立上车工程](../../samples/bringup/README.md) 中的相应目录。修改中央配置后重新构建相关固件。

## 共用配置与接线

物理配置使用 [gimbal.overlay](../../config/vehicle/installed/gimbal.overlay)、系统配置使用同目录的 `base.conf` 和 `gimbal.conf`。标定、硬件驱动、PID、关节限位分别来自 [calibration.hpp](../../include/robotics/vehicle/calibration.hpp)、[gimbal_profile.hpp](../../include/robotics/vehicle/gimbal_profile.hpp)；手动策略来自 [manual_profile.hpp](../../include/robotics/vehicle/manual_profile.hpp)。阶段程序与最终应用共用这些配置。

| 资源 | 固定用途 |
| --- | --- |
| UART5 / DBUS | DR16 遥控 |
| USART10 PE3 TX / PE2 RX，460800 8N1 | v4 板间 UART |
| UART7 PE8 TX / PE7 RX，115200 8N1 | DAPLink 单口诊断；TX 接 DAPLink RX 并共地 |
| CAN1 / CAN2 | 小 Yaw / Pitch 电机 |
| CAN3，1 Mbps | 头部 DM IMU，主动上报，从机 0x1 / 反馈 0x11 |

UART7 被诊断独占，视觉不能再沿用这个端口。DAPLink 是否实际接到 UART7、IMU 安装变换和实车机械端点仍需按现场记录确认；共享配置的确认值保留现值，不能代替实板记录。

## 运行与停止

左右拨杆 Down，摇杆与拨轮归中保持 500 ms，再把左拨杆切到 Middle 解锁。右横杆控制小 Yaw，右纵杆控制 Pitch；左摇杆前后/横移各限 ±0.3 m/s，拨轮自转限 ±0.5 rad/s。左拨杆 Down 或遥控过期撤销运行；不支持自动视觉档。

双轴惯性输出有效且云台执行处于 Active 时，关节中心误差驱动大 Yaw 回中，限速 ±0.6 rad/s、斜坡 0.8 rad/s²。保留当前对端的线上反号适配 `big_yaw_wire_direction = -1`；不擅自统一两端正方向。协议速度是大 Yaw 关节侧 rad/s，见 [v4 协议契约](../../docs/protocols/interboard-v4.md)。头部 IMU 过期、初始化失败、输入撤销等情况下仍产生 `Disabled + 0` 请求。

应用循环每 5 ms 更新一次，每 10 ms 发布大 Yaw 请求及行走命令。请求携带原始输入与控制命令年龄，发送不会延长旧运动的有效期。链路曾上线后丢失或对端 boot 改变，会撤销本地解锁，必须归中重新解锁；本地初次调试不要求对端已上线。底盘自行约束 boot、授权、命令时效和反馈恢复。

## 诊断

只修改 [diagnostics_config.hpp](../../include/robotics/vehicle/diagnostics_config.hpp) 中的 `diagnostic_output`，然后重新编译烧录，选择文本日志或 VOFA JustFloat。使用同一 DAPLink COM 口与共享波特率；VOFA 固件不输出系统文本。初始化失败仍继续报告失败步骤与错误码。

本应用提交的小 Yaw 诊断目标/反馈单位均为 rad，`enabled` 表示双轴执行 Active，`valid` 表示小 Yaw 测量有效，`age_ms` 为头部姿态年龄。通道顺序及布局版本沿用共享 [诊断说明](../../samples/bringup/README.md)。诊断字段不等于对端内部 PID 观测，也不证明实际停止时限或惯性稳定性已通过。

尚未进行实板验证。射击真实热量/许可/归位、视觉参考会话与功率实测均属于独立后续能力。
