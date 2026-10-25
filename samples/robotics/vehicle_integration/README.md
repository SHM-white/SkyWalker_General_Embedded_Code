# 双主控整车组合（历史宏组合入口）

历史记录，更新日期：2026-10-25。本目录保留旧 `common/vehicle_bench.hpp` 运行体和 `VEHICLE_*` 角色/场景宏，用于理解旧实现。它不再与 `applications/sentry_gimbal`、`applications/sentry_chassis` 共享运行入口；两个最终应用已经各自在自己的 `src/main.cpp` 中装配资源并执行控制循环。

现场上车请选择 [samples/bringup 独立工程](../../bringup/README.md)，按诊断、遥控/IMU、单轴、板间观察、大 Yaw、组合的顺序推进。最终应用分别见 [云台](../../../applications/sentry_gimbal/README.md) 和 [底盘](../../../applications/sentry_chassis/README.md)。不要通过本目录的角色宏或附加 conf 选择新的上车阶段。

## 当前配置与历史实现的区别

新入口使用 [共享实车配置目录](../../../config/vehicle/installed/README.md)，其代码基线为：

| 板卡 | 资源 | 当前固定用途 |
| --- | --- | --- |
| 云台 | CAN1 / CAN2 / CAN3 | 小 Yaw / Pitch / 头部 CAN IMU |
| 云台 | UART5 / USART10 / UART7 | DR16 / 板间 460800 / DAPLink 诊断 115200 |
| 底盘 | CAN1 / CAN2 / CAN3 | 四舵向 / 四轮驱 / 大 Yaw |
| 底盘 | USART1 / UART7 | 板间 460800 / DAPLink 诊断 115200 |

旧文档曾把底盘轮驱写成 CAN3、大 Yaw 写成 CAN2；当前代码分配已是上表所列 CAN2 轮驱、CAN3 大 Yaw。云台新入口的头部 IMU 是 CAN3 主动上报，从机 0x1、反馈 0x11；本历史 sample 的 `prj.conf` 仍选择 RS485 头部 IMU，且 `app.overlay` 仍包含旧公共 overlay，不能凭本 sample 的构建结果认定与新应用接线一致。旧公共 overlay 中的 UART7 视觉、USART2 RS485 IMU 和默认控制台映射属于历史资源布局。

中央 [calibration.hpp](../../../include/robotics/vehicle/calibration.hpp) 当前值为：`connections_confirmed=true`、`imu_mounting_confirmed=true`、`shooter_constraints_confirmed=true`、`power_model_calibrated=false`。旧说明中的“确认标志默认全为 false”已经过期；这些 bool 保留代码现值，并非本次已完成现场标定的证明。新最终云台也不会因为发射确认标志为 true 而启动发射机构。

## 旧构建记录

以下命令仅记录历史入口，已退出现场推荐步骤；其 overlay、宏和额外 conf 不构成新固件的配置基线：

```sh
# 历史云台角色，勿用作本次上车步骤
west build -b dm_mc02/stm32h723xx samples/robotics/vehicle_integration -d ../build/vehicle_gimbal
# 历史底盘角色，勿用作本次上车步骤
west build -b dm_mc02/stm32h723xx samples/robotics/vehicle_integration -d ../build/vehicle_chassis -- -DEXTRA_CONF_FILE=chassis.conf
```

`pitch_free.conf`、`vision_observe.conf`、`vision_execute.conf`、`power.conf`、`shooting.conf` 和 `diagnostic.conf` 同属旧宏组合方式。新的独立工程通过目录固定入口，不要求操作者叠加这些文件。新诊断出口由单个 `diagnostic_output` 代码变量选择，见共享配置说明；旧 sample 的控制台遥测不继承这一保证。

## 保留的实现边界

历史运行体包含 `CommandManager`、惯性适配、云台执行器、大 Yaw 回中与板间授权/boot/年龄处理，也包含尚未接入真实来源的 `PendingPowerSource`、`PendingHeatSource` 与 `PendingDialHomeSource`。功率预算不等于实测功率；默认无效测量不能成为功率闭环已完成的证据。加载发射仍需真实热量/许可和拨盘参考，不能从任意故障后位置伪造归位。

视觉参考会话、真实弹速/弹数、功率模型与机构标定仍需独立完成。历史编译记录、功能开关和中央确认 bool 均不等于本次实板输入、单机构或组合已通过；本目录未因文档更新新增实板结论。
