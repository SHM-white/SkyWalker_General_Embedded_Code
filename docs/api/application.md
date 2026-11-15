# 双主控应用与执行器：接口与调用参考

当前正式应用概览按 `81578dd`（2026-11-15）源码整理。两个 sentry 应用分别在自己的 `main.cpp` 中装配，不再调用 `samples::vehicle::run()`。完整源码、物理资源和参数见文末链接。

**接入状态：应用已装配视觉、导航、外置头部 IMU 和发射执行器，实板待确认。** 板载 BMI088 提供载体角速度，底盘使用 v4 世界角速度及原生大 Yaw 角反馈。27 项编译、链接通过是 2026-10-25 旧基线；本次文档更新未重新构建、测试、烧录或实板验证，实测功率闭环仍未完成。

## 职责与关联

| 入口 | 自行装配的对象 | 固定职责 |
| --- | --- | --- |
| `applications/sentry_gimbal/src/main.cpp` | RemoteReceiver、CommandManager、板载 BMI088、外置头部 IMU、视觉/导航来源、Yaw/Pitch 和发射电机与执行器、双 Yaw 分配器、UART Endpoint、Diagnostics | 遥控与自主来源仲裁、双轴及发射控制、大 Yaw 世界速度和底盘请求 |
| `applications/sentry_chassis/src/main.cpp` | 四舵向/四轮驱电机、三个 CanBus、SwerveHardware、ChassisExecutor、大 Yaw 电机/执行器、UART Endpoint、Diagnostics | 消费真实板间操作授权，执行四轮与大 Yaw，回传生产状态 |
| `samples/bringup/<阶段>/src/main.cpp` | 只装配对应阶段需要的对象 | 观察、单机构与逐级组合；目录固定角色和机构 |

对象构造、attach/start、线程和循环归各入口所有；应用间只共享纯配置及已有驱动、通信、控制器和执行器 API。`include/robotics/vehicle/` 中的 profile 不隐藏静态电机、启动线程或无限循环。

输入 / 依赖：[命令服务](command.md)、[板间接口](interboard.md)、[线协议 v4](../protocols/interboard-v4.md)、[惯性适配](inertial.md)、[云台执行器](gimbal.md)、[底盘](chassis.md)、[大 Yaw](big-yaw.md)。发射接口及限制见[发射接口](shooter.md)和[云台应用](../../applications/sentry_gimbal/README.md)。

## 装配与调用顺序

1. 从 [独立上车入口表](../../samples/bringup/README.md) 选择当前阶段，不用角色宏、锁 Pitch 宏或额外 conf 改造一个公共入口。
2. CMake 固定加载 `config/vehicle/installed/base.conf`、对应板级 conf 与本地 `prj.conf`；本地 overlay 包含对应共享物理 overlay。核对候选接线及中央标定。
3. 尽早启动 `Diagnostics`，再构造长期存活的设备、接收器、Motor、Group、CanBus 和执行器。需要 DMA 的缓冲保持静态 `__nocache` 生命周期。
4. 注册输入来源并启动服务；把本阶段的电机 attach 到各物理总线，start 后 begin 控制器。记录初始化错误，保留无输出诊断循环。
5. 最终应用主循环调用 `endpoint.poll()` 并读取一致快照；云台 5 ms、底盘 2 ms 绝对节拍。接收、仲裁、CAN I/O、IMU 采集和诊断发送由相应模块 worker 推进。
6. 在同一执行线程暂存本周期各机构目标，再对每条物理 CAN 提交一次。观察线程复制状态，不更新输入年龄或运行授权。

各独立上车工程可使用不同的固定装配和通信调度，以各自源码为准。观察工程不会因为物理设备树声明了电机总线就自动注册电机。

## 当前配置来源

| 位置 | 内容 |
| --- | --- |
| `config/vehicle/installed/` | 两板资源、DMA、基础系统输出策略与可选 CAN IMU 后端 |
| `include/robotics/vehicle/calibration.hpp` | ID、方向、传动、零点、限位、安装变换与确认值 |
| `gimbal_profile.hpp` / `chassis_profile.hpp` / `swerve_profile.hpp` / `big_yaw_profile.hpp` | 共享硬件、PID 与执行器参数 |
| `manual_profile.hpp` | 最终手动应用命令、惯性与回中策略 |
| `bringup_policy.hpp` / `bringup_chassis_policy.hpp` | 阶段保守动作幅度与电流上限 |
| `diagnostics_config.hpp` | 唯一的 Log/Vofa 编译期选择、诊断波特率和候选版本标识 |

当前 `connections_confirmed`、`imu_mounting_confirmed`、`shooter_constraints_confirmed` 为 true，`power_model_calibrated` 为 false。这些是沿用的代码值，不是本次实板通过记录。中央参数修改后重建受影响的阶段和应用，旧固件不会自动更新。

云台 CAN1 为小 Yaw 和发射电机，CAN2 为 Pitch，CAN3 为外置头部 IMU，板间为 USART10；视觉使用 USB CDC。底盘 CAN1/2/3 为舵向/轮驱/大 Yaw，板间 USART1。诊断候选 UART7、115200 8N1，实际接线待确认；当前云台启用诊断，底盘默认关闭。

## 授权、时效与观测

遥控解锁及自主档掉线保持的具体条件见[云台应用拨杆说明](../../applications/sentry_gimbal/README.md#拨杆与巡航模式)；左 Down 撤销运行，板间离线或对端重启也有独立恢复边界。板间还保留 boot、命令/原始输入年龄与执行状态生产年龄检查；各字段实际契约见 [v4 规范](../protocols/interboard-v4.md)。

`RunStatus::stamp` 由执行 owner 生产。`SnapshotCache<T>::publish/snapshot` 和 Endpoint 只交换值副本，读取或重发不延长生产数据寿命。电机反馈等待、用户主动停止和输入失效是不同状态，观察时同时看来源年龄、反馈有效性、reason 与执行器状态。

诊断 API、单线程实例所有权及错误边界见[观测接口](telemetry.md)。启用诊断后 Log/Vofa 统一走共享诊断口；不在同口额外 `vofa_init()` 或启动 console/shell。无效反馈有 valid/age，底盘协议摘要尚未提供实测速度/功率，默认零不能当作测量。

## 构建示例

在仓库目录执行，两个应用各自使用固定产物目录：

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
west build -b dm_mc02/stm32h723xx applications/sentry_chassis -d ../build/applications/sentry_chassis
```

诊断输出选择示例位于共享配置头，修改后按原命令重编译、烧录：

```cpp
inline constexpr DiagnosticOutput diagnostic_output = DiagnosticOutput::Log;
// 看波形时将同一处改为 DiagnosticOutput::Vofa。
```

历史 `samples/robotics/vehicle_integration` 仍保留公共 `vehicle_bench.hpp` 和宏组合，用于理解旧实现；其 `run(power_source, heat_source, home_source)` 不是两个最终应用的当前入口。现场操作以独立工程 README 为准，不沿用历史额外 conf 组合。

## 正文与源码

- [双主控说明](../applications/dual-controller.md) · [独立上车入口](../../samples/bringup/README.md) · [共享物理配置](../../config/vehicle/installed/README.md)
- [云台 main](../../applications/sentry_gimbal/src/main.cpp) · [底盘 main](../../applications/sentry_chassis/src/main.cpp)
- [中央标定](../../include/robotics/vehicle/calibration.hpp) · [诊断接口](../../include/robotics/vehicle/diagnostics.hpp)
- [执行器](../modules/robotics/executors.md) · [历史整车 sample](../../samples/robotics/vehicle_integration/README.md)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
