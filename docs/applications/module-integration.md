# 模块联动与调用路线

当前正式应用概览按 2026-11-15（代码基线 `dd3f673`）整理。当前两个 sentry 应用各自持有 `main.cpp`、设备实例、初始化顺序和执行循环；上车阶段也分别有独立工程。共同使用 [实车配置与 profile](../../config/vehicle/installed/README.md) 以及已有驱动、协议、控制器和执行器 API，不再由公共 `vehicle_bench::run()` 调度最终应用。

| 层 | 当前职责与边界 |
| --- | --- |
| Remote / IMU 来源 | 保存实际采样 stamp、质量和参考；读取快照不续期 |
| RcControlAdapter | 对 DR16 输入执行归中与解锁；导航档允许满足条件的掉线保持，细节见云台应用 |
| CommandManager | 后台采样与仲裁，并接受执行循环发布的真实操作门控 |
| 航向适配 / 运动学 | 正式云台使用外置头部 IMU 与板载 BMI088，底盘将云台系速度变到车体系 |
| Executor | 检查命令、来源、许可和反馈时效，保存目标并处理停止与恢复边界 |
| Motor / CanBus | 保存电机意图，推进协议使能、反馈和总线恢复；由应用协调每周期提交 |
| InterBoardEndpoint | 编解码 v4、boot 绑定、转发年龄和状态生产时效；不代替执行器判定 |
| Diagnostics | 接受观测快照，统一日志/JustFloat 输出；不写目标或影响解锁 |

## 当前云台链

[云台 main](../../applications/sentry_gimbal/src/main.cpp) 装配 DR16、板载 BMI088、CAN3 外置头部 IMU、视觉与导航来源、双轴及发射执行器；视觉和导航来源同时受设备树别名与 Kconfig 控制。遥控或自主命令经 `CommandManager` 仲裁，双 Yaw 分配器生成小 Yaw 和大 Yaw 运动意图。外置 IMU 提供视觉世界姿态参考，板载 BMI088 提供载体航向与角速度。

```text
遥控 / 导航 / 视觉 → CommandManager → 云台 / 发射执行器
头部 IMU + 载体 BMI088 → 视觉适配与双 Yaw 分配
行走、大 Yaw 和载体航向 → USART10 Endpoint → 底盘
底盘裁判原始帧 → 云台重组并转发至导航
```

控制循环为 5 ms；板间请求每 10 ms 提交。授权、新鲜度和参考有效性决定实际输出，详见[云台应用](../../applications/sentry_gimbal/README.md)与[双主控应用](dual-controller.md)。

## 当前底盘链

[底盘 main](../../applications/sentry_chassis/src/main.cpp) 显式创建八个舵轮 Motor、Group、SwerveHardware、ChassisExecutor、大 Yaw Motor/BigYawExecutor 和三条物理 CAN。CAN1 是四舵向，CAN2 是四轮驱，CAN3 是大 Yaw；旧“CAN3 轮驱、CAN2 大 Yaw”的表述已过期。

```text
USART1 endpoint → OperatorControl + boot/年龄检查
  0x0101 云台系平移 + 大 Yaw 原生角 → 车体系 → ChassisExecutor → 八电机目标
  0x0402 世界角速度 + 双板 BMI088 角速度 → 补偿限幅 → BigYawExecutor
  单一执行循环 → CAN1 / CAN2 / CAN3 每周期各提交一次
  真实执行状态与实测大 Yaw 速度 → endpoint / 诊断
```

底盘为 2 ms 绝对节拍，命令需同时满足对端在线、接收方 boot、真实操作授权和对应时效。大 Yaw 保留来源年龄、命令年龄、许可与本轴反馈检查；不能用一条持续心跳保活过期运动。初次看到 peer boot 或 boot 改变时，两执行器跨越恢复边界。当前入口不启用功率预算/实测闭环，按中央舵向 2.0 A、轮驱 20.0 A 上限运行手动机械链。

执行器与电机恢复各自管理故障边界；普通电机恢复不会改写板间 boot 或生成新的 RC 授权。物理总线故障仍会影响该总线成员，不能据此承诺所有机构故障完全隔离。授权撤销、实际板重启、链路失效与输入过期由应用和协议共同处理。

## 配置、诊断与工程选择

[gimbal.overlay / chassis.overlay](../../config/vehicle/installed/README.md) 固定实车资源；`base.conf` 与板级 conf 固定系统和驱动设置。`calibration.hpp` 提供 ID、方向、传动比、零点、限位和安装变换，相关 profile 提供 PID 与策略值。阶段可以选择更保守输出上限，不能复制标定或改变接线来让某个 sample 单独运行。

云台板间口为 USART10、底盘板间口为 USART1，均为 460800 8N1；两板 DAPLink 候选诊断口统一为 UART7、115200 8N1。现场接线尚未实查，UART7 由诊断独占，不留视觉复用别名。当前云台启用诊断，底盘默认关闭；启用 `CONFIG_SKYWALKER_VEHICLE_DIAGNOSTICS` 的工程可修改 [diagnostics_config.hpp](../../include/robotics/vehicle/diagnostics_config.hpp) 的 `diagnostic_output` 并重编译，选择 Log 或 Vofa。

诊断从初始化开始接受身份、失败步骤和错误码；独立 worker 发送固定队列中的观测。队列满时丢观测并累计统计，控制线程不等待串口。VOFA 模式丢弃日志与 printk 文本，console、shell、banner 均关闭；无效测量保留有效位/NaN 和年龄，复位后没有上次 RAM 故障记录。上车工程通道布局见 [入口表](../../samples/bringup/README.md)，正式云台的 32 通道布局见 [VOFA 通道表](../dev/云台双轴响应VOFA通道说明.md)。

操作者选择具体目录和固定构建产物，先观察、再单机构、再组合；不通过 `VEHICLE_*` 角色/场景宏切换新应用。[vehicle_integration](../../samples/robotics/vehicle_integration/README.md) 与原电机持续指令[实施指南](../dev/电机持续指令与独立自动恢复重构实施指南.md)保留历史上下文，当前接线和入口以本次独立工程与共享配置为准。

## 未完成的验证

视觉、导航、裁判与发射执行器已有应用装配；云台发射的热量与许可要求当前关闭，实测功率闭环、拨盘参考和有载发射仍需单独确认。代码接入不等于现场验收通过。

2026-10-25 的旧基线有 27 个工程编译通过记录；本轮尚未重新构建、测试、烧录或运行实板机构动作/故障场景。中央确认 bool 与编译通过均不能升级为接线、机构方向、停止时限或组合性能已经通过，仍须按阶段记录实车结论。完整应用关系见 [双主控应用](dual-controller.md)，协议布局和时效语义见 [interboard-v4](../protocols/interboard-v4.md)。
