# 模块联动与调用路线

更新日期：2026-10-25。当前两个 sentry 应用各自持有 `main.cpp`、设备实例、初始化顺序和执行循环；上车阶段也分别有独立工程。共同使用 [实车配置与 profile](../../config/vehicle/installed/README.md) 以及已有驱动、协议、控制器和执行器 API，不再由公共 `vehicle_bench::run()` 调度最终应用。

| 层 | 当前职责与边界 |
| --- | --- |
| Remote / IMU 来源 | 保存实际采样 stamp、质量和参考；读取快照不续期 |
| RcControlAdapter | 对新鲜 DR16 输入执行安全档归中和解锁；失联/撤权后重新解锁 |
| CommandManager | 后台采样与仲裁，并接受执行循环发布的真实操作门控 |
| 惯性适配 / 运动学 | 将惯性目标变换成机械命令，或将底盘速度分配到舵轮；不创建授权 |
| Executor | 检查命令、来源、许可和反馈时效，保存目标并处理停止与恢复边界 |
| Motor / CanBus | 保存电机意图，推进协议使能、反馈和总线恢复；由应用协调每周期提交 |
| InterBoardEndpoint | 编解码 v4、boot 绑定、转发年龄和状态生产时效；不代替执行器判定 |
| Diagnostics | 接受观测快照，统一日志/JustFloat 输出；不写目标或影响解锁 |

## 当前云台链

[云台 main](../../applications/sentry_gimbal/src/main.cpp) 在创建业务对象前启动诊断，再显式创建 DR16、CAN3 头部 DM IMU、CommandManager、CAN1 小 Yaw、CAN2 Pitch 和对应控制器。它不创建发射电机、视觉或裁判接收器。

```text
DR16 → 真实遥控解锁 → CommandManager 外部门控
     → CommandSnapshot + 头部 IMU + 关节反馈
     → InertialGimbalAdapter → GimbalExecutor → CAN1 / CAN2
     → 小 Yaw 中心误差 → YawCenteringController → BigYawRequest
CommandSnapshot.chassis → ChassisControl
遥控原始 stamp / clear 事件 → OperatorControl
```

执行循环为 5 ms 绝对节拍；同一循环推进 endpoint，每 10 ms 提交行走和大 Yaw 请求，周期末各提交一次 CAN1、CAN2。大 Yaw 回中以双轴惯性输出有效且云台 Active 为前提，默认最大 ±0.6 rad/s、斜坡 0.8 rad/s²；实际头部稳定误差不再作为额外回中启动条件。保留当前线路反号适配，不在此处擅自改对端的物理正方向。

关节测量与目标以驱动标定坐标为基础，头部 IMU 参考及安装变换保留于惯性层。云台 CAN3 采用主动 CAN 四元数报告，从机 0x1 / 反馈 0x11，不再把该 CAN 设备交给 RS485 Source。

## 当前底盘链

[底盘 main](../../applications/sentry_chassis/src/main.cpp) 显式创建八个舵轮 Motor、Group、SwerveHardware、ChassisExecutor、大 Yaw Motor/BigYawExecutor 和三条物理 CAN。CAN1 是四舵向，CAN2 是四轮驱，CAN3 是大 Yaw；旧“CAN3 轮驱、CAN2 大 Yaw”的表述已过期。

```text
USART1 endpoint → OperatorControl + boot/年龄检查
  ChassisControl → ChassisExecutor → SwerveChassis → 八电机目标
  BigYawRequest  → BigYawExecutor → 大 Yaw 速度目标
  单一执行循环 → CAN1 / CAN2 / CAN3 每周期各提交一次
  真实执行状态与实测大 Yaw 速度 → endpoint / 诊断
```

底盘为 2 ms 绝对节拍，命令需同时满足对端在线、接收方 boot、真实操作授权和对应时效。大 Yaw 保留来源年龄、命令年龄、许可与本轴反馈检查；不能用一条持续心跳保活过期运动。初次看到 peer boot 或 boot 改变时，两执行器跨越恢复边界。当前入口不启用功率预算/实测闭环，按中央舵向 0.8 A、轮驱 0.5 A 上限运行手动机械链。

执行器与电机恢复各自管理故障边界；普通电机恢复不会改写板间 boot 或生成新的 RC 授权。物理总线故障仍会影响该总线成员，不能据此承诺所有机构故障完全隔离。授权撤销、实际板重启、链路失效与输入过期由应用和协议共同处理。

## 配置、诊断与工程选择

[gimbal.overlay / chassis.overlay](../../config/vehicle/installed/README.md) 固定实车资源；`base.conf` 与板级 conf 固定系统和驱动设置。`calibration.hpp` 提供 ID、方向、传动比、零点、限位和安装变换，相关 profile 提供 PID 与策略值。阶段可以选择更保守输出上限，不能复制标定或改变接线来让某个 sample 单独运行。

云台板间口为 USART10、底盘板间口为 USART1，均为 460800 8N1；两板 DAPLink 候选诊断口统一为 UART7、115200 8N1。现场接线尚未实查，UART7 由诊断独占，不留视觉复用别名。只改 [diagnostics_config.hpp](../../include/robotics/vehicle/diagnostics_config.hpp) 的 `diagnostic_output`，重编译烧录后即可选 Log 或 Vofa，不叠加额外配置。

诊断从初始化开始接受身份、失败步骤和错误码；独立 worker 发送固定队列中的观测。队列满时丢观测并累计统计，控制线程不等待串口。VOFA 模式丢弃日志与 printk 文本，console、shell、banner 均关闭；无效测量保留有效位/NaN 和年龄，复位后没有上次 RAM 故障记录。通道布局及单位见 [独立上车入口表](../../samples/bringup/README.md)。

操作者选择具体目录和固定构建产物，先观察、再单机构、再组合；不通过 `VEHICLE_*` 角色/场景宏切换新应用。[vehicle_integration](../../samples/robotics/vehicle_integration/README.md) 与原电机持续指令[实施指南](../dev/电机持续指令与独立自动恢复重构实施指南.md)保留历史上下文，当前接线和入口以本次独立工程与共享配置为准。

## 尚未接入的能力

视觉观察/执行、真实裁判约束、实测功率控制、真实热量与拨盘归位支持的有载发射没有接入当前两个最终应用。库 API、旧 sample 的开关和 Pending 来源不能充当完成状态；后续应分别建立明确输入与资源的独立入口，再接最终组合。

本次 27 个相关工程编译完成，未运行实板机构动作或故障场景。中央确认 bool 与编译通过均不能升级为接线、机构方向、停止时限或组合性能已经通过，仍须按阶段记录实车结论。完整应用关系见 [双主控应用](dual-controller.md)，协议布局和时效语义见 [interboard-v4](../protocols/interboard-v4.md)。
