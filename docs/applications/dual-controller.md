# 双主控整车应用

更新日期：2026-10-25。[云台应用](../../applications/sentry_gimbal/README.md) 与 [底盘应用](../../applications/sentry_chassis/README.md) 已各自在自己的 `src/main.cpp` 中装配设备、启动来源并运行控制循环。当前交付是手动双轴惯性云台、大 Yaw 回中发送和底盘机械执行链。

[旧 vehicle_integration](../../samples/robotics/vehicle_integration/README.md) 保留历史宏组合运行体；两个最终应用已经不调用它的 `vehicle_bench.hpp::run()`。现场选择 [samples/bringup 独立工程](../../samples/bringup/README.md) 的目录，不用角色宏、场景宏或叠加 conf 切换应用。

## 资源与执行归属

| 板卡 | 输入与通信 | 执行资源 |
| --- | --- | --- |
| 云台 | UART5 DR16；CAN3 头部 DM IMU；USART10 板间 UART | CAN1 小 Yaw、CAN2 Pitch |
| 底盘 | USART1 板间 UART，消费云台发送的真实操作授权 | CAN1 四舵向、CAN2 四轮驱、CAN3 大 Yaw |
| 两板 | UART7 `diagnostic-uart`，115200 8N1，经 DAPLink 观察 | 诊断 worker 独占发送，不承担控制 |

云台板间 USART10 使用 PE3 TX / PE2 RX、460800 8N1、DMA 通道 3/4；UART7 为 PE8 TX / PE7 RX，无 DMA。头部 IMU 使用 CAN3、1 Mbps、主动四元数上报，从机 0x1 / 反馈 0x11。底盘保留 USART1 板间映射及 DMA 3/4。具体接线与所有权见 [installed 配置说明](../../config/vehicle/installed/README.md)；DAPLink 实际是否接 UART7 仍待现场核对。

```text
云台 main（5 ms 绝对节拍）
  endpoint.poll → 板间接收与异步发送推进
  DR16 快照 → RcControlAdapter → CommandManager 外部门控
  CommandSnapshot + 头部 IMU + 两轴机械反馈
    → InertialGimbalAdapter → GimbalExecutor
    → YawCenteringController → 大 Yaw 关节速度请求
  每 10 ms 提交大 Yaw 请求、行走命令；每周期提交 CAN1/CAN2 各一次
  执行状态 → endpoint / DAPLink 诊断

底盘 main（2 ms 绝对节拍）
  endpoint.poll → 接收快照 → 操作授权、boot 与时效检查
  行走命令 → ChassisExecutor → SwerveChassis → 八电机暂存目标
  大 Yaw 请求 → BigYawExecutor → 独立关节速度内环
  每周期提交 CAN1/CAN2/CAN3 各一次 → 状态与实测大 Yaw 反馈
```

当前两个应用都在自己的 main 循环中推进 endpoint，没有旧公共运行体的独立通信/遥测线程。遥控和 IMU 接收器、CommandManager、Motor/CanBus 仍有各自模块工作线程；异步 UART 的发送期限不表示控制线程同步等待串口发送完成。电机目标和 CAN 提交由应用执行循环唯一写入，诊断 worker 只发送已提交的观测。

## 输入授权、停止与大 Yaw

遥控左右拨杆 Down、摇杆和拨轮归中保持 500 ms，再把左拨杆切到 Middle 解锁。左 Down、遥控过期或已建立链路后的对端掉线/boot 改变会撤销运行；重新归中解锁后才能继续。底盘没有伪造本地永久授权，依靠 `OperatorControl` 中的真实输入序号、年龄和目标 boot。

云台右横/纵杆分别控制小 Yaw / Pitch，手动上限为 1.0 / 0.8 rad/s；左摇杆平移各限 ±0.3 m/s，拨轮自转限 ±0.5 rad/s。头部姿态和关节反馈必须新鲜，惯性目标先变换为机械关节速度，再由机械执行器施加标定限位。

大 Yaw 回中根据小 Yaw 独立中心误差计算，要求双轴惯性输出有效且云台执行处于 Active；不是等待头部姿态误差已进入稳定阈值才发送。当前回中限速 ±0.6 rad/s、斜坡 0.8 rad/s²，保留对端约定的 `big_yaw_wire_direction = -1`。线路速度为大 Yaw 关节侧 rad/s，不能当作电机转子 rpm 或头部对地角速度。

v4 大 Yaw 请求保留原始来源/命令/许可年龄与接收方 boot 绑定。发送不会刷新旧运动的原始年龄，底盘同时检查自身反馈和执行恢复条件；心跳在线不替代命令新鲜度。输入撤销、IMU 或初始化失败时仍提交停止请求，`Disabled + 0` 与启用速度内环的零速目标不同。详细字节布局、序号和同序号降级停止规则见 [板间 v4 契约](../protocols/interboard-v4.md)。电机恢复不会替用户创建新的输入授权。

## 共享配置与诊断

两应用与上车工程固定引用 [config/vehicle/installed](../../config/vehicle/installed/README.md) 的 `base.conf`、板级 conf 和 overlay。标定与纯配置来自 `include/robotics/vehicle/` 中的 [calibration.hpp](../../include/robotics/vehicle/calibration.hpp)、`gimbal_profile.hpp`、`manual_profile.hpp`、`chassis_profile.hpp`、`swerve_profile.hpp` 和 `big_yaw_profile.hpp`。共享范围是数值与无副作用配置工厂，设备实例、启动顺序和执行循环由每个工程持有。

中央连接、IMU 安装和发射约束标志当前为 true，功率模型标志为 false；这说明当前编入的配置，不能当作本次已完成现场确认。硬件候选版本和未验证标定版本由 [diagnostics_config.hpp](../../include/robotics/vehicle/diagnostics_config.hpp) 明示。改中央配置后，相关阶段与最终应用都需重新编译烧录。

同一头文件中的 `diagnostic_output` 是日志/波形的唯一选择：设为 `DiagnosticOutput::Log` 或 `DiagnosticOutput::Vofa`，再按原命令重编译烧录。两种模式均使用 UART7 DAPLink COM 口；console、shell、启动 banner 和直写 UART 日志后台关闭，专用后台在 VOFA 模式丢弃文本。初始化失败仍有状态/错误观测；固定队列拥塞丢观测并计数，不阻塞控制。通道顺序、有效位、年龄与每个阶段的单位见 [上车说明](../../samples/bringup/README.md)。UART7 不再提供视觉别名，后续视觉必须先安排独立资源。

## 构建与交付边界

从仓库目录运行，按应用使用独立构建目录：

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
west build -b dm_mc02/stm32h723xx applications/sentry_chassis -d ../build/applications/sentry_chassis
```

本次重构的 27 个相关工程已完成编译；没有执行实板输入、机构动作或故障恢复验证。当前最终云台不创建视觉、裁判、摩擦轮或拨盘；最终底盘使用中央低电流手动策略，未启用实测功率闭环。真实热量/许可/拨盘参考、视觉会话和功率模型仍是后续独立能力，不靠追加旧 conf 宣称完成。上车结果按阶段单独记录，编译结果不能记为硬件通过。
