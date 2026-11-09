# 双主控整车应用（当前正式入口）

核对日期：2026-11-10；代码基线：`dev@bad529abe5d682797ca27d49078bbf0a9b00266b`，该提交已同步到 `main`。两个应用分别在自己的 `main.cpp` 装配，不调用旧 `vehicle_bench.hpp::run()`。以下是**软件当前实现**，不表示已经实板验收。独立惯性试验与后续视觉方案不能直接当作正式应用的接入状态。

## 两板职责与物理资源

| 板卡 | 入口与当前输入 | 执行与通信 |
| --- | --- | --- |
| 云台 | [sentry_gimbal](../../applications/sentry_gimbal/README.md)：UART5 DR16、SPI 板载 BMI088（随大 Yaw 转动） | CAN1 小 Yaw、CAN2 Pitch；USART10 PE3/PE2 板间 v4、460800 8N1；UART7 诊断 |
| 底盘 | [sentry_chassis](../../applications/sentry_chassis/README.md)：USART1 板间 v4、底盘板载 BMI088 | CAN1 四舵向、CAN2 四轮驱、CAN3 大 Yaw；UART7 诊断 |
| 暂未装配 | 当前正式云台不实例化 CAN3 外置头部 IMU、视觉接收器、裁判接收器或发射执行器 | CAN3 的独立 DM IMU 驱动/样例仍可保留；功能存在不代表正式入口启用 |

两板 UART7 `diagnostic-uart` 默认候选 115200 8N1，Log/VOFA 二选一，DAPLink 实际接线仍须现场确认。连接与参数见[共享配置](../../config/vehicle/installed/README.md)；正式云台接入范围以应用 `main.cpp` 为准，不要仅根据 overlay 或 Kconfig 中的驱动存在推断实例已启动。

## 当前控制数据流

```text
云台 main：5 ms 控制、10 ms 板间目标发布
  DR16 → CommandManager 与真实遥控授权
  小 Yaw / Pitch 编码器 + 板载 BMI088 载体角速度
       → 编码器手动双轴目标 → GimbalExecutor → CAN1 / CAN2
  小 Yaw 相对中心偏角 + 有效载体航向 → 回中外环
       → 0x0402 大 Yaw 世界角速度请求（最大 1 rad/s）
  板载 BMI088 航向/去偏角速度 → 0x0404（32B）
  云台系行走目标 → 0x0101（32B）
  → USART10 Endpoint v4

底盘 main：2 ms 控制
  USART1 Endpoint v4 → boot / 命令年龄 / 许可检查
  0x0101 云台系平移 + 大 Yaw 原生角与标定前向零位
       → 底盘系速度 → ChassisExecutor → 四舵轮
  0x0402 世界角速度 + 双板真实陀螺观测
       → 补偿为关节相对速度、按电机上限限幅
       → BigYawExecutor → CAN3
  各物理 CAN 每周期由应用唯一提交者 commit
```

本地小 Yaw/Pitch 手动控制不把外置 CAN3 IMU 或板载航向有效性作为启动前提。板载航向有效时，小 Yaw 通过补偿载体角速度来维持世界系手动速度意图；板载航向失效时**大 Yaw 撤权**，不应将板载航向解释为枪口独立头部姿态。板载 EKF 静态零偏初始化、100 ms 短时 Degraded 预测窗口及诊断编号见[云台 README](../../applications/sentry_gimbal/README.md)。

大 Yaw 回中参数：中心偏角增益 6.0，世界系限速 1 rad/s，加速度限幅 4 rad/s²；需要小 Yaw 有效反馈、本地轴运行、板载航向新鲜及真实对端状态。底盘的世界角速度补偿使用底盘 BMI088 与云台发来的 `0x0404`，再按关节限幅饱和。平移必须使用 **`native_position_rad`** 与同坐标系的前向零位；大 Yaw 原生角失效时清零平移，不依赖历史连续角展开参考。舵轮具有最终优化转角下的驱动联锁（0.10/0.40 rad）与逐轮 cos³ 补偿，见[移植验收记录](../dev/哨兵参考工程移植验收.md)。

## 手动授权与协议边界

上电静止约 2 秒，**左 Down、右 Middle**，摇杆/拨轮归中至少 500 ms；再左 Down→Middle 解锁、左 Up 运行。右摇杆控制小 Yaw/Pitch，左摇杆控制前后和横移（各最大 0.3 m/s）；进入手动后右拨杆必须先经过 Middle，再主动切 Down 才请求小陀螺 0.5 rad/s。左 Down 主动停止；曾上线链路丢失或对端 Boot ID 改变要重新解锁。

当前线上仍为与 `JJL0612/sentry_onlychassis@dcc8d4e` 对齐的**参考 v4**：`0x0402` 只使用 mode=0/1，mode=1 是**头部对世界角速度意图**；`0x0404` 为 32 字节板载载体航向/去偏角速度，**不是旧 SkyWalker `0x0405/40B`**；`0x0101` 的平移位于云台系；独立 `0x0301` 授权不是该参考底盘的运行前提。协议详见[字节合同](../protocols/interboard-v4.md)，不能只烧录一端就改变协议。

## 构建、观测与未完成项

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
west build -b dm_mc02/stm32h723xx applications/sentry_chassis -d ../build/applications/sentry_chassis
```

诊断输出由 [diagnostics_config.hpp](../../include/robotics/vehicle/diagnostics_config.hpp) 选择，固定 19 通道；云台/底盘通道 13、18 的含义并不相同，分别以两个应用 README 为准。发射、视觉、实测功率闭环与实板方向/停机时限验收均未因本次分支同步而完成。2026-10-25 的旧基线有 27 项编译/链接记录，**本轮没有重新执行构建、测试、烧录或硬件验证**；请以 CI 与现场验收记录确认新基线。
