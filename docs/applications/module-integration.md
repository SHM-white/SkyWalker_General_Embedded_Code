# 模块联动与调用路线

独立实板样例和双主控 applications 复用公开模块。应用创建设备、总线、故障组与线程；机构执行器检查反馈、准备参考、处理恢复并暂存输出，周期末由板级线程提交每条 CAN。

## 公共对象的调用顺序

| 阶段 | 对象 | 调用方责任 |
|---|---|---|
| 原始输入 | RemoteReceiver、RefereeReceiver、VisionReceiver、ImuReceiver | 保留原始 stamp、质量与参考代次 |
| 仲裁 | RemoteSource、VisionSource、RefereePermissionSource、CommandManager | 启动前注册，唯一 worker 周期采样 |
| 快照 | CommandSnapshot、RobotCommand、SnapshotCache | 非消费式读取，各消费者检查原始数据年龄 |
| 参考适配 | InertialGimbalAdapter、YawCenteringController | 区分机械角与头部惯性角；参考变化重新准备 |
| 机构执行 | GimbalExecutor、ChassisExecutor、BigYawExecutor、ShooterExecutor | 独立 RecoveryGate、握手监督、失效撤销 |
| 板级提交 | Motor、Group、CanBus | attach/start；每个执行周期每条物理 CAN 一次 commit |
| 板间传输 | InterBoardEndpoint、codec、parser、link | 原始年龄、boot ID、独立恢复代次和 V2 能力检查 |

CommandManager 提供后台服务，CommandArbiter 提供同步决策。快照读取不会消费命令；命令生产暂停后即使仲裁或通信仍运行，机构也不能把旧输入重新视为新鲜。

## 云台与头部参考

`gimbal_control` 直接用遥控验证机械双轴。`command_gimbal` 接入真实命令服务和可选裁判、视觉来源。`inertial_gimbal` 先锁定 Pitch，消费随头部运动的外置 IMU，验证转载体时小 Yaw 的补偿。

~~~text
头部惯性目标 + 原始命令时间 + 头部 IMU
  → InertialGimbalAdapter
  → 机械角速度目标
  → GimbalExecutor → GimbalAxis → Motor

小 Yaw 中心误差 + 头部稳定状态
  → YawCenteringController
  → V2 大 Yaw 请求
  → 底盘 BigYawExecutor → 速度内环
~~~

底盘车体、大 Yaw 载体、头部各有安装变换和参考 ID；机械编码器零点、小 Yaw 回中中心和惯性参考会话分别保存。IMU 断流或参考变化暂停惯性控制、回中和发射，轮控继续按自己的有效条件判断。

## 共享 CAN 与发射

`gimbal_shared_can` 将小 Yaw、两摩擦轮与拨盘接到同一 DJI 总线；Pitch 使用独立 DM 总线。`shooter_bench` 专门验证动作条件，不把 CAN 联通视为实际发射验收。

~~~text
仲裁后云台命令 → GimbalExecutor ──┐
发射事件/持续请求 → ShooterExecutor ├→ 板级执行线程 → CAN 各一次 commit
实际权限/热量/云台状态 ────────────┘
~~~

云台、摩擦轮和拨盘有独立故障组。拨盘须先满足摩擦轮就绪和业务条件；单发按事件编号去重，恢复后不重放旧事件。有载模式必须提供真实拨盘参考与热量；空载台架的相对坐标重建是单独配置。

## 四舵轮与功率

`swerve` 验证单舵轮跨两 CAN；`four_swerve` 使用完整 SwerveChassis 和八电机组。`chassis_power` 将真实裁判预算与测量链路接到 ChassisExecutor，观察缩放、功率与舵向误差。

~~~text
底盘命令 + 权限 + 预算 + 实测功率
  → ChassisExecutor → SwerveChassis / ChassisPowerLimiter
  → SwerveHardware → 八电机组
  → 舵向 CAN1 / 驱动 CAN2
~~~

预算与功率测量分别检查新鲜度。估算功率和完整模型标定均有明确配置门禁，不能以固定预算冒充实际测量。

## 双主控整车

`vehicle_integration` 和两个 applications 共用运行时，按手动、视觉观察、视觉执行、发射配置逐级组合。通信线程只读取/发布快照，5 ms 执行线程推进本板全部执行器；实际参考、源时间与独立恢复代次贯穿整条链路。

入口与阶段配置见[双主控整车框架](dual-controller.md)和[样例索引](../getting-started/samples.md)。全部标定 TODO 与实板验收步骤见[逐级实施指南](../dev/项目优化与逐级整车验证样例实施指南.md)。
