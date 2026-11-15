# 机构执行器、惯性适配与双 Yaw 分配

应用装配概览按 `dd3f673`（2026-11-15）更新；执行器接口细节仍以源码为准。公共执行器 API 与正式应用装配是两回事。执行器位于 `include/robotics/` 与 `lib/robotics/`；[云台应用](../../../applications/sentry_gimbal/README.md)与[底盘应用](../../../applications/sentry_chassis/README.md)各自在自己的 `src/main.cpp` 中构造对象、初始化资源并执行控制循环。它们不调用公共 `vehicle_bench.hpp`。应用持有 Motor、Group、CanBus 和输入来源，完成 attach/start，并在所有机构暂存输出后对每条物理 CAN commit 一次。执行器不创建或提交 CAN 总线。

应用与[独立上车工程](../../../samples/bringup/README.md)共享 [installed 物理配置](../../../config/vehicle/installed/README.md)及 `include/robotics/vehicle/` 中的标定与纯配置工厂。正式云台使用 CAN1 小 Yaw及发射电机、CAN2 Pitch、板载 SPI BMI088 和 CAN3 外置头部 IMU；底盘使用 CAN1 舵向、CAN2 轮驱、CAN3 大 Yaw。UART7 为两板 DAPLink 诊断候选口，板间固定为云台 USART10、底盘 USART1。资源共用不会替代现场接线和标定确认。

所有有状态 update、begin、suspend 和 status 引用访问由一个执行线程拥有。跨线程使用 SnapshotCache 或 Endpoint 的值副本；读者不刷新生产时间。SnapshotCache 争锁使用有界短自旋，首次尚无发布时 snapshot 返回 -EAGAIN，ISR 调用返回 -EWOULDBLOCK。初始化配置失败保持 Blocked；运行意图由新鲜输入决定，电机等待/故障不阻止其他轴持续生产目标。Group 只是批量启停，Motor/CAN 独立恢复。

## 公共输入与状态

| 字段 | 含义 |
|---|---|
| command.stamp | 命令生产时间，ms 与 sequence |
| source_stamp | 遥控/视觉原始生产时间，core::Stamp 使用 µs |
| permission | 实际输出许可与其原始时间；require_permission 决定是否必须存在 |
| transport_ready | 本地拓扑/I/O 配置是否能够执行；不等于每台电机反馈都在线 |
| emergency_stop / clear_estop | 输入级撤销/清除请求；不代替物理动力切断 |
| RunStatus.requested | 当前输入运行意图 |
| member_count / active_count / waiting_count | 成员诊断统计，不构成全体就绪门禁 |
| RunStatus.stamp | 执行线程的实际状态生产时间与序号；通信不能重新盖戳 |
| ready / state / reason / error | 当前观测，不能据此停止上层目标生产 |

持续调用 update 后读取完整返回值记录诊断。只有用户停止、输入过期/权限撤销/急停或初始化配置错误才撤销对应业务输出；电机协议恢复和 CAN TX 错误保留运行意图。`begin()` 返回 0 代表配置成功，尚无真实电机反馈也可以成功。输入超时不是等待 Motor ready 的理由。

## 双轴 GimbalExecutor

头文件：[gimbal_executor.hpp](../../../include/robotics/gimbal/gimbal_executor.hpp)。构造注入 yaw/pitch 两个 Motor、批量 Group、两套 PositionMotor/GimbalAxis 配置与 Executor::Config。

| API | 契约 |
|---|---|
| `int begin()` | 在应用 attach/start 后配置两轴，不 enable、不 commit；配置/控制器错误以负 errno 返回 |
| `RunStatus update(const GimbalExecutionInputs &, core::TimeUs now_us)` | 检查输入年龄/许可/周期，持续更新两轴机械目标；now_us 为本地单调微秒时间 |
| `RunStatus suspend(now_us, reason, error=0, blocked=false)` | 显式撤销输入；不是电机掉线后的周期恢复操作 |
| `yawTargetRad()` / `pitchTargetRad()` | 执行线程内读取机械目标，rad |
| `const RunStatus &status()` | 仅执行线程读取内部引用；对外发布值副本 |

`yaw_output_valid`、`pitch_output_valid` 可由惯性适配器声明本轴数学输入是否够用；当前正式云台同时使用分配器关节轨迹、手动编码器轴目标和视觉姿态外环，三者按本周期命令来源选择。`GimbalAxis::poll()` 当前返回 `feedback_healthy/error`，没有 `ready_for_enable`。有限位轴须使用真实标定范围与可信参考；Rate 目标时间轴不因电机离线重置，PositionMotor 在本轴恢复时自动重建控制历史。

## InertialGimbalAdapter

头文件：[inertial_gimbal.hpp](../../../include/robotics/gimbal/inertial_gimbal.hpp)。`update(const InertialGimbalInputs &, now_us)` 接收头部 IMU Snapshot、yaw/pitch MotorSnapshot、惯性 GimbalCommand、原始 source_stamp 和 prerequisites_ready，返回 InertialGimbalOutput；`suspend(now_us, reason, error)` 撤销业务输入。

输出 `command` 是给 GimbalExecutor 的关节 Rate 命令，`source_stamp` 保留原输入时间，两个 output_valid 分别表示数学测量足够，`stabilization_valid` 用于回中/供弹业务。适配器只做坐标与目标计算，没有电机/总线所有权。

库 Config 默认头部 IMU 超时 20 ms，机械测量 50 ms，来源/命令 100 ms，周期上限 20 ms；Pitch 默认锁定。**当前正式云台已实例化 `InertialGimbalAdapter`（`head_control`）并每周期 update**，视觉 Yaw 参考来自分配器的 `head_reference_*`，Pitch 使用头部四元数测得的关节保持；两个惯性上车工程可分别固定为仅 Yaw 或双轴，不靠锁轴宏选择。配置包括两轴方向、姿态误差增益、gyro damping、惯性 Pitch 范围、稳定误差阈值和姿态质量要求。IMU frame_id/epoch 是坐标身份；视觉目标与头部参考必须匹配，不能把世界角直接送机械编码器位置。电机恢复不重定义惯性业务目标。

## 双 Yaw 分配与 BigYawExecutor

头文件：[dual_yaw_motion_allocator.hpp](../../../include/robotics/gimbal/dual_yaw_motion_allocator.hpp)、[big_yaw_executor.hpp](../../../include/robotics/execution/big_yaw_executor.hpp)。

`DualYawMotionAllocator::update(const DualYawAllocatorInputs &, now_us)` 把头部世界速率意图拆成小 Yaw 关节轨迹与大 Yaw 世界意图：`head` 是经行程/制动包络约束后小 Yaw 仍能实现的头部速率，`big.rate_rad_s` 发往底盘；`joint_rate_rad_s` 与 `joint_position_rad` 已按 `joint_direction` 只换算一次到编码器坐标。手动中心项、视觉角度误差、载体加速度窗、制动包络与静止锁存均在分配器内完成，正式参数见 `dualYawAllocatorConfig()`。

`YawCenteringController`（[yaw_centering.hpp](../../../include/robotics/gimbal/yaw_centering.hpp)）已作废，仅保留给历史样例和 `tests/robotics_contracts`；`manualCenteringConfig()` 不代表当前回中路径，不要再按旧接口新增调用。

`BigYawExecutor` 注入独立 Motor、VelocityMotor::Config 与自身 Config；`begin()` 配置，`update(const BigYawExecutionInputs &, now_us)` 消费请求，`suspend(...)` 撤销输入，`feedback()` 返回 BigYawFeedback 值副本。默认控制连续旋转速度，不需要绝对机械零点，不从八电机轮控状态取得许可。可选 heading_lock 后端消费独立头部 IMU，以世界航向 P/实测角速度 PI 产生 MIT 力矩；只有 MIT 模式和参考已确认才允许配置，不自动回退。当前模式与验收前提见[底盘应用 README](../../../applications/sentry_chassis/README.md)和[板间 v4 契约](../../protocols/interboard-v4.md)。

v4 的 BigYawRequest 保留 receiver_boot_id、source_sequence 和来源/命令/权限年龄。resume_generation 已删除；库不以电机 ready/armed 作为发送授权。独立双 Yaw 上车入口额外要求真实反馈新鲜才发非零速度，缺测量时最多发已授权零速 FollowCenter。真实板重启、链路或输入过期仍使请求失效；当前应用的操作授权发布器在曾上线的对端掉线或 boot 改变时撤销本地 RC 解锁，需要重新归中解锁。BigYawFeedback 保留真实状态生产年龄与实际速度 valid，未获得有效测量时不得把目标速度作为反馈。速度单位、接口轴到关节轴换算与线上方向适配见[协议 v4 契约](../../protocols/interboard-v4.md)。

## ChassisExecutor 与 SwerveHardware

头文件：[chassis_executor.hpp](../../../include/robotics/chassis/chassis_executor.hpp)、[swerve_hardware.hpp](../../../include/robotics/chassis/swerve_hardware.hpp)。

`ChassisExecutor(SwerveHardware &, const SwerveChassis::Config &, const Config &)` 注入硬件映射，不绑定 Endpoint。`begin()` 校验硬件/运动学/功率配置；`update(const ChassisExecutionInputs &, now_us)` 消费 command、source_stamp、permission、power_budget、measured_power 与运行/急停条件，返回 RunStatus。`output()`、`feedback()`、`effortScale()`、`steerEffortScale()` 和 `powerMeasurement()` 为执行线程观察入口。

SwerveHardware 的 Motor 顺序为舵向 FL/FR/RL/RR，再轮驱 FL/FR/RL/RR。`begin()` 校验，`read(ChassisFeedback &)` 逐轴填有效性和执行版本，`stage(output, steer_scale, drive_scale)` 逐轴暂存/作废 effort，`suspend()` 明确批量停止。它不 commit CAN。

舵向需要新鲜绝对角与速度，轮驱只需要速度；ModuleFeedback 的 steer_valid/drive_valid 和 ModuleOutput 的 steer_output_valid/drive_output_valid 相互独立。SwerveModule 有最短转向、翻转迟滞（1.658/1.483 rad）、转向斜坡及 Hold/Coast；不使用四轮全体对齐互锁，只按最终优化角的 cos³ 逐轮缩速（`cosine_compensation` 默认 true）。0.10/0.40 rad 对齐联锁已删除。转向掉线不让轮驱停止目标生产。位置/角速度分别为 rad/rad·s⁻¹，轮线速度为 m/s；几何占位值须实测。

功率模式需要新鲜真实预算与 PowerMeasurement；`IPowerMeasurementSource::sample()` 返回的原始 stamp 不可刷新。预算不是实测功率。`power_control_calibrated=false` 时使用台架独立缩放和电流上限；功率控制/模型标定与 estimated power 开关必须按实物确认。公共 Config 的默认缩放/上限不同于整车 profile，实际中央 profile 当前舵向 2.0 A、轮驱 20.0 A、独立缩放为 1。

## ShooterExecutor

头文件：[shooter_executor.hpp](../../../include/robotics/shooter/shooter_executor.hpp)。构造注入左/右摩擦轮和拨盘 Motor、摩擦对/拨盘两个 Group、速度/位置控制配置；`begin()` 只配置，`update(const ShooterExecutionInputs &, now_us)` 返回 ShooterStatus，`suspend(...)` 撤销输入。

输入包括 command/source_stamp、真实 permission、ShooterHeatState、带时间的 dial_home_reference、云台 RunStatus、allow_feed 和急停条件。输出分为 friction/feed 状态，另有 friction_ready、dial_busy、jammed、last_event_id、shots、dial_target_rad 与 reserved_heat。摩擦对和拨盘分别执行；共享 DJI CAN 在所有机构 update 后统一 commit。

摩擦速度稳定驻留、热量余量、拨盘可信原点、头部稳定/云台状态均属于发射业务条件。单发以 fire_event_id 与原始事件时间去重；忙碌、失效或恢复中不能执行的旧事件消费丢弃，恢复不重放。shots 是软件执行统计，不能直接当裁判实测弹数。卡滞与热量业务策略按现行实现保留，电机自动清错不能代替解除机械卡弹。

当前最终云台已构造 ShooterExecutor、摩擦轮和拨盘，并注册视觉/导航来源、绑定板间裁判许可；发射执行器仍设 `require_heat=false`、`require_permission=false`，热量/发射许可和拨盘归位输入尚未接通。最终底盘使用受中央电流上限约束的手动执行配置，`require_power_budget=false`、`power_control_calibrated=false`、`allow_estimated_power=false`，未接通实测功率。

历史 [vehicle_integration](../../../samples/robotics/vehicle_integration/README.md) 仍通过 `vehicle_bench.hpp` 的 `run(IPowerMeasurementSource *, IShooterHeatSource *, IDialHomeSource *)` 注入来源；其默认 PendingPowerSource、PendingHeatSource、PendingDialHomeSource 无效。该历史宏组合入口不再决定两个最终应用的装配方式。视觉 AB 反馈所需的可信弹速/弹数与显式视觉参考会话仍待补齐，不能通过打开历史功能开关认定已完成。

## 当前正式云台的周期组合

```text
RC/CommandManager 快照 → GimbalCommand（手动/巡航/视觉）
DualYawMotionAllocator.update() → 小 Yaw 关节轨迹 + 大 Yaw 世界意图
板载 BMI088 → 载体航向/角速度 → 0x0404 与分配器载体补偿
外置头部 IMU → InertialGimbalAdapter（视觉姿态外环，yaw_reference_*）
GimbalExecutor.update() → CAN1/CAN2 分别 commit
0x0402 大 Yaw 意图 + 0x0101 行走 → Endpoint.poll()
```

当前正式云台每周期调用 `head_control.update()`；独立惯性样例仍可使用同一 `InertialGimbalAdapter` API。板载 EKF 的短暂 Degraded 预测不等于独立头部姿态，准确的有效性和诊断判定见[云台应用 README](../../../applications/sentry_gimbal/README.md)。底盘按[参考 v4](../../protocols/interboard-v4.md)消费云台系行走、板载航向与大 Yaw 世界速度命令。

配置阶段与两板分工见[双主控应用](../../applications/dual-controller.md)，持续控制/取消版本见[电机工作流](../../guides/motor-workflow.md)。诊断 Log/VOFA 的单变量选择及固定通道见[上车入口说明](../../../samples/bringup/README.md)。软件入口存在、编译通过与单舵轮调通均不代表整车、视觉、功率或有载发射已验收。
