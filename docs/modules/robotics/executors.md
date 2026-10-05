# 机构执行器、惯性适配与大 Yaw 回中

本文对齐 `main@99a97c9`（2026-10-05）。执行器位于 `include/robotics/` 与 `lib/robotics/`；两套 sentry 应用仅调用公共 `samples/robotics/common/vehicle_bench.hpp`。应用持有 Motor、Group、CanBus、输入来源和线程，完成 attach/start，并在所有机构暂存输出后对每条物理 CAN commit 一次。执行器不创建或提交 CAN 总线。

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

`yaw_output_valid`、`pitch_output_valid` 由惯性适配器声明本轴数学输入是否够用，独立作废对应计算输出。`GimbalAxis::poll()` 当前返回 `feedback_healthy/error`，没有 `ready_for_enable`。有限位轴须使用真实标定范围与可信参考；Rate 目标时间轴不因电机离线重置，PositionMotor 在本轴恢复时自动重建控制历史。

## InertialGimbalAdapter

头文件：[inertial_gimbal.hpp](../../../include/robotics/gimbal/inertial_gimbal.hpp)。`update(const InertialGimbalInputs &, now_us)` 接收头部 IMU Snapshot、yaw/pitch MotorSnapshot、惯性 GimbalCommand、原始 source_stamp 和 prerequisites_ready，返回 InertialGimbalOutput；`suspend(now_us, reason, error)` 撤销业务输入。

输出 `command` 是给 GimbalExecutor 的关节 Rate 命令，`source_stamp` 保留原输入时间，两个 output_valid 分别表示数学测量足够，`stabilization_valid` 用于回中/供弹业务。适配器只做坐标与目标计算，没有电机/总线所有权。

默认头部 IMU 超时 20 ms，机械测量 50 ms，来源/命令 100 ms，周期上限 20 ms；Pitch 默认锁定。配置包括两轴方向、姿态误差增益、gyro damping、惯性 Pitch 范围、稳定误差阈值和姿态质量要求。IMU frame_id/epoch 是坐标身份；视觉目标与头部参考必须匹配，不能把世界角直接送机械编码器位置。电机恢复不重定义惯性业务目标。

## 小 Yaw 外环与 BigYawExecutor

头文件：[yaw_centering.hpp](../../../include/robotics/gimbal/yaw_centering.hpp)、[big_yaw_executor.hpp](../../../include/robotics/execution/big_yaw_executor.hpp)。

`YawCenteringController::update(const YawCenteringInputs &, now_us)` 使用已标定小 Yaw 中心、关节角及其采样时间、原始来源、head_stable 与 permission_valid，生成限速/斜坡后的大 Yaw 角速度。中心与编码器零点分开标定。默认死区 0.05 rad、迟滞 0.025 rad、kp=0.5、最大 0.3 rad/s、加速度 0.4 rad/s²。`reset()` 清外环历史。

`BigYawExecutor` 注入独立 Motor、VelocityMotor::Config 与自身 Config；`begin()` 配置，`update(const BigYawExecutionInputs &, now_us)` 消费请求，`suspend(...)` 撤销输入，`feedback()` 返回 BigYawFeedback 值副本。它只控制连续旋转速度，不需要绝对机械零点，不从八电机轮控状态取得许可。

v4 的 BigYawRequest 保留 receiver_boot_id、source_sequence 和来源/命令/权限年龄。resume_generation 已删除；发送不等电机 ready/armed。真实板重启、链路或输入过期仍使请求失效。BigYawFeedback 保留真实状态生产年龄与实际速度 valid，未获得有效测量时不得把目标速度作为反馈。

## ChassisExecutor 与 SwerveHardware

头文件：[chassis_executor.hpp](../../../include/robotics/chassis/chassis_executor.hpp)、[swerve_hardware.hpp](../../../include/robotics/chassis/swerve_hardware.hpp)。

`ChassisExecutor(SwerveHardware &, const SwerveChassis::Config &, const Config &)` 注入硬件映射，不绑定 Endpoint。`begin()` 校验硬件/运动学/功率配置；`update(const ChassisExecutionInputs &, now_us)` 消费 command、source_stamp、permission、power_budget、measured_power 与运行/急停条件，返回 RunStatus。`output()`、`feedback()`、`effortScale()`、`steerEffortScale()` 和 `powerMeasurement()` 为执行线程观察入口。

SwerveHardware 的 Motor 顺序为舵向 FL/FR/RL/RR，再轮驱 FL/FR/RL/RR。`begin()` 校验，`read(ChassisFeedback &)` 逐轴填有效性和执行版本，`stage(output, steer_scale, drive_scale)` 逐轴暂存/作废 effort，`suspend()` 明确批量停止。它不 commit CAN。

舵向需要新鲜绝对角与速度，轮驱只需要速度；ModuleFeedback 的 steer_valid/drive_valid 和 ModuleOutput 的 steer_output_valid/drive_output_valid 相互独立。SwerveModule 有最短转向、翻转迟滞、转向斜坡及 Hold/Coast；无全体对齐互锁、drive_ready 或 cos 缩速。转向掉线不让轮驱停止目标生产。位置/角速度分别为 rad/rad·s⁻¹，轮线速度为 m/s；几何占位值须实测。

功率模式需要新鲜真实预算与 PowerMeasurement；`IPowerMeasurementSource::sample()` 返回的原始 stamp 不可刷新。预算不是实测功率。`power_control_calibrated=false` 时使用台架独立缩放和电流上限；功率控制/模型标定与 estimated power 开关必须按实物确认。公共 Config 的默认缩放/上限不同于整车 profile，实际中央 profile 当前舵向 0.8 A、轮驱 0.5 A、独立缩放为 1。

## ShooterExecutor

头文件：[shooter_executor.hpp](../../../include/robotics/shooter/shooter_executor.hpp)。构造注入左/右摩擦轮和拨盘 Motor、摩擦对/拨盘两个 Group、速度/位置控制配置；`begin()` 只配置，`update(const ShooterExecutionInputs &, now_us)` 返回 ShooterStatus，`suspend(...)` 撤销输入。

输入包括 command/source_stamp、真实 permission、ShooterHeatState、带时间的 dial_home_reference、云台 RunStatus、allow_feed 和急停条件。输出分为 friction/feed 状态，另有 friction_ready、dial_busy、jammed、last_event_id、shots、dial_target_rad 与 reserved_heat。摩擦对和拨盘分别执行；共享 DJI CAN 在所有机构 update 后统一 commit。

摩擦速度稳定驻留、热量余量、拨盘可信原点、头部稳定/云台状态均属于发射业务条件。单发以 fire_event_id 与原始事件时间去重；忙碌、失效或恢复中不能执行的旧事件消费丢弃，恢复不重放。shots 是软件执行统计，不能直接当裁判实测弹数。卡滞与热量业务策略按现行实现保留，电机自动清错不能代替解除机械卡弹。

整车 `run(IPowerMeasurementSource *, IShooterHeatSource *, IDialHomeSource *)` 支持注入真实来源；默认 PendingPowerSource、PendingHeatSource、PendingDialHomeSource 无效，功率/有载发射配置因缺测量保持不可执行。视觉 AB 反馈还缺可信弹速/弹数与显式视觉参考会话，TX 默认关闭。

## 在应用周期中组合

```cpp
// application 已静态创建 hardware/adapter，并完成 attach/start/begin。
// source、IMU、MotorSnapshot 均保留原采样时间。
const auto adapted = adapter.update(inertial_inputs, now_us);
gimbal_inputs.command = adapted.command;
gimbal_inputs.source_stamp = adapted.source_stamp;
gimbal_inputs.yaw_output_valid = adapted.yaw_output_valid;
gimbal_inputs.pitch_output_valid = adapted.pitch_output_valid;
const auto gimbal_status = hardware.gimbal.update(gimbal_inputs, now_us);
const auto shooter_status = hardware.shooter.update(shooter_inputs, now_us);
const int published = hardware.commit(); // DJI CAN1 和 Pitch CAN2 各一次
// published 错误记录诊断；输入/急停仍由本周期显式撤销。
```

这是公共 vehicle_bench 的周期组织示意，构造、设备和权限上下文以其源码为准。配置阶段、两板分工及默认阻断见[双主控应用](../../applications/dual-controller.md)，持续控制/取消版本见[电机工作流](../../guides/motor-workflow.md)。软件入口存在与单舵轮调通均不代表整车、视觉、功率或有载发射已验收。
