# 开发专题与状态记录

本目录保存设计方案、实施记录、调参手册和现场排障记录。查当前接口、启动顺序和返回值时，以源码及[文档中心](../README.md)为准；本目录不定义现行 API 契约。

索引按 `dd3f673`（2026-11-15）整理。状态定义：

- **现行参考**：正文与当前代码一致，可作为操作或调参依据。
- **待实施/决策**：方案尚未落地，或仍是待填决策清单。
- **现场排障**：针对具体故障的记录，步骤可复用，但通道号和参数需按当前固件替换。
- **已落地历史**：方案已实现，正文保留旧接口、旧参数或旧结论，只能用于追溯；当前行为看“现行入口”。
- **已归档**：内容已被完全推翻，见 [archive/](archive/README.md)。

## 现行参考

| 文档 | 用途 | 核对依据 |
|---|---|---|
| [云台双轴响应 VOFA 通道说明](云台双轴响应VOFA通道说明.md) | 正式云台 schema 113、32 通道映射、有效位与窗口事件 | `include/robotics/vehicle/diagnostics.hpp`、`applications/sentry_gimbal/src/main.cpp` |
| [云台双轴 MPC 响应逐轮调参手册](云台双轴MPC响应逐轮调参手册.md) | 按轮次采集、判读与回退调参 | schema 113 与 `gimbal_profile.hpp` |
| [云台 MPC 轨迹跟踪响应优化实施指南](云台MPC轨迹跟踪响应优化实施指南.md) | 控制结构与 B1/B2/C 已实施说明 | `gimbal_axis.hpp`、`gimbal_executor.hpp` |
| [云台 MPC 测试 Pitch 最低点排查指南](云台MPC测试Pitch最低点排查指南.md) | Pitch 最低点与参考异常的排查 | 文首已按 schema 113 更新 |
| [实车诊断输出编译开关](实车诊断输出编译开关.md) | Log/Vofa 单一输出选择与空实现 | `diagnostics.hpp`、`lib/robotics/CMakeLists.txt` |
| [CAN1 单 3508 电机样例实现指南](CAN1单3508电机样例实现指南.md) | 最小 CAN 速度闭环教学 | `samples/motor/m3508_can1_speed_control/` |
| [MC02-WS2812 状态灯](MC02-WS2812状态灯.md) | 灯语与驱动边界 | `drivers/led/status_led.cpp` |
| [卡尔曼滤波设备树取舍分析与迁移指南](卡尔曼滤波设备树取舍分析与迁移指南.md) | 为何使用 C 库而非 DTS 驱动 | `include/control/kalman_filter.h`、`lib/control/kalman_filter.c` |

## 待实施 / 待决策

| 文档 | 当前状态 |
|---|---|
| [云台 GimbalAxis 与 Gimbal 分层重构方案](云台GimbalAxis与Gimbal分层重构方案.md) | `GimbalAxis` 已实现；`Gimbal`/`DualYawCoordinator` 层尚未落地，可继续作为后续任务输入。 |
| [门控、回退与许可删留决策清单（180 项）](SkyWalker_门控删留决策清单_180项.md) | 基于 `main@233cb5349328` 的审查快照，删留决定待逐项填写；行号链接不代表当前 HEAD。 |

## 现场排障记录

| 文档 | 适用性 |
|---|---|
| [双路电机启动一秒停机定位](双路电机启动一秒停机定位.md) | 2026-10-05 串口现场记录；`ready/Group` 语境已过时，CAN 排查思路仍可复用。 |
| [外置 IMU 新固件 RS485 协议不兼容与 NaN 定位指南](外置IMU新固件RS485协议不兼容与NaN定位指南.md) | RS485 主动上报改为请求应答的定位过程；实车头部 IMU 已迁到 CAN3。 |
| [大 Yaw 突发跟转诊断操作指南](大Yaw突发跟转诊断操作指南.md) | 双板抓波形流程可复用；通道表基于旧 schema 102/103/104/106，使用前必须换用当前 schema 113/109。 |
| [舵轮零指令停机现场](舵轮零指令停机现场.md) | CAN2 Bus-off/零指令停机样本。 |
| [rm_typec 设备树 SPI 频率属性诊断指南](rm_typec设备树SPI频率属性诊断指南.md) | 编辑器 binding 误报排查；BMI08x SPI 节点仍存在。 |

## 已落地历史

以下文档的方案已实现，但正文含旧 API、旧参数或已删除机制。只用于理解演进；当前行为以“现行入口”为准。

| 文档 | 现行入口与过时点 |
|---|---|
| [命令来源注册与后台仲裁服务实施指南](命令来源注册与后台仲裁服务实施指南.md) | `command_manager.hpp`；已新增 Navigation 角色、`setOperatorGate()`、`setPlantState()`，正文容量/角色描述过时。 |
| [云台视觉姿态回传与外置 IMU 接管方案](云台视觉姿态回传与外置IMU接管方案.md) | `gimbal_profile.hpp` 的 `headSensorConfig()`；当前为 CAN3 `DmImuCanSource` 主动四元数上报。 |
| [双 Yaw 速度加速度分配与视觉轨迹控制方案](双Yaw速度加速度分配与视觉轨迹控制方案.md) | `dual_yaw_motion_allocator.hpp`；正文“尚未接入”已过时，分配器已在正式链路。 |
| [哨兵参考工程对照与大 Yaw 原生速度模式](哨兵参考工程对照与大Yaw原生速度模式.md) | `motion_profile.hpp` 默认 `BigYawBackend::NativeVelocity`；正文“默认关闭”过时。 |
| [哨兵参考工程移植验收](哨兵参考工程移植验收.md) | `calibration.hpp`；候选零位/增益已被后续标定取代，当前前向零位 0.048638 rad。 |
| [哨兵小陀螺变速提速与前进方向校准实施指南](哨兵小陀螺变速提速与前进方向校准实施指南.md) | `spinRateCurveRadS()` 与 `SpinSpeedProfile`；固定 0.5 rad/s 已被变速曲线取代。 |
| [哨兵空闲巡航与搜索模式规划](哨兵空闲巡航与搜索模式规划.md) | 已实现 `AutonomousMode::Cruise/CruiseSpin`、`GimbalBehavior::Scan`；正文“暂不实现”过时。 |
| [哨兵巡航模式与遥控掉线保持实施指南](哨兵巡航模式与遥控掉线保持实施指南.md) | 巡航与掉线保持已实现；正文 schema 111→112 的升级建议过时，当前云台为 113。 |
| [多品牌电机驱动重构架构与手工实施指南](多品牌电机驱动重构架构与手工实施指南.md) | `Motor/Group/CanBus` 架构仍有效；`ready()`/`clearFault()` 已删除。 |
| [大 Yaw 编码器展开与世界航向闭环标定排障](大Yaw编码器展开与世界航向闭环标定排障.md) | `native_unwrapped_position_rad`、`BigYawWorldHeadingFollower` 仍有效；文中零位为历史值。 |
| [安全策略收敛与执行层自恢复实施指南](安全策略收敛与执行层自恢复实施指南.md) | `GlobalSafetyManager` 已删除；现行是 CommandManager 与逐轴自动恢复。 |
| [小 Yaw 机械中心标定指南](小Yaw机械中心标定指南.md) | 中心标定思路可复用；数据流仍写 `YawCenteringController`，现由分配器回中，中心值见 `calibration.hpp`。 |
| [板间通信统一传输接口与三种后端实施指南](板间通信统一传输接口与三种后端实施指南.md) | 三种后端仍在 `include/communication/interboard/`；线上协议固定 v4。 |
| [正式哨兵键鼠控制启用指南](正式哨兵键鼠控制启用指南.md) | `input_profile` 已在应用 main 默认设置，手工步骤已完成。 |
| [电机 Session 首批实现与接入说明](电机Session首批实现与接入说明.md) | `MotorSession` 已无定义，仅作迁移记录。 |
| [电机与 CAN 控制链收敛重构方案](电机与CAN控制链收敛重构方案.md) | 持续目标与逐轴恢复已落地；正文把 `MotorSession` 当已实现的部分过时。 |
| [电机持续指令与独立自动恢复重构实施指南](电机持续指令与独立自动恢复重构实施指南.md) | 主要契约已落地；正文仍称两个 sentry main 调用 `vehicle_bench`，实际各自持有 `main.cpp`。 |
| [电机驱动审查复核与线程弹性实施指南](电机驱动审查复核与线程弹性实施指南.md) | 审查基线 `bc1e07b` 的 F1–F6 已随电机重构变化，现行 API 无 `ready()`/`clearFault()`。 |
| [M2006 拨弹盘连续位置目标与卡弹恢复实施指南](M2006拨弹盘连续位置目标与卡弹恢复实施指南.md) | 连续目标与卡弹策略已落地，但正文 `drive.ready()`/Stopping 状态机基于旧驱动；当前样例只用 `enable()/update()` 与快照状态。 |
| [舵轮参考移植](舵轮参考移植.md) | 应用已各自装配；`vehicle_bench` 只被 `samples/robotics/vehicle_integration` 使用。 |
| [舵轮已调通源码对照与修改建议](舵轮已调通源码对照与修改建议.md) | 2026-10-02 参考对照；当前无 `ready/Group` 联锁，cos³ 与翻转迟滞成为现行行为。 |
| [逐模块上车与协议配置收敛方案](逐模块上车与协议配置收敛方案.md) | 21 个 bringup 工程与共享配置已落地；正文仍保留“建议新建/尚未实施”语境。 |
| [项目优化与逐级整车验证样例实施指南](项目优化与逐级整车验证样例实施指南.md) | `RecoveryGate` 已删除；现行恢复契约见电机主题文档。 |
| [鼠标发射手势与单双发控制实施指南](鼠标发射手势与单双发控制实施指南.md) | `MouseShooterGesture` 已由 `CommandArbiter` 使用，正文“业务代码尚未修改”过时。 |
| [三源命令管理 sample 手工实施指南](三源命令管理sample手工实施指南.md) | `samples/robotics/command_manager/` 已存在；接线与操作以 sample README 为准。 |
| [SkyWalker 双 Yaw 加速度协同控制实施方案](SkyWalker_双Yaw加速度协同控制_实施方案.md) | 分配器已实现；参数以 `dualYawAllocatorConfig()` 为准。 |
| [SkyWalker 底盘方向偏移与小陀螺漂移修复执行任务书](SkyWalker_底盘方向偏移与小陀螺漂移修复_执行任务书.md) | 展开位置与航向跟随已落地；文中示例零位/行号过时。 |
| [SkyWalker 门控精简修改方案（按 dev 批注）](SkyWalker_门控精简修改方案_按dev批注.md) | P1 编译期检查与逐轮 cos³ 已落地；0.10/0.40 rad 对齐联锁已删除。 |
| [samples 遥控控制统一改造指南](samples遥控控制统一改造指南.md) | 协议标识已从 3 升级为 v4；当前 sample 目录结构与遥控操作以各自 README 为准。 |

## 已归档

以下 6 篇已被当前代码完全推翻，移入 [archive/](archive/README.md)，不再作为任何操作依据：

- 云台遥控样例安全策略阅读指南
- 双 Yaw 回滚与控制分工分析
- 命令管理器调用接口简化实施指南
- 哨兵 IMU 航向控制验收
- 小 Yaw 低速偏慢的定位与提速参数指南
- 当前项目完成度分析与后续实施清单

## 维护约定

- 新增方案先在本索引登记状态；方案落地后补“现行入口”，并删除正文中已失效的施工步骤。
- 被新方案完全取代时移入 `archive/`，同时更新所有指向它的链接。
- 本目录不复制 `docs/api` 的签名、默认值或错误码；需要契约时链接过去。
- 现场排障记录必须标注所依据的 schema/固件版本，避免旧通道表被直接复用。
