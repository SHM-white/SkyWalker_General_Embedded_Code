# 命令持续控制与电机独立恢复

更新日期：2026-10-25。CommandManager 持续采样来源、仲裁并发布 CommandSnapshot。消费者分别检查最终命令和原始来源年龄。电机暂时掉线不能撤销来源、清掉用户运行意图或停止仍可计算的目标生产。

`sourceStamp()` 位于 `command_source.hpp`，提取遥控/视觉原始生产时间。快照读取、通信重发和 CAN commit 都不续期原输入。用户主动停止、急停、来源停更以及真实板启动身份变化仍按输入有效性处理。

MotorSession、RecoveryGate 和旧 Group 授权状态机已删除。Executor 直接检查用户意图和命令年龄，每周期更新各轴；Motor/CAN 独立清错、使能、重试。单台电机的恢复不创建新的操作授权，也不要求用户重新解锁。这个结论不适用于真实输入或板间上下文失效：当前双板操作授权在曾上线的对端掉线或 boot 改变后撤销，必须重新归中解锁。

RunStatus.requested、member_count/active_count/waiting_count 是观测。state、ready 和 waiting reason 用于诊断，不是上层是否继续生产目标的统一条件；具体业务可以把真实测量、头部稳定性作为回中或供弹的输入条件。commit 的传输错误记录到诊断，不因单次 TX 错误调用全机构 suspend；suspend 用于真实输入停止和初始化配置错误。

SwerveHardware 逐轴读反馈、逐轴暂存或作废计算输出。舵向需要绝对角和速度，轮驱只需要速度，四模块没有全体事务回滚、对齐门控或余弦缩速。GimbalAxis 保存业务目标；有效 Rate 输入的目标时间轴不因单轴执行掉线而重建。InertialGimbalAdapter 用两轴 output_valid 表达数学测量是否足够，电机执行状态不创建新输入边界；IMU frame_id/epoch 仍是真实坐标身份。

板间使用 v4，已删除 resume_generation。Endpoint 按实际 boot/link 建立上下文；生产序号、原始 source/command age 与接收缓存年龄共同保护输入有效期。库不要求 BigYaw ready/armed 才能发送请求；独立 `gimbal_dual_yaw` 上车策略对非零速度额外要求真实远端反馈新鲜，无反馈时最多发送已授权的零速 FollowCenter。`Disabled + 0` 和启用速度控制下的零速请求不同，详见[协议 v4 契约](../../protocols/interboard-v4.md)。

两个最终 sentry 应用均在自己的 `src/main.cpp` 中装配并执行，不再调用公共 vehicle_bench。云台使用物理遥控与 `manualCommandConfig()` 的 `require_external_run_gate=true`，每周期通过 `setOperatorGate()` 同步本地解锁/清故障会话；输入过期或安全档撤销授权。底盘不创建 CommandManager，只消费真实板间 OperatorControl、行走命令和大 Yaw 请求，并分别检查 boot、年龄和当前执行条件。

当前最终应用是手动机械链：云台不构造视觉、裁判或发射接收/执行对象，底盘不启用实测功率闭环；这些后续能力不从缺省零值、确认 bool 或功能开关推导为已可执行。历史 [vehicle_integration](../../../samples/robotics/vehicle_integration/README.md) 仍保留公共运行体及宏组合；历史 shooter 的离散事件消费规则仍须遵守，恢复不补执行离线期间的旧动作。

硬件与控制参数来自[共享实车配置](../../../config/vehicle/installed/README.md)和 `include/robotics/vehicle/`；现场按[独立上车入口](../../../samples/bringup/README.md)选择具体模块与输出上限。旧[持续指令实施指南](../../dev/电机持续指令与独立自动恢复重构实施指南.md)记录先前迁移背景，当前装配以应用和上车入口为准。编译记录不能代替实板断电恢复、电气接线和停止时限确认。
