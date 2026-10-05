# 命令持续控制与电机独立恢复

CommandManager 持续采样来源、仲裁并发布 CommandSnapshot。消费者分别检查最终命令和原始来源年龄。电机暂时掉线不能撤销来源、清掉用户运行意图或停止目标生产。

sourceStamp 位于 command_source.hpp，提取遥控/视觉原始生产时间。快照读取、通信重发和 CAN commit 都不续期原输入。用户主动停止、急停、来源停更以及实际板启动身份变化仍按输入有效性处理。

MotorSession、RecoveryGate、Group 授权状态机已删除。Executor 直接检查用户意图和命令年龄，每周期更新各轴；Motor/CAN 独立清错、使能、重试。无恢复生产时间边界，不等新的恢复 generation，不要求用户重新解锁。

RunStatus.requested、member_count/active_count/waiting_count 是观测。state、ready 和 waiting reason 只用于诊断，不能决定上层是否继续提交目标。commit 的传输错误记录到诊断，不调用全机构 suspend；suspend 用于真实输入停止和初始化配置错误。

SwerveHardware 逐轴读反馈、逐轴提交或作废计算输出。舵向需要绝对角和速度，轮驱只需要速度，四模块不再全体事务回滚。没有对齐门控或余弦缩速。

GimbalAxis 保存业务目标，Rate 目标继续推进。InertialGimbalAdapter 保留目标，用两轴 output_valid 表达数学测量是否足够；电机执行状态不创建新输入边界。IMU frame_id/epoch 仍是真实坐标身份。

板间全部使用 v4，删除 resume_generation。Endpoint 仅按实际 boot/link 建立上下文，BigYaw 请求发送不依赖被控电机反馈 ready/valid。原始 source age、command age、producer sequence 与接收缓存年龄保护输入有效期。

两个 sentry 应用均使用公共 vehicle_bench，历史未构建的私有 Executor 已删除。保留原 shooter 业务动作和离散事件消费规则，仅迁移电机执行契约，不补执行离线期间的动作。

文件级步骤、布局和现场验收见[详细实施指南](../../dev/电机持续指令与独立自动恢复重构实施指南.md)。本次编译不能代替实板断电恢复和电气问题验收。
