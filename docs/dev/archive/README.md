# 已归档开发记录

本目录保存已被当前代码推翻、不再作为操作依据的历史文档。它们只用于追溯设计演进；查当前行为请使用每篇“替代入口”中的现行文档或源码。

截至 `dd3f673`（2026-11-15）归档以下 6 篇：

| 文档 | 归档原因 | 替代入口 |
|---|---|---|
| [云台遥控样例安全策略阅读指南](云台遥控样例安全策略阅读指南.md) | 全文围绕已删除的 `ready()`、`Group.clearFault()` 与全局恢复代次 | [云台应用 README](../../../applications/sentry_gimbal/README.md)、[命令恢复](../../modules/robotics/command-recovery.md) |
| [双 Yaw 回滚与控制分工分析](双Yaw回滚与控制分工分析.md) | “正式路径不再调用 `DualYawMotionAllocator`”的结论已被推翻；零位数值也已更新 | [双 Yaw 分配接口](../../api/big-yaw.md)、[云台应用 README](../../../applications/sentry_gimbal/README.md) |
| [命令管理器调用接口简化实施指南](命令管理器调用接口简化实施指南.md) | 方案以已删除的 `GlobalSafetyManager` 决策为输入 | [命令来源与后台服务](../../modules/robotics/command-service.md)、[命令接口](../../api/command.md) |
| [哨兵 IMU 航向控制验收](哨兵IMU航向控制验收.md) | 正文依赖已不在协议表中的 `0x0405`/`mode=2` 扩展 | [板间 v4 契约](../../protocols/interboard-v4.md)、[底盘应用 README](../../../applications/sentry_chassis/README.md) |
| [小 Yaw 低速偏慢的定位与提速参数指南](小Yaw低速偏慢的定位与提速参数指南.md) | “`yaw_encoder_rate_rad_s=0.2` 是硬上限”的结论已失效，当前 bringup 策略为 4.0 rad/s | [云台应用 README](../../../applications/sentry_gimbal/README.md)、`include/robotics/vehicle/manual_profile.hpp` |
| [当前项目完成度分析与后续实施清单](当前项目完成度分析与后续实施清单.md) | 2026-09-30 源码快照，`GlobalSafetyManager`、RS485 头部 IMU 等前提均已变化 | [文档中心](../../README.md)、[双主控应用](../../applications/dual-controller.md) |

归档文档不保证内部链接可用，也不随代码继续更新。需要恢复其中某项结论时，先在当前源码中重新核实。
