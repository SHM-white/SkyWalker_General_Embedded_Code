# 开发专题与状态记录

本目录保存设计方案、实施记录和指定时间点的分析。查当前接口、启动顺序和返回值时以源码及[模块文档](../README.md)为准。

当前接口以 `main@99a97c9` 为基线；旧方案中的 MotorSession、ready/clearFault、Group 故障传播和恢复授权 generation 已被持续目标与逐轴自动恢复替代，历史正文仅保留演进背景。

## 已落地的实现记录

| 记录 | 当前代码入口 | 说明 |
|---|---|---|
| [samples 遥控与统一板间协议](samples遥控控制统一改造指南.md) | samples/robotics/common/、lib/communication/interboard_* | 机构样例使用物理遥控与独立诊断构建；记录中的标识 3 已被 v4 替代；当前契约见板间模块文档，实板接线与标定待确认 |
| [双主控逐级整车验证实施](项目优化与逐级整车验证样例实施指南.md) | samples/robotics/ 的逐级样例、include/robotics/vehicle/ | 首包与后续惯性控制、大 Yaw、共享 CAN、四舵轮、发射和整车框架已直接实现；实板与标定 TODO 待落实 |
| [命令来源注册与后台仲裁服务](命令来源注册与后台仲裁服务实施指南.md) | include/robotics/command/command_manager.hpp、receiver_sources.hpp | 已实现的来源注册、后台 worker、CommandSnapshot 与同步 CommandArbiter 分层 |
| [三源命令管理台架原施工方案](三源命令管理sample手工实施指南.md) | samples/robotics/command_manager/ | 方案已落地；当前接线、配置和操作以 sample README 为准 |
| [电机驱动复核记录](电机驱动审查复核与线程弹性实施指南.md) | drivers/motor/、tests/motor/regression/ | 旧版待修清单属于历史检查点；当前代码与回归入口应以现行电机文档为准 |

## 尚待核验或实施

| 文档 | 当前待办 |
|---|---|
| [云台 GimbalAxis 与 Gimbal 分层方案](云台GimbalAxis与Gimbal分层重构方案.md) | 机械双轴、惯性适配和大小 Yaw 协调框架已实现，硬件标定与实板闭环待确认。 |
| [云台遥控样例安全策略阅读指南](云台遥控样例安全策略阅读指南.md) | 真实接线、可触达急停/复位输入和实机安全行为仍待确认。 |
| [rm_typec SPI 属性诊断指南](rm_typec设备树SPI频率属性诊断指南.md) | 编辑器 binding 警告来源和目标板构建结果待核验。 |
| [多品牌电机驱动架构手工指南](多品牌电机驱动重构架构与手工实施指南.md) | 核心对象已落地；方案的目标契约不自动代表当前 API，待办以电机主题文档为准。 |
| [安全策略与执行层恢复方案](安全策略收敛与执行层自恢复实施指南.md) | 设计说明保留为演进记录；当前实现状态见双主控应用和命令恢复文档。 |
| [当前项目完成度分析](当前项目完成度分析与后续实施清单.md) | 2026-09-30 的源码快照，仅作历史分析，不代表当前 HEAD 状态。 |
