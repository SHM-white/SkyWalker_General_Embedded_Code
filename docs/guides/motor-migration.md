# 电机持续目标与逐轴恢复：旧代码迁移

适用于源码基线 `main@99a97c9`（2026-10-05）。本次改变了运行意图、命令接收、恢复和共享总线的责任边界；不能只替换方法名后保留旧的全员就绪判断。

阅读顺序：[公共电机模型](../modules/drivers/motor.md) → [控制封装](../modules/control/motor-control.md) → [周期工作流](motor-workflow.md)。签名与调用示例见 [DJI](../api/motor-dji.md)、[DM](../api/motor-dm.md)、[速度 / 位置控制器](../api/motor-control.md)。

## 旧假设到当前契约

| 旧代码 / 假设 | 当前契约 | 应用需要修改什么 |
| --- | --- | --- |
| MotorSession、RecoveryGate 或 Group 管整个机构恢复 | 已删除；Motor / CanBus 独立自动恢复 | 移除组级故障锁存和设备恢复后的人工重新授权路径 |
| `ready()` / `clearFault()` / preflight 决定能否写目标 | 公共 ready / clearFault 已移除；合法目标持续接收 | 配置阶段校验能力，运行时持续生产，不以 Active / ready 门控 |
| enable 成功等于已上线或已运动 | enable 保存用户持续运行意图 | 分别显示 `enabled_requested`、实际 `state`、反馈与输出有效性 |
| update / setter 失败或一轴离线就停止所有轴 | 等待设备 / 参考时合法目标仍被接受 | 分别记录每轴结果；不提前 return 导致其他轴和 CAN 跳过 |
| Group 将一台故障传播给其他成员 | Group 只批量 enable / disable 和统计 | 批量停止只能来自明确业务意图，不能沿用组级恢复状态机 |
| 重复 commit 能刷新命令期限 | 原目标时间来自 setter | 真实控制周期生产新目标；总线重发不续期 |
| 反馈计算完成后取当前 generation 再提交 | 输出绑定计算所用快照的 `enable_generation` | 使用同一快照；控制封装已通过专属 producer 执行 |
| 丢失多圈位置后首帧重新当零点 | 连续参考等待可信来源，绝对参考按模式恢复 | 明确选取位置模式；只在有依据时 reseed |
| reset 能重新定义 StartupRelative 原点 | reset 只重置本轴控制历史 | 保留可信原点；恢复不采集新的任意零点 |
| 恢复后执行掉线期间积压的目标 | 只保留最新有效命令，年龄仍有效 | 不在应用排队补发旧轨迹；恢复不会复活停止前命令 |
| DM SaveZero 可用于自动恢复 | 自动恢复不 SaveZero | 持久零点是单独硬件操作，不用于软件重试 |
| 一个共享 DJI 帧中的停机轴要求全帧为零 | 按槽独立决定输出 | 保留其他正常端点目标；统一由 CanBus 编码和发送 |
| 板间 resume_generation 或电机反馈 ready 决定能否发送 | v4 无 resume_generation；原生产年龄仍受约束 | 同步升级两板；保留 boot、producer sequence 和原年龄，不重新盖时间 |

## 周期任务的基本形状

```cpp
// 构造、attach/start、configure 已完成；对象长期存活。
// input_valid 来自真正的用户/外部来源有效期，不来自电机状态。
void controlTick(bool input_valid, float left_target, float right_target, float dt_s) {
    if (input_valid) {
        recordCallError(left_motor.enable());
        recordCallError(right_motor.enable());
        recordCallError(left_axis.update(left_target, dt_s));
        recordCallError(right_axis.update(right_target, dt_s));
    } else {
        recordCallError(left_motor.disable());
        recordCallError(right_motor.disable());
    }
    recordCallError(can_bus.commit().error);
}
```

这是已有对象上的周期片段，`recordCallError` 为应用自己的诊断函数。多个物理 CAN 分别 commit；一条失败也不能漏掉另一条。本地固定目标每周期生产；遥控 / 板间目标必须检查真实输入生产时间，不能重复读取缓存就视为新输入。

停止请求返回成功、停止帧 TX 完成、驱动确认 Disabled 与机械停止是不同观测。停止前已有在途帧可能迟到；有机械互锁需求时在业务层表达共同停止意图，不能把独立恢复误当成机械联动保证。

## 迁移待办

- [ ] 核对型号、模式、ID、Timing、物理 CAN 所有权与 attach/start 初始化错误。
- [ ] 删除已移除的会话、组级恢复、人工 clearFault、ready/preflight 和恢复 generation 使用。
- [ ] 把运行意图改为来自启停操作、急停和真实输入有效期；不由反馈状态反向生成。
- [ ] 所有轴持续接收目标；每条 CAN 每周期统一 commit，结果逐项记录。
- [ ] 自写反馈控制使用同一快照的计算代次；控制器绑定后不混用普通 setter。
- [ ] 分别展示目标接收、输出有效、反馈有效、实际状态、参考与停止进度。
- [ ] 检查失联重接、单轴故障、总线恢复、停止后新目标及位置参考丢失场景。
- [ ] 同步两板 v4 和原生产年龄；联动机构仍由业务层表达明确停止策略。
- [ ] 按[文档维护清单](../maintenance.md)同步接口、主题页、样例、应用说明与在线图。

这些勾选项是迁移和验收要求，不表示本次已经完成实板验收。新模型默认在条件恢复后执行最新有效目标，现场上电前应按对应样例核对接线、模式、单位、限幅与机械条件。
