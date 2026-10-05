# 持续目标与独立执行骨架

真实CommandManager线程每10ms仲裁，底盘和云台两个独立5ms消费者读取同一非消耗快照。合成操作员由ICommandSource提供，执行器仅保存内存目标，无电机、CAN、遥控器或IMU接线。

执行器分别保存最新目标和requested；数学参考缺失或一次周期过长只改变active_count/waiting_count诊断，目标继续被接收。没有RecoveryGate、恢复generation、全员就绪或模拟使能屏障。源过期和明确Disabled才取消运行意图并清目标。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/execution_skeleton -d build/execution_skeleton
```

默认20s一轮自动演练：

| 时间 | 事件 | 预期 |
| --- | --- | --- |
| 0～3s | 正常输入与执行 | 两消费者直接接收目标，独立Active |
| 3～4.5s | 输入生产暂停，返回EAGAIN | 原始source年龄增长，100ms后目标失效；仲裁轮询不能续期 |
| 4.5～7s | 输入恢复 | 自动接受新目标，无恢复授权边界 |
| 7～8.5s | 云台消费者及状态生产暂停 | 云台status_seq冻结，100ms后日志valid失效；底盘继续 |
| 11～11.15s | 底盘消费者停顿超过25ms | 周期超限计数增长，本周期等待Cycle，目标仍接收；下一正常周期继续 |
| 14～15s | 云台数学参考无效 | 云台requested与目标继续，只有模拟执行等待Reference；底盘继续 |

遥测每250ms显示requested、实际活动/等待成员数、source/command/status原始年龄、目标与周期超限。SnapshotCache复制快照不修改producer stamp，心跳或读取不能续期执行状态。

关闭演练：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/execution_skeleton -d build/execution_skeleton_steady -- -DCONFIG_EXECUTION_SKELETON_AUTORUN=n
```

这里的目标只是模拟数据，不能据此断言真实电机恢复已经验证。真实控制封装还需要本轴反馈与可信参考，实际发送由现有Motor/CAN工作线程处理。尚未运行本次演练或取得实板记录。
