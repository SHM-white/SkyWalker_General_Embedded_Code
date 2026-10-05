# 电机持续控制、逐轴恢复与并发调用

上层持续表达运行意图和最新目标，每台电机独立处理实际执行和恢复。设备不存在、掉线或报告故障不会要求上层重新启动，也不会由 Group 传播停机。主动停止、急停和真实输入过期由输入入口处理。

旧代码适配先看[电机迁移指引](motor-migration.md)。型号与字节编码见 [DJI](../modules/drivers/motor-dji.md)、[DM](../modules/drivers/motor-dm.md)，PID 封装见[电机控制](../modules/control/motor-control.md)。

## 对象与阅读前提

先阅读[公共电机模型](../modules/drivers/motor.md)：输入入口决定意图，机构产生目标，控制器按本轴反馈计算，Motor 保存命令，CanBus 统一发布并逐轴恢复，Group 只做显式批量操作。

## 从初始化到持续运行

1. 静态构造电机、总线和控制器。
2. 给对应物理 CanBus attach 全部端点，然后 start。
3. 配置控制器。能力／参数错误属于初始化错误。
4. 用户启动后持续 enable 和 setter/update，设备尚未有反馈也接受。
5. 每个物理 CAN 每周期 commit 一次。
6. 底层独立建立实际执行条件；反馈恢复后控制器自行重置本轴历史。

```cpp
// 所有错误都记录，但本轴等待不得跳过其他轴。
void tick(bool requested, float first_target, float second_target, float dt) {
    if (requested) {
        recordCallError(first_motor.enable());
        recordCallError(second_motor.enable());
        recordCallError(first_axis.update(first_target, dt));
        recordCallError(second_axis.update(second_target, dt));
    } else {
        recordCallError(first_motor.disable());
        recordCallError(second_motor.disable());
    }
    recordCallError(bus.commit().error);
}
```

`requested` 只由输入意图和有效期改变，不由 MotorState 改变。`recordCallError()` 是应用自己的错误记录接口。两个轴位于不同物理 CAN 时，两条总线分别 commit，不能因为第一个结果失败漏掉第二条。

本地固定目标由周期任务持续生产；没有新的控制台按键不等于命令源失联。遥控、板间或其他外部命令则保留原始生产时间，不能重复读取旧输入并重新盖上当前时间。

## 三个成功结果

| 调用 | 返回成功的含义 |
| --- | --- |
| setter／update | 目标已接受；暂不可执行也属于成功接收 |
| commit | 已发布最新总线命令快照；Recovering 期间也接受 |
| CAN TX 完成 | 这帧实际完成发送 |

这些结果均不代表机械已经到位或停止。

setter 只拒绝本次非法调用，如非有限值、超过明确数值范围、协议不支持或写入者不匹配。设备状态不作为上层命令写入的准入条件。

## 命令时间与计算依据

每轴只保留最新目标，setter 的生产时间不被 commit 或恢复续期。反馈计算输出必须携带同一快照的执行代次；详见[命令与代次](../modules/drivers/motor.md#命令取消版本与计算依据)。

## 停止与逐轴恢复

设备离线、驱动故障和总线恢复期间持续生产目标；输入过期或用户停止才撤销意图。恢复只影响本轴或本 CAN，连续位置失效需可信参考。停止报告和各事件语义见[公共停止与恢复契约](../modules/drivers/motor.md#停止与恢复)。

## 共享总线发布

同一 CAN 的所有轴先更新，再统一 commit；多个物理 CAN 分别发布。DJI 共享帧按槽判断，一个端点停止仍保留其他槽的有效目标。版本复核、停止确认、在途帧和总线恢复详见[发送契约](../modules/drivers/motor.md#发送候选与共享-dji-帧)。

## 并发约定与期限

- 同一个控制器的 configure/reset/update 串行执行。telemetry 锁只保护快照读取。
- 同一电机由一个普通目标写入者负责，producer 绑定只处理身份，不附带温度／超速授权。
- 不同线程使用不同物理 CAN 可以独立 update／commit。
- 多线程共享 CAN 时指定统一发布者，或串行化整组 setter/update 和 commit。内部短锁不提供整个业务周期的事务性。
- commit 收集整条总线，未协调会把不同周期的轴目标混在一起。
- 对必须同周期配对的轴由应用协调；独立轴不增加全员就绪屏障。

Timing 采用具名字段：feedback_timeout_ms、command_timeout_ms、enable_timeout_ms、retry_interval_ms。DJI 默认为 20、10、100、100 ms；J4310 默认为 50、20、3000、100 ms。

BusOptions 默认 TX 等待 2 ms、控制器恢复间隔 100 ms。每轮 RX 处理有界，协议工作有界，下一次等待考虑当前尝试与重试期限。控制器状态仍采用短周期探测；忙碌时有界让出 CPU。实际调度和延迟依赖板卡 tick、优先级与 CAN 负载。

## 诊断入口

先看 `enabled_requested` 和最近命令序号，确认输入入口是否持续生产；再看反馈年龄、实际 state、output_permitted、retry_count 和位置参考。最后看 BusStatus.state、last_tx 和 last_recovery。

历史错误和当前状态分别理解：总线已经 Running 时，last_recovery 保留上次故障并不表示目前仍然故障。一个成员掉线连带其他总线停止时，检查应用输入撤权或主动 disable 路径；Group 自身没有故障传播。

此文描述软件契约。实际 CAN 电气故障、模式配置、连续坐标可信来源和机械执行需要对应硬件确认；自动恢复不能修正错误接线、协议量化范围或机构坐标。
