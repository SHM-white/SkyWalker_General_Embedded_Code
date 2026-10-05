# 达妙 DM CAN 电机驱动

实现位于 `drivers/motor/{motor,can_bus,group}.cpp` 和 `drivers/motor/dm/dm_protocol.cpp`；接口位于 `include/drivers/motor/`。当前支持 J4310-2EC-V1.1。配置在 C++ 中构造，设备树提供物理设备。

调用顺序为配置 → Motor → CanBus attach/start → 持续 enable 与 setter/update → commit。目标提交不等待电机反馈。掉线、握手和真实驱动故障期间，上层继续提交；底层逐电机自动重试、清错和使能。详细并发约定见[电机工作链路](../../guides/motor-workflow.md)。

## 模式和配置

持久模式须与电机调试助手一致，分别用 `dm::j4310Mit()`、`j4310PositionVelocity()`、`j4310Velocity()` 创建配置。

```cpp
using namespace skywalker;
static motor::Motor drive{motor::dm::j4310Mit({
    .id = 1, .master_id = 0x11,
    .position_max_rad = 12.5f,
    .velocity_max_rad_s = 30.0f,
    .torque_max_nm = 10.0f,
    .torque_limit_nm = 1.0f,
    .timing = {.feedback_timeout_ms = 50, .command_timeout_ms = 20,
               .enable_timeout_ms = 3000, .retry_interval_ms = 100},
})};
static motor::CanBus bus{DEVICE_DT_GET(DT_NODELABEL(can1))};
```

ID 范围为 1–15，Master ID 是标准 CAN 反馈 ID。PMAX、VMAX、TMAX 使用设备实际值，既决定命令量化，也参与反馈解码。软件力矩上限为正且不超过 TMAX。

初始化时 attach 全部电机，再 start 物理总线。控制循环持续调用 enable、模式对应 setter、commit；无需先取得设备反馈。Group 可以批量启停跨 CAN 成员，但不等待或联动成员实际状态。

## 模式与字节布局

| 模式 | 标准 CAN ID | DLC | 数据 | 命令 |
| --- | ---: | ---: | --- | --- |
| MIT | motor ID | 8 | 位置 16 bit，速度/KP/KD/力矩各 12 bit | `setTorque()`、`setMit()` |
| 位置速度 | `0x100 + ID` | 8 | little-endian float 位置与速度上限 | `setPositionVelocity()` |
| 速度 | `0x200 + ID` | 4 | little-endian float 速度 | `setVelocity()` |

MIT 有符号工程量通过线性偏移量化到无符号字段：`(value - min) × ((1 << bits) - 1) / (max - min)`。KP 范围为 0–500，KD 为 0–5。量化和字节打包由已有协议函数完成，应用只传工程单位。

`setTorque()` 生成零 KP/KD、零位置速度、仅前馈力矩的 MIT 命令。软件位置／速度 PID 输出 N·m 时，使用携带实际反馈执行代次的两参数力矩提交。原生模式直接目标使用单参数 setter。

| 特殊命令 | 末字节 | 用途 |
| --- | ---: | --- |
| ClearError | `0xFB` | 清驱动故障 |
| Enable | `0xFC` | 使能 |
| Disable | `0xFD` | 失能 |
| SaveZero | `0xFE` | 持久化零点，仅底层协议提供 |

特殊帧前七字节为 `FF`。自动恢复只使用协议清错、使能、停止及必要探测，不自动 SaveZero。

## 反馈和逐轴恢复

反馈 ID 为 Master ID，`data[0]` 高四位为设备状态，低四位为 motor ID。共享 Master ID 的设备按帧内 ID 分发；重复 ID、命令冲突及 TX/RX 冲突在 start 时报告配置错误。

快照的 `enabled_requested` 表示上层意图；`state` 和 `output_permitted` 描述实际执行。`feedback.valid`、`feedback_fresh` 描述测量有效性。原生 MOS／转子温度和设备状态仅在对应 valid 标记为真时使用。

设备报告过压、欠压、过流、过温、通信丢失或过载时，本轴 Fault；底层限频 ClearError，得到对应操作之后的真实 Disabled 反馈，再自动 Enable。故障持续则继续本轴重试，其他电机正常运行。

Enable 与反馈顺序用 callback 顺序号确认，不用毫秒相等推断先后。缺少反馈也允许发起协议尝试，设备稍后接入可自动运行。必要的探测发送零力矩／速度，位置速度模式只在知道原生位置时使用当前位置；不伪造零位置。

`feedback.position_rad` 是连续参考，`native_position_rad` 是本次解码的原生位置。多圈参考中断后不能自动猜回原坐标；速度控制恢复，依赖多圈参考的轴只等待自己的可信参考。显式 reseed 后继续最新目标。

## 命令接收、停止和诊断

合法 setter 在设备离线和 Fault 时返回 0。负返回值表示参数、模式、范围或写入者错误，不能用于撤销其他轴。commit 返回发布结果，Recovering 期间持续覆盖最新命令。重复 commit 不刷新原命令年龄。

停止取消此前命令与旧协议操作；持续已停 disable 幂等。停止后写入的新目标保存在缓存，之后 enable 才运行。`StopProgress` 的 Pending、TxComplete、DriveConfirmed、Unreachable 分别表示等待发送、发送完成、真实 Disabled 确认、暂未确认，不能当成机械停止测量。

正常掉电恢复无需控制台重新复位。观察最近命令序号、运行意图、反馈年龄、实际状态、retry_count、BusStatus.last_tx 和 last_recovery。实际 CAN 整体不可用时，该总线无法执行；其他总线持续工作。

## 资源和使用

```conf
CONFIG_CAN=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DM=y
```

J4310 默认反馈 50 ms、命令 20 ms、协议等待 3000 ms、重试间隔 100 ms，使用具名 Timing。每条 CAN 一个工作线程，协议任务限频轮转并与正常目标交替，缺失设备不会占满发送机会。

首次使用核对 CAN、ID、Master ID、持久模式和 PMAX/VMAX/TMAX。恢复后继续运动是默认行为，现场应在能够正常运动的条件下接入电机电源。

样例：[dm_mit_control](../../../samples/motor/dm_mit_control/)、[dm_velocity_control](../../../samples/motor/dm_velocity_control/)、[dm_position_control](../../../samples/motor/dm_position_control/)、[dm_mit_velocity_control](../../../samples/motor/dm_mit_velocity_control/)、[dm_mit_position_control](../../../samples/motor/dm_mit_position_control/)。
