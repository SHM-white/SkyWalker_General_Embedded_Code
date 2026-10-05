# DJI 电机与共享 CAN 总线：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

统一 Motor/CanBus 持续接收目标，设备独立自动恢复；effort 单位 A。

**接入状态：已有代码。** 源码与台架入口已有；模式、ID、方向、范围和机械参数按实物确认。设备掉线不撤销其他轴。

## 职责与关联

Motor 保存运行意图、最新命令和真实反馈；CanBus 处理物理 I/O、限频协议重试和独立恢复；Group 只做显式批量操作。

输入 / 依赖：[板级：MC02 / RoboMaster Type-C](boards.md)

消费者：[VelocityMotor / PositionMotor 硬件闭环](motor-control.md)、[舵轮底盘与功率缩放](chassis.md)

## 接口契约

### 1. dji::Config dji::gm6020(const Gm6020Options &options); dji::Config dji::m3508(const M3508Options &options); dji::Config dji::m2006(const M2006Options &options)

```cpp
dji::Config dji::gm6020(const Gm6020Options &options); dji::Config dji::m3508(const M3508Options &options); dji::Config dji::m2006(const M2006Options &options)
```

构建不同型号的值配置，然后传给 Motor；工厂不创建设备、不发 CAN、不验证全部选项。

| 参数 | 含义与边界 |
| --- | --- |
| `options.id` | GM6020 为 1–7，其余为 1–8。 |
| `options.current_limit_a` | 软件电流上限：GM6020 ≤3 A，M3508 ≤20 A，M2006 ≤10 A，均须 >0。 |
| `options.gear_ratio` | M3508 默认 19，M2006 默认 36；电机轴/输出轴转速比，必须 >0。 |
| `GM6020 专有选项` | encoder_zero_ticks 0–8191 定义绝对零点；current_mode_confirmed 须确认固件电流模式后置 true。 |
| `options.timing` | 反馈、命令、稳定恢复、使能超时，单位 ms，全部须 >0。 |

**返回 / 输出：** 返回 dji::Config；Motor(dji::Config) 保存一份配置。

**线程 / 时序：** 初始化阶段纯函数；构造后的 Motor 不可拷贝/移动。

**错误 / 边界：** 配置错误在 CanBus::start() 暴露；GM6020 未确认电流模式为 -EINVAL。

### 2. int dji::describe(const Config &config, Descriptor &out)

```cpp
int dji::describe(const Config &config, Descriptor &out)
```

计算反馈 CAN ID、命令帧 ID 和帧内槽位。GM6020 反馈 0x204+id，命令 0x1FE/0x2FE；M3508/M2006 反馈 0x200+id，命令 0x200/0x1FF。

| 参数 | 含义与边界 |
| --- | --- |
| `config` | DJI 配置。 |
| `out` | 成功时写入描述；command_slot 为 0–3，同一帧最多四电机。 |

**返回 / 输出：** 0=成功；描述不代替完整配置验证。

**线程 / 时序：** 无 I/O 的纯调用；通常应用不必自己映射 ID。

**错误 / 边界：** -ERANGE：型号/电机 ID 不合法。不同协议 TX/RX 冲突最终由总线 start() 检查。

### 3. bool dji::decodeFeedback(const can_frame &frame, RawFeedback &out); int dji::buildCommandFrame(can_frame &frame, uint16_t command_id, const int16_t command_raw[4])

```cpp
bool dji::decodeFeedback(const can_frame &frame, RawFeedback &out); int dji::buildCommandFrame(can_frame &frame, uint16_t command_id, const int16_t command_raw[4])
```

底层协议工具：解码编码器/RPM/原始电流/温度；把四个原始电流槽位编码成命令帧。正常应用使用 Motor，避免绕过权限和限幅。

| 参数 | 含义与边界 |
| --- | --- |
| `frame` | 标准 CAN 数据帧。 |
| `out` | 成功解码后的原始协议数据，不是 SI 单位输出轴反馈。 |
| `command_id` | DJI 的合法组帧 ID。 |
| `command_raw` | 四个槽位的 int16 原始值，按高字节在前编码。 |

**返回 / 输出：** decodeFeedback 返回是否帧合法；buildCommandFrame 返回 0 或负 errno。

**线程 / 时序：** 纯编解码，不执行 CAN 发送；驱动内部在 I/O 线程调用。

**错误 / 边界：** 非法帧 decode 为 false；不由这些工具维护 freshness/Group/命令超时。

### 4. int Motor::setCurrent(float ampere); int Motor::setCurrent(float ampere, uint64_t sampled_enable_generation)

```cpp
int Motor::setCurrent(float ampere); int Motor::setCurrent(float ampere, uint64_t sampled_enable_generation)
```

设置 DJI 电流目标；反馈计算结果必须携带计算时 snapshot.enable_generation，不能计算后重新贴新版本。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非有限 -EINVAL，明确范围 -ERANGE，模式不支持 -ENOTSUP，producer 不匹配 -EACCES；离线/非 Active 不拒绝合法目标。

### 5. MotorInfo Motor::info() const; MotorSnapshot Motor::snapshot() const; bool Motor::active() const

```cpp
MotorInfo Motor::info() const; MotorSnapshot Motor::snapshot() const; bool Motor::active() const
```

查询能力、参数和状态值副本；enabled_requested 是运行意图，output_permitted/feedback_fresh 是实际执行条件，retry_count 与 last_fault 是诊断。

**返回 / 输出：** 能力/状态值副本或实际 active 布尔值；快照读取不生产新反馈。

**线程 / 时序：** 允许多读取线程，短锁保护；有状态输出只有一个生产者。

**错误 / 边界：** 检查 feedback.valid 和原始 timestamp；实际状态不能替代运行意图。

### 6. int Motor::enable(); int Motor::disable()

```cpp
int Motor::enable(); int Motor::disable()
```

enable 幂等保存运行意图，离线也可接受；disable 在真→假时取消旧命令和旧协议操作，停止后重复 disable 不清掉之后新提交的目标。Group 成员也可直接操作。

**返回 / 输出：** 0 请求已接受，不证明机械执行或制动。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** -EACCES：CAN 未启动；disable 版本耗尽 -EOVERFLOW。

### 7. int Motor::reseedPosition(double known_position_rad)

```cpp
int Motor::reseedPosition(double known_position_rad)
```

用可信机械依据建立本轴连续坐标；不向固件保存零点，不影响其他轴。

| 参数 | 含义与边界 |
| --- | --- |
| `known_position_rad` | 已确认输出轴坐标 rad，不能凭断电前角度猜圈数。 |

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法/越界/能力或参考条件错误返回负 errno；见 Motor 实现。

### 8. int Motor::invalidateComputedEffort()

```cpp
int Motor::invalidateComputedEffort()
```

仅作废计算 effort，保留运行意图；反馈不足时应使旧计算输出失效。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 已绑定控制器 producer 时普通调用可能 -EACCES；封装内部使用其 producer。

### 9. CanBus(const device *can, BusOptions options = {}); int CanBus::attach(Motor &motor); int CanBus::start()

```cpp
CanBus(const device *can, BusOptions options = {}); int CanBus::attach(Motor &motor); int CanBus::start()
```

每物理 CAN 一个 owner。初始化 attach 全部端点，再 start I/O 工作线程；DJI/DM 可共享。

| 参数 | 含义与边界 |
| --- | --- |
| `can` | 实际独占 Zephyr CAN 控制器 |
| `options` | TX timeout 默认 2 ms，恢复重试默认 100 ms |

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 设备、拓扑、ID、参数错误由 attach/start 返回；不可在运行中增加成员。

### 10. CommitResult CanBus::commit(); BusStatus CanBus::status() const

```cpp
CommitResult CanBus::commit(); BusStatus CanBus::status() const
```

commit 发布完整总线最新快照，Recovering 时继续接受。status 含 latest_submitted_sequence、last_tx、rx_overflows、rx_invalid_frames、superseded_batches 和 last_recovery。

**返回 / 输出：** CommitResult.error/sequence 表示发布结果；CAN TX 完成另看 last_tx。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 未启动/永久配置错误返回调用错误；成功不代表远端收到或执行。

### 11. Group(Motor &first, Others &...others); Group(std::span<Motor *const> members); int Group::enable(); void Group::disable(); GroupStatus Group::status() const

```cpp
Group(Motor &first, Others &...others); Group(std::span<Motor *const> members); int Group::enable(); void Group::disable(); GroupStatus Group::status() const
```

固定成员批量启停和统计，无故障传播、就绪屏障、成员所有权或独立恢复状态。

**返回 / 输出：** enable 返回首个调用错误并继续其他成员；status 为成员/意图/实际/离线/故障计数。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 固定列表配置错误或 Motor 调用错误；没有 ready/active/clearFault 公共组接口。

## 调用示例

### 持续运行意图与最新目标

```cpp
// 先 bus.attach(motor)、bus.start()，电机固件模式匹配。
if (run_requested) {
    const int enabled = motor.enable();
    const int accepted = motor.setCurrent(0.1f);
    // 分别记录调用错误，设备离线不终止周期。
} else {
    (void)motor.disable();
}
const auto published = bus.commit();
const auto actual = motor.snapshot();
```

直接协议目标片段；effort=A。绑定 VelocityMotor/PositionMotor 后应改用 axis.update，不混用 producer。

## 调用顺序

1. 按真实模式/ID/量程静态构造 Motor、CanBus。
2. attach 所有端点，再 start；不等待设备全部上线。
3. 运行意图有效时每周期 enable + setter/update + commit。
4. 各电机独立探测、清错、使能，反馈恢复后执行最新未过期目标。
5. 主动停止取消旧目标；停止进展单独观测，机械刹停不是 TX 成功。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `Timing` | 具名反馈/命令/使能/重试期限；原目标时间不由恢复或 commit 续期。 |
| `Effort 单位` | A；应用限幅与控制输出必须使用同单位。 |
| `物理总线 owner` | 每 CAN 一个 CanBus，所有机构先 stage，统一每周期 commit 一次。 |

- 没有公共 Motor::ready/clearFault；Group 不传播成员故障。
- setter 接受、commit 发布、CAN TX 完成、机械执行是不同结果。
- 共享 DJI 帧只把不可执行槽置零，其他槽保留目标。
- 连续参考丢失不能猜圈数；可信 reseed 仅影响本轴。
- 历史 last_fault/last_recovery 不等于当前仍在故障。

## 正文与源码

- [DJI Markdown](../modules/drivers/motor-dji.md)
- [电机操作顺序](../guides/motor-workflow.md)

- [dji_motor.hpp](../../include/drivers/motor/dji_motor.hpp)
- [dji_protocol.hpp](../../include/drivers/motor/dji_protocol.hpp)
- [motor.cpp](../../drivers/motor/motor.cpp)
- [can_bus.cpp](../../drivers/motor/can_bus.cpp)
- [main.cpp](../../samples/motor/dji_unified/src/main.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
