# 达妙电机：MIT / 速度 / 位置速度：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

统一 Motor/CanBus 持续接收目标，设备独立自动恢复；effort 单位 N·m。

**接入状态：已有代码。** 源码与台架入口已有；模式、ID、方向、范围和机械参数按实物确认。设备掉线不撤销其他轴。

## 职责与关联

Motor 保存运行意图、最新命令和真实反馈；CanBus 处理物理 I/O、限频协议重试和独立恢复；Group 只做显式批量操作。

输入 / 依赖：[板级：MC02 / RoboMaster Type-C](boards.md)

消费者：[VelocityMotor / PositionMotor 硬件闭环](motor-control.md)

## 接口契约

### 1. dm::Config dm::j4310Mit(const J4310Options &options); dm::Config dm::j4310Velocity(const J4310Options &options); dm::Config dm::j4310PositionVelocity(const J4310Options &options)

```cpp
dm::Config dm::j4310Mit(const J4310Options &options); dm::Config dm::j4310Velocity(const J4310Options &options); dm::Config dm::j4310PositionVelocity(const J4310Options &options)
```

构建 MIT、速度或位置速度模式配置；主控工厂不会把驱动器固件自动切换模式。

| 参数 | 含义与边界 |
| --- | --- |
| `options.id` | 电机 ID 1–15。 |
| `options.master_id` | 驱动器反馈的标准 CAN ID 0–0x7FF；不能与总线 TX ID 冲突。 |
| `position_max_rad / velocity_max_rad_s / torque_max_nm` | 真实固件 PMAX/VMAX/TMAX，须为正且匹配工具中已保存的值。 |
| `torque_limit_nm` | 应用软件力矩上限 >0 且 ≤TMAX。 |
| `timing` | feedback_timeout_ms / command_timeout_ms / enable_timeout_ms / retry_interval_ms；DJI 20/10/100/100，J4310 50/20/3000/100 ms。 |

**返回 / 输出：** 返回 dm::Config；交给 Motor(dm::Config)。

**线程 / 时序：** 初始化阶段纯配置；对象全固件生命周期存活。

**错误 / 边界：** 全部配置验证在 CanBus::start()；不支持型号/模式 -EINVAL；范围错误 -ERANGE。

### 2. int dm::describe(const Config &config, Descriptor &out); int dm::controlFrameId(ControlMode mode, uint16_t motor_id, uint16_t &out)

```cpp
int dm::describe(const Config &config, Descriptor &out); int dm::controlFrameId(ControlMode mode, uint16_t motor_id, uint16_t &out)
```

计算三模式控制帧 ID：MIT=id，位置速度=0x100+id，速度=0x200+id；反馈 ID 来自 master_id，并用帧内 motor_id 分流。

| 参数 | 含义与边界 |
| --- | --- |
| `config / mode` | 必须与固件持久化模式相同。 |
| `motor_id` | 电机地址1–15。 |
| `out` | 成功时写描述或控制帧 ID。 |

**返回 / 输出：** 0=成功，负 errno=参数错误。

**线程 / 时序：** 纯调用，无 CAN I/O。

**错误 / 边界：** -EINVAL/-ERANGE：型号、模式或地址不合法。CanBus 允许不同 DM 电机共用反馈 master_id，但帧内 motor_id 必须不同。

### 3. int dm::buildMitFrame(uint16_t motor_id, const Limits &limits, const MitCommand &command, can_frame &out)

```cpp
int dm::buildMitFrame(uint16_t motor_id, const Limits &limits, const MitCommand &command, can_frame &out)
```

MIT 量化编码工具；位置16位，速度/kp/kd/力矩各12位。正常应用直接使用 Motor::setMit。

| 参数 | 含义与边界 |
| --- | --- |
| `motor_id` | 电机地址。 |
| `limits` | 匹配固件的 PMAX/VMAX/TMAX。 |
| `command` | MIT五字段。 |
| `out` | 编码后的标准 CAN 帧。 |

**返回 / 输出：** 0=完成编帧，负 errno=无效/超范围；不发送。

**线程 / 时序：** 驱动 I/O 线程内部使用的纯编码。

**错误 / 边界：** 协议范围不等于应用 torque_limit_nm；绕过 Motor 会绕开生命周期和软件力矩限幅。

### 4. int dm::buildVelocityFrame(uint16_t motor_id, float velocity_rad_s, can_frame &out); int dm::buildPositionVelocityFrame(uint16_t motor_id, float position_rad, float velocity_rad_s, can_frame &out)

```cpp
int dm::buildVelocityFrame(uint16_t motor_id, float velocity_rad_s, can_frame &out); int dm::buildPositionVelocityFrame(uint16_t motor_id, float position_rad, float velocity_rad_s, can_frame &out)
```

编码速度/位置速度模式的浮点命令帧；仅构造字节，不建立整车权限。

| 参数 | 含义与边界 |
| --- | --- |
| `motor_id` | 电机地址。 |
| `position_rad / velocity_rad_s` | 对应物理量；应用层额外限幅由 Motor 执行。 |
| `out` | 目标CAN帧。 |

**返回 / 输出：** 0=完成编帧。

**线程 / 时序：** 纯调用，无 I/O。

**错误 / 边界：** 地址/非有限输入错误返回负 errno；主控应调用 Motor setter，以免漏 PMAX/VMAX 校验。

### 5. int dm::buildSpecialFrame(ControlMode mode, uint16_t motor_id, SpecialCommand command, can_frame &out); int dm::decodeFeedback(const can_frame &frame, uint8_t expected_motor_id, const Limits &limits, DecodedFeedback &out); bool dm::isFaultStatus(DriveStatus status)

```cpp
int dm::buildSpecialFrame(ControlMode mode, uint16_t motor_id, SpecialCommand command, can_frame &out); int dm::decodeFeedback(const can_frame &frame, uint8_t expected_motor_id, const Limits &limits, DecodedFeedback &out); bool dm::isFaultStatus(DriveStatus status)
```

特殊命令 Enable/Disable/ClearError/SaveZero、反馈状态和温度解码。SaveZero 是底层协议能力，统一 Motor 不公开保存硬件零点操作。

| 参数 | 含义与边界 |
| --- | --- |
| `mode / motor_id` | 真实固件模式和地址。 |
| `command` | 特殊协议命令枚举。 |
| `frame / expected_motor_id / limits` | 标准反馈帧、预期帧内电机地址和真实量程。 |
| `out / status` | 编码/解码输出或待分类驱动状态。 |

**返回 / 输出：** 构帧/解码返回0或负errno；isFaultStatus 返回故障判断。

**线程 / 时序：** 纯编解码；驱动将回调入队后在I/O线程解析。

**错误 / 边界：** 不能仅以 CAN TX完成认为已禁用：DM stop 的 DriveConfirmed 才表示后续反馈确认禁用；Unreachable 表示无法确认。

### 6. int Motor::setTorque(float newton_meter); int Motor::setTorque(float newton_meter, uint64_t sampled_enable_generation)

```cpp
int Motor::setTorque(float newton_meter); int Motor::setTorque(float newton_meter, uint64_t sampled_enable_generation)
```

MIT 纯力矩目标，computed effort 使用计算快照执行版本。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非有限 -EINVAL，明确范围 -ERANGE，模式不支持 -ENOTSUP，producer 不匹配 -EACCES；离线/非 Active 不拒绝合法目标。

### 7. int Motor::setMit(const dm::MitCommand &command)

```cpp
int Motor::setMit(const dm::MitCommand &command)
```

MIT 位置/速度/kp/kd/前馈目标，量化范围须匹配真实固件 PMAX/VMAX/TMAX。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非有限 -EINVAL，明确范围 -ERANGE，模式不支持 -ENOTSUP，producer 不匹配 -EACCES；离线/非 Active 不拒绝合法目标。

### 8. int Motor::setVelocity(float rad_s); int Motor::setPositionVelocity(float rad, float max_rad_s)

```cpp
int Motor::setVelocity(float rad_s); int Motor::setPositionVelocity(float rad, float max_rad_s)
```

固件原生速度或位置速度模式；主控模式配置不自动修改固件模式。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非有限 -EINVAL，明确范围 -ERANGE，模式不支持 -ENOTSUP，producer 不匹配 -EACCES；离线/非 Active 不拒绝合法目标。

### 9. MotorInfo Motor::info() const; MotorSnapshot Motor::snapshot() const; bool Motor::active() const

```cpp
MotorInfo Motor::info() const; MotorSnapshot Motor::snapshot() const; bool Motor::active() const
```

查询能力、参数和状态值副本；enabled_requested 是运行意图，output_permitted/feedback_fresh 是实际执行条件，retry_count 与 last_fault 是诊断。

**返回 / 输出：** 能力/状态值副本或实际 active 布尔值；快照读取不生产新反馈。

**线程 / 时序：** 允许多读取线程，短锁保护；有状态输出只有一个生产者。

**错误 / 边界：** 检查 feedback.valid 和原始 timestamp；实际状态不能替代运行意图。

### 10. int Motor::enable(); int Motor::disable()

```cpp
int Motor::enable(); int Motor::disable()
```

enable 幂等保存运行意图，离线也可接受；disable 在真→假时取消旧命令和旧协议操作，停止后重复 disable 不清掉之后新提交的目标。Group 成员也可直接操作。

**返回 / 输出：** 0 请求已接受，不证明机械执行或制动。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** -EACCES：CAN 未启动；disable 版本耗尽 -EOVERFLOW。

### 11. int Motor::reseedPosition(double known_position_rad)

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

### 12. int Motor::invalidateComputedEffort()

```cpp
int Motor::invalidateComputedEffort()
```

仅作废计算 effort，保留运行意图；反馈不足时应使旧计算输出失效。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 已绑定控制器 producer 时普通调用可能 -EACCES；封装内部使用其 producer。

### 13. CanBus(const device *can, BusOptions options = {}); int CanBus::attach(Motor &motor); int CanBus::start()

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

### 14. CommitResult CanBus::commit(); BusStatus CanBus::status() const

```cpp
CommitResult CanBus::commit(); BusStatus CanBus::status() const
```

commit 发布完整总线最新快照，Recovering 时继续接受。status 含 latest_submitted_sequence、last_tx、rx_overflows、rx_invalid_frames、superseded_batches 和 last_recovery。

**返回 / 输出：** CommitResult.error/sequence 表示发布结果；CAN TX 完成另看 last_tx。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 未启动/永久配置错误返回调用错误；成功不代表远端收到或执行。

### 15. Group(Motor &first, Others &...others); Group(std::span<Motor *const> members); int Group::enable(); void Group::disable(); GroupStatus Group::status() const

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
    const int accepted = motor.setTorque(0.1f);
    // 分别记录调用错误，设备离线不终止周期。
} else {
    (void)motor.disable();
}
const auto published = bus.commit();
const auto actual = motor.snapshot();
```

直接协议目标片段；effort=N·m。绑定 VelocityMotor/PositionMotor 后应改用 axis.update，不混用 producer。

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
| `Effort 单位` | N·m；应用限幅与控制输出必须使用同单位。 |
| `物理总线 owner` | 每 CAN 一个 CanBus，所有机构先 stage，统一每周期 commit 一次。 |

- 没有公共 Motor::ready/clearFault；Group 不传播成员故障。
- setter 接受、commit 发布、CAN TX 完成、机械执行是不同结果。
- 共享 DJI 帧只把不可执行槽置零，其他槽保留目标。
- 连续参考丢失不能猜圈数；可信 reseed 仅影响本轴。
- 历史 last_fault/last_recovery 不等于当前仍在故障。

## 正文与源码

- [DM Markdown](../modules/drivers/motor-dm.md)
- [三模式台架说明](../../samples/motor/DM_J4310_EXAMPLES.md)

- [dm_motor.hpp](../../include/drivers/motor/dm_motor.hpp)
- [dm_protocol.hpp](../../include/drivers/motor/dm_protocol.hpp)
- [motor.cpp](../../drivers/motor/motor.cpp)
- [can_bus.cpp](../../drivers/motor/can_bus.cpp)
- [main.cpp](../../samples/motor/dm_mit_control/src/main.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
