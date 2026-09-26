# SkyWalker `dev` 分支安全链路与双 CAN 改造指导

> 目标：在**不推翻现有双主控架构**的前提下，补齐板间心跳参与安全判定、云台板对底盘健康状态的全局认知，并让底盘 `DjiChassisHardware` 同时兼容 1 条或 2 条物理 CAN。
>
> 基线仓库：`SHM-white/SkyWalker_General_Embedded_Code`
>
> 基线分支：`dev`
>
> 基线提交：`429fd2205985c11ec051cd5acd0ee2dcb87b4451`
>
> 本文是**施工规格**，可以直接交给其他 AI 逐项修改代码。除本文明确列出的接口外，不允许借机重构现有框架。

---

## 0. 本次改造的边界

本次只处理三个问题：

1. **底盘 LocalSafety 增加 heartbeat freshness gate**
   - heartbeat 与 command freshness 独立；
   - 心跳失效时底盘必须本地 `Disable`；
   - 心跳恢复后，不能立即沿用旧命令恢复输出；
   - 必须重新经过稳定新命令计数。

2. **GlobalSafetyManager 增加 chassis peer / chassis feedback health**
   - 云台板必须知道底盘板是否在线；
   - 云台板必须知道底盘板反馈是否新鲜；
   - 底盘掉线或明显不可执行时，只禁用底盘，不影响仍然健康的小 Yaw；
   - 全局状态应进入 `Degraded`，而不是继续假装整车 `Active`。

3. **DjiChassisHardware 支持 1 / 2 条 CAN**
   - 现有单 CAN 配置必须继续工作；
   - 允许 8 个电机分布到两条物理 CAN；
   - CAN 拓扑只属于 hardware/application 层；
   - `SwerveChassis`、`SwerveModule`、`CommandManager`、板间协议都不感知 CAN 数量。

### 0.1 明确不做

本次禁止顺手实现：

- Vision / Auto CommandSource
- Shooter
- 大 Yaw / Pitch
- 新的裁判系统消息
- 功率模型标定
- zbus 替换 `Latest<T>`
- 把 `CommandRouter` 从 application 抽成通用库
- Swerve 算法重写
- 电机 PID 参数重调
- 改变现有 UART 帧格式版本
- 为了“双 CAN”把控制器/运动学拆成两份
- 修改驱动层使其感知“舵电机”“驱动电机”

如果施工中发现必须做上述内容才能完成本任务，应先停止并说明原因，而不是直接扩大修改范围。

---

# 1. 修改前架构确认

当前正确的数据流应保持：

```text
云台板
────────────────────────────────────────

DR16
  ↓
RemoteService
  ↓
RemoteState
  ↓
ManualCommandMapper
  ↓
OperatorIntent
  ↓
GlobalSafetyManager
  ↓
CommandManager
  ↓
RobotCommand
  ↓
CommandRouter
  ├──────────────→ LocalGimbalCommand → GimbalLocalSafety → YawGimbal
  │
  └──────────────→ RemoteChassisControl
                            ↓
                       InterBoard UART
                            ↓

底盘板
────────────────────────────────────────

InterBoard UART
  ↓
InterBoardLink
  ↓
RemoteChassisControl
ChassisConstraint
BoardHeartbeat
  ↓
ChassisLocalSafety
  ↓
SwerveChassis
  ↓
ChassisPowerLimiter
  ↓
DjiChassisHardware
  ↓
DJI Bus × 1 or 2
```

需要补充的安全链路为：

```text
底盘板 BoardHeartbeat ──────────────┐
                                   ↓
                            GlobalSafetyManager
                                   ↓
                           chassis Active/Disable

底盘板 ChassisFeedbackSummary ─────┘


云台板 BoardHeartbeat
        ↓ UART
底盘板 InterBoardLink
        ↓
ChassisLocalSafety
        ↓
heartbeat fresh ?
        ├─ no  → Disable + TransportUnavailable
        └─ yes → 继续检查 command / power / feedback / hardware
```

---

# 2. 总体接口原则

## 2.1 heartbeat 与 command 必须是两套 freshness

禁止：

```cpp
bool link_ok = command_fresh;
```

也禁止：

```cpp
收到 ChassisControl 就等于 peer online
```

二者语义不同：

```text
heartbeat fresh
= 对端任务/通信链路仍活着

command fresh
= 对端最近真正产生过新的控制意图
```

即使 UART task 不断重发旧 command，也不能刷新 command freshness。

现有协议已经保留 producer command sequence，这个行为必须保持。

---

## 2.2 Safety 层只接收状态，不依赖 UART 类

禁止让：

```cpp
ChassisLocalSafety
GlobalSafetyManager
```

直接 include：

```cpp
InterBoardLink
AsyncUart
```

Safety 只接收：

```text
MessageStamp
ExecutionState
SafetyAction
bool / enum 状态
```

由 application 把通信层 snapshot 传进 Safety。

---

## 2.3 底盘掉线不得拖死本地云台

全局安全策略：

```text
remote 正常
referee 允许
gimbal 本地健康
chassis 掉线
```

期望：

```text
gimbal  = Active / Hold
chassis = Disable
global state = Degraded
```

而不是：

```text
gimbal  = Disable
chassis = Disable
```

底盘通信故障不是全车急停。

---

## 2.4 EStop 仍然最高优先级

本次新增 heartbeat / peer health 不得改变：

```text
EmergencyStop > 其他所有条件
```

全局/本地 EStop latch 逻辑保持现有设计。

---

# 3. Task A：ChassisLocalSafety 增加 heartbeat freshness gate

## 3.1 需要修改的文件

至少修改：

```text
include/robotics/messages/safety.hpp
include/robotics/safety/chassis_local_safety.hpp
lib/robotics/safety.cpp
applications/sentry_chassis/src/board_config.hpp
applications/sentry_chassis/src/main.cpp
```

建议同步修改：

```text
samples/robotics/command_safety/*
双主控框架使用说明.md
```

## 3.2 修改 `LocalSafetyInputs`

文件：

```text
include/robotics/messages/safety.hpp
```

增加：

```cpp
MessageStamp peer_heartbeat_stamp{};
```

推荐最终接口：

```cpp
struct LocalSafetyInputs {
    std::uint64_t now_ms = 0;

    std::uint64_t receiver_boot_id = 0;
    std::uint64_t command_boot_id = 0;

    std::uint32_t resume_generation = 0;
    std::uint32_t command_generation = 0;

    MessageStamp peer_heartbeat_stamp{};
    MessageStamp command_stamp{};

    SafetyAction global_action = SafetyAction::Disable;

    bool power_allowed = false;
    bool feedback_fresh = false;
    bool hardware_ready = false;
    bool armed = false;
    bool config_valid = true;
    bool emergency_stop_requested = false;
};
```

### 约束

`peer_heartbeat_stamp`：

- 表示本板最近一次收到**合法 peer heartbeat** 的本地接收时间；
- 必须使用 `InterBoardLink::latestHeartbeat()` 中 heartbeat 的 `stamp`；
- 不允许使用发送端 uptime 进行 freshness 判断；
- 不允许收到普通 control frame 时刷新它。

---

# 4. 修改 `ChassisLocalSafety::Config`

文件：

```text
include/robotics/safety/chassis_local_safety.hpp
```

修改为：

```cpp
struct Config {
    std::uint32_t command_timeout_ms = 100;
    std::uint32_t heartbeat_timeout_ms = 100;
    std::uint32_t stable_command_count = 3;
};
```

要求：

```cpp
command_timeout_ms > 0
heartbeat_timeout_ms > 0
stable_command_count > 0
```

当前 heartbeat TX 周期约 20 ms，因此初始建议：

```cpp
heartbeat_timeout_ms = 100;
```

也就是允许大约 5 个 heartbeat 周期的短暂抖动。

---

# 5. 修改 `ChassisLocalSafety::evaluate()`

文件：

```text
lib/robotics/safety.cpp
```

## 5.1 新增 freshness

```cpp
const bool heartbeat_fresh =
    isFresh(i.peer_heartbeat_stamp,
            i.now_ms,
            config_.heartbeat_timeout_ms);

const bool command_fresh =
    isFresh(i.command_stamp,
            i.now_ms,
            config_.command_timeout_ms);
```

原来的 `fresh` 建议重命名成 `command_fresh`。

## 5.2 修改参数校验

```cpp
if (!config_.command_timeout_ms ||
    !config_.heartbeat_timeout_ms ||
    !config_.stable_command_count ||
    i.global_action > SafetyAction::Active)
    return -EINVAL;
```

## 5.3 heartbeat stale 时必须清稳定计数

改为：

```cpp
if (!heartbeat_fresh ||
    !command_fresh ||
    !context ||
    !i.power_allowed ||
    i.global_action != SafetyAction::Active) {
    count_ = 0;
}
```

这条必须保证：heartbeat 断开后形成新的 recovery boundary。

---

# 6. LocalSafety 推荐判定优先级

```text
1. Invalid Configuration
2. Emergency Stop
3. Peer Heartbeat Stale
4. Power Disabled / Stale
5. Feedback / Hardware Not Ready
6. Command Stale
7. Boot / Generation Context Mismatch
8. Stable Command Count 未满足
9. Active
```

推荐代码结构：

```cpp
if (!i.config_valid) {
    n.state = ExecutionState::ConfigBlocked;
    n.active_reasons = InvalidConfiguration;
}
else if (estop_latched_) {
    n.state = ExecutionState::EStopLatched;
    n.active_reasons = EmergencyStop;
}
else if (!heartbeat_fresh) {
    n.state = ExecutionState::Waiting;
    n.active_reasons = TransportUnavailable;
}
else if (!i.power_allowed) {
    n.state = ExecutionState::Waiting;
    n.active_reasons = PowerDisabled;
}
else if (!i.feedback_fresh || !i.hardware_ready) {
    n.state = ExecutionState::Recovering;
    n.active_reasons = FeedbackStale;
}
else {
    n.state = ExecutionState::Ready;

    if (!command_fresh)
        n.active_reasons |= CommandStale;

    if (!context || count_ < config_.stable_command_count)
        n.active_reasons |= RecoveryBoundary;

    if (i.global_action != SafetyAction::Active)
        n.active_reasons |= OperatorDisabled;

    if (!n.active_reasons) {
        n.action = SafetyAction::Active;
        n.state = i.armed
                      ? ExecutionState::Active
                      : ExecutionState::Ready;
    }
}
```

heartbeat stale 时绝不能 `Hold`。

---

# 7. 在底盘 application 接入 heartbeat stamp

文件：

```text
applications/sentry_chassis/src/main.cpp
```

构造 `LocalSafetyInputs` 时增加：

```cpp
input.peer_heartbeat_stamp = rx.peer.stamp;
```

保留：

```cpp
input.command_stamp = rx.command.command.stamp;
```

必须是两个独立 stamp。

---

# 8. 增加底盘 heartbeat timeout 配置

文件：

```text
applications/sentry_chassis/src/board_config.hpp
```

增加：

```cpp
heartbeat_timeout_ms = 100;
```

推荐：

```cpp
inline constexpr std::uint32_t
    command_timeout_ms = 100,
    heartbeat_timeout_ms = 100,
    permission_timeout_ms = 300,
    feedback_stable_ms = 30,
    recovery_retry_ms = 100;
```

---

# 9. 不再使用脆弱的 aggregate 位置初始化

增加字段以后，不建议继续：

```cpp
ChassisLocalSafety safety({ ... });
```

改成：

```cpp
ChassisLocalSafety::Config safety_config{};
safety_config.command_timeout_ms =
    board_config::command_timeout_ms;
safety_config.heartbeat_timeout_ms =
    board_config::heartbeat_timeout_ms;
safety_config.stable_command_count = 3;

ChassisLocalSafety safety(safety_config);
```

---

# 10. Task A 验收条件

| heartbeat | command | context | stable count | 结果 |
|---|---|---|---:|---|
| fresh | fresh | valid | >=3 | Active |
| stale | fresh | valid | >=3 | Disable + TransportUnavailable |
| fresh | stale | valid | >=3 | Disable + CommandStale |
| fresh | fresh | invalid | >=3 | Disable + RecoveryBoundary |
| fresh | fresh | valid | 1~2 | Disable + RecoveryBoundary |
| stale→fresh | 旧 command 重发 | valid | 旧 count=3 | 仍不得 Active |
| stale→fresh | 连续 3 个新 command sequence | valid | 3 | 才允许 Active |

额外要求：

```text
heartbeat stale
→ hardware output 最终为 0 / stopped
→ 不依赖云台板再发送一条“STOP”
```

---

# 11. Task B：GlobalSafetyManager 纳入底盘 peer / feedback health

## 11.1 目标

GlobalSafety 新增：

```text
底盘 heartbeat 是否新鲜
底盘 feedback 是否新鲜
底盘当前 execution state 是否可执行
```

GlobalSafety 只决定逻辑层是否继续向底盘下发 Active command；它不是底盘最后一道保护。

---

# 12. 修改 `GlobalSafetyInputs`

文件：

```text
include/robotics/messages/safety.hpp
```

推荐增加：

```cpp
MessageStamp chassis_heartbeat_stamp{};
MessageStamp chassis_feedback_stamp{};

ExecutionState chassis_execution_state =
    ExecutionState::Waiting;

std::uint32_t chassis_active_reasons = 0;
```

推荐完整形态：

```cpp
struct GlobalSafetyInputs {
    std::uint64_t now_ms = 0;

    bool command_source_fresh = false;
    bool operator_motion_enabled = false;
    bool emergency_stop_requested = false;

    OutputPermission gimbal_power{};
    OutputPermission chassis_power{};
    OutputPermission shooter_power{};

    MessageStamp chassis_heartbeat_stamp{};
    MessageStamp chassis_feedback_stamp{};

    ExecutionState chassis_execution_state =
        ExecutionState::Waiting;

    std::uint32_t chassis_active_reasons = 0;
};
```

这里传 stamp 而不是 `bool chassis_online`，让 freshness policy 继续归 SafetyManager。

---

# 13. 修改 `GlobalSafetyManager::Config`

文件：

```text
include/robotics/safety/global_safety_manager.hpp
```

增加：

```cpp
std::uint32_t chassis_heartbeat_timeout_ms = 100;
std::uint32_t chassis_feedback_timeout_ms = 100;
```

推荐：

```cpp
struct Config {
    bool require_referee_for_motion = false;

    std::uint32_t permission_timeout_ms = 300;

    std::uint32_t chassis_heartbeat_timeout_ms = 100;
    std::uint32_t chassis_feedback_timeout_ms = 100;
};
```

---

# 14. GlobalSafety 的底盘 health 策略

保留现有：

```cpp
n.gimbal = action(i.gimbal_power, true);
n.chassis = action(i.chassis_power, false);
n.shooter = action(i.shooter_power, false);
```

然后只对 `n.chassis` 增加 transport / health clamp。

```cpp
const bool chassis_heartbeat_fresh =
    isFresh(i.chassis_heartbeat_stamp,
            i.now_ms,
            config_.chassis_heartbeat_timeout_ms);

const bool chassis_feedback_fresh =
    isFresh(i.chassis_feedback_stamp,
            i.now_ms,
            config_.chassis_feedback_timeout_ms);
```

推荐：

```cpp
if (!chassis_heartbeat_fresh) {
    n.chassis = SafetyAction::Disable;
    n.active_reasons |= TransportUnavailable;
}
else if (!chassis_feedback_fresh) {
    n.chassis = SafetyAction::Disable;
    n.active_reasons |= FeedbackStale;
}
else {
    switch (i.chassis_execution_state) {
    case ExecutionState::Ready:
    case ExecutionState::Active:
        break;

    case ExecutionState::ConfigBlocked:
        n.chassis = SafetyAction::Disable;
        n.active_reasons |= InvalidConfiguration;
        break;

    case ExecutionState::EStopLatched:
        n.chassis = SafetyAction::Disable;
        n.active_reasons |= EmergencyStop;
        break;

    case ExecutionState::Waiting:
    case ExecutionState::Recovering:
    default:
        n.chassis = SafetyAction::Disable;
        n.active_reasons |= i.chassis_active_reasons;
        break;
    }
}
```

`Waiting / Recovering` 时仍要 `Disable`，但不要为了“必须有 reason”制造假故障。

---

# 15. GlobalSafety 的 state 规则

关键场景：

```text
gimbal = Active
chassis = Disable
chassis reason = TransportUnavailable
```

必须：

```cpp
state = SafetyState::Degraded;
```

不能因为底盘掉线就强制：

```cpp
gimbal = Disable;
```

---

# 16. 云台 application 接入 peer heartbeat / chassis feedback

文件：

```text
applications/sentry_gimbal/src/main.cpp
```

当前已有：

```cpp
Latest<BoardHeartbeat> peer_heartbeat;
Latest<ChassisFeedbackSummary> chassis_feedback;
```

在 `commandTask()` 内增加：

```cpp
BoardHeartbeat peer{};
ChassisFeedbackSummary feedback{};
```

循环：

```cpp
peer_heartbeat.get(peer);
chassis_feedback.get(feedback);
```

然后：

```cpp
input.chassis_heartbeat_stamp = peer.stamp;
input.chassis_feedback_stamp = feedback.stamp;
input.chassis_execution_state = feedback.execution_state;
input.chassis_active_reasons = feedback.active_reasons;
```

不要求 application 预先算 freshness。

---

# 17. 修改云台 board_config

文件：

```text
applications/sentry_gimbal/src/board_config.hpp
```

增加：

```cpp
chassis_heartbeat_timeout_ms = 100;
chassis_feedback_timeout_ms = 100;
```

推荐：

```cpp
inline constexpr std::uint32_t
    permission_timeout_ms = 300,
    command_timeout_ms = 100,
    chassis_heartbeat_timeout_ms = 100,
    chassis_feedback_timeout_ms = 100;
```

---

# 18. 修改 GlobalSafetyManager 构造方式

改成明确赋值：

```cpp
GlobalSafetyManager::Config safety_config{};
safety_config.require_referee_for_motion =
    board_config::require_referee_for_motion;

safety_config.permission_timeout_ms =
    board_config::permission_timeout_ms;

safety_config.chassis_heartbeat_timeout_ms =
    board_config::chassis_heartbeat_timeout_ms;

safety_config.chassis_feedback_timeout_ms =
    board_config::chassis_feedback_timeout_ms;

GlobalSafetyManager safety(safety_config);
```

---

# 19. GlobalSafety 与 CommandRouter 的关系保持不变

`CommandRouter` 仍然只负责：

```text
RobotCommand
+
GlobalSafetyDecision
+
peer boot/generation context

→ local command
→ remote command
```

错误做法：

```cpp
if (!peer_online)
    CommandRouter 自己把 vx/vy/wz 改 0;
```

Router 不是策略层。

---

# 20. Task B 验收矩阵

| Remote | Referee | chassis heartbeat | chassis feedback | chassis state | gimbal | chassis | global |
|---|---|---|---|---|---|---|---|
| OK | OK | fresh | fresh | Ready | Active | Active | Active |
| OK | OK | stale | 任意 | 任意 | Active | Disable | Degraded |
| OK | OK | fresh | stale | 任意 | Active | Disable | Degraded |
| OK | OK | fresh | fresh | Recovering | Active | Disable | Degraded |
| OK | OK | fresh | fresh | ConfigBlocked | Active | Disable | Degraded |
| OK | OK | fresh | fresh | EStopLatched | 按本地/全局规则 | Disable | 非完整 Active |
| stale | OK | fresh | fresh | Ready | Hold/Disable | Disable | 非 Active |

重点：

```text
chassis 掉线
≠
gimbal 必须掉线
```

---

# 21. Task C：`DjiChassisHardware` 支持 1 / 2 条 CAN

## 21.1 当前限制

当前 hardware 初始化时要求所有 descriptor 使用同一物理 CAN。本次只在：

```text
applications/sentry_chassis/src/chassis_hardware.*
```

解除限制。

禁止修改：

```text
SwerveChassis
SwerveModule
SwerveKinematics
CommandManager
InterBoard protocol
```

---

# 22. 双 CAN 目标行为

## 22.1 单 CAN

```text
CAN1
├─ steer FL
├─ steer FR
├─ steer RL
├─ steer RR
├─ drive FL
├─ drive FR
├─ drive RL
└─ drive RR
```

结果：

```text
bus_count = 1
```

## 22.2 双 CAN

允许：

```text
CAN1 → 4 steer
CAN2 → 4 drive
```

也允许任意其他 1/2 CAN 分配。

Hardware 根据：

```cpp
Descriptor::can
```

自动分组，CAN 拓扑由设备树决定。

---

# 23. 修改 `DjiChassisHardware` 成员

文件：

```text
applications/sentry_chassis/src/chassis_hardware.hpp
```

推荐：

```cpp
class DjiChassisHardware {
public:
    static constexpr std::size_t kMaxBuses = 2;

    ...

private:
    std::array<const device *, 8> motors_;
    std::array<skywalker::motor::dji::Descriptor, 8> descriptors_{};

    std::array<std::uint64_t, 8> stamps_{};
    std::array<std::uint64_t, 8> first_stamps_{};

    std::array<const device *, kMaxBuses> can_devices_{};
    std::array<std::uint8_t, 8> motor_bus_index_{};

    std::array<skywalker::motor::dji::Bus, kMaxBuses> buses_{};
    std::array<skywalker::motor::dji::FlushReport, kMaxBuses> reports_{};

    std::size_t bus_count_ = 0;

    ...
};
```

删除原来的单个：

```cpp
Bus bus_{};
FlushReport report_{};
```

---

# 24. 增加私有 helper

建议增加：

```cpp
int findOrCreateBus(const device *can);
int stopAllBuses();
```

示意：

```cpp
int DjiChassisHardware::findOrCreateBus(const device *can) {
    if (!can)
        return -ENODEV;

    for (std::size_t i = 0; i < bus_count_; ++i) {
        if (can_devices_[i] == can)
            return static_cast<int>(i);
    }

    if (bus_count_ >= kMaxBuses)
        return -ENOTSUP;

    const std::size_t index = bus_count_++;
    can_devices_[index] = can;

    const int ret = buses_[index].init(can);

    if (ret < 0) {
        --bus_count_;
        can_devices_[index] = nullptr;
        return ret;
    }

    return static_cast<int>(index);
}
```

如果 `Bus::init()` 失败，要恢复 `bus_count_`。

---

# 25. 重写 `init()` 的 CAN 分组部分

保留所有现有检查：

```text
motor 非空
device ready
direction 必须 ±1
describe 成功
capability 检查
current limit 检查
```

删除“所有电机必须和第 0 个 motor 同 CAN”的限制。

对每个 motor：

```cpp
const int bus_index =
    findOrCreateBus(descriptors_[i].can);

if (bus_index < 0)
    return bus_index;

motor_bus_index_[i] =
    static_cast<std::uint8_t>(bus_index);

ret = buses_[bus_index].attach(motors_[i]);

if (ret < 0)
    return ret;
```

最终：

```cpp
bus_count_ >= 1
bus_count_ <= 2
```

---

# 26. 双 CAN 下的 ID 冲突检查

这是本次最容易漏掉的地方。

不同 CAN 上允许相同：

```text
feedback_id
command_id + command_slot
```

因此检查必须加入 CAN 维度。

推荐：

```cpp
for (unsigned j = 0; j < i; ++j) {
    if (motors_[i] == motors_[j])
        return -EINVAL;

    const bool same_bus =
        descriptors_[i].can == descriptors_[j].can;

    if (!same_bus)
        continue;

    if (descriptors_[i].feedback_id ==
        descriptors_[j].feedback_id)
        return -EINVAL;

    if (descriptors_[i].command_id ==
            descriptors_[j].command_id &&
        descriptors_[i].command_slot ==
            descriptors_[j].command_slot)
        return -EINVAL;
}
```

---

# 27. 重写 `suspend()`

任何一个 bus / motor 出错时，要尽最大努力停止所有 bus。

推荐：

```cpp
int DjiChassisHardware::stopAllBuses() {
    int first_error = 0;

    for (std::size_t i = 0; i < bus_count_; ++i) {
        const int ret = buses_[i].stop(reports_[i]);

        if (ret < 0 && first_error == 0)
            first_error = ret;
    }

    return first_error;
}
```

`suspend()`：

```cpp
int DjiChassisHardware::suspend() {
    ready_ = false;
    armed_ = false;
    stable_ = false;
    next_retry_ms_ = 0;

    if (!initialized_ || stopped_)
        return 0;

    stopped_ = true;

    return stopAllBuses();
}
```

安全动作必须 best-effort 覆盖所有 CAN。

---

# 28. 重写 `pollRecovery()`

双 CAN：

```text
for every active CAN:
    pollCanRecovery(can)

for every bus:
    if Fault:
        recover(bus)

所有 bus 都成功
↓
read all 8 motor feedback
↓
measurement reference reset
↓
stable feedback
↓
ready
```

示意：

```cpp
for (std::size_t i = 0; i < bus_count_; ++i) {
    int ret = control::pollCanRecovery(can_devices_[i]);

    if (ret < 0)
        return fail_recovery(ret);

    if (buses_[i].state() ==
        motor::dji::BusState::Fault) {

        ret = buses_[i].recover(reports_[i]);

        if (ret < 0)
            return fail_recovery(ret);
    }
}
```

`resetMeasurementReference()` 仍按 8 个 motor 调用，不需要按 Bus 改写。

---

# 29. 重写 `arm()`

只有所有 bus 都成功 arm，整个 chassis 才算 armed。

```cpp
for (std::size_t i = 0; i < bus_count_; ++i) {
    const int ret = buses_[i].arm(reports_[i]);

    if (ret < 0 || !reports_[i].zero_sent) {
        stopAllBuses();

        ready_ = false;
        armed_ = false;
        stopped_ = true;

        return ret < 0 ? ret : -EIO;
    }
}

armed_ = true;
stopped_ = false;
```

如果 CAN1 arm 成功、CAN2 arm 失败，必须立即 stop 全部 bus。

---

# 30. 重写 `apply()`

前半段继续：

```text
ChassisOutput
→ 8 个 current
→ finite / current limit 检查
→ setCurrent × 8
```

然后：

```cpp
int first_error = 0;

for (std::size_t i = 0; i < bus_count_; ++i) {
    const int ret = buses_[i].flush(reports_[i]);

    if (ret < 0 && first_error == 0)
        first_error = ret;
}

if (first_error < 0) {
    suspend();
    return first_error;
}

return 0;
```

任意 bus 失败后，整个 chassis 进入 fail-safe。

---

# 31. `read()` 基本不需要变化

继续保持 motor 维度读取：

```cpp
motor::readFeedback(motors_[i], ...)
```

保留：

```text
feedback timestamp
motor state
capability
finite
velocity safety
temperature safety
estimated power
```

CAN 拓扑不要渗透到 `ChassisFeedback`。

---

# 32. `ready()` / `armed()` 仍然代表整个底盘

禁止让上层根据：

```text
CAN1 armed
CAN2 armed
```

分别决定运动。

任意 Bus 不健康：

```text
整个 hardware not ready / not armed
```

---

# 33. 双 CAN 与 DeviceTree

不需要在 `SwerveChassis::Config` 里增加 CAN 配置。

CAN 来源仍然是：

```text
motor device
    ↓
motor::dji::describe()
    ↓
Descriptor::can
```

真实双 CAN 只需要在：

```text
applications/sentry_chassis/app.overlay
```

把不同 motor 实例挂到对应 CAN controller。

alias：

```text
steer_fl
steer_fr
steer_rl
steer_rr
drive_fl
drive_fr
drive_rl
drive_rr
```

保持不变。

---

# 34. 双 CAN 错误场景

### 34.1 一个 Bus fault

```text
CAN1 normal
CAN2 fault
```

结果：

```text
hardware.suspend()
armed = false
ready = false
```

不得让 CAN1 长期继续驱动部分轮子。

### 34.2 一个 Bus recovery 成功，一个失败

整个 chassis 继续 `Recovering`。

### 34.3 3 条不同 CAN

当前 V1：

```cpp
return -ENOTSUP;
```

### 34.4 单 CAN 回归

原来的 8 motors → same CAN 必须继续工作。

---

# 35. 本次不要改板间帧格式

现有 heartbeat 已经包含：

```text
sender_role
safety_state
ready
sync_requested
active_reasons
sender_uptime
sender_boot_id
resume_generation
```

本次直接利用：

```text
heartbeat.stamp
```

因此 `InterBoard protocol version`、heartbeat payload、CRC、frame header 均不需要修改。

---

# 36. 不要用 sender uptime 判断 timeout

错误：

```cpp
now_ms - heartbeat.sender_uptime_ms
```

正确：

```cpp
isFresh(
    heartbeat.stamp,
    local_now_ms,
    timeout_ms);
```

因为 heartbeat stamp 是接收板自己的 local receive time。

---

# 37. heartbeat 与 producer command sequence 的关系

必须继续维持：

```text
heartbeat sequence
≠
command producer sequence
```

UART 重发旧 command 不能让底盘认为收到了新的控制意图。

现有：

```text
RemoteChassisControl.command.stamp.sequence
```

继续作为稳定 command 计数依据。

---

# 38. 推荐修改顺序

## Step 1

只改：

```text
LocalSafety heartbeat gate
```

目标：底盘独立具备 heartbeat stale 自动停机。

## Step 2

只改：

```text
GlobalSafety chassis peer / feedback health
```

目标：云台板正确报告 `Degraded`，并停止发送 Active chassis intent。

## Step 3

只改：

```text
DjiChassisHardware 1/2 CAN
```

目标：运动学与命令层完全无感。

## Step 4

只做：

```text
sample
README
日志
回归检查
```

不要把前三步揉成一个巨大 patch。

---

# 39. 建议日志

云台板每秒状态日志建议至少包含：

```text
global safety state
gimbal action
chassis action
peer heartbeat age / online
chassis feedback state
active reasons
```

底盘板建议至少保留：

```text
execution state
active reasons
generation
last accepted command
recovery error
bus_count
```

初始化成功时打印：

```text
DJI chassis hardware buses=1
```

或：

```text
DJI chassis hardware buses=2
```

---

# 40. 建议增加的纯逻辑验证

## 40.1 heartbeat timeout

```text
t=0    heartbeat fresh
t=0    command seq=1
t=10   command seq=2
t=20   command seq=3
→ Active

停止 heartbeat
t>heartbeat_timeout
→ Disable + TransportUnavailable
```

## 40.2 heartbeat 恢复但 command 未更新

```text
heartbeat 恢复
重复发送 producer seq=3
```

要求仍然 `Disable`。

直到：

```text
seq=4
seq=5
seq=6
```

才重新 Active。

## 40.3 GlobalSafety peer offline

输入：

```text
remote fresh
operator enabled
referee output enabled
gimbal permission fresh
chassis permission fresh
chassis heartbeat stale
```

输出：

```text
gimbal = Active
chassis = Disable
state = Degraded
reason includes TransportUnavailable
```

## 40.4 feedback stale

heartbeat 正常但 `ChassisFeedbackSummary.stamp` stale：

```text
chassis = Disable
reason includes FeedbackStale
```

---

# 41. 双 CAN 验证建议

## 41.1 单 CAN DeviceTree

```text
8 motors → CAN1
```

应用能构建，运行后 `bus_count = 1`。

## 41.2 双 CAN DeviceTree

临时配置：

```text
4 motors → CAN1
4 motors → CAN2
```

应用能构建，运行后 `bus_count = 2`。

## 41.3 相同 ID 跨 CAN

允许：

```text
CAN1 motor feedback ID = X
CAN2 motor feedback ID = X
```

## 41.4 相同 ID 同 CAN

拒绝：

```text
CAN1 motor A feedback ID = X
CAN1 motor B feedback ID = X
```

---

# 42. 小 Yaw 当前实机优先级

本次修改完成后，真实硬件验证仍然优先小 Yaw：

```text
1. 单板刷 sentry_gimbal
2. 暂时允许 bench 配置
3. 验证 RemoteService
4. 验证 CommandManager
5. 验证 GimbalLocalSafety
6. 验证 YawGimbal
7. 验证 command timeout → Hold/Disable
8. 再接第二块底盘板测试 UART heartbeat
```

真实底盘没准备好时，`SwerveChassis` / `DjiChassisHardware` 先以编译和逻辑验证为主。

---

# 43. 双板 UART 实机验证顺序

```text
Step A
只跑 heartbeat
```

确认双方互相 online。

```text
Step B
停止云台板 link task / 拔 UART
```

确认底盘：

```text
TransportUnavailable
→ Disable
```

```text
Step C
恢复 UART
```

确认不能瞬间 Active，必须经过新的 command sequence 稳定计数。

```text
Step D
停止底盘板
```

云台板必须：

```text
GlobalSafety.state = Degraded
chassis = Disable
gimbal 仍可工作
```

---

# 44. Safety reason 使用约束

现有 reason 足够：

```text
OperatorDisabled
CommandStale
PowerDisabled
PowerStale
FeedbackStale
TransportUnavailable
EmergencyStop
InvalidConfiguration
RecoveryBoundary
PowerBudgetStale
```

映射：

```text
heartbeat stale         → TransportUnavailable
chassis feedback stale  → FeedbackStale
command stale           → CommandStale
boot/generation mismatch→ RecoveryBoundary
referee output disabled → PowerDisabled
referee data stale      → PowerStale
power constraint stale  → PowerBudgetStale
```

本次不要新增 reason，除非现有 reason 确实无法表达。

---

# 45. 对 `InterBoardLink` 的约束

本次通常不需要修改：

```text
lib/communication/interboard_link.cpp
```

现有关键行为应保留：

```text
1. 首次 heartbeat 前拒绝普通消息
2. peer boot ID 改变后清空旧 control/constraint/feedback
3. frame sequence 去重
4. producer command sequence 不由 TX 重发刷新
5. peerOnline() 基于 heartbeat stamp
```

---

# 46. 对 `CommandManager` 的约束

本次不修改其职责：

```text
OperatorIntent
+
GlobalSafetyDecision
→ RobotCommand
```

禁止 `CommandManager` 检查 UART、heartbeat、CAN。

---

# 47. 对 `SwerveChassis` 的约束

本次完全不应该知道：

```text
heartbeat
UART
CAN1/CAN2
referee
GlobalSafety
```

它的输入依然只有：

```cpp
ChassisCommand
ChassisFeedback
dt
```

输出：

```cpp
ChassisOutput
```

如果双 CAN 修改触碰 `lib/robotics/swerve.cpp`，通常说明分层做错了。

---

# 48. 对 `DjiChassisHardware` 的职责约束

它可以知道：

```text
8 个 motor device
Descriptor
CAN device
Bus
feedback timeout
current limit
temperature
hardware recovery
```

它不应该知道：

```text
RemoteState
OperatorIntent
CommandManager
GlobalSafetyManager
RefereeState
UART packet
```

---

# 49. 最终期望架构

```text
                    ┌──────────────────────┐
                    │      DR16 / VTM      │
                    └──────────┬───────────┘
                               ↓
                      ManualCommandMapper
                               ↓
                         OperatorIntent
                               ↓
                  ┌─────────────────────────┐
                  │   GlobalSafetyManager   │
                  │                         │
                  │ remote health           │
                  │ referee permission      │
                  │ chassis heartbeat       │
                  │ chassis feedback        │
                  └───────────┬─────────────┘
                              ↓
                       CommandManager
                              ↓
                        CommandRouter
                     ┌────────┴─────────┐
                     ↓                  ↓
                 Local Yaw       RemoteChassisControl
                     ↓                  ↓
            GimbalLocalSafety       UART + CRC
                     ↓                  ↓
                 YawGimbal       ChassisLocalSafety
                                         │
                               heartbeat fresh?
                               command fresh?
                               context valid?
                               power allowed?
                               hardware ready?
                                         ↓
                                  SwerveChassis
                                         ↓
                               ChassisPowerLimiter
                                         ↓
                               DjiChassisHardware
                                  ┌──────┴──────┐
                                  ↓             ↓
                               DJI Bus 0     DJI Bus 1
                                  ↓             ↓
                                CAN A         CAN B
```

---

# 50. 最终验收标准

- [ ] `dev` 当前单 CAN 配置仍能构建
- [ ] `LocalSafetyInputs` 有独立 heartbeat stamp
- [ ] heartbeat stale 会触发 `TransportUnavailable`
- [ ] heartbeat stale 会清空 stable command count
- [ ] heartbeat 恢复后旧 command 不会直接恢复 Active
- [ ] GlobalSafety 能看到 chassis heartbeat freshness
- [ ] GlobalSafety 能看到 chassis feedback freshness
- [ ] chassis 掉线时 gimbal 不被无条件 Disable
- [ ] chassis 掉线时 global state 能体现 `Degraded`
- [ ] `CommandRouter` 不承担 Safety policy
- [ ] `SwerveChassis` 不感知 UART / CAN topology
- [ ] `DjiChassisHardware` 自动识别 1 / 2 条 CAN
- [ ] 同一 CAN 内重复 feedback ID / command slot 被拒绝
- [ ] 不同 CAN 上相同 ID 可以存在
- [ ] 任意一个 Bus arm/flush/recover 失败会使整个 chassis fail-safe
- [ ] `suspend()` 会 best-effort stop 所有 Bus
- [ ] 不允许只剩部分舵轮继续输出
- [ ] 3 条 CAN 返回 `-ENOTSUP`
- [ ] 板间协议版本和 payload 不因本任务发生无关变化
- [ ] 现有 boot ID / resume generation / producer sequence 机制保留

---

# 51. 给实施 AI 的最终任务说明

```text
基于 dev 分支提交
429fd2205985c11ec051cd5acd0ee2dcb87b4451
实施《SkyWalker_dev安全链路与双CAN改造指导.md》。

要求：

1. 严格限定修改范围为：
   - ChassisLocalSafety heartbeat freshness gate
   - GlobalSafetyManager chassis peer/feedback health
   - DjiChassisHardware 1/2 CAN 支持
   - 与以上改动直接相关的 application 配置、sample、文档

2. 不修改现有总体架构：
   Remote → Intent → GlobalSafety → CommandManager
   → CommandRouter → Local/Remote execution。

3. heartbeat freshness 与 command freshness 必须独立。

4. heartbeat stale 后底盘必须本地 Disable，并清空稳定新命令计数。

5. UART 恢复后不得因为重复发送旧 command 而立即重新 Active。

6. chassis peer / feedback 故障只禁用 chassis；
   如果 gimbal 本身健康，允许 gimbal 继续运行，
   GlobalSafetyState 应反映 Degraded。

7. 双 CAN 只在 DjiChassisHardware 层处理。
   SwerveChassis/SwerveModule/CommandManager 不得感知 CAN 数量。

8. DjiChassisHardware 自动按 Descriptor::can 将 8 个电机分配到最多 2 个 Bus。
   单 CAN 必须兼容。

9. 任意一个 Bus 故障时整个 chassis fail-safe，
   不允许部分轮组继续长期输出。

10. stop/suspend 必须 best-effort 操作所有 Bus，
    不能遇到第一个错误就跳过剩余 Bus。

11. 同一 CAN 内检查 feedback ID / command slot 冲突；
    不同 CAN 上允许相同 ID。

12. 不改变现有 InterBoard protocol frame format，
    直接使用 heartbeat 本地 receive stamp 做 timeout。

13. 修改完成后先给出：
    - 修改文件列表
    - 每个接口的最终定义
    - Safety 状态迁移说明
    - 单 CAN / 双 CAN 数据流说明
    - 构建结果
    - 尚未实机验证的部分

14. 不要顺手实现 Vision、Shooter、Auto、功率模型标定或其他功能。
```

---

# 52. 推荐后续路线

本次改造完成以后，架构层暂时冻结。

```text
P0  小 Yaw 单板实机
 ↓
P1  双板 UART heartbeat / timeout 实机
 ↓
P2  双板 Command + Constraint 实机
 ↓
P3  单个 SwerveModule 实机
 ↓
P4  四 Swerve + 双 CAN
 ↓
P5  功率模型标定
 ↓
P6  大 Yaw / Pitch / Shooter
 ↓
P7  Vision / Autonomous
```

在 P4 之前，不建议再次大改总体架构。

---

**文档结束。**
