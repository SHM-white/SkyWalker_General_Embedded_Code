# SkyWalker 双主控分布式机器人架构施工规格

> 目标仓库：`SHM-white/SkyWalker_General_Embedded_Code`
> 机器人：RoboMaster 哨兵
> 主控拓扑：**云台主控板 + 底盘主控板**
> 板间通信：**UART 双向通信**
> 当前可测试硬件：**小 Yaw 云台**
> 当前不可完整测试硬件：**4 舵 + 4 驱动底盘**
> 本文用途：**统一架构与手工施工规格；默认支持暂停后自动恢复**

---

快速入口：[可行性与源码证据](#review) · [安全分级](#safety) · [电机断电恢复](#power-recovery) · [施工顺序](#stages) · [底层改造](#lifecycle) · [专项验收](#acceptance)

<a id="review"></a>

# 0. 文档范围与可行性结论

本文是机器人架构施工的唯一主文档，合并原应用层与消息规格、舵轮分阶段指南、双主控命令路由指南，并修订仓库已有的同名综合草稿。保留消息、遥控、裁判、云台、舵轮、双板通信、施工顺序和验证内容；已存在的控制库不再列为待重写模块。

**结论：双板部署可行；旧版安全生命周期不能直接用于比赛应用，必须先补齐可恢复运行。** 算法分层没有要求一次错误后结束线程，但旧规格的 `Disabled → motor.stop()`、笼统的严重故障锁存，以及现有 wrapper 的一次性 begin，组合起来确实可能让正常电机断电变成无法自动恢复。

本次是文档合并与设计修订，业务源码未修改，未执行构建或硬件测试。本文中的拟新增接口是施工契约，不代表仓库已实现。仓库 AGENTS.md 与古法编程模式继续有效；文档内的代码和施工步骤不构成修改源码的授权。

当前明确假设：两板通过串口双向通信；暂把遥控、裁判和决策入口放在云台板。实际接线不同可移动输入服务。用户提出的“比赛中电机电源切断而主控仍运行”作为必须支持的工况；本文不推断具体赛季、处罚原因、供电时序或裁判协议字段布局。

阅读顺序：先看第 14～20 节及第 61 节的恢复策略，再按第 64 节施工；数值控制细节见第 73 节，文件级修改见第 74 节，断电专项验收见第 75 节。

## 0.1 本次确认的代码事实

| 位置                                                                 | 当前行为                                                                        | 对施工的影响                                     |
| -------------------------------------------------------------------- | ------------------------------------------------------------------------------- | ------------------------------------------------ |
| `lib/control/motor_control.cpp`，`MotorRuntime::cycle/send/fail` | dt、读反馈和发送错误最终进入 Fault，并调用 backend.stop                         | 不能只在上层忽略返回值，内部已经退出 Running     |
| 同文件，`VelocityMotor::begin/PositionMotor::begin`                | begin_attempted_ 一旦置位，再调用返回 -EALREADY；update 非 Running 返回 -EACCES | 重复 begin 或清一个上层标志不能恢复              |
| `lib/control/dji_motor_backend.cpp`，prepare                       | 等待反馈约 2 秒后返回超时                                                       | 电机稍晚上电可耗尽唯一启动机会                   |
| `lib/control/dm_motor_backend.cpp`，prepare                        | 最多约 3 秒并发送禁用探测                                                       | 等待电源应改成持续服务状态，不是终止应用         |
| `drivers/motor/dji/dji_bus.cpp`                                    | flush 失败会 Fault；已有 recover，stop 保留已有 Fault                           | 必须按总线实际状态恢复，不能在 Safe 时调用 flush |
| `drivers/motor/dji/dji_motor.cpp`，getStateImpl                    | 软件 fault_latched 优先于反馈新鲜度                                             | recover 前不能把 getState()==Ready 设为必要条件  |
| `drivers/motor/dm/dm_bus.cpp`，recover                             | 有发送清错/禁用与等待新反馈阶段，可返回 -EINPROGRESS/-EAGAIN                    | 这两个返回值应继续调度；不能当成又一次永久故障   |
| `samples/motor/dji_speed_control/src/main.cpp`                     | begin/update 出错后 return；示例还有限时运行                                    | 台架示例的退出策略不能照搬到比赛常驻任务         |

这些是源码路径推导，不能据此断言以前实车故障只有这一个原因。CAN 控制器 bus-off 也不等于电机永久损坏；Zephyr 有独立恢复接口及模式约束，见[官方 CAN API](https://docs.zephyrproject.org/latest/doxygen/html/group__can__controller.html)。本地 `../zephyr/include/zephyr/drivers/can.h` 也可核对；项目版本以 west.yml 锁定提交为准。

---

# 1. 当前代码基线

仓库已有：

```text
Motor Driver
├── DJI M3508 / M2006 / GM6020
├── DM J4310
├── Motor capability
├── continuous position
├── absolute single-turn position
└── Bus lifecycle

Control
├── PID
├── Feedforward
├── Slew Rate
├── Angle
├── motor_velocity
├── motor_position
├── VelocityMotor
└── PositionMotor

Sensor
├── BMI088
├── Kalman
└── EKF
```

已存在核心类：

```cpp
skywalker::control::MotorBackend
skywalker::control::DjiMotorBackend
skywalker::control::DmMotorBackend
skywalker::control::VelocityMotor
skywalker::control::PositionMotor
```

`PositionMotor` 已支持以下坐标模式，但其可恢复生命周期尚未实现：

```cpp
enum class PositionReference {
    StartupRelative,
    DriverContinuous,
    AbsoluteNearest,
};
```

因此小 Yaw 复用已有控制算法，并先完成第 74 节生命周期改造：

```text
PositionMotor + AbsoluteNearest
```

禁止重新写一套位置环。

---

# 2. 总体架构

```text
                ┌─────────────────────────┐
                │       云台主控板         │
                │                         │
DR16 ──────────►│ Remote                  │
Referee ───────►│ Referee                 │
Vision ────────►│ Vision (future)         │
                │                         │
                │ GlobalSafetyManager     │
                │ CommandManager          │
                │ CommandRouter           │
                │     │          │        │
                │     │          └────────┼─► InterBoardLink ───┐
                │     ▼                   │                      │
                │ YawGimbal               │                      │
                │                         │                      │
                └─────────────────────────┘                      │
                                                               UART
                ┌─────────────────────────┐                      │
                │       底盘主控板         │◄─────────────────────┘
                │                         │
                │ InterBoardLink          │
                │ ChassisLocalSafety      │
                │ SwerveKinematics        │
                │ 4 × SwerveModule        │
                │ Shared DJI Bus          │
                │ 4 steer + 4 drive       │
                │ ChassisFeedback         │
                └─────────────────────────┘
```

核心原则：

```text
云台板决定“整车想做什么”
底盘板决定“底盘怎样实时、安全地做到”
UART 只传高层语义命令
实时 PID / 舵轮解算 / CAN 留在底盘板
```

---

# 3. 两块板职责

## 3.1 云台板

负责：

```text
DR16
裁判系统
视觉/自动控制入口
GlobalSafetyManager
ManualCommandMapper
CommandManager
CommandRouter
小 Yaw 控制
板间通信
全车状态汇总
```

不负责：

```text
4 舵轮运动学
底盘 8 电机 PID
底盘 CAN flush
底盘本地故障兜底
```

## 3.2 底盘板

负责：

```text
板间命令接收
板间 heartbeat
命令 freshness
ChassisLocalSafety
SwerveKinematics
4 × SwerveModule
底盘控制器
共享 DJI Bus
8 电机状态
功率限制
底盘反馈
```

不负责：

```text
DR16 协议
裁判协议原始解析
视觉协议
全局模式仲裁
CommandManager
```

---

# 4. 依赖方向

必须保持：

```text
Input
 ↓
Decoder
 ↓
State / Intent
 ↓
Global Safety
 ↓
CommandManager
 ↓
RobotCommand
 ↓
CommandRouter
 ├── 本地命令槽
 └── 远端待发槽
       ↓
   InterBoardLink
       ↓
   Local Safety
       ↓
   Subsystem
       ↓
   Controller
       ↓
   Motor / Driver
```

禁止反向依赖。

---

# 5. 推荐目录

```text
include/
├── communication/
│   ├── remote/
│   │   ├── dr16_decoder.hpp
│   │   └── remote_service.hpp
│   ├── referee/
│   │   ├── referee_protocol.hpp
│   │   ├── referee_parser.hpp
│   │   └── referee_service.hpp
│   └── interboard/
│       ├── interboard_protocol.hpp
│       ├── interboard_codec.hpp
│       ├── interboard_parser.hpp
│       └── interboard_link.hpp
│
├── robotics/
│   ├── messages/
│   │   ├── common.hpp
│   │   ├── remote.hpp
│   │   ├── referee.hpp
│   │   ├── command.hpp
│   │   ├── feedback.hpp
│   │   ├── safety.hpp
│   │   └── interboard.hpp
│   ├── command/
│   │   ├── manual_command_mapper.hpp
│   │   └── command_manager.hpp
│   ├── safety/
│   │   ├── global_safety_manager.hpp
│   │   ├── gimbal_local_safety.hpp
│   │   └── chassis_local_safety.hpp
│   ├── gimbal/
│   │   └── yaw_gimbal.hpp
│   ├── swerve/
│   │   ├── swerve_types.hpp
│   │   ├── swerve_kinematics.hpp
│   │   ├── swerve_module.hpp
│   │   └── swerve_chassis.hpp
│   └── chassis/
│       └── chassis_power_limiter.hpp
│
lib/
├── communication/
└── robotics/

applications/
├── sentry_gimbal/
└── sentry_chassis/
```

两个主控必须是**两个独立 Zephyr application**，共享同一个 module/library。

---

# 6. 全局接口规范

公开接口单位统一：

```text
rad
rad/s
m/s
A
N·m
W
J
ms
s
```

字段名必须包含单位：

```cpp
yaw_rad
velocity_rad_s
power_w
timeout_ms
```

可能失败的函数：

```text
0       成功
< 0     -errno
```

实时控制路径禁止动态分配。

`lib/robotics/*` 禁止：

```text
创建线程
sleep
读取 UART
访问 zbus
DEVICE_DT_GET
读取系统时间
直接操作 CAN
```

---

# 7. 公共消息时间戳

`include/robotics/messages/common.hpp`

```cpp
namespace skywalker::robotics {

enum class ControlSource : std::uint8_t {
    None = 0,
    Remote,
    KeyboardMouse,
    Vision,
    Autonomous,
};

struct MessageStamp {
    std::uint64_t timestamp_ms = 0;
    std::uint32_t sequence = 0;
    bool valid = false;
};

inline bool isFresh(
    const MessageStamp &stamp,
    std::uint64_t now_ms,
    std::uint32_t timeout_ms)
{
    return stamp.valid &&
           now_ms >= stamp.timestamp_ms &&
           now_ms - stamp.timestamp_ms <= timeout_ms;
}

}
```

板间消息 freshness 必须用：

```text
本地接收时间
```

不得用远端 uptime 判断 freshness。

---

# 8. RemoteState

```cpp
enum class RcSwitch : std::uint8_t {
    Unknown = 0,
    Up,
    Middle,
    Down,
};

struct RemoteAnalog {
    std::int16_t right_x = 0;
    std::int16_t right_y = 0;
    std::int16_t left_x = 0;
    std::int16_t left_y = 0;
    std::int16_t wheel = 0;
};

struct RemoteMouse {
    std::int16_t x = 0;
    std::int16_t y = 0;
    std::int16_t z = 0;
    bool left = false;
    bool right = false;
};

struct RemoteKeyboard {
    std::uint16_t bits = 0;
};

struct RemoteState {
    RemoteAnalog analog{};
    RcSwitch left_switch = RcSwitch::Unknown;
    RcSwitch right_switch = RcSwitch::Unknown;
    RemoteMouse mouse{};
    RemoteKeyboard keyboard{};
    MessageStamp stamp{};
    bool online = false;
};
```

RemoteState 不出现：

```text
vx
yaw target
fire rate
```

---

# 9. DR16 Decoder

```cpp
class Dr16Decoder {
public:
    static constexpr std::size_t kFrameSize = 18;

    struct Config {
        std::int16_t channel_center = 1024;
        std::int16_t channel_min = 364;
        std::int16_t channel_max = 1684;
        std::int16_t center_deadband = 10;
    };

    explicit Dr16Decoder(const Config &config);

    int reset();

    int decodeFrame(
        const std::uint8_t *data,
        std::size_t size,
        std::uint64_t timestamp_ms,
        robotics::RemoteState &out);

    std::uint32_t validFrameCount() const;
    std::uint32_t invalidFrameCount() const;

private:
    Config config_{};
    std::uint32_t valid_frames_ = 0;
    std::uint32_t invalid_frames_ = 0;
    std::uint32_t sequence_ = 0;
};
```

必须 transactional：

```text
完整解码到临时对象
校验成功
最后一次性写 out
```

---

# 10. RemoteService

职责：

```text
UART bytes
→ 帧同步
→ Dr16Decoder
→ latest RemoteState
```

```cpp
class RemoteService {
public:
    struct Config {
        std::uint32_t offline_timeout_ms = 100;
    };

    RemoteService(
        const device *uart,
        const Dr16Decoder::Config &decoder_config,
        const Config &config);

    int init();

    int processBytes(
        const std::uint8_t *data,
        std::size_t size,
        std::uint64_t now_ms);

    int snapshot(
        std::uint64_t now_ms,
        robotics::RemoteState &out) const;

    bool online(std::uint64_t now_ms) const;

private:
    const device *uart_ = nullptr;
    Dr16Decoder decoder_;
    Config config_{};
    robotics::RemoteState latest_{};
};
```

UART callback 只搬数据，不做业务。

---

# 11. OperatorIntent

```cpp
enum class OperatorMode : std::uint8_t {
    Safe = 0,
    Manual,
    Auto,
};

struct OperatorIntent {
    OperatorMode mode = OperatorMode::Safe;
    ControlSource source = ControlSource::None;

    float chassis_vx_norm = 0.0f;
    float chassis_vy_norm = 0.0f;
    float chassis_wz_norm = 0.0f;

    float gimbal_yaw_rate_norm = 0.0f;
    float gimbal_pitch_rate_norm = 0.0f;

    bool friction_requested = false;
    bool fire_requested = false;

    MessageStamp stamp{};
};
```

归一化范围：

```text
[-1,+1]
```

---

# 12. ManualCommandMapper

```cpp
class ManualCommandMapper {
public:
    struct Config {
        float analog_deadband = 0.03f;
        float mouse_yaw_scale = 0.002f;
        float mouse_pitch_scale = 0.002f;
    };

    explicit ManualCommandMapper(const Config &config);

    int map(
        const RemoteState &remote,
        OperatorIntent &out) const;

private:
    Config config_{};
};
```

V1 推荐：

```text
左拨杆 Down   Safe
左拨杆 Middle Manual
左拨杆 Up     Auto
```

---

# 13. RobotCommand

## Chassis

```cpp
enum class ChassisMode : std::uint8_t {
    Disabled = 0,
    BodyVelocity,
    Spin,
};

struct ChassisCommand {
    ChassisMode mode = ChassisMode::Disabled;
    float vx_m_s = 0.0f;
    float vy_m_s = 0.0f;
    float wz_rad_s = 0.0f;
    ControlSource source = ControlSource::None;
    MessageStamp stamp{};
};
```

## Gimbal

```cpp
enum class GimbalMode : std::uint8_t {
    Disabled = 0,
    Hold,
    Rate,
    AbsoluteAngle,
};

struct GimbalCommand {
    GimbalMode mode = GimbalMode::Disabled;
    float yaw_target_rad = 0.0f;
    float yaw_rate_rad_s = 0.0f;
    float pitch_target_rad = 0.0f;
    float pitch_rate_rad_s = 0.0f;
    ControlSource source = ControlSource::None;
    MessageStamp stamp{};
};
```

## Shooter

```cpp
enum class ShooterMode : std::uint8_t {
    Disabled = 0,
    Ready,
    FireSingle,
    FireContinuous,
};

struct ShooterCommand {
    ShooterMode mode = ShooterMode::Disabled;
    float fire_rate_hz = 0.0f;
    float requested_bullet_speed_m_s = 0.0f;
    ControlSource source = ControlSource::None;
    MessageStamp stamp{};
};
```

## Robot

```cpp
struct RobotCommand {
    ChassisCommand chassis{};
    GimbalCommand gimbal{};
    ShooterCommand shooter{};
    MessageStamp stamp{};
};
```

---

<a id="safety"></a>

# 14. 安全语义：输出许可与程序存活分开

安全层只决定本周期允许什么输出，不负责杀线程、退出 main、重启主控或销毁硬件对象。恢复是一条常驻运行路径。

```cpp
enum class SafetyAction : std::uint8_t {
    Disable = 0, // 禁止驱动力输出，可恢复；不是析构或终止
    Hold,        // 有供电且反馈可信时闭环保持，仍会输出电流
    Active,      // 执行当前合法目标
};
enum class SafetyState : std::uint8_t {
    Boot = 0, Waiting, Ready, Active, Degraded, EmergencyStop, ConfigBlocked,
};
enum class ExecutionState : std::uint8_t {
    Waiting = 0, Recovering, Ready, Active, EStopLatched, ConfigBlocked,
};
```

每个子系统一个 ExecutionState，不叠加预测风险评分、历史错误积分熔断、入侵检测、认证失败锁死或无限增长的故障层级。CRC、长度和有限值检查用于避免把损坏数据作为电机目标；它们不证明被攻击，也不触发永久停机。

`Disable/Hold/Active` 不能用枚举大小作机械意义上的安全排序：Hold 是带电闭环，裁判关闭输出或反馈过期时也不能 Hold。先求交集：全局是否允许输出、本地供电与反馈是否可用；通过后再选择 Hold 或 Active。

**等待多久、恢复失败多少次，都不把正常离线升级成不可恢复错误。** 长期不能恢复时仍保持局部禁止输出并限频报告，通信、诊断和恢复任务继续运行。

---

# 15. 故障分级与原因记录

| 观测                                        | 当前动作                                                  | 恢复与作用范围                                    |
| ------------------------------------------- | --------------------------------------------------------- | ------------------------------------------------- |
| 单个 UART CRC/长度错误、未知消息            | 丢弃该帧，保留未过期的上一条有效目标                      | 下一合法帧自然恢复；计数只诊断                    |
| 一个电机反馈帧无效                          | 不覆盖最后有效反馈                                        | 在反馈有效期内继续；超过期限暂停所属子系统        |
| 一次发送队列忙或暂时不可写                  | 记录、下一调度周期发最新目标                              | 底层若已 Fault，进入本地恢复；不退出线程          |
| 命令实际过期、遥控实际离线                  | 底盘停止运动；云台可在有电且反馈有效时 Hold               | 新鲜有效命令稳定到达后自动恢复                    |
| 裁判输出禁用、已知电机电源关闭              | 对应供电域 Disable，Waiting                               | 等允许输出和反馈恢复，自动重新准备                |
| 电机反馈持续离线，原因不明                  | 受影响子系统 Waiting，不能假称已确认断电                  | 持续监听反馈并自动尝试恢复                        |
| CAN bus-off、UART RX 停止                   | 对受影响链路/子系统暂停输出并重启收发或恢复控制器         | 有界、限频重试；没有“重试 N 次就永久锁死”       |
| 单次 dt 越界或计算无效                      | 丢弃本次计算，清旧输出；恢复有效反馈后 reset 时间与控制器 | 只暂停所属子系统，不带大 dt 硬算，不退出          |
| 有效硬件过温/欠压报告                       | 暂停该域，不持续顶着故障使能                              | 条件恢复且满足器件要求后自动恢复；温度可用回差    |
| 显式急停                                    | EStopLatched，撤销相应/全车输出                           | 急停输入已解除且收到明确复位才解除；线程一直活着  |
| 配置单位错、设备不存在、ID 冲突、能力不支持 | ConfigBlocked，仅阻止相关子系统                           | 修正配置/装配后重新初始化；并非把离线判为配置错误 |

四舵轮第一版不做缺一轮继续行驶；关键轮反馈过期时整个底盘暂停。但不因此关闭独立云台、遥控或板间通信。真实物理损坏需维修，不能靠清错证明修好；也不能用“未知错误可能很严重”来新增永久锁存规则。

诊断结构至少区分当前原因 `active_reasons`、历史累计计数 `stats` 和明确急停锁存 `estop_latched`。恢复后清当前原因，保留统计；禁止 `if (历史active_reasons != 0) 永久禁止输出`。保存第一原因和最后一次恢复结果，防止“掉电”被随后出现的“零帧发送失败”覆盖。

---

# 16. GlobalSafetyManager：每个输出域单独授权

运行在决策输入所在板，默认云台板。输入应包含有效性，不要把“未知”默认当作供电允许。

```cpp
struct OutputPermission {
    bool valid = false;
    bool enabled = false;
    MessageStamp stamp{}; // 本地该字段最后一次真实更新
};
struct GlobalSafetyInputs {
    std::uint64_t now_ms = 0;
    bool command_source_fresh = false;
    bool operator_motion_enabled = false;
    bool emergency_stop_requested = false;
    OutputPermission gimbal_power{}, chassis_power{}, shooter_power{};
};
struct GlobalSafetyDecision {
    SafetyAction gimbal = SafetyAction::Disable;
    SafetyAction chassis = SafetyAction::Disable;
    SafetyAction shooter = SafetyAction::Disable;
    SafetyState state = SafetyState::Waiting;
    std::uint32_t active_reasons = 0;
    MessageStamp stamp{};
};
class GlobalSafetyManager {
public:
    struct Config { bool require_referee_for_motion = false; };
    explicit GlobalSafetyManager(const Config &config);
    int evaluate(const GlobalSafetyInputs &input, GlobalSafetyDecision &out);
    int clearEmergencyStop(bool estop_input_released);
};
```

`evaluate`：有效输入返回 0，即使结果是 Waiting/Disable；非法参数返回 -EINVAL 且不提交部分结果。纯逻辑、单线程使用，不访问 UART、电机或系统时钟。急停复位在输入仍有效时返回 -EBUSY，否则清锁存并回 Waiting，不直接 arm。

比赛运行启用 require_referee_for_motion；输出许可字段过期则暂时禁止相关输出，字段恢复后自动恢复。台架模式显式允许未接裁判，并从日志显示该配置。若已接裁判且收到有效禁止输出，无论哪种模式都执行禁止。

全局许可不能依赖远端已经 armed：否则形成“云台等底盘 armed 才允许，底盘等允许才 arm”的死循环。命令通路应持续发送当前意图和许可；远端未就绪只影响其实际执行与反馈展示。

---

# 17. 去抖、超时和恢复默认策略

- 一帧损坏不是离线。只以最后一条合法数据的本地接收年龄判断实际断流；坏帧不能刷新时间。
- 命令 100 Hz / 100 ms 超时、心跳 50 Hz / 200 ms 仅为调试起点，不能照搬到电机反馈。电机反馈使用实际驱动配置并依据闭环需求测量。
- 反馈判 stale 后不额外等待一长段“去抖”，避免重复宽限；去抖主要用于重新允许输出。
- 恢复示例：命令连续 3 条有效且间隔未超时；每台必要电机收到不同时间戳的新反馈并稳定 20～50 ms。不是把同一缓存读三次就算三条新数据。
- 示例恢复尝试间隔 100 ms，失败后最多退让到 1 s；不超过此上限，不因重试总次数进入锁死。后台每轮操作也必须有时间上界。
- 数据里的 NaN/Inf、无效模式直接拒绝当次数据；合法目标超过操作限幅则按约定 clamp。不能把正常饱和当成硬件严重故障。
- 短暂抖动后使用当前有效命令，速度/力矩按既定斜坡恢复。掉线并不是急停，不要求操作员每次重新拨动开关。
- 显式急停始终立即撤销输出，不用去抖延迟响应；普通 Safe 拨杆位置只是可恢复的运行禁用，不等同锁存急停。

阈值必须写成配置并显示当前阻止原因、剩余等待条件；不能散落在各层隐藏地重复增加一份恢复门槛。最终数值以控制周期、反馈周期、停车响应和实机结果确定。

---

# 18. GimbalLocalSafety：保持与撤销输出

输入：当前全局许可、命令有效性、本地真实反馈新鲜度、驱动状态、供电许可和急停。输出：SafetyAction、ExecutionState、active_reasons；在云台控制线程评估，无 I/O、无阻塞。

有电、反馈可信而目标源暂时丢失：首次进入 Hold 时把目标设为当前角，后续保持这个固定目标。不能每周期重写为测量角而导致始终没有保持误差。

裁判禁用、电机离线或反馈过期：Disable → Waiting，禁止 Hold。云台物理支撑/限位仍按机构设计，代码不能在断电后承诺维持姿态。

恢复后先以重新读到的实际姿态初始化目标与 PID，再按当前 Rate/AbsoluteAngle 命令限速变化；不追赶失联期间累计的旧目标。配置错误只阻止云台，其他任务继续。

---

# 19. ChassisLocalSafety：底盘持续运行状态机

```text
Waiting
  └─ 非配置阻塞、非急停、输出许可恢复 → Recovering
Recovering
  ├─ 恢复暂未完成 / 反馈未齐 → 保持，限频继续
  ├─ 电源再次禁止 / 链路条件失效 → Waiting
  └─ 总线可用 + 新反馈稳定 + 控制器重置 → Ready
Ready
  └─ 当前许可 + 恢复边界之后的新命令 + 本地 arm 成功 → Active
Active
  └─ 命令过期 / 电源禁止 / 反馈过期 / 运行失败 → Waiting
任意状态
  └─ 显式急停 → EStopLatched → 明确复位后 Waiting
```

ConfigBlocked 只由明确配置错误进入，不由“等反馈太久”进入。四轮就绪指本地反馈与能力就绪，不依赖全局允许之后才会出现的 armed 标志。

`evaluate(inputs, decision)` 在有效输入下返回 0，等待也是正常结果。硬件恢复由本板唯一硬件所有者调度，不能在 Safety.evaluate 内调用 CAN。输出状态、原因和新鲜反馈一起发布到云台板。

命令中安全许可、模式、速度与本地接收时间放在同一快照中。接收命令时允许更新等待状态下的命令缓存，不因本板尚未 armed 丢掉所有命令；执行端只消费恢复边界后的命令，避免互相等待。

---

# 20. 数据新鲜度不能相互代替

心跳新鲜不等于命令新鲜，命令新鲜也不等于电机反馈新鲜。仅心跳存活而 command_task 停止时，底盘应因命令过期进入可恢复等待。

每种消息单独维护序号及最后一次有效接收时间。重复发送相同 command_sequence 不刷新命令有效期；只有命令生产者产生新目标快照才推进命令序号。发送线程不能为旧目标不断重新编号。

裁判输出许可、功率限制、热量等也各自记录最后真实更新，不能用任意一帧裁判消息刷新整个 RefereeState。跨板转发时保留 valid，并携带字段数据年龄；接收侧把转发前年龄与本地经过时间相加，不能把持续转发的旧许可当作持续更新。

---

# 21. CommandManager

```cpp
class CommandManager {
public:
    struct Config {
        float max_chassis_vx_m_s = 3.0f;
        float max_chassis_vy_m_s = 3.0f;
        float max_chassis_wz_rad_s = 6.0f;

        float max_gimbal_yaw_rate_rad_s = 3.0f;
        float max_gimbal_pitch_rate_rad_s = 2.0f;

        std::uint32_t input_timeout_ms = 100;
    };

    explicit CommandManager(const Config &config);

    int reset(std::uint64_t now_ms);

    int step(
        const OperatorIntent &operator_intent,
        const GlobalSafetyDecision &safety,
        std::uint64_t now_ms,
        RobotCommand &out);

private:
    Config config_{};
    std::uint32_t sequence_ = 0;
};
```

CommandManager 不允许：

```text
UART write
CAN
Motor API
```

---

# 22. 命令出口：第一版静态绑定

云台命令写本地最新值槽；底盘命令写串口发送最新值槽。第一版可用普通成员函数，不要求 本地云台出口 等虚接口、动态注册表或通用消息代理。

```cpp
struct RouteReport {
    int local_gimbal_result = 0;
    int remote_chassis_result = 0;
};
```

提交成功表示进入本地缓存，不代表远端已经执行。队列满返回 -EAGAIN，底盘远端提交失败不阻止云台本地提交。发射机构第一版只保留状态接口，离散事件以后单独建队列。

---

# 23. CommandRouter

建议放在 `applications/sentry_gimbal/src/command_router.*`，因为实际路由需要接触应用通道；CommandManager 与纯 robotics 库不依赖 zbus。

```cpp
class CommandRouter {
public:
    RouteReport route(const RobotCommand &command,
                      const GlobalSafetyDecision &safety);
};
```

由 command_task 唯一调用，非阻塞，只复制值，不同步等待 UART。依次尝试两个出口并分别报告结果，不因第一个失败直接 return。安全许可和子命令一起提交，接收侧不再次走发送路由，避免回环。

---

# 24. 板间协议最小范围

V1 只传：同步/心跳、底盘目标、底盘约束、底盘摘要；固定两端角色，不做任意节点发现、加密认证、远程升级或万能 RPC。包错误只丢弃或重新同步。

显式逐字段编码，禁止把 C++ struct 的原始内存发送。整数规定小端，浮点规定 IEEE 754 binary32，编译期检查平台表示；读取后拒绝非有限值、非法枚举与越界长度。

以下布局是新设计，尚未确认旧串口协议。若找到旧工程，应先确定是否需要兼容，再决定沿用其帧层；不能一边套用旧协议一边改 wire 布局。

---

# 25. Frame V1：固定字节布局

| 字节偏移   | 内容                                   |
| ---------- | -------------------------------------- |
| 0..1       | 0xA5、0x5A                             |
| 2          | version = 1                            |
| 3          | sender_role：1 云台、2 底盘            |
| 4..5       | message_id，u16 LE                     |
| 6..7       | payload_length，u16 LE，最大 128       |
| 8..11      | frame_sequence，u32 LE；各消息类型独立 |
| 12..12+N-1 | N 字节 payload                         |
| 12+N..13+N | CRC16，u16 LE                          |

总长 14+N，最大 142 字节；接收缓存可留 160 字节空间。CRC 选定为多项式 0x1021、初值 0xFFFF、不反射、xorout=0，覆盖帧头至负载最后一个字节，CRC 自身不参与。双方编码器共享这份定义，不能复用名字相同但参数不同的 CRC 函数。

编码在输出容量不足时返回 -ENOSPC，非法字段 -EINVAL；成功返回实际字节数。解码接口遵循各自声明，不把“成功字节数”误按通用 0 成功接口处理。

失步逐字节查找帧头；长度非法或 CRC 失败从候选起点后继续扫描，不清空整个串口服务。组帧等待也应有上界，不能让一个假长度一直阻塞后续合法帧。

---

# 26. Message IDs

```cpp
enum class MessageId : std::uint16_t {
    Heartbeat         = 0x0001,

    ChassisControl    = 0x0101,
    ChassisConstraint = 0x0102,
    ChassisFeedback   = 0x0103,
    ChassisFault      = 0x0104,

    GimbalFeedback    = 0x0201,

    SystemEvent       = 0x0301,
};
```

---

# 27. Heartbeat 与自动同步

```cpp
enum class BoardRole : std::uint8_t { Unknown=0, GimbalController=1, ChassisController=2 };
struct BoardHeartbeat {
    BoardRole role = BoardRole::Unknown;
    SafetyState safety_state = SafetyState::Waiting;
    std::uint32_t active_reasons = 0;
    std::uint32_t sender_uptime_ms = 0; // 仅诊断
    std::uint64_t sender_boot_id = 0;
    std::uint32_t resume_generation = 0;
    bool ready = false;
    bool sync_requested = false;
    MessageStamp stamp{};
};
```

wire：u8 role、u8 state、u8 ready、u8 sync_requested、u32 active_reasons、u32 uptime_ms、u64 sender_boot_id、u32 resume_generation，共 24 字节。底盘心跳中的 sender_boot_id 即发送命令时要回显的 receiver_boot_id；云台也发布自己的启动 ID，以区分重启和重复同步请求。底盘按云台 boot_id 对同步请求幂等处理，重复请求不会重复暂停；确认新的启动上下文时才重置该方向的序号基准。

推荐 50 Hz；200 ms 心跳超时只影响在线诊断，实际运动仍独立检查命令。心跳与命令缓存不能共用一个 last_rx_ms。

---

# 28. RemoteChassisControl

```cpp
struct RemoteChassisControl {
    ChassisCommand command{};
    SafetyAction global_action = SafetyAction::Disable;
    std::uint32_t active_reasons = 0;
    std::uint64_t receiver_boot_id = 0;
    std::uint32_t resume_generation = 0;
    MessageStamp stamp{}; // 接收板时间，不是对端 uptime
};
```

wire 共 36 字节：u8 mode、u8 source、u8 action、u8 reserved=0，f32 vx_m_s/vy_m_s/wz_rad_s，u32 active_reasons、u32 command_sequence、u64 receiver_boot_id、u32 resume_generation。

frame_sequence 是通信帧计数，command_sequence 是命令生产者的更新计数。只有后者真正推进才更新运动命令时间。云台看到底盘新的恢复上下文后，从当前输入重新产生目标并回显 boot_id/generation；不用操作员手动确认。

模式、许可、数据、序号和接收时间是同一快照。对端尚未 Ready 时仍可缓存/显示数据，但本地不执行；不能拒收所有“尚未 armed”的命令而阻断恢复。

---

# 29. ChassisConstraint

```cpp
struct ChassisConstraint {
    OutputPermission output{};
    bool power_valid = false;
    float power_limit_w = 0.0f;
    float buffer_energy_j = 0.0f;
    std::uint32_t output_age_ms = 0;
    std::uint32_t power_age_ms = 0;
    MessageStamp stamp{};
};
```

wire：u8 output_valid、u8 output_enabled、u8 power_valid、u8 reserved=0，f32 power_limit_w、f32 buffer_energy_j，u32 output_age_ms、u32 power_age_ms，共 20 字节。

Command 是目标，Constraint 是许可与额度，Safety 决定本地能否执行。字段年龄以源端最近真实更新计算，源端持续转发不能变成新测量。接收端年龄 = 发送时年龄 + 本地接收后经过时间，另为链路在途延迟留实测余量；需严格时限时再引入时间同步。

比赛配置下输出许可过期则 Waiting；功率约束过期使用事先验证过的保守额度，尚未验证时暂停底盘输出，数据恢复后自动恢复。不能把默认 0 W 当成合法有效读数一直锁着电机，也不能把未知许可默认 true。

---

# 30. ChassisFeedbackSummary

反馈包含本地执行状态、当前原因、armed、ready、最后接受的 command_sequence、各数据有效性，以及实测/估计的 vx、vy、wz、功率。不要用一个 healthy=false 同时表示电源正常关闭、数据失效、配置错和线程死亡。

建议 wire 32 字节：u8 execution_state、u8 safety_state、u8 ready、u8 armed，u32 active_reasons、u32 last_command_sequence、u32 valid_fields，f32 vx_m_s、f32 vy_m_s、f32 wz_rad_s、f32 power_w。

valid_fields 的 bit0 表示速度估计有效，bit1 表示功率测量有效；没有里程计/功率测量时发 0 且对应位清零，不伪造零速度观测。云台以收到合法反馈的本地时间判断在线；状态 Waiting 的板仍然是 online。

---

# 31. InterBoardCodec

```cpp
class InterBoardCodec {
public:
    int encodeHeartbeat(...);
    int encodeChassisControl(...);
    int encodeChassisConstraint(...);
    int encodeChassisFeedback(...);

    int decodeHeartbeat(...);
    int decodeChassisControl(...);
    int decodeChassisConstraint(...);
    int decodeChassisFeedback(...);
};
```

必须显式：

```text
storeLe16
storeLe32
loadLe16
loadLe32
storeFloatLe
loadFloatLe
```

禁止未对齐 reinterpret_cast。

---

# 32. InterBoardParser

```cpp
class InterBoardParser {
public:
    struct Stats {
        std::uint32_t valid_frames = 0;
        std::uint32_t crc_errors = 0;
        std::uint32_t length_errors = 0;
        std::uint32_t version_errors = 0;
        std::uint32_t unknown_messages = 0;
    };

    int reset();

    int consume(
        const std::uint8_t *data,
        std::size_t size,
        std::uint64_t local_receive_ms);

    bool popFrame(
        FrameMeta &meta,
        std::uint8_t *payload,
        std::size_t capacity,
        std::size_t &payload_size);

    const Stats &stats() const;
};
```

Parser 只解帧，不执行业务。

---

# 33. InterBoardLink

```cpp
class InterBoardLink {
public:
    struct Config {
        std::uint32_t tx_timeout_ms = 20;
    };

    InterBoardLink(
        const device *uart,
        BoardRole local_role,
        const Config &config);

    int init();

    int processRxBytes(
        const std::uint8_t *data,
        std::size_t size,
        std::uint64_t now_ms);

    int sendHeartbeat(...);
    int sendChassisControl(...);
    int sendChassisConstraint(...);
    int sendChassisFeedback(...);

    bool peerOnline(
        std::uint64_t now_ms,
        std::uint32_t timeout_ms) const;

    int latestHeartbeat(...);
    int latestChassisControl(...);
    int latestChassisConstraint(...);
    int latestChassisFeedback(...);
};
```

---

# 34. 连续目标与离散事件

连续速度、心跳、状态和约束使用最新值覆盖，不为每条目标做 ACK 和重传。发送忙时下一轮发送最新值，不能积压一串过期目标。

重启/恢复上下文同步是一次状态交接，不是每帧控制 ACK。同步失败保持 Waiting 并继续尝试，不进入不可解除锁存。

单发一次、保存零点、清故障属于事件；未来使用事件 ID、确认与去重，并明确重启后不自动重放。RobotCommand 内不能用周期重复的 FireSingle 模式实现“一次”；第一版不执行该模式，等事件通路建立后移入事件接口。恢复后不补发断电期间的射击。

---

# 35. UART 与 CAN 带宽预算

板间可从 460800 baud、8N1、异步收发开始实测，但板型、引脚、电平和真实旧协议尚未确认。不要因为候选速率看起来够就承诺稳定。

本协议底盘目标完整帧为 50 B，100 Hz 时单向占 50000 bit/s；38 B 心跳 50 Hz 为 19000 bit/s；34 B 约束 50 Hz 为 17000 bit/s。云台发送方向合计约 86000 bit/s，在 460800 下约 18.7%，还需给其他消息、突发与调度留余量。底盘反馈 46 B×50 Hz 加心跳约 42000 bit/s。上述按每字节 10 位计，半双工要合计双向及转向开销。

CAN 按每条物理总线分别估算帧长、帧率和重发余量，并测发送等待、错误计数和线程耗时。把云台搬到另一板不会降低仍挂在底盘同一 CAN 上的八个电机负载。若拆两条 CAN，只改硬件设备映射、Bus 管理和同步提交，不改舵轮数学模型。

UART RX 回调只转交 `buf + offset` 到 `len` 所描述的字节片段及缓冲所有权，不假设一次回调就是一帧。RX 停止后由通信任务重新启用；TX 完成/中止前不重用缓冲。可参考 lib/vofa/vofa.c 的设备使用，但不能照搬其文本分行逻辑和偏移处理。

---

# 36. zbus 边界

zbus 只允许存在：

```text
applications/*
```

不进入：

```text
lib/robotics/*
lib/communication parser/*
```

云台板 channels：

```text
remote_state
referee_state
operator_intent
global_safety
robot_command
gimbal_command
chassis_feedback
```

底盘板：

```text
remote_chassis_control
chassis_constraint
chassis_feedback
```

---

# 37. 云台板线程

```text
remote_task
referee_task
interboard_rx_task
command_task       100 Hz
gimbal_task        200 Hz
interboard_tx_task 50~100 Hz
```

command_task：

```text
Remote snapshot
Referee snapshot
ChassisFeedback snapshot
ManualCommandMapper
GlobalSafetyManager
CommandManager
CommandRouter
```

gimbal_task：

```text
GimbalCommand
GlobalSafety
Motor health
GimbalLocalSafety
YawGimbal.update
```

---

# 38. 底盘板线程

```text
interboard_rx_task
chassis_task       200 Hz
interboard_tx_task 50 Hz
```

chassis_task：

```text
RemoteChassisControl
Constraint
heartbeat freshness
command freshness
read motor feedback
local safety
swerve step
setCurrent ×8
shared bus.flush
feedback summary
```

---

# 39. YawGimbal

```cpp
enum class YawTopology : std::uint8_t {
    Continuous = 0,
    Limited,
};

struct YawGimbalConfig {
    YawTopology topology = YawTopology::Continuous;

    float min_angle_rad = -3.1415926f;
    float max_angle_rad =  3.1415926f;

    float max_rate_rad_s = 3.0f;

    bool hold_on_zero_rate = true;
};

class YawGimbal {
public:
    YawGimbal(
        control::PositionMotor &motor,
        const YawGimbalConfig &config);

    int validate() const;
    int begin();

    int update(
        const GimbalCommand &command,
        SafetyAction action,
        float dt_s);

    int stop();

private:
    control::PositionMotor &motor_;
    YawGimbalConfig config_{};

    double target_angle_rad_ = 0.0;
    bool initialized_ = false;
};
```

---

# 40. Yaw SafetyAction 与可恢复电机接口

```text
Active: 使用当前有效 GimbalCommand，正常 update
Hold:   供电与反馈允许时保持进入 Hold 那一刻的角度
Disable: suspend 输出，控制线程继续 poll，条件好转后 resume
```

这里的 suspend/resume 是待实现接口，当前 PositionMotor 只有一次性 begin/stop。不能把旧接口改个名字就宣称可恢复；具体所需改动见第 74 节。stop 保留给应用主动结束，不作为正常电源禁用的默认路径。

YawGimbal 的 begin 只完成一次对象配置与注册；电机未供电返回等待状态，通过周期服务继续准备，不能阻止通信线程启动。YawGimbal.update 即使无法控制，也应将状态交给常驻服务而非结束线程。

---

# 41. Yaw Rate

```text
target += rate * dt
```

rate clamp：

```text
±max_rate
```

Limited 模式再 clamp angle。

Continuous 拓扑的 AbsoluteAngle 模式复用 `PositionReference::AbsoluteNearest`；Limited 拓扑按第 73.2 节选择限位内连续目标，YawGimbal 不重新实现角度数学函数。

---

# 42. Swerve Types

```cpp
struct ModuleLocation {
    float x_m = 0.0f;
    float y_m = 0.0f;
};

struct ModuleTarget {
    float angle_rad = 0.0f;
    float wheel_velocity_m_s = 0.0f;
};

using ModuleTargets =
    std::array<ModuleTarget, 4>;
```

坐标：

```text
+x 前
+y 左
+z 上
+wz 逆时针
```

---

# 43. SwerveKinematics

```cpp
class SwerveKinematics {
public:
    struct Config {
        std::array<ModuleLocation, 4> locations{};
        float max_wheel_velocity_m_s = 0.0f;
        float stationary_epsilon_m_s = 0.01f;
    };

    explicit SwerveKinematics(const Config &config);

    int validate() const;

    int reset(const ModuleTargets &current);

    int solve(
        const ChassisCommand &command,
        ModuleTargets &out);

private:
    Config config_{};
    std::array<float, 4> last_angle_rad_{};
    bool initialized_ = false;
};
```

公式：

```text
wheel_vx = vx - wz*y
wheel_vy = vy + wz*x

speed = hypot(wheel_vx,wheel_vy)
angle = atan2(wheel_vy,wheel_vx)
```

零速：

```text
保持 last angle
```

超速：

```text
所有轮统一 scale
```

---

# 44. SwerveModule

后续接口：

```cpp
struct ModuleFeedback {
    float steer_absolute_rad = 0.0f;
    float steer_continuous_rad = 0.0f;
    float steer_velocity_rad_s = 0.0f;
    float drive_velocity_rad_s = 0.0f;
};

struct ModuleOutput {
    float optimized_angle_rad = 0.0f;
    float optimized_wheel_velocity_m_s = 0.0f;

    float steer_continuous_target_rad = 0.0f;
    float drive_target_rad_s = 0.0f;

    float steer_effort = 0.0f;
    float drive_effort = 0.0f;
};

class SwerveModule {
public:
    struct Config {
        float wheel_radius_m = 0.0f;
        control_motor_position_config steer{};
        control_motor_velocity_config drive{};
    };

    explicit SwerveModule(const Config &config);
    int validate() const;
    int reset(const ModuleFeedback &feedback);

    int step(
        const ModuleTarget &target,
        const ModuleFeedback &feedback,
        float dt_s,
        ModuleOutput &out);
};
```

Module 禁止知道：

```text
CAN
device*
GM6020
M3508
UART
```

---

# 45. 舵角优化

```text
error = shortest_angle(target,current)

if abs(error) > pi/2:
    target += pi
    wheel_velocity *= -1
```

`== pi/2` V1 不翻转。

---

# 46. SwerveChassis

```cpp
struct ChassisFeedback {
    std::array<ModuleFeedback,4> module{};
};

struct ChassisOutput {
    ModuleTargets target{};
    std::array<ModuleOutput,4> module{};
};

class SwerveChassis {
public:
    struct Config {
        SwerveKinematics::Config kinematics{};
        std::array<SwerveModule::Config,4> modules{};
    };

    explicit SwerveChassis(const Config &config);
    int validate() const;
    int reset(const ChassisFeedback &feedback);

    int step(
        const ChassisCommand &command,
        const ChassisFeedback &feedback,
        float dt_s,
        ChassisOutput &out);
};
```

---

# 47. 底盘硬件层与共享 Bus

`applications/sentry_chassis/src/chassis_hardware.*` 拥有实际电机、每条 CAN 唯一 Bus 及恢复状态。不能用 8 个单电机独占 DjiMotorBackend 占同一条总线。若混用电机类型，硬件适配层明确电流/力矩单位。

```cpp
class DjiChassisHardware {
public:
    int init(); // 仅配置/注册，不长时间等待电机供电
    int read(ChassisFeedback &out); // 失败不提交半套反馈
    int suspend(); // 撤销全部目标；幂等，保留接收与绑定
    int pollRecovery(std::uint64_t now_ms); // 0 已安全就绪，-EAGAIN/-EINPROGRESS 继续等待
    int arm(); // 只从 Ready/Safe 进入，先确认零输出
    int apply(const ChassisOutput &output);
};
```

这些接口由一个硬件线程调用；恢复操作不得与 apply 并行。控制周期整批读取反馈 → 算四轮 → 验证整批输出 → 全部 setCurrent → 每条 Bus 一次 flush。不能某个轮失败后仍发送混合新旧的一组电流。

Bus 当前可能同步发送并等待，pollRecovery 的非阻塞契约需要检查/拆分底层等待；只把阻塞函数放进 poll 并不满足周期要求。可由唯一硬件服务线程执行有界恢复，持续报告进度，控制逻辑以 Waiting 跳过输出。

软件 Bus recover 和 Zephyr CAN 控制器 bus-off recover 是两层不同操作。驱动支持自动恢复时观察其完成；手动模式才按实际 Kconfig/驱动接口调用 can_recover，禁止在控制线程 K_FOREVER 等待。

---

# 48. 裁判系统

暂按裁判 UART 接云台板部署；接线不同只移动输入服务。

公共状态：

```cpp
struct RefereePowerState {
    float chassis_power_w = 0.0f;
    float chassis_power_limit_w = 0.0f;
    float buffer_energy_j = 0.0f;
    MessageStamp stamp{}; // 该功率字段组真实更新时间
};

struct RefereeRobotState {
    std::uint8_t robot_id = 0;
    OutputPermission chassis_output{};
    OutputPermission gimbal_output{};
    OutputPermission shooter_output{};
};

struct RefereeState {
    RefereeRobotState robot{};
    RefereePowerState power{};
    MessageStamp stamp{};
    bool online = false;
};
```

协议 command id / payload layout 只能存在于：

```text
communication/referee/
```

---

# 49. RefereeParser

```cpp
class RefereeParser {
public:
    struct Stats {
        std::uint32_t valid_frames = 0;
        std::uint32_t crc8_errors = 0;
        std::uint32_t crc16_errors = 0;
        std::uint32_t length_errors = 0;
        std::uint32_t unknown_commands = 0;
    };

    int reset();

    int consume(
        const std::uint8_t *data,
        std::size_t size,
        std::uint64_t timestamp_ms);

    const RefereeState &state() const;
    const Stats &stats() const;
};
```

Parser 不允许直接改 motor/command。

---

# 50. 裁判与底盘关系

```text
RefereeState
   ├──► GlobalSafety
   └──► ChassisConstraint
            ↓ UART
         ChassisPowerLimiter
```

不要：

```text
RefereeParser → setCurrent
```

---

# 51. ChassisPowerLimiter

后续：

```cpp
struct ChassisPowerInput {
    float measured_power_w = 0.0f;
    float power_limit_w = 0.0f;
    float buffer_energy_j = 0.0f;
};

struct ChassisPowerDecision {
    float effort_scale = 1.0f;
};

class ChassisPowerLimiter {
public:
    int reset();

    int step(
        const ChassisPowerInput &input,
        float dt_s,
        ChassisPowerDecision &out);
};
```

V1 可用保守 scale 作为台架限幅起点；电流统一缩放并不证明严格满足整机功率限制，比赛前须校准功率模型与实测动态。超级电容策略后续独立升级。

---

# 52. 云台板初始化与常驻运行

先完成静态配置校验、创建应用生命周期对象、注册消息与硬件接收通道，再启动遥控、裁判、板间通信、控制及恢复服务。默认 Disable/Waiting，不能在没有反馈时先 Hold。

电机准备由控制/硬件服务逐步进行；电机未上电不阻止其余线程运行，也没有“启动等两秒超时就退出”的比赛逻辑。UART 暂时不可用时相应服务持续尝试恢复。真正的设备配置缺失只标记相关功能 ConfigBlocked。

所有硬件对象长寿命且单一所有者；恢复不析构，不重复注册 CAN filter，不重复 claim，不用重新 new 一组 wrapper 绕过 begin_attempted_。

---

# 53. 底盘上电与恢复顺序

```text
1. 配置校验、创建并唯一绑定 Bus/电机，启动 UART 与状态上报
2. 无供电或禁止输出时保持 Waiting，持续接收反馈
3. CAN 控制器若 bus-off，先按驱动模式恢复控制器
4. 软件 Bus 若 Fault，在允许的零输出/禁用操作条件下执行 recover
5. 读取原始反馈有效位/时间戳，等本轮所需电机反馈稳定
6. reset 舵向连续参考、全部 PID、滤波器和控制时间基准
7. Ready；等待恢复边界之后生成的新命令与当前输出许可
8. arm 并确认结果，从零驱动目标开始按斜坡恢复
```

第 4 步不能依赖 getState()==Ready，因为软件故障锁存可能正是 Ready 无法出现的原因。恢复只做零输出/禁用和必要清错，不授权非零运动。

若收到 DM 明确硬件错误码，先按错误种类确认可清除条件；不得每个控制周期无条件 ClearError。正在过温时先等待降温，正在欠压时先等待电源恢复。第 4 步暂时失败保持后台等待，不退出。

---

# 54. 第一条实机链：小 Yaw

先打通：

```text
DR16
→ RemoteState
→ ManualCommandMapper
→ GlobalSafetyManager
→ CommandManager
→ CommandRouter
→ GimbalCommand
→ GimbalLocalSafety
→ YawGimbal
→ PositionMotor
→ Motor
```

这是最优先施工对象。

---

# 55. 小 Yaw 测试顺序

```text
1. 现有 position sample
2. YawGimbal Hold
3. YawGimbal 固定 Rate
4. DR16 只打印
5. DR16 → Command 只打印
6. DR16 → Yaw
7. 拔 DR16 测 Hold
8. EStop 测 Disable
```

首次：

```text
max yaw rate <= 0.5 rad/s
保持低电流限制
```

---

# 56. 板间测试顺序

在底盘没装好前，两板先做：

```text
A. Heartbeat
B. ChassisControl encode/decode
C. heartbeat 正常但 command 停止
D. command timeout 必须触发
E. Feedback 回传
F. 拔 UART 线
G. peer offline
```

这是底盘实机前必须完成的安全测试。

---

# 57. SwerveKinematics 无硬件测试

固定场景：

```text
forward
left
rotate
forward+rotate
zero
overspeed
```

验收：

```text
forward: 4轮 0°
left: 4轮 +90°
rotate: 4轮切向
zero: 保持上一次角
overspeed: 统一缩放
```

---

# 58. Kconfig

建议：

```text
CONFIG_SKYWALKER_LIB_COMMUNICATION
CONFIG_SKYWALKER_LIB_ROBOTICS

CONFIG_SKYWALKER_REMOTE_DR16
CONFIG_SKYWALKER_REFEREE
CONFIG_SKYWALKER_INTERBOARD

CONFIG_SKYWALKER_ROBOTICS_COMMAND
CONFIG_SKYWALKER_ROBOTICS_SAFETY
CONFIG_SKYWALKER_ROBOTICS_GIMBAL
CONFIG_SKYWALKER_ROBOTICS_SWERVE
```

不要一类一个超细 Kconfig。

---

# 59. Devicetree

基础 board DTS 只描述板。

真实机器人连接放 application overlay。

云台：

```text
remote-uart
referee-uart
interboard-uart
yaw-motor
```

底盘：

```text
interboard-uart
steer-fl/fr/rl/rr
drive-fl/fr/rl/rr
```

具体 UART、引脚、电机 ID 根据真实接线配置。

---

# 60. 推荐周期

初始值：

```text
Yaw control             200 Hz
CommandManager          100 Hz
ChassisControl UART     100 Hz
Heartbeat                50 Hz
ChassisFeedback          50 Hz
Chassis control         200 Hz

Remote timeout          100 ms
Command timeout         100 ms
Heartbeat timeout       200 ms

InterBoard UART      460800 baud
```

后续实测调整。

---

<a id="power-recovery"></a>

# 61. 比赛电机断电与恢复：必须支持的完整路径

## 61.1 断电阶段

裁判禁用位有效时，记录 `RefereeOutputDisabled`，对应供电域立即撤销输出；这不是 motor damaged，也不是急停锁存。许可信息晚到、暂时缺失或裁判链路失效时，实际反馈过期也足以让该域进入 Waiting，不需要先确认断电原因。

清应用、wrapper、Bus 的非零待发目标与 PID 积分；冻结运动目标累积。CAN 发送已入硬件队列的旧非零帧也要处理：核对当前 CAN 驱动取消/停止发送的能力，限定在途数量和时限，恢复前确认旧队列已排空/取消且零输出建立。只清应用变量不能证明旧帧不会在供电恢复瞬间被发出。

零输出/禁用帧可以尽力发一次。电机电源切断时，零帧发送可能失败；记录 output_delivery_unknown，不因此进入新的永久锁存，也不把“发送失败”当成已成功物理刹停。控制失联时依赖电调自身的命令超时行为，具体停机时延必须实测。

接收中断、命令处理、裁判状态、心跳和监控继续；总线恢复限频。系统看门狗反映任务是否按期推进，不以“电机有没有反馈”作为喂狗条件。确实线程卡死可由看门狗处理，正常 Waiting 必须视为任务健康。

## 61.2 电源恢复阶段

有效输出许可恢复并不保证电机已经启动。继续等待新反馈/总线恢复；确认后按第 53 节顺序重新准备。恢复准备完成时产生一次 `resume_generation` 并回传云台，云台用当前输入生成一条携带该 generation 的新命令；底盘接受后自动 arm。这个自动往返用于避免执行断电前排队命令，不要求用户重新拨开关、不要求手工清普通故障。

同一次恢复期间 generation 保持不变，仅在后续真实断流/禁用之后再次进入 Ready 时更新；重复坏帧不能不断使 generation 自增而让恢复永远追不上。持续许可和新命令不依赖对端 armed；本地 Ready 也不依赖远端先发非零命令，避免双向等待。

驱动速度从零参考开始斜坡恢复；舵角从恢复后的实际角开始；发射事件不补发。若要求遥控回中才恢复，应作为明确可选的人机策略，默认比赛自动控制流程不增加这一门槛。急停仍须明确复位。

## 61.3 位置恢复不能仅清 Fault

电机或反馈中断期间机构可能移动，连续编码器的跨圈累积可能不可信。DJI 当前连续角用相邻帧差值展开；恢复时应重新建立基准，不把失联前后的两个采样强行当成相邻运动。DM backend 的 unwrap 状态也要重置。

1:1 固定零点单圈能力可用于重新确定舵向，并求当前位置附近的连续目标；原始编码器单位和减速比仍按驱动语义。减速转子的单圈角不能凭空恢复输出轴多圈位置；有限位云台若没有可恢复位置传感器，只让该轴等待实际校准，不把整个机器人锁死。

## 61.4 自动恢复不等于反复冲击

对暂时通信失败自动恢复；对当前仍存在的硬件故障等待条件解除，不反复输出大电流试探。恢复尝试限制频率、单步耗时和恢复斜坡，但不设置次数耗尽后永久停机。日志只在状态切换和低频汇总时输出，禁止错误刷屏反过来饿死控制任务。

---

# 62. 板间重启、失联与重新同步

发送方 uptime 仅用于诊断，不以一次回退直接判定重启：它可能绕回或来自迟到帧。固定两板链路使用自动重新同步协议，所有握手均由固件完成，超时限频重试且永不进入“认证失败锁死”。

接收板每次冷启动提供新的 boot_id（如可用的硬件随机非零 64 位值；不是密码）。真实断流时清旧命令；每次重新完成准备进入 Ready 时推进 resume_generation。发送方命令必须回显接收板的 boot_id/resume_generation。接收侧只接受匹配当前状态的新命令；不匹配只丢帧并发送当前同步状态。

云台冷启动使用自己的新 boot_id 请求同步，底盘仅在首次识别这个启动上下文时进入 Waiting、清旧速度并在准备完成后回当前 generation；收到匹配响应后云台从实时输入重新产生命令。相同同步请求应幂等，不能每重发一次就再次清已经恢复的运动。序号按每类消息分别判断，采用无符号差值 `0 < new-old < 2^31`；重复序号不刷新年龄。

如果板上没有可靠 boot_id 来源，必须另行选定并验证启动握手/缓存清空策略；不能用固定 boot_id 宣称已保证跨重启隔离。这是旧缓存处理，不是防破解机制。CRC 与字段校验仍只服务于传输正确性。

---

# 63. 禁止事项

```text
Remote 直接 motor.update
Referee 直接 setCurrent
CommandManager 直接 UART
Swerve 直接 UART
8×DjiMotorBackend 同总线
板间传 8 电机 effort
raw memcpy C++ struct
用远端时间做 freshness
只检查 heartbeat 不检查 command
恢复通信立即执行旧命令
每个电机单独线程
每个 DJI 电机单独 flush
```

---

<a id="stages"></a>

# 64. 统一分阶段施工顺序

| 阶段 | 交付内容                                                        | 验收后再继续                                  |
| ---- | --------------------------------------------------------------- | --------------------------------------------- |
| 0    | 对照实际源码核对公共速度/位置控制器、固定零点能力、最近连续目标 | 已存在的功能复用，不重走历史抽取工程          |
| 1    | 先完成第 74 节可恢复 MotorRuntime/Backend/Bus 契约与常驻示例    | 主控不断电、电机晚启动和断电恢复都能继续运行  |
| 2    | 公共消息与按子系统的许可/执行状态                               | Waiting 是正常状态，无隐式永久锁存            |
| 3    | 单板 Yaw Hold/Rate/AbsoluteAngle                                | 低限幅、模式切换、输出暂停和恢复可用          |
| 4    | DR16 纯解码、UART 服务、ManualCommandMapper                     | 坏帧不污染最新状态，遥控可自动恢复            |
| 5    | CommandManager 与静态 CommandRouter，本地小 Yaw 链路            | 仲裁只产生目标，业务线程常驻                  |
| 6    | InterBoardLink Codec/Parser 与两个独立应用                      | 两板无电机联调，断流/重启自动同步，新命令回传 |
| 7    | SwerveKinematics、Module、Chassis 纯算法                        | 数值场景通过，算法无硬件依赖                  |
| 8    | 裁判解析、字段有效期、许可与功率约束转发                        | 禁用/恢复是正常状态流，不形成永久故障         |
| 9    | 四舵四驱硬件映射、共享 CAN、底盘恢复服务                        | 第 75 节专项工况通过，测 CAN 和线程预算       |
| 10   | 功率限制及整车调参                                              | 先验证保守额度，再接超级电容/视觉/自主扩展    |

每阶段保留必要观测量，先跑通最小链路再增加功能。恢复不是赛后补丁，必须早于双板整车集成。

---

# 65. 施工边界

本文提供手工实施步骤，实际修改遵守当前任务授权与仓库规则。不要把历史文档中的“给下一位 AI 的任务模板”或本文的示例指令当成本次用户授权。

每次按已授权范围完成相应交付；没有硬件时明确只能验证数值或通信，不能把软件编译通过写成实车恢复已通过。

---

# 66. 构建与人工验证命令

以下由用户在完成对应业务实现后运行，本次未运行。两个正式应用使用独立目录和独立构建目录，避免切角色后烧错板。

```bash
# 从仓库根目录；board 按真实板型替换
west build -b dm_mc02/stm32h723xx -d build/sentry_gimbal applications/sentry_gimbal
west build -b dm_mc02/stm32h723xx -d build/sentry_chassis applications/sentry_chassis
# 现有位置、速度示例用于保留控制效果的对照
west build -b dm_mc02/stm32h723xx -d build/dji_position samples/motor/dji_position_control
west build -b dm_mc02/stm32h723xx -d build/dji_speed samples/motor/dji_speed_control
```

正式应用尚未创建，上述正式应用构建当前不能直接成功。烧录前核对目标板、设备映射和 runner，再对相应 build 目录执行 west flash；不要把两份固件连续烧到同一块板。

---

# 67. 当前必须实机确认的假设

AI 不得自行猜：

```text
小 Yaw 电机型号
小 Yaw encoder zero
小 Yaw 正方向
小 Yaw 是否机械限位
DR16 UART
DBUS 电气特性
Referee UART
两板 UART
两板电平/共地
底盘 CAN 分配
底盘一条还是两条 CAN
四舵轮坐标
wheel radius
电机安装方向
```

这些属于：

```text
application / devicetree
```

不属于通用 robotics 库。

---

# 68. 如果未来底盘拆两条 CAN

架构不变。

硬件层改为：

```cpp
motor::dji::Bus steer_bus;
motor::dji::Bus drive_bus;
```

然后：

```text
steer setCurrent ×4
drive setCurrent ×4

steer_bus.flush()
drive_bus.flush()
```

`SwerveChassis` 完全不变。

---

# 69. 如果未来增加第三块板

例如：

```text
云台
底盘
电源/超级电容
```

只新增：

```text
PowerCommandSink
PowerFeedback
MessageId
InterBoard route
```

CommandManager 不应因此重写。

---

# 70. 最终验收清单

- [ ] 主文档唯一，三个原施工文档内容已归并，不再平行维护。
- [ ] 两板独立应用，共享算法库，板间只传高级目标和状态。
- [ ] MotorRuntime 已支持等待与恢复，不以重复 begin/重建对象绕过生命周期。
- [ ] 普通丢包、命令超时、供电关闭、可恢复 CAN 故障不锁存、不退出任务。
- [ ] 急停须明确复位，普通 Safe 模式不要求清故障。
- [ ] 裁判输出禁用/恢复与字段有效性闭环完整，无 healthy/armed 等待死锁。
- [ ] 自动恢复使用当前命令、重置 PID/展开基准，清理旧发送队列，不补射击事件。
- [ ] 底盘故障影响底盘域，独立云台和通信仍运行。
- [ ] 速度/位置公共控制器保留原效果；舵轮角度、半径、方向和电流单位确认。
- [ ] UART、CAN 带宽和线程时间预算实测；恢复期间日志与重试有界。
- [ ] 第 75 节主控不断电、电机断电等所有目标工况完成实机验收。

---

# 71. 后续扩展边界

VisionCommand、AutoCommand 都进入 CommandManager，先检查所选来源的真实新鲜度，再产生 RobotCommand。自主模式不应被硬编码的 require_remote=true 永久卡住；所选命令源策略应与比赛操作方案一致，显式急停仍独立生效。

底盘跟随云台使用 GimbalFeedback 中的云台相对底盘角，在决策侧把操纵速度转成底盘坐标系：vx_body=cos(yaw)*vx_gimbal-sin(yaw)*vy_gimbal，vy_body=sin(yaw)*vx_gimbal+cos(yaw)*vy_gimbal。相对角过期时暂停跟随模式或按明确策略回到车体系；不把云台绝对航向直接当相对角。

暂不实现第三块板、通用路由注册、远程配置写入、加密认证、风险预测及三轮容错行驶。确有新需求时扩展局部接口，不为假想攻击给普通比赛通信加永久封锁。

---

# 72. 合并后的架构原则

CommandManager 决定目标，Router 分发到固定执行板，InterBoardLink 传输值，Subsystem 本地完成实时控制。Safety 决定当前输出，Recovery 负责条件恢复后的重新准备，两者都不终止业务任务。

机器人暂时不能运动时，仍应能报告原因、接收数据、等待电源并自行恢复。只有明确急停需要操作员复位；配置无法满足时阻止相关功能并显示具体缺项。既不能把陈旧目标当新命令，也不能把正常电源管理当不可逆故障。

---

# 73. 公共控制与舵轮施工细节（合并原分阶段指南）

## 73.1 已存在的控制库直接复用

实际接口以 `include/control/motor_velocity.h`、`motor_position.h`、`angle.h` 为准。旧指南中“从 sample 新抽取速度/位置环”“新增固定零点能力”属于历史施工阶段，当前已有对应实现。不得因为合并文档又创建一套平行控制器。

速度环保留：参考速度斜坡、测量低通、soft deadband、前馈 PID、实际输出限幅和内部观测量。软死区为 `abs(error)<=deadband → 0`，否则 `sign(error)*(abs(error)-deadband)`，不能换成超过阈值后保留完整误差的硬死区。

位置环组合公共速度环：位置 PID 产生速度参考，再由速度控制器产生 effort。DJI effort 单位 A，DM MIT 单位 N·m，适配层必须明确单位和限幅；不能用同一数字在两种后端间直接替换。

```cpp
// 已有接口的典型调用形状；放在模块 reset / step 中。
control_motor_velocity_reset(&drive_state, feedback.drive_velocity_rad_s, 0.0f);
control_motor_position_reset(&steer_state, &steer_config,
                             feedback.steer_continuous_rad,
                             feedback.steer_velocity_rad_s);
```

reset 使用当前测量初始化滤波器、PID 历史和微分基准，积分清零，驱动参考从零开始。无效 dt 不塞进 PID；调度恢复时重置 previous_ms，不能把几十秒停电时间作为一次 dt。失败不提交状态与输出的事务语义是已有 C API 契约。

## 73.2 连续角、固定零点角与恢复

`position_rad` 保留驱动连续输出轴坐标，`absolute_position_rad` 是有固定装配零点且满足能力条件的单圈角。两者不能互相替换。DJI 当前仅在相应固定零点传感能力且减速比 1:1 时报告 FeedbackAbsolutePosition；不要仅凭型号名称假设能力。

装配零点计算：先把 raw 与 zero 都提升成有符号整数再相减，再 wrap 到半圈范围，最后乘 `2*pi/ticks_per_turn`。例如原始 10、零点 20，应得到 -10 ticks，不能在无符号数里先减成很大的正数。装配方向、零点与减速比放机器人 overlay/配置，不硬编码到 SwerveModule。

最近连续目标直接复用：

```cpp
float target_continuous_rad = 0.0f;
int ret = control_angle_nearest_continuous_target(
    requested_absolute_rad, measured_absolute_rad,
    measured_continuous_rad, &target_continuous_rad);
```

例如当前单圈 +179°、请求 -179°，连续目标应位于当前位置约 +2°，不能绕 -358°。该例用度便于理解，接口仍输入弧度。DM unwrap 的周期依实际协议 PMAX，不能机械套 2π。掉电恢复后重新播种展开基准，无法观测的圈数不得伪造。

有机械限位的云台不能无条件采用环形最短路径：目标与整个运动路径都必须处于限位内。Continuous 可以复用 AbsoluteNearest；Limited 必须基于已校准的连续坐标选定限位内目标，再交给 DriverContinuous 模式，避免跨 ±π 绕到限位外。第一版小 Yaw 的实际拓扑先由机械结构确认。

## 73.3 SwerveModule 的完整计算顺序

模块保存配置、舵向位置环状态、驱动速度环状态与 initialized，不保存 device、CAN、UART、线程或系统时间。公开接口为 validate/reset/step，参数见第 44 节；配置在构造时传入且完整复制。

1. 校验目标、反馈、dt、轮半径、控制配置；轮半径必须有限且大于零。
2. 求目标角与测量绝对角最短误差；`abs(error)>pi/2` 时目标加 π 并把线速度反号，等于 π/2 时不翻转。
3. wrap 优化角，调用最近连续目标函数，运行舵向位置环。
4. 驱动目标 `rad/s = m/s / wheel_radius_m`；反馈若已经是减速器输出轴速度，不重复乘减速比。
5. 运行驱动速度环，将结果组装到临时 ModuleOutput，全部成功再提交两个控制器状态与输出。

```text
next_steer = steer_state
next_drive = drive_state
计算优化角、连续目标、舵向和驱动输出
任一步失败 → 返回错误，原状态和 out 不变
全部成功 → 提交 next_steer、next_drive 和 out
```

一轮当前 10°、目标 170°、速度 +3 m/s，可优化成约 -10° 与 -3 m/s。若 90° 附近实测出现反复翻转，再加入可配置的小回差，先解决实际抖动，不凭空加入大状态机。

速度 sample 与位置 sample 回接公共控制器后，要在原电机上人工对照 P/I/D、前馈、目标/测量速度、输出、饱和、dt、反馈年龄，确认原有效果。sample 的限时退出和出错 return 不迁移到正式应用。

## 73.4 SwerveKinematics 与 SwerveChassis

统一数组顺序 FL、FR、RL、RR；坐标 +x 向前、+y 向左、+wz 逆时针。每模块保存实际 `(x_m,y_m)`，不只存 L/W。矩形底盘可设 FL=(+L/2,+W/2)、FR=(+L/2,-W/2)、RL=(-L/2,+W/2)、RR=(-L/2,-W/2)。

```text
wheel_vx_i = vx - wz*y_i
wheel_vy_i = vy + wz*x_i
speed_i = hypot(wheel_vx_i, wheel_vy_i)
angle_i = atan2(wheel_vy_i, wheel_vx_i)
```

任一轮超过最大轮速时全部按同一比例缩放，不能单独裁剪某一轮。轮速接近零时保持上一次目标角，速度置零；reset 时将 last_angle 初始化为实际舵角，避免 atan2(0,0) 导致四轮回零。

SwerveChassis 持有一份 Kinematics 与四个 Module。一次 step 中先复制全部算法状态；运动学和四模块全部成功才整体提交，不能前三轮积分已更新、第四轮失败后留下半套状态。输出计算成功后再让硬件层整批提交；真实 CAN 多帧无法物理原子发送，部分发送失败要撤销本周期并进入可恢复暂停，不能声称事务化算法保证总线原子性。

必须保留的数值用例：前进四轮 0°；左移四轮 +90°；旋转四轮切向；平移叠加旋转；零速保持舵角；超速统一缩放；跨 ±π 最短路径；多圈连续参考；非法半径；未 reset 返回 -EACCES；一个模块失败不改变其余模块状态。

## 73.5 原规格其他接口约定

- `Dr16Decoder::decodeFrame`：先写临时 RemoteState；成功 0 并更新 stamp/sequence，失败 -EINVAL/-EBADMSG 且 out 不变。channel_center/min/max 和拨杆映射沿用第 9、12 节，实机确认方向。UART 参数须确认校验位、停止位及 DBUS 电气路径，不能只看波特率。不要给没有 CRC 的协议虚构 CRC 检查。
- `RemoteService::snapshot`：无有效数据 -EAGAIN；过期可输出带 online=false 的诊断快照并返回 -ESTALE，消费者必须检查有效性。一次候选坏帧不直接把整个服务关闭。
- `ManualCommandMapper::map`：归一化模拟量到 [-1,1]，处理死区与鼠标倍率，只产生意图，不操作电机；Auto 未实现时保持安全目标，不猜测自主行为。
- `RefereeParser`：按实际使用版本的帧头、长度、CRC8/CRC16 和命令布局增量解析；未知命令跳过，分片与粘包可继续。裁判协议版本尚未确认，施工时再依据该版本官方协议固定 command_id/字节布局，不沿用过期资料。
- `InterBoardParser`：解析器仅一个线程拥有；固定容量帧队列，容量不足记录并重新同步，禁止把缺失字节拼成合法新帧。没有完整帧时 popFrame=false，不是应用失败。
- zbus 只放应用任务；每板独立 channel 实例。Latest State 用于连续状态，事件用固定队列。线程间复制快照，避免无锁共享可变对象引用。

---

<a id="lifecycle"></a>

# 74. 可恢复生命周期：必须先动手的文件清单与接口契约

## 74.1 修改位置与顺序

| 顺序 | 现有/拟建文件                                                                    | 手工修改要求                                                                                  |
| ---- | -------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| 1    | `include/control/motor_backend.hpp`、`lib/control/motor_control.cpp`         | 分离一次配置绑定与周期准备；增加 Waiting/Recovering/Ready，fail 不再把所有错误做终局处理      |
| 2    | `include/control/dji_motor_backend.hpp`、`lib/control/dji_motor_backend.cpp` | 复用 Bus::recover；从非阻塞反馈快照判断候选恢复，避免 Ready 锁存死锁；控制器 bus-off 单独处理 |
| 3    | `include/control/dm_motor_backend.hpp`、`lib/control/dm_motor_backend.cpp`   | 复用分阶段恢复，处理 -EINPROGRESS/-EAGAIN；重置 unwrap，硬件错误按实际状态清除                |
| 4    | `include/control/velocity_motor.hpp`、`position_motor.hpp` 及实现            | 增加 suspend/poll/resume，reset PID/时间/坐标，保留 begin 作为一次性配置入口                  |
| 5    | `drivers/motor/dji/dji_bus.cpp`、`dm/dm_bus.cpp` 及必要内部接口              | 仅在现有 API 无法表达撤销待发/有界恢复/重新播种时补齐；不删边界与单位检查                     |
| 6    | `samples/motor/dji_speed_control/src/main.cpp`、位置与 DM 对应示例             | 提供常驻恢复验证路径，保留原轨迹和观测；有限时演示明确标注用途                                |
| 7    | 拟建`applications/sentry_*/src/`                                               | 唯一硬件所有者、常驻状态机、恢复调度；一个子系统未就绪不阻塞其他线程启动                      |

配置、对象所有权和设备回调只建立一次。`begin_attempted_` 可继续防重复绑定，但暂时没有电机反馈不应消耗“之后不能再运行”的机会。不要仅把它置回 false；backend 的 claimed、Bus 状态和驱动回调仍然存在。

## 74.2 建议新增接口形状

以下为待实现契约，不能直接调用当前代码中的同名成员：

```cpp
enum class PauseReason {
    OperatorDisabled, RefereeDisabled, CommandTimeout,
    FeedbackTimeout, TransportTemporary, InvalidCycle, EmergencyStop
};

// MotorRuntime / wrapper 层公开形状
int configure();                     // 校验、唯一绑定；不等待电机供电
int suspend(PauseReason reason);      // 撤销输出，保留对象、回调和恢复机会
int poll(std::uint64_t now_ms);       // 推进恢复，不直接产生运动目标
int resume();                        // 条件允许且控制器已重置，arm 并确认零输出
ExecutionState state() const;

// Backend 新增/拆分操作的形状
int pollPrepare(std::uint64_t now_ms); // prepare 原阻塞流程拆成有界步骤
int recover(std::uint64_t now_ms);     // 仅恢复通信/禁用态，不等于允许运动
int resetMeasurementReference();      // 在新反馈基础上重新建立展开基准
```

| 接口                      | 结果与状态                                                       | 边界与线程约束                                             |
| ------------------------- | ---------------------------------------------------------------- | ---------------------------------------------------------- |
| configure                 | 0 → Waiting；明确配置错 → ConfigBlocked                        | 一个所有者线程；重复调用不再次 claim，可返回 -EALREADY     |
| suspend                   | 无论零帧能否送达，都先撤销软件非零目标并进入 Waiting/急停态      | 幂等，重复调用不重启恢复计时；发送失败返回 errno 仅供诊断  |
| poll                      | 0 表示 Ready；-EAGAIN/-EINPROGRESS 表示继续；暂时 I/O 错仍可重试 | 每轮有预算，禁止 while 一直等；连续调用不会重复注册资源    |
| resume                    | 0 → Active；暂不可用 -EAGAIN 保持等待；急停未清 -EACCES         | 先验证当前许可、新命令和新反馈，reset 后 arm；失败回恢复态 |
| resetMeasurementReference | 0 重新播种；缺新反馈 -EAGAIN；无所需位置能力 -ENOTSUP            | 非运动状态调用；不修改装配零点，不伪造未知多圈位置         |

`-EINVAL/-ERANGE/-EHOSTDOWN` 不能仅按 errno 全局决定严重度。同一个 -ERANGE 可以来自错误配置，也可能是本次 dt 越界。调用位置与具体原因共同分类：配置错误阻止配置，本次运行数据无效暂停本周期。数据记录必须能区分。

现有 Bus 的 Fault 可以先保留为底层暂时禁止发送状态，由恢复服务自动 recover，未必需要重写所有驱动枚举。关键是不能把底层 Fault 无条件复制成整车人工锁存。底层 recover 成功也不自动恢复旧目标。

## 74.3 常驻控制流程示例

放入正式应用的控制/硬件服务任务，下面的函数名为流程伪代码：

```text
循环运行，每次按固定周期调度：
    读取当前许可、命令和反馈快照
    若显式急停：
        只在进入时 suspend(EmergencyStop)
        继续报告与处理复位；不退出循环
    否则若供电许可禁止或过期：
        只在进入等待时 suspend(具体原因)
        持续接收；不生成非零目标，不 arm
    否则若 Active 且命令或必要反馈过期：
        suspend(具体原因)，下轮进入恢复
    否则若本地尚未 Ready/Active：
        按重试时间调用 poll(now)
        成功后重新播种反馈、reset 控制器和时间，公布新的恢复 generation
    否则若 Ready 且有匹配 generation 的新命令：
        resume；成功从零参考按斜坡进入 Active
    否则若 Active：
        用实际正常 dt 执行 step/update
        失败则记录原因并 suspend，下一周期继续恢复路径
    发布执行状态与原因
    等待下一周期
```

命令超时但供电正常时恢复探测不能依赖先收到可执行目标或 getState()==Ready；即使当前命令还未通过 generation 校验，也必须能够完成硬件准备并发布 Ready。在 Ready 等待期间若反馈重新过期，只回本地恢复；不会 arm。电源仍明确禁止时不自动 arm、不持续发送使能；接收与必要诊断始终工作。每次状态转换只 reset 一次，不能每轮 reset 导致永远无法累计稳定时间。

## 74.4 恢复路径自检点

每一步完成后检查：没有实际电机时仍周期上报 Waiting；恢复重试间隔有上限；PID 不在 Waiting 中积分；不同供电域独立；getState 的软件 Fault 不阻止 recover；DM 恢复的中间返回码不导致 return；普通错误不触发主控重启；对象、回调和 CAN filter 数量不随恢复次数增长。

---

<a id="acceptance"></a>

# 75. 比赛断电、噪声与自恢复专项验收

以下是交付前由用户执行的验证清单，不是本次已经完成的测试。先机械支撑、低限幅、准备物理断电，确认两板逻辑供电与电机供电能分别控制；先验证无动力状态，再让电机参与。

| 工况                                             | 必须观察到的结果                                   | 失败通常定位到                                       |
| ------------------------------------------------ | -------------------------------------------------- | ---------------------------------------------------- |
| 主控先启动，电机 30 秒后才上电                   | 等待期间心跳/遥控/诊断持续；上电后自动准备与恢复   | 一次性 begin、启动超时 return、阻塞初始化            |
| 运行中裁判输出禁用，主控保持供电                 | 对应域撤销输出并 Waiting，正常报告禁止原因         | Disabled 调用了终止性 stop、错误全局传播             |
| 主控保持供电，电机断电 1 秒、30 秒、数分钟后恢复 | 不耗尽重试次数；新反馈后重置参考并自动执行新命令   | 永久锁存、重试耗尽、dt 没重置、控制器旧积分          |
| 裁判消息晚于真实断电，或没有可用许可消息         | 反馈超时仍能暂停；原因可显示未知，不判断永久损坏   | 必须收到“已断电证明”才允许等待的错误依赖           |
| 恢复许可已经 true，电机还未启动                  | 保持 Waiting 而不反复使能冲击，反馈到来后恢复      | 许可被误当作物理供电/反馈保证                        |
| 等待时 getState 持续 Fault，但原始新反馈已经到来 | 自动 recover 后重新评估 Ready                      | Ready→recover 的死循环                              |
| 一帧 CRC 错/未知消息/丢帧                        | 计数增加，上一命令未过期则继续；下一合法帧正常     | 坏帧触发全车 disable、累计错误熔断                   |
| UART 断开超过命令有效期再接回                    | 底盘暂停；自动同步、接受新目标并恢复，无手工清错   | 一次 session 失败永久锁死、重复握手不断改 generation |
| 只停 command_task，心跳继续                      | 命令过期底盘暂停；命令生产恢复后可恢复             | 心跳刷新了命令时间、旧目标被重新编号                 |
| 普通 Safe→Manual/Auto 切换                      | 无需急停复位，当前合法命令恢复                     | 把日常禁用和 EmergencyStop 混为一谈                  |
| 一个必要舵轮离线再恢复                           | 整个底盘暂停后恢复，独立云台/通信持续              | 一轮错误关全车、缺轮继续运动                         |
| 一次明显调度延迟或发送忙                         | 可以短暂暂停，但下轮进入恢复，不结束任务           | runtime.fail 终局化、过大 dt 补算                    |
| CAN bus-off 后链路恢复                           | 控制器恢复、软件 Bus 恢复分步完成；后台有界        | 只清 Bus Fault、不恢复 CAN 控制器；恢复中阻塞        |
| 底盘或云台单板重启                               | 另一板存活，旧命令无效；同步后自动恢复             | uptime 回绕误判、旧命令序号/缓存混入新会话           |
| 断电期间手动转动舵轮，恢复供电                   | 舵角参考重新建立，无追赶旧多圈目标                 | 丢帧前后错误 unwrap、旧 PID/目标保留                 |
| 显式急停按下、再松开                             | 不自行运动；明确复位且条件正常后恢复，任务始终在线 | 急停被普通噪声触发、复位直接沿用旧速度               |
| 裁判只更新其他字段，输出许可字段停止更新         | 许可独立超时；真实许可更新后恢复                   | 使用总 RefereeState 时间戳刷新旧许可                 |
| 多次上述断电恢复循环                             | 内存、注册回调数、日志速率与线程周期无持续增长     | 重建对象、重复注册、错误日志风暴                     |

每次记录：本地运行时间（证明未重启）、执行状态、暂停原因、最后有效反馈年龄、命令序号、恢复 generation、Bus/CAN 状态、重试次数（只统计）、恢复耗时与第一周期目标/输出。正式验收要求供电恢复到可运动的实际时延可解释，不能只看到一行“recover success”。

---

# 76. 合并映射与尚未验证内容

| 原文档/草稿                    | 本文归并位置                            | 消除的冲突                                                     |
| ------------------------------ | --------------------------------------- | -------------------------------------------------------------- |
| 机器人应用层与消息架构施工规格 | 1～13、21～23、36～41、48～60、71、73.5 | 单板部署改为双板；Disabled 不再终结 wrapper；不重复建立控制库  |
| 舵轮控制架构分阶段施工指南     | 42～47、64、68、73、74                  | 旧阶段 1～4 改为基线复用；任意错误整车停止改为所属域可恢复暂停 |
| 双主控命令路由实施指南         | 23～35、61～62、75                      | 普通断链自动同步恢复，无需人工重新使能；序号和数据年龄统一     |
| 已有同名双主控综合草稿         | 全文作为统一入口修订                    | 取消笼统严重故障锁存；消除 Safe/Hold/stop 与供电工况的矛盾     |

尚需实机确认：板型、UART 电平与引脚、DR16 DBUS 接线、旧串口协议、裁判实际协议版本与输出字段、每条 CAN 电机分配、驱动无命令时的响应、控制器在途帧清理、恢复步骤耗时、boot_id 可用来源、舵轮装配零点/半径/方向、云台限位与断电后可观测位置。本文不虚构这些已验证。

本次只整理根目录 Markdown 施工文档；业务源码、测试、脚本、配置、设备树、构建文件以及仓库规则均未修改。
