# SkyWalker 双主控分布式机器人架构施工规格

> 目标仓库：`SHM-white/SkyWalker_General_Embedded_Code`  
> 机器人：RoboMaster 哨兵  
> 主控拓扑：**云台主控板 + 底盘主控板**  
> 板间通信：**UART 双向通信**  
> 当前可测试硬件：**小 Yaw 云台**  
> 当前不可完整测试硬件：**4 舵 + 4 驱动底盘**  
> 本文用途：**可直接交给其他 AI/Codex 按阶段施工的接口级规范**

---

# 0. 执行前规则

执行 AI 必须先读：

```text
AGENTS.md
.agents/skills/ancient-programming/SKILL.md
```

仓库默认启用“古法编程模式”。若要真正修改业务源码，用户必须在当次任务明确写：

```text
本任务禁用古法编程模式。
```

否则只能写实施说明，不能修改业务代码。

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

`PositionMotor` 已支持：

```cpp
enum class PositionReference {
    StartupRelative,
    DriverContinuous,
    AbsoluteNearest,
};
```

因此小 Yaw 第一版直接复用：

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
 ├── Local Sink
 └── Remote Sink
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
│   │   ├── command_manager.hpp
│   │   ├── command_sink.hpp
│   │   └── command_router.hpp
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

# 14. Safety 核心语义

```cpp
enum class SafetyAction : std::uint8_t {
    Disable = 0,
    Hold,
    Active,
};

enum class SafetyState : std::uint8_t {
    Boot = 0,
    Safe,
    Ready,
    Degraded,
    Fault,
    EmergencyStop,
};
```

安全级别：

```text
Disable < Hold < Active
```

本地 Safety 只能保持或降低，不能提升全局安全级别。

---

# 15. Safety Fault Mask

```cpp
enum SafetyFault : std::uint32_t {
    SafetyFaultNone              = 0,

    SafetyFaultRemoteOffline     = 1u << 0,
    SafetyFaultRefereeOffline    = 1u << 1,
    SafetyFaultInterboardOffline = 1u << 2,
    SafetyFaultCommandTimeout    = 1u << 3,

    SafetyFaultGimbalMotor       = 1u << 4,
    SafetyFaultChassisMotor      = 1u << 5,
    SafetyFaultCan               = 1u << 6,
    SafetyFaultUart              = 1u << 7,
    SafetyFaultController        = 1u << 8,

    SafetyFaultEmergencyStop     = 1u << 31,
};
```

---

# 16. GlobalSafetyManager

运行在云台板。

```cpp
struct GlobalSafetyInputs {
    std::uint64_t now_ms = 0;

    bool initialization_complete = false;

    bool remote_online = false;
    bool referee_online = false;

    bool gimbal_healthy = false;
    bool chassis_peer_online = false;
    bool chassis_healthy = false;

    bool referee_gimbal_output_enabled = true;
    bool referee_chassis_output_enabled = true;
    bool referee_shooter_output_enabled = true;

    bool emergency_stop_requested = false;
};

struct GlobalSafetyDecision {
    SafetyState state = SafetyState::Boot;

    SafetyAction gimbal = SafetyAction::Disable;
    SafetyAction chassis = SafetyAction::Disable;
    SafetyAction shooter = SafetyAction::Disable;

    std::uint32_t fault_mask = SafetyFaultNone;

    MessageStamp stamp{};
};

class GlobalSafetyManager {
public:
    struct Config {
        bool require_remote = true;
        bool require_referee_for_motion = false;
        bool allow_gimbal_when_chassis_offline = true;
    };

    explicit GlobalSafetyManager(const Config &config);

    int reset(std::uint64_t now_ms);

    int evaluate(
        const GlobalSafetyInputs &input,
        GlobalSafetyDecision &out);

    int clearLatchedFaults();

private:
    Config config_{};
    SafetyState state_ = SafetyState::Boot;
    std::uint32_t latched_faults_ = 0;
    std::uint32_t sequence_ = 0;
};
```

---

# 17. Global Safety 默认策略

优先级：

```text
EmergencyStop
>
严重本地硬件 Fault
>
初始化未完成
>
Remote Offline
>
Peer Offline
>
Referee Output Disable
>
Normal
```

典型行为：

```text
Normal:
    gimbal  Active
    chassis Active
    shooter Active

Remote Offline:
    gimbal  Hold
    chassis Disable
    shooter Disable

Chassis Peer Offline:
    gimbal  Active/Hold
    chassis Disable

EmergencyStop:
    all Disable
```

EmergencyStop 必须 latch，不能自动恢复。

---

# 18. GimbalLocalSafety

```cpp
struct GimbalLocalSafetyInputs {
    SafetyAction global_action = SafetyAction::Disable;

    bool motor_ready = false;
    bool feedback_fresh = false;
    bool controller_ok = false;

    bool local_emergency_stop = false;
};

struct GimbalLocalSafetyDecision {
    SafetyAction action = SafetyAction::Disable;
    std::uint32_t fault_mask = 0;
};

class GimbalLocalSafety {
public:
    int evaluate(
        const GimbalLocalSafetyInputs &input,
        GimbalLocalSafetyDecision &out) const;
};
```

优先：

```text
local EStop
>
motor fault
>
feedback stale
>
controller fault
>
global action
```

---

# 19. ChassisLocalSafety

运行在底盘板，是底盘最终安全裁决。

```cpp
struct ChassisLocalSafetyInputs {
    std::uint64_t now_ms = 0;

    SafetyAction global_action = SafetyAction::Disable;

    bool peer_heartbeat_fresh = false;
    bool chassis_command_fresh = false;

    bool can_healthy = false;
    bool all_motors_healthy = false;
    bool controllers_ok = false;

    bool local_emergency_stop = false;
};

struct ChassisLocalSafetyDecision {
    SafetyAction action = SafetyAction::Disable;
    SafetyState state = SafetyState::Boot;
    std::uint32_t fault_mask = 0;
};

class ChassisLocalSafety {
public:
    struct Config {
        std::uint32_t heartbeat_timeout_ms = 200;
        std::uint32_t command_timeout_ms = 100;
    };

    explicit ChassisLocalSafety(const Config &config);

    int reset();

    int evaluate(
        const ChassisLocalSafetyInputs &input,
        ChassisLocalSafetyDecision &out);

private:
    Config config_{};
};
```

优先级：

```text
Local EStop
>
CAN Fault
>
Motor Fault
>
Controller Fault
>
Command Timeout
>
Heartbeat Timeout
>
Global Disable
>
Global Hold
>
Active
```

---

# 20. Heartbeat 与 Command Timeout 必须分离

以下情况必须被识别：

```text
Heartbeat 正常
CommandTask 卡死
```

此时：

```text
peer online = true
command fresh = false
```

底盘必须停车。

因此：

```text
heartbeat freshness != command freshness
```

两者不能合并。

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

# 22. CommandSink

```cpp
class IGimbalCommandSink {
public:
    virtual ~IGimbalCommandSink() = default;
    virtual int submit(const GimbalCommand &command) = 0;
};

class IChassisCommandSink {
public:
    virtual ~IChassisCommandSink() = default;

    virtual int submit(
        const ChassisCommand &command,
        SafetyAction global_action,
        std::uint32_t global_fault_mask) = 0;
};
```

---

# 23. CommandRouter

```cpp
class CommandRouter {
public:
    CommandRouter(
        IGimbalCommandSink &gimbal_sink,
        IChassisCommandSink &chassis_sink);

    int route(
        const RobotCommand &command,
        const GlobalSafetyDecision &safety);

private:
    IGimbalCommandSink &gimbal_sink_;
    IChassisCommandSink &chassis_sink_;
};
```

云台：

```text
LocalGimbalSink
```

底盘：

```text
RemoteChassisSink
```

CommandManager 完全不知道底盘在远端板。

---

# 24. 板间协议原则

禁止：

```cpp
uart_send(
    reinterpret_cast<uint8_t*>(&command),
    sizeof(command));
```

必须显式 Wire Protocol。

要求：

```text
SOF
protocol version
message id
length
sequence
sender uptime
payload
CRC16
```

---

# 25. Frame V1

```text
0       0xA5
1       0x5A
2       protocol_version
3       flags
4..5    message_id LE
6..7    payload_length LE
8..9    sequence LE
10..13  sender_uptime_ms LE
14..    payload
last2   CRC16
```

最大 payload：

```text
128 B
```

最大 frame：

```text
160 B
```

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

# 27. Heartbeat

```cpp
enum class BoardRole : std::uint8_t {
    Unknown = 0,
    GimbalController,
    ChassisController,
};

struct BoardHeartbeat {
    BoardRole role = BoardRole::Unknown;
    SafetyState safety_state = SafetyState::Boot;
    std::uint32_t fault_mask = 0;
    std::uint32_t sender_uptime_ms = 0;
    MessageStamp stamp{};
};
```

推荐：

```text
50 Hz
timeout 200 ms
```

---

# 28. RemoteChassisControl

```cpp
struct RemoteChassisControl {
    ChassisCommand command{};

    SafetyAction global_action = SafetyAction::Disable;

    std::uint32_t global_fault_mask = 0;

    MessageStamp stamp{};
};
```

wire payload：

```text
u8  chassis_mode
u8  source
u8  global_action
u8  reserved

f32 vx
f32 vy
f32 wz

u32 global_fault_mask
u32 command_sequence
```

---

# 29. ChassisConstraint

Command、Constraint、Safety 必须分离。

```cpp
struct ChassisConstraint {
    bool valid = false;

    bool referee_output_enabled = true;

    float power_limit_w = 0.0f;
    float buffer_energy_j = 0.0f;

    MessageStamp stamp{};
};
```

含义：

```text
Command:
    想做什么

Constraint:
    最多允许什么

Safety:
    现在能不能做
```

---

# 30. ChassisFeedbackSummary

```cpp
struct ChassisFeedbackSummary {
    float estimated_vx_m_s = 0.0f;
    float estimated_vy_m_s = 0.0f;
    float estimated_wz_rad_s = 0.0f;

    float measured_power_w = 0.0f;

    bool healthy = false;
    bool armed = false;

    SafetyState safety_state = SafetyState::Boot;

    std::uint32_t fault_mask = 0;

    MessageStamp stamp{};
};
```

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

# 34. 周期控制不做 ACK

以下消息：

```text
Heartbeat
ChassisControl
ChassisFeedback
Constraint
```

按 Latest State 处理。

丢一帧：

```text
等待下一帧
```

不 retry。

后续只有：

```text
保存零点
写配置
进入 bootloader
清故障
```

这类离散事件才做 ACK。

---

# 35. UART 参数建议

板间：

```text
460800 baud
8N1
DMA
```

如果验证稳定再：

```text
921600
```

板间消息带宽远低于此速率。

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

# 40. Yaw SafetyAction

```text
Active:
    执行 GimbalCommand

Hold:
    首次进入时锁当前角度
    后续保持

Disable:
    motor.stop()
```

日常 Remote 短暂掉线：

```text
Hold
```

真正：

```text
EStop / Motor fault
```

才 Disable。

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

AbsoluteAngle 模式依赖 `PositionMotor::AbsoluteNearest`，YawGimbal 不重复 shortest-path。

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

# 47. 底盘硬件层

真实底盘不要使用：

```text
8 × DjiMotorBackend
```

因为当前 `DjiMotorBackend` 是单电机独占 Bus 模型。

底盘必须：

```text
shared dji::Bus
```

接口：

```cpp
class DjiChassisHardware {
public:
    struct Devices {
        const device *can = nullptr;
        std::array<const device *,4> steer{};
        std::array<const device *,4> drive{};
    };

    int init();
    int waitForFeedback(std::uint32_t timeout_ms);
    int read(ChassisFeedback &out);
    int arm();
    int apply(const ChassisOutput &output);
    int stop();
    bool healthy() const;

private:
    motor::dji::Bus bus_{};
};
```

apply：

```text
setCurrent ×8
↓
bus.flush ×1
```

---

# 48. 裁判系统

裁判 UART 在云台板。

公共状态：

```cpp
struct RefereePowerState {
    float chassis_power_w = 0.0f;
    float chassis_power_limit_w = 0.0f;
    float buffer_energy_j = 0.0f;
};

struct RefereeRobotState {
    std::uint8_t robot_id = 0;
    bool chassis_output_enabled = false;
    bool gimbal_output_enabled = false;
    bool shooter_output_enabled = false;
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

V1 简单 scale，超级电容策略后续独立升级。

---

# 52. 云台板初始化顺序

```text
device ready
message bus
RemoteService
RefereeService
InterBoardLink
Yaw MotorBackend
PositionMotor
YawGimbal
ManualCommandMapper
GlobalSafetyManager
GimbalLocalSafety
CommandManager
CommandRouter
启动线程
```

上电默认：

```text
Gimbal Hold
Chassis Disable
Shooter Disable
```

---

# 53. 底盘板初始化顺序

未来：

```text
device ready
message bus
InterBoardLink
DjiChassisHardware.init
等待 8 电机反馈
SwerveChassis.validate
读初始反馈
SwerveChassis.reset
ChassisLocalSafety.reset
hardware.arm
等待 fresh heartbeat + fresh command
再允许 Active
```

禁止使用上次缓存旧命令。

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

# 61. Recovery 规则

非锁存：

```text
Remote offline
Interboard timeout
Referee offline
```

允许恢复。

锁存：

```text
EmergencyStop
严重 CAN fault
严重 motor fault
```

必须显式 clear。

任何掉线恢复后：

```text
必须收到新鲜的新命令
```

不能恢复旧速度。

---

# 62. 对端重启

Heartbeat 中携带：

```text
sender_uptime_ms
```

如果突然回退：

```text
认为 peer reboot
```

立即：

```text
旧 command invalid
LocalSafety Disable
等待新命令
```

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

# 64. 分阶段施工顺序

## 阶段 1：公共消息

实现：

```text
common
remote
command
feedback
safety
interboard
```

只定义结构体/枚举。

---

## 阶段 2：Safety

实现：

```text
GlobalSafetyManager
GimbalLocalSafety
ChassisLocalSafety
```

用纯 fixture 验证。

---

## 阶段 3：YawGimbal

实现：

```text
YawGimbal
硬编码 sample
```

先验证真实小 Yaw。

---

## 阶段 4：DR16 Decoder

```text
bytes → RemoteState
```

---

## 阶段 5：Remote UART

只打印，不控电机。

---

## 阶段 6：Manual + CommandManager

```text
Remote
→ Intent
→ Safety
→ RobotCommand
```

只打印。

---

## 阶段 7：DR16 控制小 Yaw

完成第一条实机链。

---

## 阶段 8：InterBoard Codec/Parser

纯 buffer 测试：

```text
encode → decode
CRC
错误长度
错误版本
```

---

## 阶段 9：双板 UART

先只有：

```text
Heartbeat
ChassisControl
Feedback
```

---

## 阶段 10：SwerveKinematics

纯算法。

---

## 阶段 11：Referee Parser

离线 fixture。

---

## 阶段 12：Referee UART

真实裁判。

---

## 阶段 13：ChassisConstraint

```text
Referee → UART → Chassis
```

---

## 阶段 14：SwerveModule / Chassis

接近底盘硬件可测时做。

---

## 阶段 15：真实底盘 Hardware

顺序：

```text
1 module
→ 2 modules
→ 4 modules
```

---

## 阶段 16：Power Limiter / SuperCap

普通功率限制先完成，再做超级电容。

---

# 65. 每阶段 AI 工作流程

```text
1. 读 AGENTS.md
2. 读 ancient-programming skill
3. 确认本任务是否禁用古法模式
4. 读本文
5. 读本阶段真实代码
6. 只实现本阶段
7. 构建
8. 检查 git diff
9. 汇报
10. 停止
```

禁止自动继续下一阶段。

---

# 66. 每阶段汇报格式

```text
已修改文件
新增接口
保留的旧行为
构建命令
构建结果
未验证硬件假设
下一阶段名称
```

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

## 双板

```text
[ ] 两板独立 application
[ ] 高层命令跨板
[ ] PID 本地执行
[ ] UART 掉线底盘自行停车
```

## Safety

```text
[ ] GlobalSafety
[ ] GimbalLocalSafety
[ ] ChassisLocalSafety
[ ] Active/Hold/Disable
[ ] fault mask
[ ] heartbeat timeout
[ ] command timeout
[ ] EStop latch
[ ] stale command 不自动恢复
```

## Command

```text
[ ] OperatorIntent
[ ] CommandManager
[ ] RobotCommand
[ ] CommandRouter
[ ] Local/Remote Sink
```

## InterBoard

```text
[ ] protocol version
[ ] message id
[ ] length
[ ] sequence
[ ] CRC16
[ ] explicit endian
[ ] heartbeat
[ ] chassis control
[ ] constraint
[ ] feedback
```

## Gimbal

```text
[ ] PositionMotor reused
[ ] Hold
[ ] Rate
[ ] Absolute
[ ] DR16 控制
[ ] Remote offline Hold
[ ] EStop Disable
```

## Chassis

```text
[ ] Kinematics
[ ] zero speed hold angle
[ ] uniform scale
[ ] Module
[ ] Chassis
[ ] shared DJI Bus
[ ] local safety
```

## Referee

```text
[ ] parser
[ ] CRC
[ ] stable RefereeState
[ ] constraint 转发
[ ] 不直接操作电机
```

---

# 71. 给下一位 AI 的任务模板

```text
请严格按照
《SkyWalker 双主控分布式机器人架构施工规格》
执行。

本任务禁用古法编程模式。

只执行：
“阶段 1：公共消息”。

要求：
1. 先读取 AGENTS.md。
2. 读取 .agents/skills/ancient-programming/SKILL.md。
3. 读取施工规格。
4. 读取当前仓库实际代码。
5. 不提前实现后续阶段。
6. 完成后构建受影响目标。
7. 检查 git diff。
8. 汇报修改文件、接口、构建结果和未验证假设。
9. 然后停止。
```

后续把：

```text
阶段 1
```

依次换成：

```text
阶段 2 Safety
阶段 3 YawGimbal
阶段 4 DR16
...
```

---

# 72. 最终架构原则

```text
Global Safety：
决定“整车逻辑上允不允许”

Local Safety：
决定“本板硬件现在到底能不能执行”

Command：
描述“想做什么”

Constraint：
描述“最多允许什么”

InterBoard：
只搬稳定、高层语义

Subsystem：
本地完成实时控制
```

最终要达到：

```text
云台板死机
→ 底盘自行停机

UART 断线
→ 底盘自行停机

底盘板异常
→ 云台知道并降级

底盘机械形式变化
→ CommandManager 不变

新增第三块板
→ 扩展 Router / Link，不重写全架构
```

这就是后续所有施工必须保持的边界。
