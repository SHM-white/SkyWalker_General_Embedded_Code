# SkyWalker 机器人应用层、消息系统与子系统架构施工规格

> 目标仓库：`SHM-white/SkyWalker_General_Embedded_Code`
> 基线：当前 `main/dev` 同步版本
> 文档定位：**可直接交给另一个 AI / Codex 执行的架构与接口施工规格**
> 当前硬件条件：**小 Yaw 云台已装好，可优先实机测试；底盘尚不具备完整实机测试条件，因此先完成基础运动学与控制框架，硬件绑定后置。**

---

# 0. 执行前强制要求

## 0.1 仓库当前仍启用“古法编程模式”

执行 AI 在开始任何源码修改前，必须先读取：

```text
AGENTS.md
.agents/skills/ancient-programming/SKILL.md
```

当前仓库默认规则是：

```text
业务源码只读
只允许写根目录 Markdown 施工指南
```

因此，如果执行 AI 要按照本文真正修改源码，用户必须在该次任务中明确写：

```text
本任务禁用古法编程模式。
```

如果用户没有明确写这句话，执行 AI **不得修改业务源码**，只能继续写说明文档。

---

# 1. 当前代码基线与关键事实

当前仓库已经具备较成熟的底层能力：

```text
Motor Driver
├── DJI M3508 / M2006 / GM6020
├── DM J4310
├── 连续位置
├── 固定零点单圈绝对角
├── 速度 / 电流 / 温度反馈
└── Bus 生命周期

Control
├── PID
├── Feedforward
├── Slew Rate
├── Angle
├── motor_velocity
├── motor_position
├── VelocityMotor
└── PositionMotor

Sensors
├── BMI088
├── EKF
└── Kalman
```

已经有：

```cpp
skywalker::control::VelocityMotor
skywalker::control::PositionMotor
skywalker::control::MotorBackend
skywalker::control::DjiMotorBackend
skywalker::control::DmMotorBackend
```

其中 `PositionMotor` 已支持三种位置语义：

```cpp
enum class PositionReference {
    StartupRelative,
    DriverContinuous,
    AbsoluteNearest,
};
```

因此，小 Yaw 云台如果使用拥有固定零点单圈角能力的电机，可以直接基于：

```text
PositionMotor
+
PositionReference::AbsoluteNearest
```

建立第一套真实机器人子系统。

## 1.1 当前应用层缺口

当前真正缺少的是：

```text
Remote / DR16
Referee System
Robot Command
Safety / Supervisor
Gimbal
Chassis
Shooter
Vision / Auto control interface
Application orchestration
Inter-thread messaging
```

因此，下一阶段不要继续堆“单电机 sample”。

目标应该变成：

```text
底层驱动已经能让电机转
        ↓
现在开始让“机器人”工作
```

---

# 2. 总体设计原则

后续整个项目强制采用以下单向依赖：

```text
┌────────────────────────────────────────────┐
│              Application                   │
│  线程 / 设备绑定 / 初始化 / 生命周期 / 故障  │
└────────────────────┬───────────────────────┘
                     │
                     ▼
┌────────────────────────────────────────────┐
│        Command / Supervisor Layer          │
│   人想做什么 + 规则允许什么 + 安全状态       │
└────────────────────┬───────────────────────┘
                     │
          ┌──────────┼────────────┐
          ▼          ▼            ▼
       Gimbal      Chassis      Shooter
          │          │            │
          ▼          ▼            ▼
       Control / Robotics algorithms
                     │
                     ▼
              Motor / Sensor API
                     │
                     ▼
                 Drivers
```

必须遵守：

```text
遥控器 ≠ 控制器
裁判系统 ≠ 控制器
消息总线 ≠ 业务逻辑
电机驱动 ≠ 机器人子系统
Application ≠ 算法库
```

核心语义：

```text
Remote
= 人想干什么

Referee
= 比赛规则允许什么

Command Manager
= 机器人最终决定干什么

Subsystem
= 机械机构应该怎么动作

Controller
= 误差怎么变成 effort

Driver
= effort 怎么变成总线帧
```

---

# 3. 最终目标数据流

```text
                     ┌───────────────┐
                     │ DR16 / DT7    │
                     └───────┬───────┘
                             │
                             ▼
                       RemoteState
                             │
                             ▼
                    ManualCommandMapper
                             │
                             ▼
                       OperatorIntent
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
      Vision             Auto Control        Other Input
        │                    │                    │
        └────────────────────┼────────────────────┘
                             ▼
                       CommandManager
                             ▲
                             │
                ┌────────────┴────────────┐
                │                         │
          SafetyDecision             RefereeState
                ▲                         ▲
                │                         │
          SafetyManager          Referee Parser
                                          ▲
                                          │
                                   Referee UART

CommandManager 输出：

┌─────────────────┐
│ ResolvedCommand │
├─────────────────┤
│ chassis         │
│ gimbal          │
│ shooter         │
└────────┬────────┘
         │
         ├──────────────► YawGimbal
         │
         ├──────────────► SwerveChassis
         │
         └──────────────► Shooter
```

---

# 4. 推荐最终目录

不要一次性全部创建。严格按后文阶段逐步建立。

```text
include/
├── control/
│   └── ...                       # 已存在
│
├── communication/
│   ├── remote/
│   │   ├── dr16_decoder.hpp
│   │   └── remote_service.hpp
│   │
│   └── referee/
│       ├── referee_crc.hpp
│       ├── referee_protocol.hpp
│       ├── referee_parser.hpp
│       └── referee_service.hpp
│
├── robotics/
│   ├── messages/
│   │   ├── common.hpp
│   │   ├── remote.hpp
│   │   ├── referee.hpp
│   │   ├── command.hpp
│   │   └── feedback.hpp
│   │
│   ├── command/
│   │   ├── manual_command_mapper.hpp
│   │   ├── command_manager.hpp
│   │   └── safety_manager.hpp
│   │
│   ├── gimbal/
│   │   └── yaw_gimbal.hpp
│   │
│   └── swerve/
│       ├── swerve_types.hpp
│       ├── swerve_kinematics.hpp
│       ├── swerve_module.hpp
│       └── swerve_chassis.hpp
│
lib/
├── communication/
│   ├── remote/
│   └── referee/
│
└── robotics/
    ├── command/
    ├── gimbal/
    └── swerve/

application/
├── boards/
│   └── <board>.overlay
├── src/
│   ├── main.cpp
│   ├── message_bus.cpp
│   ├── remote_task.cpp
│   ├── referee_task.cpp
│   ├── command_task.cpp
│   ├── gimbal_task.cpp
│   └── chassis_task.cpp
├── CMakeLists.txt
└── prj.conf

samples/
├── communication/
│   ├── dr16_decoder/
│   └── referee_parser/
│
├── robotics/
│   ├── yaw_gimbal/
│   └── swerve_kinematics/
```

---

# 5. 全局编码约束

这些规则对所有后续代码生效。

## 5.1 单位

公开接口强制使用 SI：

```text
角度      rad
角速度    rad/s
线速度    m/s
加速度    m/s²
电流      A
力矩      N·m
功率      W
时间      ms / s，字段名必须带单位
```

禁止：

```cpp
float yaw;       // 不知道是度还是 rad
float speed;     // 不知道是 rpm 还是 rad/s
float power;     // 单位不明确
```

必须：

```cpp
float yaw_rad;
float velocity_rad_s;
float chassis_power_w;
```

## 5.2 错误码

所有可能失败的接口统一：

```text
成功：0
失败：负 errno
```

推荐：

```text
-EINVAL   输入非法
-ENODEV   设备不存在
-EAGAIN   数据暂不可用
-ETIMEDOUT 数据超时
-EACCES   生命周期不允许
-ENOTSUP  能力不支持
-EIO      I/O / 总线错误
-EFAULT   内部状态异常
```

禁止自创：

```cpp
return -10086;
```

## 5.3 动态内存

实时控制路径禁止：

```text
new
delete
malloc
free
std::vector 动态扩容
std::string 临时拼接
```

优先：

```text
std::array
固定长度 C 数组
栈对象
静态对象
```

## 5.4 线程规则

算法类：

```text
不创建线程
不 sleep
不读系统时间
不直接访问 zbus
不直接 LOG
```

线程、周期和消息转发属于：

```text
application/
```

---

# 6. 公共消息契约

新增：

```text
include/robotics/messages/common.hpp
include/robotics/messages/remote.hpp
include/robotics/messages/referee.hpp
include/robotics/messages/command.hpp
include/robotics/messages/feedback.hpp
```

namespace：

```cpp
namespace skywalker::robotics
```

---

## 6.1 common.hpp

建议完整接口：

```cpp
#pragma once

#include <cstdint>

namespace skywalker::robotics {

enum class ControlSource : std::uint8_t {
    None = 0,
    Remote,
    KeyboardMouse,
    Vision,
    Auto,
};

enum class SafetyState : std::uint8_t {
    Boot = 0,
    Safe,
    Ready,
    Fault,
    EmergencyStop,
};

struct MessageStamp {
    std::uint64_t timestamp_ms = 0;
    std::uint32_t sequence = 0;
    bool valid = false;
};

}
```

约束：

```text
timestamp_ms
= 该状态真正被更新的时间
不是“被消费者读取的时间”

sequence
= 每次成功更新 +1

valid
= 是否至少收到过一帧有效数据
```

---

# 7. DR16 / DT7 遥控器层

遥控器拆成：

```text
UART transport
      ↓
DR16 Decoder
      ↓
RemoteState
      ↓
ManualCommandMapper
```

绝对禁止：

```text
UART callback
    ↓
直接 motor.update()
```

---

## 7.1 RemoteState

`include/robotics/messages/remote.hpp`

```cpp
#pragma once

#include <cstdint>
#include <robotics/messages/common.hpp>

namespace skywalker::robotics {

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

    bool pressed(std::uint16_t mask) const {
        return (bits & mask) != 0u;
    }
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

}
```

这里保存的是：

```text
“遥控器状态”
```

而不是：

```text
vx
yaw
friction_on
```

---

## 7.2 Dr16Decoder

文件：

```text
include/communication/remote/dr16_decoder.hpp
lib/communication/remote/dr16_decoder.cpp
```

完整接口约束：

```cpp
#pragma once

#include <cstddef>
#include <cstdint>

#include <robotics/messages/remote.hpp>

namespace skywalker::communication::remote {

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
    bool validateChannels(...) const;
    static robotics::RcSwitch decodeSwitch(std::uint8_t value);

    Config config_{};
    std::uint32_t valid_frames_ = 0;
    std::uint32_t invalid_frames_ = 0;
    std::uint32_t sequence_ = 0;
};

}
```

### decodeFrame 语义

成功：

```text
return 0
out 完整更新
out.stamp.valid = true
out.online = true
sequence +1
```

失败：

```text
return -EINVAL / -EBADMSG
out 不得被部分修改
invalid_frames +1
```

必须：

```text
先解到局部临时对象
全部字段通过 sanity check
最后一次性赋值给 out
```

禁止：

```text
先写 out.right_x
后发现 switch 非法
然后 return
```

---

# 8. RemoteService：UART 与 decoder 的边界

新增：

```text
include/communication/remote/remote_service.hpp
lib/communication/remote/remote_service.cpp
```

推荐接口：

```cpp
#pragma once

#include <cstddef>
#include <cstdint>

#include <zephyr/device.h>

#include <communication/remote/dr16_decoder.hpp>

namespace skywalker::communication::remote {

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
    int processCandidateFrame(
        const std::uint8_t *frame,
        std::uint64_t now_ms);

    const device *uart_ = nullptr;
    Dr16Decoder decoder_;
    Config config_{};

    robotics::RemoteState latest_{};

    std::uint8_t rx_window_[Dr16Decoder::kFrameSize * 2]{};
    std::size_t rx_size_ = 0;
};

}
```

## 8.1 ISR / UART callback 规则

不要在 ISR 内：

```text
复杂解析
zbus 长时间阻塞 publish
LOG 大量输出
业务模式判断
```

推荐：

```text
UART callback
    ↓
写固定 RX buffer / FIFO
    ↓
remote_task thread
    ↓
RemoteService::processBytes()
```

---

# 9. 手动控制映射层

新增：

```text
include/robotics/command/manual_command_mapper.hpp
lib/robotics/command/manual_command_mapper.cpp
```

此层才把：

```text
摇杆
```

变成：

```text
vx / vy / wz / yaw rate
```

---

## 9.1 OperatorIntent

加入 `command.hpp`：

```cpp
#pragma once

#include <cstdint>

#include <robotics/messages/common.hpp>

namespace skywalker::robotics {

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

}
```

注意这里使用：

```text
[-1, +1]
```

归一化请求。

此处暂时不填真实 m/s。

原因：

```text
遥控器只表达人的意图强度
具体最大速度属于机器人参数
```

---

## 9.2 ManualCommandMapper

```cpp
#pragma once

#include <robotics/messages/command.hpp>
#include <robotics/messages/remote.hpp>

namespace skywalker::robotics::command {

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
    float normalizeChannel(std::int16_t raw) const;
    float applyDeadband(float value) const;

    Config config_{};
};

}
```

### 推荐 V1 模式

先固定简单逻辑：

```text
左拨杆 Down
    -> SAFE

左拨杆 Middle
    -> MANUAL

左拨杆 Up
    -> AUTO
```

先不要实现复杂多级快捷键。

---

# 10. 最终子系统命令

CommandManager 最终输出真正物理量。

---

## 10.1 ChassisCommand

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

---

## 10.2 GimbalCommand

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

当前只有小 Yaw：

```text
pitch 字段保留
但 V1 不实现 pitch
```

---

## 10.3 ShooterCommand

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

现在只定义协议，不实现 Shooter。

---

## 10.4 RobotCommand

```cpp
struct RobotCommand {
    ChassisCommand chassis{};
    GimbalCommand gimbal{};
    ShooterCommand shooter{};

    MessageStamp stamp{};
};
```

---

# 11. SafetyManager

安全状态不要散落在：

```text
remote_task
gimbal_task
chassis_task
referee_task
```

统一放入 SafetyManager。

文件：

```text
include/robotics/command/safety_manager.hpp
lib/robotics/command/safety_manager.cpp
```

---

## 11.1 Safety 输入输出

```cpp
namespace skywalker::robotics::command {

struct SafetyInputs {
    bool remote_online = false;
    bool referee_online = false;

    bool gimbal_healthy = false;
    bool chassis_healthy = false;

    bool referee_gimbal_output_enabled = true;
    bool referee_chassis_output_enabled = true;
    bool referee_shooter_output_enabled = true;

    bool emergency_stop_requested = false;

    std::uint64_t now_ms = 0;
};

struct SafetyDecision {
    SafetyState state = SafetyState::Boot;

    bool allow_gimbal = false;
    bool allow_chassis = false;
    bool allow_shooter = false;

    bool force_zero = true;

    std::uint32_t fault_mask = 0;
};

class SafetyManager {
public:
    struct Config {
        bool require_remote = true;

        // 实验室测试阶段允许没有裁判系统。
        bool require_referee_for_output = false;
    };

    explicit SafetyManager(const Config &config);

    int reset();

    int evaluate(
        const SafetyInputs &input,
        SafetyDecision &out);

private:
    Config config_{};
    SafetyState state_ = SafetyState::Boot;
};

}
```

## 11.2 V1 安全规则

优先级必须是：

```text
Emergency Stop
>
硬件 Fault
>
Remote Offline
>
Referee Output Disable
>
普通命令
```

实验室阶段：

```text
require_referee_for_output = false
```

比赛版本：

```text
require_referee_for_output = true
```

这样没插裁判系统时仍可调小 Yaw。

---

# 12. CommandManager

文件：

```text
include/robotics/command/command_manager.hpp
lib/robotics/command/command_manager.cpp
```

CommandManager 不读 UART、不直接看 CAN。

---

## 12.1 Config

```cpp
namespace skywalker::robotics::command {

class CommandManager {
public:
    struct Config {
        float max_chassis_vx_m_s = 3.0f;
        float max_chassis_vy_m_s = 3.0f;
        float max_chassis_wz_rad_s = 6.0f;

        float max_gimbal_yaw_rate_rad_s = 3.0f;
        float max_gimbal_pitch_rate_rad_s = 2.0f;

        std::uint32_t command_timeout_ms = 100;
    };

    explicit CommandManager(const Config &config);

    int reset();

    int step(
        const OperatorIntent &manual,
        const SafetyDecision &safety,
        std::uint64_t now_ms,
        RobotCommand &out);

private:
    RobotCommand makeSafeCommand(std::uint64_t now_ms) const;

    Config config_{};
    std::uint32_t sequence_ = 0;
};

}
```

V1 先只实现：

```text
Remote manual
+
Safety
```

后续再扩展：

```cpp
step(
    manual,
    auto_command,
    vision_command,
    safety,
    referee,
    now,
    out);
```

不要第一版就把所有来源塞进去。

---

# 13. 小 Yaw 云台——当前第一优先级实机子系统

这是当前最值得先完成的机器人闭环。

目标链路：

```text
DR16
 ↓
RemoteState
 ↓
OperatorIntent
 ↓
CommandManager
 ↓
GimbalCommand
 ↓
YawGimbal
 ↓
PositionMotor
 ↓
DjiMotorBackend / DmMotorBackend
 ↓
Motor
```

---

# 14. YawGimbal 接口

新增：

```text
include/robotics/gimbal/yaw_gimbal.hpp
lib/robotics/gimbal/yaw_gimbal.cpp
```

V1 直接复用现有：

```cpp
skywalker::control::PositionMotor
```

不要重新写位置环。

---

## 14.1 配置

```cpp
#pragma once

#include <cstdint>

#include <control/position_motor.hpp>
#include <robotics/messages/command.hpp>

namespace skywalker::robotics::gimbal {

enum class YawTopology : std::uint8_t {
    Continuous = 0,
    Limited,
};

struct YawGimbalConfig {
    YawTopology topology = YawTopology::Continuous;

    // Limited 模式使用。
    float min_angle_rad = -3.1415926f;
    float max_angle_rad =  3.1415926f;

    float max_rate_rad_s = 3.0f;

    // 操作手松杆时是否立即 hold 当前目标。
    bool hold_on_zero_rate = true;
};

struct YawGimbalTelemetry {
    GimbalMode mode = GimbalMode::Disabled;

    float requested_rate_rad_s = 0.0f;
    double target_angle_rad = 0.0;

    double measured_position_rad = 0.0;
    float measured_absolute_rad = 0.0f;
    float measured_velocity_rad_s = 0.0f;

    float effort_command = 0.0f;

    bool healthy = false;
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
        float dt_s);

    int stop();

    const YawGimbalTelemetry &telemetry() const;

private:
    int enterHold();
    int updateRate(float rate_rad_s, float dt_s);
    int updateAbsolute(float target_rad);
    double constrainTarget(double target_rad) const;

    control::PositionMotor &motor_;
    YawGimbalConfig config_{};

    YawGimbalTelemetry telemetry_{};

    GimbalMode last_mode_ = GimbalMode::Disabled;

    double target_angle_rad_ = 0.0;
    bool initialized_ = false;
};

}
```

---

# 15. YawGimbal 生命周期

## 15.1 validate()

检查：

```text
max_rate > 0

Limited:
    min < max
    min/max finite
```

不要检查 Motor 硬件状态。

---

## 15.2 begin()

流程：

```text
motor.begin()
    ↓
取得 motor.telemetry()
    ↓
确认 feedback valid
    ↓
target_angle = 当前实际角度
    ↓
进入 Hold
```

如果使用：

```cpp
PositionReference::AbsoluteNearest
```

则必须要求底层：

```text
FeedbackAbsolutePosition
```

---

## 15.3 Disabled

收到：

```cpp
GimbalMode::Disabled
```

V1 处理：

```text
motor.stop()
```

注意：

现有 `PositionMotor` 生命周期是：

```text
一次 begin
然后运行
stop 后不设计为重复 begin
```

因此应用层不应该通过频繁 Disabled/Enabled 来控制日常模式。

推荐区分：

```text
Safety stop
= 真正 motor.stop()

普通“锁住不动”
= Hold
```

也就是说，正常运行期间：

```text
GimbalMode::Hold
```

才是“停止转动”。

---

## 15.4 Hold

第一次进入 Hold：

```text
target_angle = 当前有效角度
```

后续：

```text
保持 target
```

---

## 15.5 Rate

输入：

```text
yaw_rate_rad_s
```

每周期：

```text
rate = clamp(rate, ±max_rate)

target += rate * dt
```

然后：

```text
motor.update(target)
```

对于 Continuous：

```text
target 可以连续累加
```

对于 Limited：

```text
clamp(min, max)
```

---

## 15.6 AbsoluteAngle

输入：

```text
yaw_target_rad
```

如果使用：

```cpp
PositionReference::AbsoluteNearest
```

则 `PositionMotor` 自己负责：

```text
固定零点单圈目标
    ↓
最近连续目标
```

YawGimbal 不重新写 shortest path。

---

# 16. 当前小 Yaw 推荐配置

现有 `samples/motor/dji_position_control` 已经证明：

```text
PositionMotor
+
AbsoluteNearest
+
GM6020 fixed-zero feedback
```

这条链可用于位置测试。

因此 V1 小 Yaw 应优先复用 sample 已经跑通的参数作为初始值。

禁止在“架构迁移”和“重新调 PID”两个任务中同时大改参数。

流程：

```text
先机械迁移
确认效果与现有 sample 接近
然后再单独调参
```

---

# 17. 底盘：当前只建立基础运动框架

底盘当前不要求真实 8 电机接入。

先完成：

```text
ChassisCommand
        ↓
SwerveKinematics
        ↓
4 × ModuleTarget
```

然后做纯算法 sample。

实际：

```text
反馈
电流控制
CAN
8 电机
功率限制
```

放到后续阶段。

---

# 18. Swerve 基础类型

新增：

```text
include/robotics/swerve/swerve_types.hpp
```

```cpp
#pragma once

#include <array>

namespace skywalker::robotics::swerve {

enum class ModuleId : std::uint8_t {
    FrontLeft = 0,
    FrontRight,
    RearLeft,
    RearRight,
};

struct ModuleLocation {
    float x_m = 0.0f;
    float y_m = 0.0f;
};

struct ModuleTarget {
    float angle_rad = 0.0f;
    float wheel_velocity_m_s = 0.0f;
};

using ModuleTargets = std::array<ModuleTarget, 4>;

}
```

坐标系强制：

```text
+x = 车体前方
+y = 车体左方
+z = 向上

wz > 0
= 从上向下看逆时针
```

---

# 19. SwerveKinematics

文件：

```text
include/robotics/swerve/swerve_kinematics.hpp
lib/robotics/swerve/swerve_kinematics.cpp
```

```cpp
#pragma once

#include <array>

#include <robotics/messages/command.hpp>
#include <robotics/swerve/swerve_types.hpp>

namespace skywalker::robotics::swerve {

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

}
```

---

# 20. Swerve 运动学公式

模块位置：

```text
(x_i, y_i)
```

底盘命令：

```text
vx
vy
wz
```

单轮：

```text
wheel_vx = vx - wz * y_i
wheel_vy = vy + wz * x_i

speed = hypot(wheel_vx, wheel_vy)
angle = atan2(wheel_vy, wheel_vx)
```

如果：

```text
speed < stationary_epsilon
```

则：

```text
speed = 0
angle = last_angle
```

不要：

```text
atan2(0,0) -> 0
```

然后让四个舵轮每次停车全部回正。

---

# 21. 最大轮速统一缩放

如果：

```text
max(speed_i) > max_wheel_velocity
```

统一：

```text
scale = max_wheel_velocity / max_speed

所有 speed_i *= scale
```

禁止：

```text
只 clamp 超限的那一个轮
```

因为这会破坏机器人速度向量比例。

---

# 22. SwerveModule：第二阶段底盘类

现在可以先定义接口，实际控制可在底盘硬件到位后实现。

```text
include/robotics/swerve/swerve_module.hpp
lib/robotics/swerve/swerve_module.cpp
```

推荐：

```cpp
#pragma once

#include <control/motor_position.h>
#include <control/motor_velocity.h>

#include <robotics/swerve/swerve_types.hpp>

namespace skywalker::robotics::swerve {

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

private:
    Config config_{};

    control_motor_position_state steer_state_{};
    control_motor_velocity_state drive_state_{};

    bool initialized_ = false;
};

}
```

这里继续保持：

```text
Module 不知道 device*
Module 不知道 CAN
Module 不知道 GM6020 / M3508
Module 不知道 dji::Bus
```

---

# 23. 舵轮 180° 优化

当前舵角：

```text
current
```

目标：

```text
target
```

计算：

```text
error = shortest_angle(target, current)
```

如果：

```text
abs(error) > pi/2
```

改成：

```text
target += pi
wheel_velocity *= -1
```

然后重新 wrap。

V1：

```text
> pi/2 才翻转
== pi/2 不翻
```

先不要增加 hysteresis。

---

# 24. SwerveChassis

```text
include/robotics/swerve/swerve_chassis.hpp
lib/robotics/swerve/swerve_chassis.cpp
```

接口：

```cpp
#pragma once

#include <array>

#include <robotics/swerve/swerve_kinematics.hpp>
#include <robotics/swerve/swerve_module.hpp>

namespace skywalker::robotics::swerve {

struct ChassisFeedback {
    std::array<ModuleFeedback, 4> module{};
};

struct ChassisOutput {
    ModuleTargets target{};
    std::array<ModuleOutput, 4> module{};
};

class SwerveChassis {
public:
    struct Config {
        SwerveKinematics::Config kinematics{};
        std::array<SwerveModule::Config, 4> modules{};
    };

    explicit SwerveChassis(const Config &config);

    int validate() const;

    int reset(const ChassisFeedback &feedback);

    int step(
        const ChassisCommand &command,
        const ChassisFeedback &feedback,
        float dt_s,
        ChassisOutput &out);

private:
    SwerveKinematics kinematics_;
    std::array<SwerveModule, 4> modules_;

    bool initialized_ = false;
};

}
```

---

# 25. 非常重要：底盘不能用 8 个独立 DjiMotorBackend

当前：

```cpp
DjiMotorBackend
```

明确是：

```text
单电机
独占 DJI Bus
```

因为 DJI 同组电机使用：

```text
一个 CAN command frame
多个 slot
```

如果同一个 CAN 上：

```text
M3508 id1
M3508 id2
M3508 id3
M3508 id4
```

每个都单独建：

```cpp
DjiMotorBackend
```

然后分别：

```text
flush()
```

会出现组帧覆盖风险。

因此：

```text
小 Yaw 单电机
-> 可以使用 DjiMotorBackend

4 steer + 4 drive
-> 不能使用 8 个 DjiMotorBackend
```

---

# 26. 未来底盘硬件层

真实底盘接入时新增：

```text
application/src/chassis_hardware.hpp
application/src/chassis_hardware.cpp
```

V1 先绑定 DJI。

```cpp
class DjiChassisHardware {
public:
    struct Devices {
        const device *can = nullptr;

        std::array<const device *, 4> steer{};
        std::array<const device *, 4> drive{};
    };

    explicit DjiChassisHardware(const Devices &devices);

    int init();

    int waitForFeedback(std::uint32_t timeout_ms);

    int read(
        skywalker::robotics::swerve::ChassisFeedback &out);

    int arm();

    int apply(
        const skywalker::robotics::swerve::ChassisOutput &output);

    int stop();

private:
    Devices devices_{};

    skywalker::motor::dji::Bus bus_{};

    skywalker::motor::dji::FlushReport report_{};

    bool initialized_ = false;
    bool armed_ = false;
};
```

## 26.1 apply()

顺序必须：

```text
setCurrent steer ×4
setCurrent drive ×4
        ↓
一次 bus.flush()
```

禁止：

```text
setCurrent
flush
setCurrent
flush
...
```

---

# 27. 裁判系统层

裁判系统严格拆成：

```text
UART bytes
   ↓
Frame Parser
   ↓
Protocol Message
   ↓
RefereeState
   ↓
Policy / Limiter
```

不要：

```text
Parser
 ↓
直接修改底盘 current
```

---

# 28. RefereeState

文件：

```text
include/robotics/messages/referee.hpp
```

不要第一版就把裁判系统所有协议字段抄进公共状态。

只抽机器人控制真正需要的稳定语义。

```cpp
#pragma once

#include <cstdint>
#include <robotics/messages/common.hpp>

namespace skywalker::robotics {

struct RefereeGameState {
    std::uint8_t game_type = 0;
    std::uint8_t game_progress = 0;

    std::uint16_t stage_remaining_s = 0;
};

struct RefereeRobotState {
    std::uint8_t robot_id = 0;

    std::uint16_t hp = 0;
    std::uint16_t max_hp = 0;

    bool chassis_output_enabled = false;
    bool gimbal_output_enabled = false;
    bool shooter_output_enabled = false;
};

struct RefereePowerState {
    float chassis_power_w = 0.0f;
    float chassis_power_limit_w = 0.0f;
    float buffer_energy_j = 0.0f;
};

struct RefereeShooterState {
    float cooling_value = 0.0f;
    float cooling_limit = 0.0f;
    float speed_limit_m_s = 0.0f;
};

struct RefereeState {
    RefereeGameState game{};
    RefereeRobotState robot{};
    RefereePowerState power{};
    RefereeShooterState shooter{};

    MessageStamp stamp{};
    bool online = false;
};

}
```

协议字段发生变化时：

```text
referee_protocol.hpp
referee_parser.cpp
```

修改。

机器人控制层：

```text
RefereeState
```

尽量保持稳定。

---

# 29. RefereeParser

新增：

```text
include/communication/referee/referee_parser.hpp
lib/communication/referee/referee_parser.cpp
```

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

    RefereeParser();

    int reset();

    int consume(
        const std::uint8_t *data,
        std::size_t size,
        std::uint64_t timestamp_ms);

    const robotics::RefereeState &state() const;

    const Stats &stats() const;

private:
    enum class ParseState : std::uint8_t {
        WaitSof,
        Header,
        Command,
        Payload,
        Tail,
    };

    int processFrame(
        const std::uint8_t *frame,
        std::size_t size,
        std::uint64_t timestamp_ms);

    int dispatch(
        std::uint16_t command_id,
        const std::uint8_t *payload,
        std::size_t size,
        robotics::RefereeState &next);

    ParseState parse_state_ = ParseState::WaitSof;

    robotics::RefereeState state_{};
    Stats stats_{};

    std::uint8_t buffer_[512]{};
    std::size_t buffered_ = 0;
    std::uint32_t sequence_ = 0;
};
```

---

# 30. 裁判解析原则

必须：

```text
CRC8 header
CRC16 whole frame
payload length validation
little-endian explicit decode
unknown command safely skip
```

禁止：

```cpp
auto *msg =
    reinterpret_cast<const PackedStruct *>(payload);
```

除非执行 AI 能严格证明：

```text
packing
alignment
endianness
compiler behavior
```

V1 推荐：

```text
显式 load_le16
显式 load_le32
memcpy float
```

避免未对齐访问。

---

# 31. 裁判协议版本隔离

当前项目面向 2026 赛季。

协议相关常量集中：

```text
referee_protocol.hpp
```

例如：

```cpp
namespace skywalker::communication::referee::protocol2026 {
...
}
```

不要把：

```text
cmd id
payload layout
```

写进：

```text
SafetyManager
CommandManager
SwerveChassis
YawGimbal
```

协议升级时只改 communication。

---

# 32. 裁判系统如何影响控制

正确：

```text
Remote:
    我要 vx=3m/s

Referee:
    当前功率限制 80W

Command:
    仍然是 vx=3m/s

Chassis power policy:
    在当前功率预算内尽力实现
```

错误：

```cpp
if (power_limit < 80)
    remote.vx *= 0.5f;
```

裁判信息是：

```text
Constraint
```

不是：

```text
Human Intent
```

---

# 33. 底盘功率限制接口

后续新增：

```text
include/robotics/chassis/chassis_power_limiter.hpp
```

建议：

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
    struct Config {
        float minimum_scale = 0.0f;
        float margin_w = 5.0f;
    };

    explicit ChassisPowerLimiter(const Config &config);

    int reset();

    int step(
        const ChassisPowerInput &input,
        float dt_s,
        ChassisPowerDecision &out);
};
```

V1 可以先不实现复杂预测器。

先：

```text
接口正确
策略简单
```

后续再换：

```text
功率模型
超级电容
预测控制
```

---

# 34. Zephyr zbus 消息系统

推荐使用 Zephyr 自带：

```text
zbus
```

而不是再自己写一套全局 MessageCenter。

原因：

```text
项目已经是 Zephyr
zbus 原生支持 channel / publish / subscriber
适合线程间低成本状态交换
```

但是：

```text
robotics/*
communication decoder/*
```

禁止包含：

```cpp
#include <zephyr/zbus/zbus.h>
```

zbus 只应该存在于：

```text
application/src/message_bus.*
application task
```

这样算法仍可脱离 RTOS 测试。

---

# 35. 推荐 zbus channels

`application/src/message_bus.cpp`

定义：

```text
remote_state_chan
referee_state_chan

operator_intent_chan
robot_command_chan

gimbal_feedback_chan
chassis_feedback_chan

system_health_chan
```

---

## 35.1 Latest State channel

这些数据只关心：

```text
最新状态
```

因此适合普通 channel：

```text
RemoteState
RefereeState
RobotCommand
GimbalFeedback
ChassisFeedback
```

如果连续发布两次，消费者只看到最新值通常没问题。

---

## 35.2 Event 不要塞 Latest State

例如：

```text
单发一次
保存零点
清故障
确认按键边沿
```

属于离散事件。

后续需要时用：

```text
k_msgq
k_fifo
zbus message subscriber
```

不要把：

```cpp
bool fire_once;
```

在 100Hz channel 里发布 20 次然后希望消费者“只触发一次”。

---

# 36. Application 线程模型

V1 推荐保持简单。

```text
remote RX
    UART event driven
    ↓
remote_task
    解析 / publish RemoteState

referee RX
    UART event driven
    ↓
referee_task
    解析 / publish RefereeState

command_task
    100 Hz
    read RemoteState
    read RefereeState
    ManualCommandMapper
    SafetyManager
    CommandManager
    publish RobotCommand

gimbal_task
    200 Hz / 5 ms
    read latest RobotCommand
    YawGimbal::update()

chassis_task
    初期 100~200 Hz
    仅纯算法 / 后续实车
```

当前小 Yaw 已有 5 ms 控制 sample，因此：

```text
gimbal_task V1 = 5 ms
```

最合理。

---

# 37. Application 初始化顺序

最终 `main.cpp` 推荐：

```text
1. device ready 检查

2. 初始化 message bus

3. 初始化 RemoteService

4. 初始化 RefereeService
   - 实验室阶段允许不存在

5. 构造 DjiMotorBackend / PositionMotor

6. 构造 YawGimbal

7. YawGimbal::validate()

8. YawGimbal::begin()

9. 初始化 ManualCommandMapper

10. 初始化 SafetyManager

11. 初始化 CommandManager

12. 启动 command / remote / referee tasks

13. 默认 RobotCommand = Safe / Hold

14. 进入调试
```

避免：

```text
刚上电
remote 还没收到数据
电机直接跑向硬编码目标
```

---

# 38. 小 Yaw 当前最小可运行闭环

第一版不需要裁判系统。

目标：

```text
DR16
 ↓
RemoteState
 ↓
ManualCommandMapper
 ↓
CommandManager
 ↓
GimbalCommand::Rate
 ↓
YawGimbal
 ↓
PositionMotor
```

推荐遥控操作：

```text
左拨杆 Down
    = 安全，不允许运动

左拨杆 Middle
    = 手动

右摇杆水平
    = yaw rate
```

手松开：

```text
yaw rate = 0
```

`YawGimbal`：

```text
保持最后目标角
```

这比“摇杆直接给电流”合理得多。

---

# 39. 小 Yaw 实机测试顺序

必须架空或保证机械安全。

## Test 1：PositionMotor 基线

先确认现有：

```text
dji_position_control
```

仍然工作。

不通过：

```text
不要继续应用层
```

---

## Test 2：YawGimbal 固定目标

暂时不用遥控器：

```text
0
+0.2 rad
-0.2 rad
```

CommandManager 可以先硬编码测试命令。

确认：

```text
方向
零点
limit
hold
```

---

## Test 3：Rate 模式

固定：

```text
+0.3 rad/s
0
-0.3 rad/s
```

检查：

```text
目标积分方向
松开后 hold
```

---

## Test 4：DR16 decoder

电机保持：

```text
Hold
```

只打印：

```text
RemoteState
```

确认：

```text
摇杆中心
摇杆正负
拨杆
掉线
```

---

## Test 5：DR16 -> Yaw

最后连接：

```text
RemoteState
    ↓
GimbalCommand
```

先限制：

```text
max yaw rate <= 0.5 rad/s
current limit 低
```

确认安全后再提高。

---

# 40. Remote Offline 行为

RemoteService：

```text
now - last_valid_frame > offline_timeout
```

则：

```text
online = false
```

CommandManager：

```text
remote offline
    ↓
OperatorIntent::Safe
```

Safety：

```text
allow_gimbal = false
```

对于已经 begin 的 PositionMotor：

V1 推荐：

```text
remote 短暂掉线
    -> Hold 当前角度
```

而不是立刻：

```text
motor.stop()
```

原因：

小云台失能后可能自然掉落/撞限位。

真正：

```text
EmergencyStop / MotorFault
```

才：

```text
motor.stop()
```

因此 SafetyDecision 可以后续进一步区分：

```text
allow_motion
allow_hold
force_disable
```

第一版如果实现这一点，建议扩展为：

```cpp
enum class OutputAction {
    Disable,
    Hold,
    Active,
};
```

但不要为了 V1 过度复杂化。

---

# 41. Devicetree 建议

不要直接继续往 board 基础 DTS 塞机器人装配信息。

机器人级绑定放：

```text
application/boards/<board>.overlay
```

推荐 aliases：

```dts
/ {
    aliases {
        remote-uart = &usart3;
        referee-uart = &usart1;
        telemetry-uart = &usart6;

        yaw-motor = &yaw_motor;
    };
};
```

具体 UART 需要根据真实接线修改。

---

# 42. DR16 UART 参数风险

当前 Type-C C 板基础 DTS 中：

```text
USART3
100000 baud
8 data bits
parity none
1 stop
```

执行 AI 不得因为“波特率看起来对”就假设 DR16 已经完全配置正确。

必须根据你真实 DR16/DBUS 接线与协议确认：

```text
baud
parity
stop bits
RX inversion / hardware path
```

常见 DBUS 配置与普通 8N1 不同。

因此第一版 DR16 工作前：

```text
先示波器/逻辑分析仪确认
再确定 overlay
```

不要为了测试遥控器破坏基础 board DTS。

---

# 43. Referee UART

裁判系统一般与 DR16 分开 UART。

application overlay 负责：

```text
referee-uart
```

协议层不应该写死：

```cpp
DEVICE_DT_GET(DT_NODELABEL(usart1))
```

而应该：

```cpp
DT_ALIAS(referee_uart)
```

这样换板不用改 parser。

---

# 44. CMake / Kconfig 规划

新增顶层：

```text
CONFIG_SKYWALKER_LIB_COMMUNICATION
CONFIG_SKYWALKER_LIB_ROBOTICS
```

再拆：

```text
CONFIG_SKYWALKER_REMOTE_DR16
CONFIG_SKYWALKER_REFEREE
CONFIG_SKYWALKER_ROBOTICS_COMMAND
CONFIG_SKYWALKER_ROBOTICS_GIMBAL
CONFIG_SKYWALKER_ROBOTICS_SWERVE
```

不要建立几十个极细 Kconfig。

V1 够用即可。

---

# 45. 推荐线程/调度边界

不要：

```text
Remote callback
    ↓
Gimbal.update()
```

正确：

```text
Remote producer
       ↓
    channel
       ↓
Command producer
       ↓
    channel
       ↓
Gimbal control thread
```

这样：

```text
遥控频率
≠
控制环频率
```

控制周期不会跟 UART 抖动。

---

# 46. Gimbal Feedback

新增：

```cpp
struct GimbalFeedback {
    double yaw_position_rad = 0.0;
    float yaw_absolute_rad = 0.0f;
    float yaw_velocity_rad_s = 0.0f;

    double yaw_target_rad = 0.0;

    GimbalMode mode = GimbalMode::Disabled;

    bool healthy = false;

    MessageStamp stamp{};
};
```

gimbal_task：

```text
YawGimbal update
    ↓
构造 GimbalFeedback
    ↓
publish
```

CommandManager 后续如果要：

```text
底盘跟随云台
```

只能订阅：

```text
GimbalFeedback
```

不要直接访问：

```text
GM6020 feedback
```

---

# 47. 底盘跟随云台的未来正确关系

未来：

```text
Remote:
    forward / strafe

GimbalFeedback:
    yaw relative to chassis

CommandManager:
    decide CHASSIS_FOLLOW_GIMBAL

ChassisCommand:
    vx
    vy
    wz
```

不要让：

```text
SwerveChassis
```

直接读取：

```text
Gimbal motor encoder
```

底盘只接受最终：

```cpp
ChassisCommand
```

---

# 48. 视觉系统未来接入

定义：

```cpp
struct VisionCommand {
    bool valid = false;

    float yaw_target_rad = 0.0f;
    float pitch_target_rad = 0.0f;

    MessageStamp stamp{};
};
```

Vision 也只进入：

```text
CommandManager
```

然后：

```text
视觉控制
遥控器控制
自动控制
```

才能统一仲裁。

禁止：

```text
视觉串口线程
    ↓
直接 motor.update()
```

---

# 49. 自动控制未来接入

同样定义：

```cpp
struct AutoCommand {
    ChassisCommand chassis{};
    GimbalCommand gimbal{};

    MessageStamp stamp{};
};
```

CommandManager 根据：

```text
OperatorMode
Safety
Auto freshness
```

选择来源。

---

# 50. CommandManager 最终扩展接口

等 V1 跑通后可扩成：

```cpp
struct CommandInputs {
    OperatorIntent manual{};
    AutoCommand autonomous{};
    VisionCommand vision{};

    SafetyDecision safety{};
    RefereeState referee{};
    GimbalFeedback gimbal{};

    std::uint64_t now_ms = 0;
};

class CommandManager {
public:
    int step(
        const CommandInputs &input,
        RobotCommand &out);
};
```

第一版禁止直接上这个最终复杂形态。

先把：

```text
Manual
+
Safety
```

跑通。

---

# 51. 事务化状态更新

所有有状态算法都应遵守：

```cpp
State next = state_;

int ret = calculate(next, ...);

if (ret < 0)
    return ret;

state_ = next;
output = next_output;
```

也就是说：

```text
step 失败
```

不能让内部状态更新了一半。

尤其：

```text
DR16 decoder
Referee parser
CommandManager
SwerveModule
```

都应注意。

---

# 52. Freshness 统一规则

每个外部来源必须有：

```text
timestamp
valid
online
```

不要只用：

```text
“变量是不是非零”
```

判断数据有效。

推荐：

```cpp
inline bool isFresh(
    const MessageStamp &stamp,
    std::uint64_t now_ms,
    std::uint32_t timeout_ms)
{
    return stamp.valid &&
           now_ms >= stamp.timestamp_ms &&
           now_ms - stamp.timestamp_ms <= timeout_ms;
}
```

---

# 53. 不允许跨层访问清单

## Remote 不允许

```text
PositionMotor
VelocityMotor
setCurrent
dji::Bus
Swerve
```

## Referee 不允许

```text
遥控器变量
PID
motor current
```

## Command 不允许

```text
CAN ID
UART
encoder tick
```

## Gimbal 不允许

```text
裁判协议 cmd id
DR16 byte layout
```

## Swerve 不允许

```text
zbus
DR16
Referee UART
GM6020 型号判断
```

---

# 54. 第一阶段施工：消息契约

新增：

```text
include/robotics/messages/common.hpp
include/robotics/messages/remote.hpp
include/robotics/messages/referee.hpp
include/robotics/messages/command.hpp
include/robotics/messages/feedback.hpp
```

本阶段：

```text
只定义结构体
不写 UART
不写线程
不写 zbus
```

验收：

```text
[ ] 所有字段单位明确
[ ] 没有 driver 类型
[ ] 没有 Zephyr device*
[ ] 没有 zbus include
[ ] 可被普通 C++ 单元编译
```

提交：

```text
feat(robotics): define application message contracts
```

---

# 55. 第二阶段施工：YawGimbal

新增：

```text
include/robotics/gimbal/yaw_gimbal.hpp
lib/robotics/gimbal/yaw_gimbal.cpp
```

新增 sample：

```text
samples/robotics/yaw_gimbal/
```

先：

```text
硬编码 GimbalCommand
```

不要等 DR16。

验收：

```text
[ ] begin 后初始 target = 当前角
[ ] Hold 不跳
[ ] Rate 正负方向正确
[ ] Rate = 0 保持
[ ] AbsoluteNearest 正常
[ ] Limited 模式不越软件限位
[ ] Fault 时 stop
```

提交：

```text
feat(gimbal): add reusable single-axis yaw controller
```

---

# 56. 第三阶段施工：DR16 Decoder

只实现：

```text
18 byte
    ↓
RemoteState
```

新增：

```text
samples/communication/dr16_decoder/
```

可使用：

```text
固定 byte fixture
```

验收：

```text
[ ] 中心值正确
[ ] 四通道符号正确
[ ] switch 三态正确
[ ] mouse/keyboard 正确
[ ] 非法 channel 拒绝
[ ] out transactional
```

提交：

```text
feat(remote): add DR16 frame decoder
```

---

# 57. 第四阶段施工：RemoteService + UART

加入 UART。

先只：

```text
RemoteState
    ↓
LOG / VOFA
```

不要控制电机。

验收：

```text
[ ] 连续接收
[ ] 无错位后长期漂帧
[ ] 拔遥控器 -> offline
[ ] 恢复 -> online
```

提交：

```text
feat(remote): add UART remote service
```

---

# 58. 第五阶段施工：ManualCommandMapper + CommandManager

新增：

```text
manual_command_mapper
safety_manager
command_manager
```

测试：

```text
remote fixture
    ↓
RobotCommand
```

验收：

```text
[ ] Safe 输出零命令
[ ] Manual 正确映射
[ ] Remote offline 安全
[ ] max rate clamp
```

提交：

```text
feat(command): add manual command and safety arbitration
```

---

# 59. 第六阶段施工：真实 DR16 控制小 Yaw

这时才连：

```text
Remote
 ↓
Command
 ↓
YawGimbal
```

首次限制：

```text
yaw rate <= 0.5 rad/s
current limit 保持低
```

硬件验收：

```text
[ ] 左拨杆安全位不运动
[ ] 右摇杆左右控制 yaw
[ ] 松杆 hold
[ ] 掉线不乱跑
[ ] 方向符合操作习惯
[ ] 不撞机械限位
```

提交：

```text
feat(application): control yaw gimbal from DR16
```

---

# 60. 第七阶段施工：SwerveKinematics

此阶段不接电机。

实现：

```text
swerve_types
swerve_kinematics
```

sample 打印：

```text
forward
left
rotate
forward + rotate
zero
```

验收：

```text
forward:
    4轮 angle=0

left:
    4轮 angle=+pi/2

rotate:
    4轮切向

zero:
    保持原舵角

over max speed:
    四轮统一 scale
```

提交：

```text
feat(swerve): add chassis kinematics
```

---

# 61. 第八阶段施工：Referee Parser

裁判硬件如果还没接，也可以通过离线 byte fixture 做。

实现：

```text
CRC
frame state machine
selected command decode
RefereeState
```

只解析当前真正需要的：

```text
game state
robot state
power
shooter heat / limit
```

不要一口气实现 UI 全协议。

提交：

```text
feat(referee): add 2026 referee stream parser
```

---

# 62. 第九阶段施工：RefereeService

UART 接入。

验收：

```text
[ ] 有效帧计数增长
[ ] CRC error 可统计
[ ] 拔线 -> offline
[ ] RobotState 正确
[ ] PowerState 正确
```

---

# 63. 第十阶段施工：SwerveModule / SwerveChassis

等：

```text
Kinematics 已确认
```

再实现：

```text
Module
Chassis
```

纯算法测试通过后，仍不要立即接 8 台电机。

---

# 64. 第十一阶段施工：真实底盘硬件

这时才写：

```text
DjiChassisHardware
```

顺序：

```text
1 module
    ↓
2 module
    ↓
4 module
```

不要第一次就：

```text
8 电机一起上
```

---

# 65. 推荐 Application V1 文件

小 Yaw 能跑起来时，application 只需要：

```text
application/
├── src/
│   ├── main.cpp
│   ├── message_bus.cpp
│   ├── remote_task.cpp
│   ├── command_task.cpp
│   └── gimbal_task.cpp
├── boards/
│   └── <board>.overlay
├── CMakeLists.txt
└── prj.conf
```

裁判、底盘 task 后续加入。

---

# 66. main.cpp 不应该做什么

最终 `main.cpp` 不应该变成：

```cpp
while (1) {
    read_uart();
    parse_remote();

    if (...) {
        if (...) {
            ...
        }
    }

    pid();
    can_send();
}
```

推荐 main：

```text
初始化
构造服务
启动线程
等待
```

业务逻辑全部拆开。

---

# 67. 推荐日志

Remote：

```text
remote online/offline
invalid frame count
```

Referee：

```text
online/offline
CRC errors
unknown cmd
```

Command：

```text
source changed
mode changed
safety state changed
```

Gimbal：

```text
begin
fault
stop
```

禁止 200Hz：

```text
LOG_INF 每周期打印
```

遥测走：

```text
VOFA
```

---

# 68. 编译与测试策略

每一阶段只构建受影响 sample。

示例：

```bash
west build -p always -b <board> samples/robotics/yaw_gimbal
```

```bash
west build -p always -b <board> samples/communication/dr16_decoder
```

```bash
west build -p always -b <board> samples/robotics/swerve_kinematics
```

application 最后：

```bash
west build -p always -b <board> application
```

执行 AI 需要先根据仓库实际 board 名确认命令，不得照抄 `<board>`。

---

# 69. 小 Yaw 上电安全

第一次 application 控制小 Yaw：

```text
1. 机械附近无人
2. 电流限值降低
3. rate 上限降低
4. 先不用遥控器
5. 固定小角度测试
6. 再用 rate
7. 再接遥控器
8. 最后测试掉线
```

必须提前准备：

```text
断电手段
急停
```

---

# 70. 执行 AI 的工作方式

执行 AI 必须：

```text
1. 读取本文
2. 读取 AGENTS.md
3. 读取 ancient-programming skill
4. 确认用户是否明确禁用古法编程模式
5. 读取当前阶段涉及的真实文件
6. 只做当前阶段
7. 不顺便实现下一阶段
8. 构建
9. 检查 git diff
10. 报告结果
```

每个阶段必须汇报：

```text
已修改文件
新增接口
保留旧行为
构建结果
未验证硬件假设
下一阶段
```

---

# 71. 执行 AI 禁止事项

禁止：

```text
一次性重构整个项目
复制另一套 RM 框架进来
自己造 RTOS
自己造 MessageCenter
遥控器直接控制电机
裁判系统直接写 PID
每个电机各开一个线程
每个 DJI 电机各自 flush
在控制层写 CAN ID
在 parser 里写机器人业务逻辑
为未来需求过度模板化
```

---

# 72. 当前最推荐的实际施工顺序

结合当前真实硬件条件：

```text
阶段 1
消息契约
        ↓
阶段 2
YawGimbal
        ↓
阶段 3
DR16 Decoder
        ↓
阶段 4
Remote UART
        ↓
阶段 5
Manual Mapper + Safety + CommandManager
        ↓
阶段 6
DR16 实机控制小 Yaw
        ↓
阶段 7
SwerveKinematics
        ↓
阶段 8
Referee Parser
        ↓
阶段 9
Referee UART
        ↓
阶段 10
SwerveModule / Chassis
        ↓
阶段 11
真实 4 舵 + 4 驱动
        ↓
阶段 12
Power / SuperCap policy
        ↓
阶段 13
Shooter / Pitch / Vision / Auto
```

这比：

```text
先把所有驱动全写完
最后一次性集成
```

风险低很多。

---

# 73. 当前最重要的架构结论

## 结论 1

```text
Remote / Referee
```

都是：

```text
输入源
```

不是控制器。

---

## 结论 2

所有来源最后都必须收敛到：

```text
RobotCommand
```

子系统不关心命令来自哪里。

---

## 结论 3

小 Yaw 当前应直接复用：

```text
PositionMotor
+
MotorBackend
```

不要再造位置环。

---

## 结论 4

底盘多 DJI 电机不能复用：

```text
8 × DjiMotorBackend
```

必须：

```text
shared dji::Bus
```

统一 flush。

---

## 结论 5

zbus 只负责：

```text
线程间搬消息
```

不要把业务逻辑塞进 listener。

---

## 结论 6

当前底盘优先实现：

```text
Command
+
Kinematics
```

而不是马上实现真实 CAN。

---

# 74. 最终验收清单

## Message

```text
[ ] RemoteState
[ ] RefereeState
[ ] OperatorIntent
[ ] RobotCommand
[ ] Feedback
[ ] SI units
```

## Remote

```text
[ ] decoder transactional
[ ] UART 与 parser 解耦
[ ] online timeout
[ ] switch / mouse / keyboard
```

## Command

```text
[ ] manual mapper
[ ] safety
[ ] command manager
[ ] remote offline safety
```

## Gimbal

```text
[ ] PositionMotor reused
[ ] Hold
[ ] Rate
[ ] Absolute
[ ] software limit
[ ] DR16 control tested
```

## Chassis

```text
[ ] kinematics independent of CAN
[ ] coordinates documented
[ ] zero speed holds angle
[ ] uniform desaturation
[ ] no DjiMotorBackend ×8
```

## Referee

```text
[ ] streaming parser
[ ] CRC
[ ] version isolation
[ ] RefereeState stable
[ ] no direct motor control
```

## Application

```text
[ ] zbus only in application messaging
[ ] control loops periodic
[ ] UART callback does minimal work
[ ] init order safe
[ ] faults stop safely
```

---

# 75. 给下一位 AI 的直接任务模板

用户可以把本文交给另一个 AI，然后使用：

```text
请严格按照《SkyWalker 机器人应用层、消息系统与子系统架构施工规格》执行。

本任务禁用古法编程模式。

只执行“阶段 1：消息契约”，不要提前实现后续阶段。

开始前：
1. 读取 AGENTS.md。
2. 读取 .agents/skills/ancient-programming/SKILL.md。
3. 读取当前仓库中涉及的实际代码。
4. 如果本文接口与当前代码已经发生冲突，以“保持本文分层原则 + 最小改动适配现状”为准。
5. 完成本阶段后构建受影响目标，检查 git diff，然后停止。

不要顺手实现下一阶段。
```

阶段 1 完成后，把：

```text
阶段 1
```

替换成：

```text
阶段 2：YawGimbal
```

逐阶段执行。

---

# 76. 未验证硬件假设

以下内容必须实机确认，不允许 AI 猜测后宣称完成：

```text
1. 小 Yaw 实际电机型号
2. 小 Yaw 编码器固定零点
3. 小 Yaw 正方向
4. 小 Yaw 是否连续旋转还是机械限位
5. DR16 实际 UART 接口
6. DR16 parity / stop bits / signal path
7. 裁判系统实际 UART 接口
8. 未来 8 个底盘电机 CAN 分配
9. 舵轮实际 wheel radius
10. 四模块真实 x/y 坐标
11. 各模块安装正方向
```

这些必须留在：

```text
application configuration / devicetree
```

不能硬编码进通用 robotics 库。

---

# 77. 文档完成后的直接建议

当前立刻开始：

```text
阶段 1：公共消息结构
```

然后：

```text
阶段 2：YawGimbal
```

因为这是目前唯一能够很快在真实硬件上形成完整验证闭环的子系统。

一旦小 Yaw 链路跑通：

```text
DR16
→ Command
→ Safety
→ Subsystem
→ Motor
```

之后底盘、Pitch、Shooter 都只是在复用同一套上层架构，而不是重新设计一次。
