# SkyWalker 舵轮控制架构施工指南

> 目标仓库：`SHM-white/SkyWalker_General_Embedded_Code`
>
> 目标分支：`dev`
>
> 本文定位：给 Codex / 人工开发者直接执行的分阶段施工规格。
>
> 核心原则：**Motor → Controller → Module → Chassis → Application**。  
> 任何阶段都不要跨层“顺手重构”下一层；每完成一层先编译、自检、必要时上硬件验证，再继续下一层。

---

## 0. 执行前约束

### 0.1 先读取仓库规则

开始前必须读取并遵守：

```text
AGENTS.md
.agents/skills/ancient-programming/SKILL.md
```

仓库当前默认启用“古法编程模式”：源码只读，只允许在仓库根目录写 Markdown 指南。

因此：

- 如果用户**没有在当前任务中明确说“本任务禁用古法编程模式”**，不要修改业务源码。
- 如果用户明确禁用古法编程模式，则只在本任务范围内按本文逐阶段施工。
- 不要自行推断用户已经授权禁用。
- 即使禁用，也仍然遵守本文的分阶段边界，不要一次性完成全部重构。

### 0.2 基线分支与现状

施工前确认工作区位于 `dev`：

```bash
git status
git branch --show-current
git log -1 --oneline
```

需要重点阅读这些现有文件：

```text
include/drivers/motor/motor.hpp

drivers/motor/dji/dji_internal.hpp
drivers/motor/dji/dji_motor.cpp
drivers/motor/dji/dji_profiles.cpp

include/control/angle.h
lib/control/angle.c

include/control/pid.h
lib/control/pid.c
include/control/feedforward_pid.h
lib/control/feedforward_pid.c
include/control/slew_rate_limiter.h
lib/control/slew_rate_limiter.c

samples/motor/dji_speed_control/src/main.cpp
samples/motor/dji_position_control/src/main.cpp

dts/bindings/motor/dji-motor-base.yaml
lib/control/CMakeLists.txt
```

当前已知架构事实：

```text
motor.hpp
  └─ 统一 Motor API
      ├─ setCurrent()
      ├─ setTorque()
      ├─ readFeedback()
      └─ getState()

lib/control/
  └─ 纯 C 数值控制库
      ├─ PID
      ├─ feedforward
      ├─ feedforward PID
      ├─ slew rate
      └─ angle

DJI motor / DJI Bus
  └─ C++ 硬件对象与 CAN 生命周期

samples/
  └─ 目前仍保存了大量本应抽入控制库的速度环、位置环逻辑
```

### 0.3 绝对禁止的架构倒退

后续施工中禁止：

```text
1. 给 motor::Api 增加 setPosition()/setVelocity()
2. 在 DJI driver 里塞 PID
3. 在 PID 内部理解“绝对角/最近路径”
4. 在 SwerveChassis 里直接调用 CAN
5. 在 SwerveChassis 里直接处理 GM6020/M3508
6. 在 SwerveModule 里直接 flush DJI Bus
7. 为了舵轮复制第二份速度 PI
8. 第一次抽取控制器时顺便重新调 PID 参数
9. 把 GM6020 型号名称作为“是否有固定零点”的判断条件
10. 一次性把所有阶段混在一个大提交里
```

---

# 1. 最终目标架构

最终依赖必须保持单向：

```text
Application / Control Thread
        │
        │ vx / vy / wz
        ▼
┌─────────────────────────────┐
│       SwerveChassis         │  C++ class
│  四轮运动学 / 轮速归一化     │
└──────────────┬──────────────┘
               │ 4 × module target
               ▼
┌─────────────────────────────┐
│       SwerveModule          │  C++ class
│ 舵角优化 / 180°翻转          │
│ 绝对角 → 最近连续目标         │
│ m/s ↔ motor rad/s           │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│        control/             │  C
│ motor_position             │
│ motor_velocity             │
│ angle / PID / FF / slew    │
└──────────────┬──────────────┘
               │ current command
               ▼
        motor::setCurrent()
               │
               ▼
        DJI Bus / other Bus
               │
               ▼
              CAN
```

最终原则：

```text
数学算法         → C
有状态控制器      → C struct + step/reset/validate
执行机构          → C++ class
机器人子系统      → C++ class
硬件驱动          → Motor API
总线生命周期      → Bus
```

---

# 2. 阶段一：抽取公共速度控制器

## 2.1 目标

把 `samples/motor/dji_speed_control/src/main.cpp` 中与“速度闭环数值计算”有关的逻辑抽进 `lib/control`。

完成后：

```text
sample
  只负责：
    读反馈
    freshness
    arm
    调 controller
    setCurrent
    flush
    telemetry
    fault stop

control_motor_velocity
  负责：
    目标速度斜坡
    速度反馈低通
    软死区
    feedforward + PID
    软件电流限幅
    transactional state
```

不要修改速度环现有行为。

## 2.2 新增文件

```text
include/control/motor_velocity.h
lib/control/motor_velocity.c
```

修改：

```text
lib/control/CMakeLists.txt
samples/motor/dji_speed_control/src/main.cpp
```

### 2.3 推荐接口

`include/control/motor_velocity.h`：

```c
#ifndef SKYWALKER_CONTROL_MOTOR_VELOCITY_H
#define SKYWALKER_CONTROL_MOTOR_VELOCITY_H

#include <stdbool.h>

#include <control/feedforward_pid.h>
#include <control/slew_rate_limiter.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    control_feedforward_pid_config regulator;
    control_slew_rate_config reference_slew;

    float measurement_filter_tau_s;
    float soft_deadband_rad_s;

    float requested_velocity_abs_max_rad_s;
    float current_abs_max_a;
} control_motor_velocity_config;

typedef struct {
    control_feedforward_pid_state regulator;
    control_slew_rate_state reference_slew;

    float filtered_velocity_rad_s;
    bool filter_initialized;
} control_motor_velocity_state;

typedef struct {
    float requested_velocity_rad_s;
    float measured_velocity_rad_s;

    float position_reference_rad;

    float dt_s;
    bool freeze_integrator;
} control_motor_velocity_input;

typedef struct {
    float velocity_reference_rad_s;
    float acceleration_reference_rad_s2;

    float filtered_velocity_rad_s;
    float velocity_error_rad_s;

    control_feedforward_pid_result regulator;

    float current_command_a;
} control_motor_velocity_output;

int control_motor_velocity_validate(
    const control_motor_velocity_config *config);

int control_motor_velocity_reset(
    control_motor_velocity_state *state,
    float measured_velocity_rad_s,
    float initial_reference_rad_s);

int control_motor_velocity_step(
    control_motor_velocity_state *state,
    const control_motor_velocity_config *config,
    const control_motor_velocity_input *input,
    control_motor_velocity_output *output);

#ifdef __cplusplus
}
#endif

#endif
```

## 2.4 行为要求

必须从现有速度 sample 原样迁移这些行为：

```text
reference slew
velocity low-pass
soft deadband
feedforward PID
software current clamp
finite 检查
requested velocity 边界检查
transactional state
```

### transactional state 必须保留

错误做法：

```c
state->reference_slew = ...;
ret = next_step(...);
if (ret < 0) {
    return ret;
}
```

因为后续失败时，状态已经被部分提交。

正确方式：

```c
control_motor_velocity_state next = *state;
control_motor_velocity_output next_output = {0};

...所有计算使用 next...

if (all_ok) {
    *state = next;
    *output = next_output;
}
```

任何失败：

```text
state 不变
output 不要求有效
```

## 2.5 soft deadband

把 sample 中类似：

```cpp
applySoftDeadband()
```

迁移成该模块私有静态函数。

语义：

```text
|error| <= deadband
    → 0

|error| > deadband
    → sign(error) * (|error| - deadband)
```

不要改成硬截断：

```text
超过 deadband 后仍保留完整 error
```

否则控制行为改变。

## 2.6 reset 语义

建议：

```c
control_motor_velocity_reset(
    state,
    measured_velocity,
    initial_reference);
```

初始化：

```text
PID measurement baseline = measured_velocity
slew current reference   = initial_reference
filter value             = measured_velocity
filter initialized       = true
```

速度 sample 当前上电行为如果以 0 为起始目标，则调用：

```c
control_motor_velocity_reset(
    &state,
    first_feedback.velocity_rad_s,
    0.0f);
```

## 2.7 sample 改造

删除 sample 内这些重复结构/函数：

```text
VelocityController
VelocityControlOutput
validateController()
resetController()
calculateVelocityCurrent()
applySoftDeadband()
```

保留：

```text
waitForFreshFeedback()
readFreshVelocityFeedback()
requestedVelocityForTime()
stopAfterFailure()
main()
VOFA telemetry
```

sample 中构建 config 即可。

## 2.8 本阶段禁止

不要：

```text
修改 PID 增益
修改 slew 参数
修改 current limit
加入位置环
加入绝对编码器逻辑
加入 SwerveModule
修改 DJI driver
```

## 2.9 验收

至少满足：

```text
[ ] control_motor_velocity_validate() 能拒绝非法配置
[ ] reset 后第一次 step 不出现未初始化滤波器
[ ] requested velocity 超界返回 -ERANGE
[ ] NaN / Inf 返回负错误码
[ ] step 中间失败不提交 state
[ ] sample 不再实现第二套速度环
[ ] sample 编译通过
[ ] 相同输入下控制行为与抽取前一致
```

完成后建议形成独立提交：

```text
refactor(control): extract reusable motor velocity controller
```

---

# 3. 阶段二：位置控制器组合速度控制器

## 3.1 目标

把位置 sample 的串级控制抽成：

```text
position controller
        ↓
requested velocity
        ↓
公共 motor_velocity
        ↓
current
```

绝对禁止在 `motor_position.c` 再复制速度 PI。

## 3.2 新增文件

```text
include/control/motor_position.h
lib/control/motor_position.c
```

修改：

```text
lib/control/CMakeLists.txt
samples/motor/dji_position_control/src/main.cpp
```

## 3.3 推荐接口

```c
#ifndef SKYWALKER_CONTROL_MOTOR_POSITION_H
#define SKYWALKER_CONTROL_MOTOR_POSITION_H

#include <control/pid.h>
#include <control/motor_velocity.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    control_pid_config position;
    control_motor_velocity_config velocity;
} control_motor_position_config;

typedef struct {
    control_pid_state position;
    control_motor_velocity_state velocity;
} control_motor_position_state;

typedef struct {
    float continuous_target_rad;
    float continuous_position_rad;
    float measured_velocity_rad_s;

    float dt_s;
} control_motor_position_input;

typedef struct {
    control_pid_result position;
    control_motor_velocity_output velocity;

    float current_command_a;
} control_motor_position_output;

int control_motor_position_validate(
    const control_motor_position_config *config);

int control_motor_position_reset(
    control_motor_position_state *state,
    const control_motor_position_config *config,
    float measured_position_rad,
    float measured_velocity_rad_s);

int control_motor_position_step(
    control_motor_position_state *state,
    const control_motor_position_config *config,
    const control_motor_position_input *input,
    control_motor_position_output *output);

#ifdef __cplusplus
}
#endif

#endif
```

## 3.4 运行逻辑

```text
continuous_target_rad
        -
continuous_position_rad
        ↓
position PID
        ↓
requested_velocity_rad_s
        ↓
control_motor_velocity_step()
        ↓
current_command_a
```

位置 PID 输出边界：

```text
[-max velocity, +max velocity]
```

真正的速度目标斜坡只放在：

```text
control_motor_velocity
```

不要在 `motor_position` 再加第二个 slew limiter。

## 3.5 保持现有位置 sample 行为

第一次封装时必须“机械搬运”，不要顺便调参。

特别注意现有 sample：

```text
位置环注释声称是 P-only
但是配置里：
    ki = 1.5
    kd = 0.8

调用时：
    freeze_integrator = true
```

因此实际行为：

```text
I 被冻结
D 仍然有效
```

所以第一次抽取必须保留：

```text
kp
ki
kd
derivative_tau
deadband
output limits
freeze_integrator
```

不要把它“纠正”为真正 P-only。

等架构迁移完成并验证行为一致后，再单独做调参任务。

## 3.6 transactional state

位置环和内部速度环必须整体事务化。

推荐：

```c
control_motor_position_state next = *state;
control_motor_position_output next_output = {0};

ret = control_pid_step(&next.position, ...);
if (ret < 0) return ret;

ret = control_motor_velocity_step(
    &next.velocity,
    &config->velocity,
    &velocity_input,
    &next_output.velocity);
if (ret < 0) return ret;

next_output.current_command_a =
    next_output.velocity.current_command_a;

*state = next;
*output = next_output;
```

## 3.7 sample 最终只负责

```text
read feedback
freshness
trajectory generation
controller step
setCurrent
flush
telemetry
fault stop
```

## 3.8 验收

```text
[ ] position controller 内没有第二份 velocity PID
[ ] velocity 环只由 control_motor_velocity_step() 实现
[ ] 现有位置 sample 参数不变
[ ] freeze_integrator 行为不变
[ ] position output 仍作为速度请求
[ ] current 输出来自公共 velocity controller
[ ] step 失败不提交 position/velocity 两层状态
[ ] sample 编译通过
```

建议提交：

```text
refactor(control): compose position controller from velocity controller
```

---

# 4. 阶段三：给 Motor Feedback 增加固定零点单圈角

## 4.1 目标

驱动同时提供：

```text
position_rad
    上电首帧为 0
    连续
    可无限累加

absolute_position_rad
    固定机械零点
    单圈
    [-pi, pi)
    掉电重启后仍可恢复方向
```

不要提供“断电保持多圈绝对位置”，因为 DJI 内置单圈编码器无法恢复圈数。

## 4.2 修改统一 Motor API

文件：

```text
include/drivers/motor/motor.hpp
```

新增 capability：

```cpp
FeedbackAbsolutePosition = 1u << 13,
```

Feedback：

```cpp
struct Feedback {
    float position_rad = 0.0f;
    float absolute_position_rad = 0.0f;

    float velocity_rad_s = 0.0f;
    float current_a = 0.0f;
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;

    std::uint32_t valid = 0;
    std::uint64_t timestamp_ms = 0;
};
```

语义务必写进注释。

## 4.3 DJI Profile 增加“传感器能力”，不要判断型号

修改：

```text
drivers/motor/dji/dji_internal.hpp
drivers/motor/dji/dji_profiles.cpp
```

推荐：

```cpp
enum class PositionSensorType : std::uint8_t {
    RelativeOnly = 0,
    FixedZeroSingleTurn,
};
```

Profile 增加：

```cpp
PositionSensorType position_sensor;
std::uint16_t encoder_ticks_per_turn;
```

注意：

```text
encoder_ticks_per_turn
```

不要继续作为 `dji_motor.cpp` 里的全局硬编码 `8192`。

DJI profile：

```text
GM6020:
    FixedZeroSingleTurn
    8192

M3508:
    RelativeOnly
    8192

M2006:
    RelativeOnly
    8192
```

即使其他 DJI 电机实际上也有单圈编码器，本阶段只开放已经明确需要、明确标定的能力，避免 API 语义混乱。

## 4.4 DTS 加装配零点

建议只在需要固定零点的型号 binding 中暴露：

```text
dts/bindings/motor/dji,gm6020-current.yaml
```

增加：

```yaml
encoder-zero-ticks:
  type: int
  required: true
  min: 0
  max: 8191
  description: >
    Encoder tick corresponding to mechanical absolute zero.
```

不要把它做成运行时 setter。

V1：

```text
零点 = build-time / devicetree configuration
```

后续如果需要现场标定，再单独设计未 arm 状态下的 calibration API。

## 4.5 DjiConfig 增加零点

```cpp
std::uint32_t encoder_zero_ticks;
```

注意兼容没有该属性的型号。

不要强迫 M3508/M2006 DTS 填一个没有意义的 zero。

可以在宏中根据 DT 属性是否存在提供默认值，例如：

```text
0
```

但只有 profile 为 `FixedZeroSingleTurn` 时才使用。

## 4.6 absolute_position 计算

输入：

```text
raw.encoder
zero_ticks
ticks_per_turn
gear ratio
```

这里必须先确认“绝对角是电机输出轴还是转子轴”。

现有 `position_rad` 和 `velocity_rad_s` 都除以：

```text
gear_ratio
```

所以统一 Motor Feedback 的位置语义应继续保持“输出轴位置”。

但是固定零点存在一个重要边界：

如果减速机构允许输出轴一圈对应电机编码器多圈，则单个转子单圈编码器无法唯一表示输出轴绝对单圈角。

因此本阶段必须明确：

```text
FixedZeroSingleTurn 只适用于编码器单圈角可以唯一映射到机构单圈角的机构。
```

GM6020 舵向通常按 1:1 输出使用时成立。

推荐算法：

```text
relative_ticks =
    raw_encoder - encoder_zero_ticks

wrap relative_ticks into:
    [-ticks_per_turn/2, ticks_per_turn/2)

absolute_motor_angle =
    relative_ticks * 2*pi / ticks_per_turn

absolute_output_angle =
    absolute_motor_angle / gear_ratio
```

但如果 `gear_ratio != 1`，必须重新检查其机械语义，不要盲目宣称仍然是完整机构绝对角。

## 4.7 连续 position 保持原语义

现有逻辑：

```text
首帧:
    total_encoder_ticks = 0

后续:
    delta = raw - last
    跨 0/8191 时修正
    total += delta
```

继续保留。

不要因为新增绝对角而让：

```text
position_rad
```

变成“相对于固定零点”。

两者必须同时存在。

## 4.8 capability

只有 profile：

```text
FixedZeroSingleTurn
```

才：

```cpp
caps |= FeedbackAbsolutePosition;
```

并在 `feedback.valid` 中标记。

## 4.9 验收

模拟关键点：

```text
zero = 4096

raw = 4096
absolute = 0

raw ≈ 4096 + 2048
absolute ≈ +pi/2

raw ≈ 4096 - 2048
absolute ≈ -pi/2

跨编码器边界时
absolute 必须连续按单圈 wrap
position_rad 必须保持连续累加
```

检查：

```text
[ ] position_rad 语义未改变
[ ] absolute_position_rad 只在 capability 支持时有效
[ ] ticks per turn 不再散落硬编码
[ ] GM6020 零点来自 DTS
[ ] 控制过程中不能动态改变零点
```

建议提交：

```text
feat(motor): add fixed-zero single-turn position feedback
```

---

# 5. 阶段四：绝对角 → 最近连续目标

## 5.1 目标

为舵向目标提供统一转换：

```text
固定零点单圈目标
      ↓
距离当前朝向最近的连续位置目标
```

PID 永远只看连续位置。

## 5.2 修改

```text
include/control/angle.h
lib/control/angle.c
```

新增：

```c
int control_angle_nearest_continuous_target(
    float requested_absolute_rad,
    float measured_absolute_rad,
    float measured_continuous_rad,
    float *continuous_target_rad);
```

## 5.3 算法

不要自己再写一套 wrap。

直接复用：

```c
control_shortest_angle_error()
```

逻辑：

```c
float error = 0.0f;

ret = control_shortest_angle_error(
    requested_absolute_rad,
    measured_absolute_rad,
    &error);

if (ret < 0)
    return ret;

target = measured_continuous_rad + error;
```

## 5.4 示例

```text
measured absolute   = +170°
measured continuous = 0°
requested absolute  = -170°

shortest error      = +20°
continuous target   = +20°
```

另一个：

```text
measured absolute   = -175°
measured continuous = +8 * 2π - 175°
requested absolute  = +175°

shortest error      = -10°
continuous target   = measured continuous - 10°
```

因此无论已经连续转了多少圈，PID 都不会遇到 `+pi/-pi` 跳变。

## 5.5 本阶段不要做

不要：

```text
把这个逻辑塞进 PID
把这个逻辑塞进 GM6020 driver
把这个逻辑做成有状态类
```

这是无状态角度数学函数。

## 5.6 验收

至少覆盖：

```text
0 → 90°
179° → -179°
-179° → 179°
连续位置已经 ±N 圈
NaN / Inf
NULL output
```

建议提交：

```text
feat(control): add nearest continuous angle target conversion
```

---


# 6. 阶段五：两个控制 sample 回接公共控制器，并人工调试恢复原有效果

> 这一阶段放在“绝对角 → 最近连续目标”完成之后、`SwerveModule` 之前。  
> 目的不是新增舵轮功能，而是先证明前四阶段抽出来的公共控制链路可以完整替代 sample 里原先直接拼接 PID / slew / feedforward 等底层模块的做法。

## 6.1 目标

重新修改两个现有测试 sample：

```text
samples/motor/dji_speed_control/
samples/motor/dji_position_control/
```

要求最终变成：

```text
dji_speed_control
    ↓
control_motor_velocity
    ↓
current_command
    ↓
motor::setCurrent()
    ↓
Bus::flush()

dji_position_control
    ↓
control_motor_position
        ↓
    control_motor_velocity
    ↓
current_command
    ↓
motor::setCurrent()
    ↓
Bus::flush()
```

sample 不再直接组合：

```text
control_pid_*
control_feedforward_pid_*
control_slew_rate_*
```

这些底层模块只能由已经抽取出的：

```text
control_motor_velocity
control_motor_position
```

内部使用。

这一阶段完成后，两个 sample 的角色应该从：

```text
“控制算法原型实现”
```

变成：

```text
“公共 Motor Controller 的硬件验证程序”
```

---

## 6.2 为什么必须在 SwerveModule 前做

如果不先做这一阶段，后续很容易出现：

```text
sample 有一套闭环
SwerveModule 又写一套闭环
```

最后变成同一个电机控制行为存在两份实现。

因此进入 `SwerveModule` 前，必须先证明：

```text
速度闭环 → 公共 velocity controller 可用
位置闭环 → 公共 position controller 可用
绝对角目标 → 公共 angle target converter 可用
```

后续 `SwerveModule` 只能组合这些已验证模块，不再复制 sample 代码。

---

## 6.3 速度 sample 的最终职责

文件重点：

```text
samples/motor/dji_speed_control/src/main.cpp
```

sample 只保留：

```text
设备获取
Bus init / attach / arm
等待 fresh feedback
读取与检查 feedback
构造 control_motor_velocity_config
reset controller
生成测试目标速度
调用 control_motor_velocity_step()
motor::setCurrent()
Bus::flush()
VOFA / log telemetry
错误安全停机
```

禁止 sample 再直接调用：

```c
control_feedforward_pid_step()
control_slew_rate_step()
control_pid_step()
```

也不要再保存：

```text
PID state
slew state
velocity filter state
```

这些全部属于：

```text
control_motor_velocity_state
```

### 速度 sample 每周期结构

应接近：

```cpp
skywalker::motor::Feedback feedback{};
ret = readFreshVelocityFeedback(...);

control_motor_velocity_input input{
    .requested_velocity_rad_s = requestedVelocityForTime(...),
    .measured_velocity_rad_s = feedback.velocity_rad_s,
    .position_reference_rad = 0.0f,
    .dt_s = dt_s,
    .freeze_integrator = false,
};

control_motor_velocity_output output{};

ret = control_motor_velocity_step(
    &controller_state,
    &controller_config,
    &input,
    &output);

ret = skywalker::motor::setCurrent(
    motor,
    output.current_command_a);

ret = dji_bus.flush(...);
```

具体字段名以阶段一最终落地接口为准，不要为了照抄本段示例而重新修改已经稳定的公共 API。

---

## 6.4 位置 sample 的最终职责

文件重点：

```text
samples/motor/dji_position_control/src/main.cpp
```

只保留：

```text
设备获取
Bus init / attach / arm
等待反馈
freshness / capability 检查
生成位置测试目标
选择目标坐标模式
必要时执行 absolute → nearest continuous target
调用 control_motor_position_step()
setCurrent()
flush()
telemetry
错误安全停机
```

禁止 sample 直接操作：

```text
position PID
velocity PID
velocity slew
feedforward PID
```

位置 sample 只允许通过：

```c
control_motor_position_step()
```

取得：

```text
current_command_a
```

---

## 6.5 位置 sample 增加目标坐标模式开关

V1 使用简单、明确的 sample 级编译期选择，不要为了这个测试程序增加复杂运行时配置系统。

建议：

```cpp
enum class PositionTargetMode {
    ContinuousRelative = 0,
    FixedZeroAbsolute,
};

constexpr PositionTargetMode kPositionTargetMode =
    PositionTargetMode::ContinuousRelative;
```

默认保持：

```text
ContinuousRelative
```

这样在重构完成后的第一次硬件对比中，可以直接和原位置 sample 的行为做 A/B 对照。

切换绝对角模式时只改：

```cpp
PositionTargetMode::FixedZeroAbsolute
```

不要在 Motor Driver 中增加：

```text
setAbsoluteMode()
setPositionMode()
```

模式只属于 sample / 上层目标解释。

---

## 6.6 连续相对角模式

`ContinuousRelative` 模式继续使用：

```text
feedback.position_rad
```

作为位置反馈。

目标也是连续坐标：

```text
continuous_target_rad
```

然后直接：

```cpp
control_motor_position_input input{
    .continuous_target_rad = requested_target,
    .continuous_position_rad = feedback.position_rad,
    .measured_velocity_rad_s = feedback.velocity_rad_s,
    .dt_s = dt_s,
};
```

这一模式的第一目标是：

> **重构后尽量恢复到重构前位置 sample 的相同行为。**

因此不要在这个模式中加入绝对零点修正、最短路转换或舵轮 180° 优化。

---

## 6.7 固定零点绝对角模式

`FixedZeroAbsolute` 模式要求电机反馈至少具备：

```text
FeedbackPosition
FeedbackAbsolutePosition
FeedbackVelocity
```

如果缺少：

```text
FeedbackAbsolutePosition
```

则 sample 必须在 arm 前拒绝运行，而不是退化成相对位置。

目标生成函数在此模式下生成：

```text
requested_absolute_rad
```

约定范围：

```text
[-pi, pi)
```

每个控制周期：

```cpp
float continuous_target_rad = 0.0f;

ret = control_angle_nearest_continuous_target(
    requested_absolute_rad,
    feedback.absolute_position_rad,
    feedback.position_rad,
    &continuous_target_rad);
```

然后把转换后的：

```text
continuous_target_rad
```

交给：

```c
control_motor_position_step()
```

完整链路：

```text
requested absolute angle
        ↓
control_angle_nearest_continuous_target
        ↓
continuous target
        ↓
control_motor_position
        ↓
control_motor_velocity
        ↓
current command
```

位置 PID 自己永远不知道“绝对角模式”。

---

## 6.8 绝对角测试轨迹

不要直接复用原来“0、1、2、3 倍目标偏移”这类可能超过单圈范围的连续位置轨迹。

绝对模式单独提供一个简单目标生成函数，例如：

```cpp
float requestedAbsolutePositionRad(std::int64_t elapsed_ms)
{
    const std::int64_t phase = elapsed_ms % 12000;

    if (phase < 3000) {
        return 0.0f;
    }
    if (phase < 6000) {
        return 1.0f;
    }
    if (phase < 9000) {
        return -1.0f;
    }
    return 2.8f;
}
```

数值只是测试轨迹示例，不是必须照搬。

关键要求：

```text
目标始终在 [-pi, pi)
至少包含一次跨 ±pi 最近路径的人工测试
```

例如后续人工测试可增加：

```text
+170° → -170°
```

期望只走约：

```text
+20°
```

而不是反向走：

```text
340°
```

---

## 6.9 capability 与 arm 前检查

速度 sample 至少检查：

```text
CommandCurrent
FeedbackVelocity
```

位置 sample 的 `ContinuousRelative` 模式至少检查：

```text
CommandCurrent
FeedbackPosition
FeedbackVelocity
```

位置 sample 的 `FixedZeroAbsolute` 模式至少检查：

```text
CommandCurrent
FeedbackPosition
FeedbackAbsolutePosition
FeedbackVelocity
```

capability 不满足时：

```text
打印明确错误
保持未 arm
退出
```

不要等控制循环开始后才失败。

---

## 6.10 telemetry 要继续保留控制器内部观测量

虽然 sample 不再直接操作 PID，但调试仍然需要看到公共控制器输出中的中间结果。

速度 sample 至少保留：

```text
requested velocity
slewed/reference velocity
raw measured velocity
filtered velocity
velocity error
P
I
D（若启用）
feedforward（若输出结构提供）
current command
saturated
feedback age
```

位置 sample 至少保留：

```text
requested target
最终 continuous target
continuous measured position
absolute position（绝对模式下）
position error
position controller output / requested velocity
velocity reference
measured velocity
velocity error
velocity P/I/D
current command
feedback age
```

如果公共输出结构目前没有某一项，不要为了 telemetry 破坏封装边界。

优先使用：

```text
control_motor_velocity_output
control_motor_position_output
```

已经公开的诊断结果。

---

## 6.11 人工调试目标

这里的“调试”明确指：

> **用户在真实硬件上人工调试。Codex 只负责把 sample 改造成可调、可观测、行为尽量等价的验证程序。**

Codex 不得声称：

```text
已经实机调好
已经恢复原有效果
已经验证稳定
```

除非用户实际提供了硬件测试结果。

### 第一步：速度 sample

先使用阶段一迁移时保留的原参数。

人工确认：

```text
目标速度方向正确
起停行为与之前接近
稳态速度与之前接近
电流限制工作正常
没有新增明显振荡/啸叫
```

如果效果不同：

```text
先检查抽取是否改变计算顺序/状态/reset/滤波/soft deadband
```

不要第一反应就重新调 PID。

只有确认算法等价、差异确实来自参数后，才人工小步调参。

### 第二步：位置 sample 相对模式

保持：

```text
PositionTargetMode::ContinuousRelative
```

人工确认：

```text
到位速度
超调
回拉
目标附近抖动
最大速度
最大电流
```

尽量达到重构前同样效果。

### 第三步：位置 sample 绝对模式

相对模式验证通过后再切：

```text
PositionTargetMode::FixedZeroAbsolute
```

先测试小角度：

```text
0° → +20° → -20°
```

再测试：

```text
+170° ↔ -170°
```

确认走最短路径。

---

## 6.12 调试期间允许改什么

允许人工调整：

```text
velocity kp / ki / kd
position kp / ki / kd
reference slew
measurement filter tau
soft deadband
software current limit
测试目标幅值
```

但是每次只改一类变量，并记录结果。

不允许因为调参困难而：

```text
重新把 PID 写回 sample
绕过 control_motor_position
在 driver 中加位置控制
把 absolute shortest-path 塞进 PID
```

如果公共控制器缺少必要参数，应修改公共控制器配置接口，而不是 sample 偷偷维护分叉逻辑。

---

## 6.13 本阶段完成标准

速度 sample：

```text
[ ] 不再直接调用 PID / FF PID / slew step
[ ] 只通过 control_motor_velocity_step() 计算电流
[ ] fresh feedback / arm / setCurrent / flush 仍在 sample
[ ] telemetry 足够人工调试
[ ] 用户实机人工调试后达到与重构前基本相同效果
```

位置 sample：

```text
[ ] 不再直接组合位置 PID + 速度 PID
[ ] 只通过 control_motor_position_step() 计算电流
[ ] 默认 ContinuousRelative 模式行为与原 sample 基本一致
[ ] 有明确模式开关可切 FixedZeroAbsolute
[ ] absolute 模式要求 FeedbackAbsolutePosition
[ ] absolute 模式通过 nearest continuous target 转换
[ ] PID 内部完全不知道 absolute angle
[ ] 用户人工验证 +170° ↔ -170° 走最近方向
```

只有两个 sample 都重新验证通过，才进入 `SwerveModule`。

建议提交可以拆成两个：

```text
refactor(sample): route speed control through motor velocity controller

refactor(sample): route position control through motor position controller
```

如果绝对模式单独改动较大，可再拆：

```text
feat(sample): add fixed-zero absolute angle position mode
```

---

# 7. 阶段六：实现 SwerveModule

到这里才开始 C++ 执行机构层。

## 7.1 目标

一个 `SwerveModule` 表示：

```text
1 × steer axis
1 × drive axis
```

输入：

```text
目标轮角
目标地面线速度
反馈
dt
```

输出：

```text
steer current
drive current
```

它不拥有 CAN Bus 生命周期。

## 7.2 新目录

建议：

```text
include/robotics/swerve/swerve_module.hpp
lib/robotics/swerve/swerve_module.cpp
```

同时新增对应 CMake：

```text
lib/robotics/CMakeLists.txt
lib/robotics/swerve/CMakeLists.txt
```

并接入：

```text
lib/CMakeLists.txt
```

不要放进 `drivers/`。

SwerveModule 不是驱动。

## 7.3 数据结构

```cpp
namespace skywalker::robotics::swerve {

struct ModuleTarget {
    float angle_rad = 0.0f;
    float wheel_velocity_m_s = 0.0f;
};

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
    float drive_motor_target_rad_s = 0.0f;

    float steer_current_a = 0.0f;
    float drive_current_a = 0.0f;
};

struct ModuleConfig {
    float wheel_radius_m = 0.0f;

    control_motor_position_config steer;
    control_motor_velocity_config drive;
};

class SwerveModule {
public:
    explicit SwerveModule(const ModuleConfig &config);

    int validate() const;

    int reset(const ModuleFeedback &feedback);

    int step(
        const ModuleTarget &target,
        const ModuleFeedback &feedback,
        float dt_s,
        ModuleOutput &output);

private:
    ModuleConfig config_{};

    control_motor_position_state steer_state_{};
    control_motor_velocity_state drive_state_{};

    bool initialized_ = false;
};

}
```

## 7.4 为什么 Module 不保存 device*

V1 强烈建议：

```text
SwerveModule 不保存：
    const device *steer_motor
    const device *drive_motor
    DJI Bus *
```

原因：

```text
1. 数值算法与硬件生命周期分离
2. PC/host 测试容易
3. 将来 steer 换达妙、drive 换 DJI，Module 不变
4. 两条 CAN 总线不会污染 Module
5. 故障停机仍由 Application 统一决定
```

也就是说：

```text
Module = 有状态的执行机构算法对象
```

而不是“硬件 Manager”。

## 7.5 舵角优化

Module 内部实现私有逻辑。

先计算原目标与当前绝对角最短误差：

```text
error = shortest_angle(target.angle, feedback.absolute)
```

如果：

```text
|error| > π/2
```

则：

```text
target angle += π
wheel velocity *= -1
```

然后角度 wrap 回：

```text
[-π, π)
```

最后再调用：

```c
control_angle_nearest_continuous_target()
```

生成连续目标。

### 边界规则

恰好：

```text
|error| == π/2
```

必须固定一种行为，不要每周期因浮点噪声来回翻转。

V1 可以定义：

```text
只有 > π/2 才翻转
```

后续若实车在 90°附近抖动，再增加 hysteresis，例如：

```text
进入翻转阈值 95°
退出翻转阈值 85°
```

第一次实现先不要加复杂状态。

## 7.6 轮速换算

Chassis 给：

```text
wheel_velocity_m_s
```

drive controller 要：

```text
motor/output rad/s
```

如果 Motor Feedback 已经通过 driver 的 `gear_ratio` 统一成“输出轴角速度”，那么：

```text
drive_target_rad_s =
    wheel_velocity_m_s / wheel_radius_m
```

不要再在 SwerveModule 重复乘减速比。

这条必须和现有 Motor API 单位语义保持一致。

## 7.7 step 顺序

推荐：

```text
validate input
    ↓
optimize target
    ↓
nearest continuous steer target
    ↓
steer position controller
    ↓
wheel m/s → rad/s
    ↓
drive velocity controller
    ↓
assemble ModuleOutput
    ↓
commit state
```

整个 Module 也应事务化：

```cpp
auto next_steer = steer_state_;
auto next_drive = drive_state_;

...计算...

全部成功后：
steer_state_ = next_steer;
drive_state_ = next_drive;
output = next_output;
```

## 7.8 reset

reset 需要：

```text
steer:
    current continuous position
    current steer velocity

drive:
    current drive velocity
    initial velocity reference = 0
```

没有 reset 前调用 step：

```text
返回 -EACCES
```

## 7.9 本阶段禁止

不要：

```text
setCurrent()
readFeedback()
Bus::flush()
k_uptime_get()
LOG_INF()
device tree 查询
线程 sleep
遥控器
底盘解算
```

## 7.10 验收

数值测试至少覆盖：

```text
当前 10°，目标 20°，+3 m/s
    → 不翻转

当前 10°，目标 170°，+3 m/s
    → 目标约 -10°
    → 速度 -3 m/s

当前 179°，目标 -179°
    → 最近路径约 +2°

当前已经连续转 5 圈
    → continuous target 仍位于当前附近

wheel radius <= 0
    → validate 失败

未 reset 直接 step
    → -EACCES
```

建议提交：

```text
feat(swerve): add reusable swerve module controller
```

---

# 8. 阶段七：实现 SwerveChassis

## 8.1 目标

Chassis 只做：

```text
vx / vy / wz
      ↓
四个轮子的目标速度向量
      ↓
四个 ModuleTarget
      ↓
4 × SwerveModule
```

不知道电机型号、CAN、PID 参数细节。

## 8.2 新增

```text
include/robotics/swerve/swerve_chassis.hpp
lib/robotics/swerve/swerve_chassis.cpp
```

## 8.3 配置不要只存 L/W

为了以后支持不同几何，建议直接存四模块坐标：

```cpp
struct ModuleLocation {
    float x_m;
    float y_m;
};

struct ChassisConfig {
    ModuleLocation front_left;
    ModuleLocation front_right;
    ModuleLocation rear_left;
    ModuleLocation rear_right;

    float max_wheel_velocity_m_s;

    ModuleConfig module_front_left;
    ModuleConfig module_front_right;
    ModuleConfig module_rear_left;
    ModuleConfig module_rear_right;
};
```

坐标统一：

```text
+x = 车体前方
+y = 车体左方
+z = 向上
+wz > 0 = 逆时针
```

这一约定必须写进头文件注释。

## 8.4 指令

```cpp
struct ChassisCommand {
    float vx_m_s = 0.0f;
    float vy_m_s = 0.0f;
    float wz_rad_s = 0.0f;
};
```

## 8.5 单轮运动学

第 i 个模块坐标：

```text
(x_i, y_i)
```

则：

```text
wheel_vx_i = vx - wz * y_i
wheel_vy_i = vy + wz * x_i

speed_i = hypot(wheel_vx_i, wheel_vy_i)
angle_i = atan2(wheel_vy_i, wheel_vx_i)
```

然后：

```text
ModuleTarget{
    angle_i,
    speed_i
}
```

## 8.6 轮速统一缩放

如果任一轮：

```text
speed_i > max_wheel_velocity
```

不要单独截断该轮。

应该统一：

```text
scale =
    max_wheel_velocity / max(speed_fl, speed_fr, speed_rl, speed_rr)

all speed_i *= scale
```

这样保持四个轮子速度向量比例，避免底盘运动方向被破坏。

## 8.7 零速度附近的舵角策略

这是舵轮必须明确的边界。

当：

```text
hypot(vx_i, vy_i)
```

非常小时，`atan2(0,0)` 虽然数学库通常返回 0，但机械上不应该强制所有舵轮回 0°。

V1 建议 Chassis 在轮速低于一个小阈值时：

```text
保持该 Module 上一次目标角
wheel speed = 0
```

因此 Chassis 需要保存：

```text
last_module_angle[4]
```

如果还未初始化，则 reset 时使用当前真实舵角作为初值。

建议配置：

```cpp
float stationary_velocity_epsilon_m_s;
```

不要把这个策略塞到 PID。

## 8.8 推荐接口

```cpp
struct ChassisFeedback {
    ModuleFeedback front_left;
    ModuleFeedback front_right;
    ModuleFeedback rear_left;
    ModuleFeedback rear_right;
};

struct ChassisOutput {
    ModuleOutput front_left;
    ModuleOutput front_right;
    ModuleOutput rear_left;
    ModuleOutput rear_right;
};

class SwerveChassis {
public:
    explicit SwerveChassis(const ChassisConfig &config);

    int validate() const;
    int reset(const ChassisFeedback &feedback);

    int step(
        const ChassisCommand &command,
        const ChassisFeedback &feedback,
        float dt_s,
        ChassisOutput &output);

private:
    ...
};
```

## 8.9 Chassis 禁止知道

```text
GM6020
M3508
motor-id
0x200
0x1FE
CAN
Bus
current raw encoding
DTS motor nodes
```

如果这些名字出现在 `swerve_chassis.cpp`，说明分层失败。

## 8.10 验收

纯平移：

```text
vx > 0
vy = 0
wz = 0

四轮：
angle = 0
speed 相等
```

横移：

```text
vx = 0
vy > 0
wz = 0

四轮：
angle = +π/2
speed 相等
```

纯旋转：

```text
vx = 0
vy = 0
wz > 0

四轮方向应切向分布
```

混合：

```text
vx + vy + wz
```

要求：

```text
[ ] 四轮结果有限
[ ] 最大速度超过限制时统一缩放
[ ] 零速度时保持上一次舵角
[ ] Chassis 不接触硬件 API
```

建议提交：

```text
feat(swerve): add four-module chassis kinematics
```

---

# 9. 阶段八：增加一个纯算法舵轮 sample

在真正接 8 台电机前，先做不依赖 CAN 的演示。

## 9.1 新增

例如：

```text
samples/control/swerve_kinematics/
```

或者：

```text
samples/robotics/swerve/
```

目的：

```text
输入若干 vx/vy/wz
打印四个目标角、轮速
```

如果方便，再加入虚拟反馈调用：

```text
SwerveChassis::reset()
SwerveChassis::step()
```

不要一上来就连接真实 8 台电机。

## 9.2 测试场景

固定打印：

```text
forward
left
rotate
forward + rotate
zero
```

检查人工可读结果。

建议提交：

```text
sample(swerve): add kinematics smoke example
```

---

# 10. 阶段九：真实 4 舵 + 4 驱动集成

只有前八阶段完成并稳定后再做。

## 10.1 DTS

建立 8 个 motor node。

目标示例：

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

GM6020：

```text
encoder-zero-ticks
```

每台独立标定。

不要在 C++ 源码写：

```text
4096
```

之类装配零点。

## 10.2 Application 保存真实设备映射

Application 层可以知道：

```text
front_left.steer → 某个 DT motor node
front_left.drive → 某个 DT motor node
```

因为这是具体机器人装配信息。

Chassis/Module 不知道。

## 10.3 每周期数据流

最终控制线程应接近：

```cpp
read steer/drive feedback × 8
        ↓
check all required capabilities
        ↓
check state/freshness
        ↓
construct ChassisFeedback
        ↓
chassis.step(command, feedback, dt, output)
        ↓
motor::setCurrent() × 8
        ↓
flush required buses
```

## 10.4 capability 检查

舵电机至少要求：

```text
FeedbackPosition
FeedbackAbsolutePosition
FeedbackVelocity
CommandCurrent
```

驱动电机至少要求：

```text
FeedbackVelocity
CommandCurrent
```

不要默认“因为它叫 GM6020 就肯定支持”。

## 10.5 Bus 归 Application 管

如果全部在一个 CAN：

```text
dji_bus.attach(...) × 8
dji_bus.arm(...)
...
setCurrent × 8
dji_bus.flush(...)
```

如果拆两条 CAN：

```text
steer_bus
drive_bus
```

则：

```text
setCurrent...
steer_bus.flush(...)
drive_bus.flush(...)
```

SwerveChassis 不改。

## 10.6 fault 策略

任意关键反馈：

```text
offline
stale
invalid
NaN
controller error
setCurrent error
flush error
```

V1 建议：

```text
整车立即进入安全停止
所有相关 bus stop / zero
```

不要尝试“坏一个轮剩三个轮继续跑”。

容错运行以后再单独设计。

## 10.7 arm 顺序

推荐：

```text
1. device ready
2. bus init/attach
3. 等全部 8 台反馈 ready
4. 检查 capability
5. 读一帧 fresh feedback
6. chassis.reset(feedback)
7. arm bus
8. command 初始保持 0
9. 进入周期控制
```

禁止：

```text
未 reset controller 就 arm 后直接给非零目标
```

## 10.8 首次上电安全

第一次硬件测试：

```text
1. 车辆架空
2. 轮胎离地
3. 电流限制设低
4. vx/vy/wz 都限小
5. 先只测试一个 Module
6. 再测试两个
7. 最后四个
8. 确认急停/断电手段随时可用
```

特别检查：

```text
舵角正方向
驱动轮正方向
模块安装坐标
encoder-zero-ticks
wheel radius
gear ratio
```

一个符号错误就可能导致舵轮优化后轮速方向相反。

---

# 11. 推荐的最终文件树

目标不是必须一次性建立，而是施工完成后大致达到：

```text
include/
├── control/
│   ├── angle.h
│   ├── pid.h
│   ├── feedforward.h
│   ├── feedforward_pid.h
│   ├── slew_rate_limiter.h
│   ├── motor_velocity.h
│   └── motor_position.h
│
├── drivers/
│   └── motor/
│       └── ...
│
└── robotics/
    └── swerve/
        ├── swerve_module.hpp
        └── swerve_chassis.hpp

lib/
├── control/
│   ├── angle.c
│   ├── pid.c
│   ├── feedforward.c
│   ├── feedforward_pid.c
│   ├── slew_rate_limiter.c
│   ├── motor_velocity.c
│   └── motor_position.c
│
└── robotics/
    └── swerve/
        ├── CMakeLists.txt
        ├── swerve_module.cpp
        └── swerve_chassis.cpp

drivers/
└── motor/
    └── dji/
        ├── dji_motor.cpp
        ├── dji_internal.hpp
        └── dji_profiles.cpp

samples/
├── motor/
│   ├── dji_speed_control/
│   └── dji_position_control/
│
└── robotics/
    └── swerve/
```

---

# 12. 各层职责总表

| 层 | 可以做 | 不可以做 |
|---|---|---|
| Motor Driver | 编码器解析、连续位置、固定零点单圈角、电流命令编码 | PID、舵轮最短路径 |
| `control/angle` | wrap、unwrap、shortest error、nearest continuous target | 操作电机 |
| Motor Velocity | 速度闭环、电流命令 | CAN、device |
| Motor Position | 位置→速度→公共速度环 | 复制速度 PI |
| SwerveModule | 舵角优化、角度目标转换、轮速单位转换、组合控制器 | CAN flush、DTS |
| SwerveChassis | vx/vy/wz→4 模块目标、速度统一缩放 | PID、电流、型号 |
| Application | 设备绑定、反馈新鲜度、arm、setCurrent、flush、故障停机 | 重写底层控制算法 |

---

# 13. Codex 每阶段工作方式

如果用户已经明确允许修改源码，则每个阶段严格执行：

```text
1. 先读本阶段涉及文件
2. 总结当前行为
3. 只实现当前阶段
4. 不实现下一阶段
5. 检查 git diff
6. 确认无无关改动
7. 给出构建命令
8. 汇报本阶段验收结果
9. 停止，等待进入下一阶段
```

每阶段结束必须报告：

```text
已修改文件
新增接口
保留的旧行为
未修改内容
构建/测试结果
仍未验证的硬件假设
下一阶段名称
```

不要自动连续施工全部阶段，除非用户再次明确要求继续。

---

# 14. 建议构建策略

实际 board 名称以当前仓库/用户环境为准。

每阶段至少单独构建受影响 sample。

示意：

```bash
west build -p always -b <board> samples/motor/dji_speed_control
west build -p always -b <board> samples/motor/dji_position_control
```

加入 swerve sample 后：

```bash
west build -p always -b <board> samples/robotics/swerve
```

如果当前 workspace 的 Zephyr 工程要求从其他目录调用，则先读取现有 README/CMake 配置，不要猜路径。

不要为了“顺手验证”运行自动格式化后覆盖大量历史文件。

---

# 15. 最终验收清单

## 控制库

```text
[ ] velocity controller 已独立
[ ] position controller 组合 velocity controller
[ ] 两者不依赖 device/CAN/Zephyr 时间 API
[ ] transactional state 完整
[ ] 原有 sample 参数第一次迁移时未变
```

## 控制 sample

```text
[ ] speed sample 只通过 motor_velocity 控制
[ ] position sample 只通过 motor_position 控制
[ ] sample 不再直接拼 PID / slew / feedforward
[ ] 两个 sample 保留 freshness / arm / setCurrent / flush / fault stop
[ ] 用户人工调试后基本恢复重构前控制效果
[ ] position sample 默认支持连续相对角模式
[ ] position sample 可切换固定零点绝对角模式
[ ] absolute 模式通过 nearest continuous target 后再进入位置控制器
```

## Motor

```text
[ ] position_rad 仍为连续相对位置
[ ] absolute_position_rad 为固定零点单圈角
[ ] capability 明确区分
[ ] 零点来自 DTS
[ ] encoder ticks per turn 来自 profile
```

## Angle

```text
[ ] shortest angle 可复用
[ ] nearest continuous target 无状态
[ ] ±π 边界行为确定
```

## Module

```text
[ ] 不保存 Bus
[ ] 不调用 setCurrent
[ ] 180°舵角优化正确
[ ] 轮速反向正确
[ ] 绝对角转换到当前附近连续目标
[ ] wheel radius 换算正确
```

## Chassis

```text
[ ] 只做运动学
[ ] 坐标系注释清楚
[ ] 四轮速度统一归一化
[ ] 零速时不乱转舵
[ ] 不出现 GM6020/M3508/CAN ID
```

## Application

```text
[ ] 一次读取 8 台反馈
[ ] freshness/fault 在应用层
[ ] chassis step 后再统一 setCurrent
[ ] 最后统一 flush
[ ] 任意关键错误安全停机
```

---

# 16. 最重要的设计结论

整个工程后续扩展时始终记住：

```text
Motor
  = “我能测什么、我能输出什么”

Controller
  = “误差如何变成电流”

Module
  = “一个机械执行机构应该怎么动作”

Chassis
  = “整车期望如何分解到各执行机构”

Application
  = “真实硬件是谁、什么时候运行、出错怎么办”
```

因此：

```text
换 GM6020 → 达妙
```

应该主要影响：

```text
Driver / Application binding
```

不应该影响：

```text
SwerveChassis
```

而：

```text
四舵轮 → 六舵轮
```

应该主要影响：

```text
Chassis geometry / module count
```

不应该逼着修改：

```text
PID
Motor API
CAN protocol
```

这就是本轮重构最重要的验收标准。

---

# 17. 推荐施工顺序摘要

严格按以下顺序：

```text
阶段 1
抽 motor_velocity
        ↓
阶段 2
抽 motor_position，并组合 velocity
        ↓
阶段 3
Motor Feedback 增 absolute position
        ↓
阶段 4
absolute → nearest continuous target
        ↓
阶段 5
两个 control sample 回接公共控制器
并由用户人工调试恢复原有效果
位置 sample 增加 absolute 模式开关
        ↓
阶段 6
SwerveModule
        ↓
阶段 7
SwerveChassis
        ↓
阶段 8
纯算法 swerve sample
        ↓
阶段 9
真实 4 steer + 4 drive 集成
```

不要跳级。

尤其不要在阶段 1/2 尚未完成时开始写舵轮解算；阶段 4 完成后还必须先完成阶段 5 的两个硬件验证 sample 回接与人工调试。只有公共速度/位置控制链路已经在原测试电机上恢复到可接受效果，才能进入 `SwerveModule`。
