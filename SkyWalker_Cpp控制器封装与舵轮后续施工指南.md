# SkyWalker 控制器 C++ 封装与后续舵轮施工指南

> 目标仓库：`SHM-white/SkyWalker_General_Embedded_Code`
>
> 目标分支：`dev`
>
> 前置条件：此前施工指南中的阶段 1～4 已完成：
>
> 1. 已抽取 `control_motor_velocity`
> 2. 已抽取 `control_motor_position`，并复用公共 velocity controller
> 3. Motor Feedback 已支持固定零点单圈绝对角
> 4. 已实现 absolute angle → nearest continuous target
>
> 本文从阶段 5 开始继续施工。
>
> **本任务如果要真正修改源码，用户必须明确授权禁用仓库默认的“古法编程模式”。**

---

# 0. 本轮架构调整

此前底层 C API 保留不动，但上层不再直接大量操作：

```text
config
state
input
output
```

四组 C 结构体。

新增一层非常薄的 C++ wrapper：

```text
C 数值实现
    ↓
C API
    ↓
C++ Controller Wrapper
    ↓
SwerveModule
    ↓
SwerveChassis
```

最终结构：

```text
lib/control/*.c
    │
    │ 纯数值算法
    ▼
control_motor_velocity
control_motor_position
    │
    │ C API
    ▼
VelocityController
PositionController
    │
    │ C++ 易用接口
    ▼
SwerveModule
    │
    ▼
SwerveChassis
    │
    ▼
Application
```

核心原则：

> **C 层是真正实现，C++ 层只是 config + state + 易用接口。**

绝对禁止在 C++ wrapper 重新实现 PID、slew、filter、feedforward。

---

# 1. 新的后续施工顺序

从阶段 4 完成后，严格按以下顺序：

```text
阶段 5
增加 C++ Controller Wrapper
VelocityController / PositionController
        ↓
阶段 6
重新修改两个 motor sample
全部通过 C++ Controller 控制
并由用户人工调试恢复原有效果
位置 sample 增加绝对角模式
        ↓
阶段 7
SwerveModule
        ↓
阶段 8
SwerveChassis
        ↓
阶段 9
纯算法 Swerve sample
        ↓
阶段 10
4 舵 + 4 驱真实硬件集成
```

进入下一阶段前，上一阶段必须完成编译和基础验收。

---

# 2. 阶段五：增加 C++ Controller Wrapper

## 2.1 目标

为以下 C API 增加易用的 C++ 封装：

```text
control_motor_velocity_*
control_motor_position_*
```

新增：

```cpp
skywalker::control::VelocityController
skywalker::control::PositionController
```

这两个类只负责：

```text
保存 config
保存 state
保存最近一次 output
简化 reset()
简化 step()
```

不负责：

```text
Motor device
CAN
Bus
时间
日志
线程
反馈 freshness
arm
setCurrent
flush
```

---

# 3. 推荐文件结构

建议直接放在现有 control include 目录：

```text
include/control/
├── motor_velocity.h
├── motor_position.h
├── velocity_controller.hpp
└── position_controller.hpp
```

V1 推荐做成：

```text
header-only
```

原因：

```text
wrapper 极薄
没有必要再增加两个 .cpp
减少 CMake 改动
调用全部可 inline
```

如果仓库现有规范更偏向 `.hpp + .cpp`，允许拆分，但不要因此增加复杂度。

---

# 4. VelocityController 设计

## 4.1 推荐类接口

```cpp
#pragma once

#include <control/motor_velocity.h>

namespace skywalker::control {

class VelocityController {
public:
    using Config = control_motor_velocity_config;
    using State = control_motor_velocity_state;
    using Input = control_motor_velocity_input;
    using Output = control_motor_velocity_output;

    explicit VelocityController(const Config &config)
        : config_(config)
    {
    }

    int validate() const
    {
        return control_motor_velocity_validate(&config_);
    }

    int reset(
        float measured_velocity_rad_s,
        float initial_reference_rad_s = 0.0f)
    {
        const int ret = control_motor_velocity_reset(
            &state_,
            measured_velocity_rad_s,
            initial_reference_rad_s);

        if (ret == 0) {
            initialized_ = true;
            last_output_ = {};
        }

        return ret;
    }

    int step(
        const Input &input,
        Output &output)
    {
        if (!initialized_) {
            return -EACCES;
        }

        const int ret = control_motor_velocity_step(
            &state_,
            &config_,
            &input,
            &output);

        if (ret == 0) {
            last_output_ = output;
        }

        return ret;
    }

    int step(
        float requested_velocity_rad_s,
        float measured_velocity_rad_s,
        float dt_s,
        Output &output)
    {
        Input input{
            .requested_velocity_rad_s = requested_velocity_rad_s,
            .measured_velocity_rad_s = measured_velocity_rad_s,
            .position_reference_rad = 0.0f,
            .dt_s = dt_s,
            .freeze_integrator = false,
        };

        return step(input, output);
    }

    const Config &config() const
    {
        return config_;
    }

    const State &state() const
    {
        return state_;
    }

    const Output &lastOutput() const
    {
        return last_output_;
    }

    bool initialized() const
    {
        return initialized_;
    }

private:
    Config config_{};
    State state_{};
    Output last_output_{};
    bool initialized_ = false;
};

} // namespace skywalker::control
```

具体字段名必须以阶段 1 最终实际接口为准。

不要为了照抄本文示例而修改已经稳定的 C API。

---

# 5. VelocityController 必须保留完整 Input 重载

不能只提供：

```cpp
step(target, measured, dt)
```

因为未来可能使用：

```text
position_reference_rad
freeze_integrator
```

例如：

```text
重力前馈
特殊运行状态冻结积分
```

因此推荐同时提供：

```cpp
int step(const Input &, Output &);
```

作为完整接口。

简化重载只是常见情况的 convenience API。

---

# 6. PositionController 设计

推荐：

```cpp
#pragma once

#include <control/motor_position.h>

namespace skywalker::control {

class PositionController {
public:
    using Config = control_motor_position_config;
    using State = control_motor_position_state;
    using Input = control_motor_position_input;
    using Output = control_motor_position_output;

    explicit PositionController(const Config &config)
        : config_(config)
    {
    }

    int validate() const
    {
        return control_motor_position_validate(&config_);
    }

    int reset(
        float measured_position_rad,
        float measured_velocity_rad_s)
    {
        const int ret = control_motor_position_reset(
            &state_,
            &config_,
            measured_position_rad,
            measured_velocity_rad_s);

        if (ret == 0) {
            initialized_ = true;
            last_output_ = {};
        }

        return ret;
    }

    int step(
        const Input &input,
        Output &output)
    {
        if (!initialized_) {
            return -EACCES;
        }

        const int ret = control_motor_position_step(
            &state_,
            &config_,
            &input,
            &output);

        if (ret == 0) {
            last_output_ = output;
        }

        return ret;
    }

    int step(
        float target_position_rad,
        float measured_position_rad,
        float measured_velocity_rad_s,
        float dt_s,
        Output &output)
    {
        Input input{
            .continuous_target_rad = target_position_rad,
            .continuous_position_rad = measured_position_rad,
            .measured_velocity_rad_s = measured_velocity_rad_s,
            .dt_s = dt_s,
        };

        return step(input, output);
    }

    const Config &config() const
    {
        return config_;
    }

    const State &state() const
    {
        return state_;
    }

    const Output &lastOutput() const
    {
        return last_output_;
    }

    bool initialized() const
    {
        return initialized_;
    }

private:
    Config config_{};
    State state_{};
    Output last_output_{};
    bool initialized_ = false;
};

} // namespace skywalker::control
```

---

# 7. Wrapper 的错误处理原则

仍然沿用项目现有：

```text
int
0 = success
<0 = errno style error
```

不要引入：

```text
exceptions
std::expected
动态异常机制
```

V1 不要改变整个项目的错误处理风格。

未 reset 就调用：

```cpp
step(...)
```

建议：

```text
-EACCES
```

非法 config：

```text
继续由 C API validate 返回
```

---

# 8. Wrapper 的 transactional 原则

真正的事务性状态由 C 核心保证。

C++ wrapper 不要自己实现另一套 rollback。

正确：

```cpp
const int ret = control_motor_velocity_step(
    &state_,
    &config_,
    &input,
    &output);

if (ret == 0) {
    last_output_ = output;
}
```

错误时：

```text
state 是否改变
```

应该由底层 C controller 的契约决定。

根据前序阶段要求：

```text
失败时 state 不提交
```

wrapper 只依赖这一契约。

---

# 9. Wrapper 禁止持有硬件

禁止：

```cpp
class VelocityController {
    const device *motor_;
    DjiBus *bus_;
};
```

禁止：

```cpp
controller.step(...)
{
    motor::setCurrent(...);
}
```

Controller 始终只是：

```text
target + feedback + dt
        ↓
current command
```

---

# 10. 阶段五验收

要求：

```text
[ ] VelocityController 存在
[ ] PositionController 存在
[ ] 两者内部直接调用对应 C API
[ ] 没有重新实现 PID
[ ] 没有重新实现 filter
[ ] 没有重新实现 slew
[ ] 没有 Motor device
[ ] 没有 CAN / Bus
[ ] 没有 Zephyr time API
[ ] 未 reset 调 step 会明确失败
[ ] 可读取 lastOutput()
[ ] 原 C API 仍然可直接使用
```

建议提交：

```text
feat(control): add C++ motor controller wrappers
```

完成后再进入 sample 改造。

---

# 11. 阶段六：重新改造两个 motor sample

目标 sample：

```text
samples/motor/dji_speed_control/
samples/motor/dji_position_control/
```

从本阶段开始：

> **sample 不再直接调用 C motor controller API。**

而是通过：

```cpp
skywalker::control::VelocityController
skywalker::control::PositionController
```

控制。

底层 C API 仍然存在，但 sample 和后续 Module 默认不直接碰：

```text
state
config
input
output
```

大量裸结构体组合。

---

# 12. 速度 sample 最终结构

原来可能类似：

```cpp
control_motor_velocity_state state{};
control_motor_velocity_input input{};
control_motor_velocity_output output{};

control_motor_velocity_step(
    &state,
    &config,
    &input,
    &output);
```

改成：

```cpp
skywalker::control::VelocityController controller{
    velocity_config
};
```

初始化：

```cpp
ret = controller.validate();

ret = controller.reset(
    feedback.velocity_rad_s,
    0.0f);
```

循环：

```cpp
control_motor_velocity_output control_output{};

ret = controller.step(
    target_velocity_rad_s,
    feedback.velocity_rad_s,
    dt_s,
    control_output);

ret = skywalker::motor::setCurrent(
    motor,
    control_output.current_command_a);

ret = dji_bus.flush(...);
```

如果 sample 需要：

```text
freeze_integrator
position_reference
```

则使用完整 Input 重载：

```cpp
VelocityController::Input input{...};

ret = controller.step(
    input,
    control_output);
```

---

# 13. 速度 sample 仍然属于 Application

必须继续负责：

```text
device ready
Bus init
attach
反馈等待
freshness
controller reset
arm
测试目标生成
setCurrent
flush
telemetry
fault stop
```

不要把这些塞进：

```cpp
VelocityController
```

---

# 14. 位置 sample 最终结构

创建：

```cpp
skywalker::control::PositionController controller{
    position_config
};
```

初始化：

```cpp
ret = controller.validate();

ret = controller.reset(
    feedback.position_rad,
    feedback.velocity_rad_s);
```

控制：

```cpp
control_motor_position_output control_output{};

ret = controller.step(
    continuous_target_rad,
    feedback.position_rad,
    feedback.velocity_rad_s,
    dt_s,
    control_output);

ret = skywalker::motor::setCurrent(
    motor,
    control_output.current_command_a);
```

---

# 15. 位置 sample 必须保留两种目标模式

增加：

```cpp
enum class PositionTargetMode : std::uint8_t {
    ContinuousRelative = 0,
    FixedZeroAbsolute,
};
```

sample 顶部提供简单模式开关：

```cpp
constexpr PositionTargetMode kPositionTargetMode =
    PositionTargetMode::ContinuousRelative;
```

默认：

```text
ContinuousRelative
```

原因：

```text
先恢复旧 sample 的控制效果
再测试 absolute 模式
```

---

# 16. ContinuousRelative 模式

直接：

```text
requested continuous target
        ↓
PositionController
```

反馈：

```text
feedback.position_rad
feedback.velocity_rad_s
```

示意：

```cpp
continuous_target_rad =
    requestedRelativePositionForTime(elapsed_ms);

ret = controller.step(
    continuous_target_rad,
    feedback.position_rad,
    feedback.velocity_rad_s,
    dt_s,
    control_output);
```

这一模式禁止：

```text
absolute position conversion
shortest path
swerve optimization
```

它就是原位置 sample 行为的重构后版本。

---

# 17. FixedZeroAbsolute 模式

绝对模式中：

```text
sample
```

负责解释目标。

Controller 仍然只接收连续角度。

完整流程：

```text
requested_absolute_rad
        ↓
feedback.absolute_position_rad
feedback.position_rad
        ↓
control_angle_nearest_continuous_target()
        ↓
continuous_target_rad
        ↓
PositionController
        ↓
current
```

示意：

```cpp
float continuous_target_rad = 0.0f;

ret = control_angle_nearest_continuous_target(
    requested_absolute_rad,
    feedback.absolute_position_rad,
    feedback.position_rad,
    &continuous_target_rad);

if (ret < 0) {
    stopAfterFailure(...);
}

ret = controller.step(
    continuous_target_rad,
    feedback.position_rad,
    feedback.velocity_rad_s,
    dt_s,
    control_output);
```

注意：

> `PositionController` 完全不知道现在是 absolute 模式。

---

# 18. absolute 模式 capability 检查

进入 arm 前检查：

```text
CommandCurrent
FeedbackPosition
FeedbackVelocity
FeedbackAbsolutePosition
```

如果：

```text
FeedbackAbsolutePosition
```

不存在：

```text
拒绝运行
不要退化
不要假装 absolute = relative
```

---

# 19. sample 禁止直接调用底层算法

阶段六完成后，sample 中不应该出现直接调用：

```c
control_pid_step()
control_feedforward_pid_step()
control_slew_rate_step()
control_motor_velocity_step()
control_motor_position_step()
```

唯一例外：

```c
control_angle_nearest_continuous_target()
```

因为它是一个无状态目标坐标转换函数，不属于 Controller object。

---

# 20. sample telemetry

C++ wrapper 必须保证仍然可以访问控制诊断。

通过：

```cpp
controller.lastOutput()
```

或者当前周期：

```cpp
control_output
```

输出。

速度 sample 至少观察：

```text
requested velocity
reference velocity
measured velocity
filtered velocity
velocity error
P
I
D
feedforward
current command
saturation
feedback age
```

位置 sample 至少观察：

```text
requested target
continuous target
continuous position
absolute position（absolute 模式）
position error
position controller output velocity
velocity reference
measured velocity
velocity error
current command
feedback age
```

不要为了方便调试：

```text
把 PID state 暴露成 public 成员
```

优先使用现有 output/result。

---

# 21. 人工调试要求

Codex 只负责：

```text
修改代码
确保结构正确
确保编译通过
保留 telemetry
提供人工测试步骤
```

真实效果由用户人工调试。

Codex 不得宣称：

```text
已经恢复原有效果
已经实机稳定
已经完成参数整定
```

除非用户实际反馈测试结果。

---

# 22. 人工调试顺序

严格：

```text
1. dji_speed_control
2. dji_position_control / ContinuousRelative
3. dji_position_control / FixedZeroAbsolute
```

不要直接从 absolute 模式开始。

---

# 23. 速度环人工调试

先保持重构前参数：

```text
kp
ki
kd
filter tau
soft deadband
slew
current limit
```

检查：

```text
目标速度方向
目标速度能否达到
稳态误差
上升时间
超调
起停
电流
振荡
```

如果和旧 sample 差别明显：

> 第一优先检查 wrapper / controller 抽取是否改变行为。

不要立即开始大幅重新调 PID。

---

# 24. 位置相对模式人工调试

模式：

```cpp
PositionTargetMode::ContinuousRelative
```

人工比较旧 sample：

```text
到位速度
超调
回拉
振荡
稳态误差
最大速度
最大电流
```

目标：

> **恢复到旧 sample 基本相同的控制效果。**

位置环此前如果存在：

```text
ki != 0
但 freeze_integrator = true
kd != 0
```

则继续保持。

不要因为注释写着 P-only 就顺手删除 D。

---

# 25. absolute 模式人工测试

相对模式稳定后切换：

```cpp
PositionTargetMode::FixedZeroAbsolute
```

测试：

```text
0° → +20°
+20° → -20°
-20° → 0°
```

再测试边界：

```text
+170° → -170°
```

期望：

```text
约 +20° 最短路径
```

反向：

```text
-170° → +170°
```

期望：

```text
约 -20°
```

不应该走：

```text
340°
```

---

# 26. 阶段六完成标准

速度 sample：

```text
[ ] 使用 VelocityController
[ ] 不直接调用 motor_velocity C step
[ ] 不直接拼 PID/slew/feedforward
[ ] freshness / arm / setCurrent / flush 仍在 sample
[ ] telemetry 足够人工调参
[ ] 用户人工确认效果基本恢复
```

位置 sample：

```text
[ ] 使用 PositionController
[ ] 不直接调用 motor_position C step
[ ] ContinuousRelative 模式可用
[ ] FixedZeroAbsolute 模式可用
[ ] absolute 模式先转 nearest continuous target
[ ] PositionController 不理解 absolute angle
[ ] 用户人工确认相对模式效果基本恢复
[ ] 用户人工确认 ±170° 跨边界走最短路径
```

建议拆提交：

```text
refactor(sample): use C++ velocity controller wrapper

refactor(sample): use C++ position controller wrapper

feat(sample): add absolute position target mode
```

---

# 27. 阶段七：SwerveModule 改用 C++ Controller

之后实现：

```cpp
class SwerveModule
```

内部不要再保存：

```text
control_motor_position_state
control_motor_velocity_state
```

改成直接组合：

```cpp
skywalker::control::PositionController steer_controller_;
skywalker::control::VelocityController drive_controller_;
```

结构：

```cpp
class SwerveModule {
public:
    ...

private:
    ModuleConfig config_;

    skywalker::control::PositionController steer_controller_;
    skywalker::control::VelocityController drive_controller_;

    bool initialized_ = false;
};
```

---

# 28. SwerveModule step 应更简单

目标：

```cpp
int SwerveModule::step(
    const ModuleTarget &target,
    const ModuleFeedback &feedback,
    float dt_s,
    ModuleOutput &output)
{
    // 1. optimize wheel angle / velocity

    // 2. absolute angle -> nearest continuous target

    // 3. steer controller

    // 4. wheel m/s -> drive rad/s

    // 5. drive controller

    // 6. assemble output
}
```

调用接近：

```cpp
ret = steer_controller_.step(
    steer_continuous_target,
    feedback.steer_continuous_rad,
    feedback.steer_velocity_rad_s,
    dt_s,
    steer_output);

ret = drive_controller_.step(
    drive_target_rad_s,
    feedback.drive_velocity_rad_s,
    dt_s,
    drive_output);
```

不再手工构造大批底层 C state。

---

# 29. SwerveModule 仍然不碰硬件

虽然已经是 C++ 类，也禁止：

```text
device *
motor::setCurrent()
DjiBus
CAN
flush
readFeedback
```

SwerveModule 输出：

```text
steer_current_a
drive_current_a
```

Application 再提交到硬件。

---

# 30. 阶段八：SwerveChassis

继续维持此前架构：

```text
ChassisCommand
vx / vy / wz
      ↓
SwerveChassis
      ↓
4 × ModuleTarget
      ↓
4 × SwerveModule
```

Chassis 不知道 Controller 内部。

因此：

```text
VelocityController / PositionController
```

不会泄漏到 `SwerveChassis` 接口。

---

# 31. 最终依赖关系

最后必须保持：

```text
control/*.c
      ↓
C API
      ↓
VelocityController / PositionController
      ↓
SwerveModule
      ↓
SwerveChassis
      ↓
Application
      ↓
Motor API
      ↓
Motor Driver / Bus / CAN
```

其中硬件命令实际上由 Application 提交，因此更准确：

```text
                       ┌──────── Motor API → Driver → CAN
                       │
Application ───────────┤
    │                  │
    ▼                  │
SwerveChassis          │
    ↓                  │
SwerveModule           │
    ↓                  │
Controllers ───────────┘ current output
```

---

# 32. 最终职责表

| 层 | 负责 | 不负责 |
|---|---|---|
| C PID / angle / FF | 纯数学 | hardware |
| C motor_velocity | 速度闭环 | Motor |
| C motor_position | 位置串级控制 | Motor |
| VelocityController | 保存 velocity config/state，简化调用 | PID 重实现、CAN |
| PositionController | 保存 position config/state，简化调用 | absolute target 解释 |
| SwerveModule | 舵角优化、目标转换、两个 Controller 组合 | Bus |
| SwerveChassis | 运动学 | PID |
| Application | feedback、freshness、arm、setCurrent、flush、fault | 重写控制算法 |

---

# 33. 重要禁止事项

整个后续施工禁止：

```text
1. 删除 C controller，只留下 C++ 实现
2. C++ wrapper 重写 PID
3. wrapper 保存 device*
4. wrapper 调 setCurrent()
5. PositionController 自己处理 absolute angle
6. sample 绕过 C++ wrapper
7. SwerveModule 绕过 wrapper 直接维护 C state
8. SwerveChassis 看到 GM6020 / M3508
9. SwerveChassis 看到 CAN ID
10. 为了调用方便给 Motor API 增 setPosition()
```

---

# 34. Codex 执行要求

如果当前任务明确允许修改源码：

```text
1. 先检查阶段 1～4 实际代码是否已完成
2. 以实际 API 字段名为准
3. 完成阶段 5 后停止
4. 编译相关 sample
5. 汇报 diff 和构建结果
6. 用户确认后再进入阶段 6
```

不要一次把阶段 5～10 全部实现。

特别是：

> **阶段 6 包含真实硬件人工调试，Codex 无法替代用户完成。**

因此 Codex 完成 sample 改造后，应输出：

```text
需要用户人工测试的步骤
应该观察哪些 telemetry
测试结果如何反馈
```

等待用户提供实机结果，再根据结果修正参数或实现。

---

# 35. 当前推荐的第一条 Codex 任务

可以直接执行：

```text
本任务禁用古法编程模式。

请读取仓库当前 dev 分支以及本施工指南。

只执行“阶段五：增加 C++ Controller Wrapper”，不要进入阶段六。

要求：
1. 保留现有 control_motor_velocity / control_motor_position C API。
2. 新增 VelocityController 和 PositionController 薄 C++ wrapper。
3. wrapper 只负责 config/state/output 生命周期与调用简化。
4. 禁止重新实现任何 PID、滤波、slew、feedforward。
5. 禁止接触 Motor device、CAN、Bus。
6. 根据仓库当前真实 C API 调整本文接口草图，不要反向修改稳定 C API 去迁就草图。
7. 完成后构建受影响目标，检查 git diff。
8. 汇报修改文件、接口、构建结果，然后停止。
```

阶段五完成并审查后，再给 Codex：

```text
只执行阶段六。
```

这样最容易控制重构质量。

---

# 36. 最终结论

本轮结构最终确定为：

```text
C 核心算法
    +
C++ 易用封装
    +
C++ Module
    +
C++ Chassis
```

而不是：

```text
全部 C
```

也不是：

```text
把所有底层算法全部重写成 class
```

这是当前工程最合适的折中：

```text
底层确定、可测试
上层调用直观
没有双份控制算法
SwerveModule 代码不会被 C struct 淹没
未来云台 / 轮腿等机构也可以直接复用 Controller wrapper
```
