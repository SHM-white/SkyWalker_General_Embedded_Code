# `calculatePositionCurrent()` 总是返回错误的诊断与修正指南

## 1. 结论

当前工作区中存在一个可以确定复现的首要原因：为了关闭摩擦前馈，两个前馈电流被设为 `0.0f`，但三个作为除数的混合区间也同时被设成了 `0.0f`：

```cpp
constexpr float kRunningFrictionCurrentA = 0.0f;
constexpr float kRunningFrictionBlendVelocityRadS = 0.0f; // 除数
constexpr float kBreakawayCurrentA = 0.0f;
constexpr float kBreakawayRequestFullRadS = 0.0f;          // 除数
constexpr float kBreakawayFadeVelocityRadS = 0.0f;         // 除数
```

这些值位于：

```text
samples/motor/dji_position_control/src/main.cpp:39-43
```

`calculateFrictionFeedforward()` 无条件使用三个区间参数做除法：

```cpp
velocity_reference_rad_s / kRunningFrictionBlendVelocityRadS
fabs(velocity_reference_rad_s) / kBreakawayRequestFullRadS
fabs(velocity_rad_s) / kBreakawayFadeVelocityRadS
```

程序启动后的前 5 秒，`requestedPositionRad()` 返回初始位置。因此第一次正常控制周期通常有：

```text
position_target_rad == feedback.position_rad
position_error == 0
position PID output == 0
velocity_reference_rad_s == 0
```

于是第一项直接成为：

```text
0.0f / 0.0f = NaN
```

即使 `kRunningFrictionCurrentA` 也是零，IEEE-754 浮点规则仍然是：

```text
0.0f * NaN = NaN
```

所以故障链是确定的：

```text
启动保持初始位置 5 秒
  -> 速度参考为 0
  -> 摩擦混合计算出现 0/0
  -> friction_feedforward_a = NaN
  -> 电流求和 = NaN
  -> clampFloat(NaN) 仍返回 NaN
  -> std::isfinite(current_command_a) == false
  -> calculatePositionCurrent() 返回 -ERANGE
  -> stopAfterFailure() 停止总线
```

在 Zephyr/newlib 常用 errno 定义中，日志里的 `cause=-34` 就是 `-ERANGE`。如果当前实际观察到的是 `-34`，它与这条路径完全吻合。

## 2. 为什么 `clampFloat()` 没有拦住 NaN

当前实现是：

```cpp
float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}
```

NaN 与任何数做 `<` 或 `>` 比较都返回 false。因此传入 NaN 时，两个分支都不会进入，函数原样返回 NaN。末尾的 `std::isfinite()` 正确地发现了问题并返回 `-ERANGE`；不应删除这个有限数检查。

## 3. 最小修正方案：电流保持为零，区间参数恢复为正数

如果当前意图只是“暂时完全关闭摩擦和起转前馈”，最小且风险最低的修改是：

- 保持两个电流幅值为 `0.0f`；
- 把三个分母恢复为原来合法的正数。

在 `samples/motor/dji_position_control/src/main.cpp` 的常量区手工改成：

```cpp
constexpr float kRunningFrictionCurrentA = 0.0f;
constexpr float kRunningFrictionBlendVelocityRadS = 0.10f;
constexpr float kBreakawayCurrentA = 0.0f;
constexpr float kBreakawayRequestFullRadS = 0.05f;
constexpr float kBreakawayFadeVelocityRadS = 0.20f;
```

两个系数为零时，最终摩擦前馈仍然是 `0 A`；但中间的比例全部保持有限，不再产生 `0/0`。

这是本次最推荐的第一步，因为它只修复非法数学运算，不改变当前想要的控制策略，也不顺带恢复原来的摩擦补偿电流。

### 自检点

修改后，在第一次控制周期检查：

```text
velocity_reference_rad_s = 0
friction_feedforward_a = 0
current_command_a 是有限数
calculatePositionCurrent() 返回 0
```

## 4. 更稳健的长期修正：关闭某项功能时根本不执行它的除法

仅恢复正分母可以立刻解决当前故障，但以后再次把区间改成零仍会复发。更稳健的实现应让“幅值为零”真正代表该前馈分支关闭。

### 4.1 改造 `calculateFrictionFeedforward()`

在同一文件中，把函数按下面思路手工修改：

```cpp
float calculateFrictionFeedforward(float velocity_reference_rad_s,
                                   float velocity_rad_s)
{
    float running_term = 0.0f;
    if (kRunningFrictionCurrentA != 0.0f) {
        const float running_blend = clampFloat(
            velocity_reference_rad_s /
                kRunningFrictionBlendVelocityRadS,
            -1.0f,
            1.0f);
        running_term =
            kRunningFrictionCurrentA * running_blend;
    }

    float breakaway_term = 0.0f;
    if (kBreakawayCurrentA != 0.0f) {
        const float direction =
            velocity_reference_rad_s > 0.0f ? 1.0f :
            velocity_reference_rad_s < 0.0f ? -1.0f : 0.0f;
        const float request_blend = clampFloat(
            std::fabs(velocity_reference_rad_s) /
                kBreakawayRequestFullRadS,
            0.0f,
            1.0f);
        const float stall_blend = clampFloat(
            1.0f - std::fabs(velocity_rad_s) /
                kBreakawayFadeVelocityRadS,
            0.0f,
            1.0f);
        breakaway_term = kBreakawayCurrentA * direction *
                         request_blend * stall_blend;
    }

    return running_term + breakaway_term;
}
```

这样：

- `kRunningFrictionCurrentA == 0` 时不计算 running 分支；
- `kBreakawayCurrentA == 0` 时不计算 breakaway 分支；
- 不会先产生 NaN 再指望乘以零把它消掉。

### 4.2 在 `validateController()` 中验证分母

长期方案还应在 arm 之前拒绝“功能已启用，但对应区间不为正”的配置。保持现有 PID 和 slew 校验后，再加入等价逻辑：

```cpp
if (kRunningFrictionCurrentA != 0.0f &&
    kRunningFrictionBlendVelocityRadS <= 0.0f) {
    return -EINVAL;
}

if (kBreakawayCurrentA != 0.0f &&
    (kBreakawayRequestFullRadS <= 0.0f ||
     kBreakawayFadeVelocityRadS <= 0.0f)) {
    return -EINVAL;
}
```

状态语义：

- 校验在 `dji_bus.arm()` 之前发生；
- 配置非法返回 `-EINVAL`，不让电机进入已使能状态；
- 配置合法时不改变任何控制器状态；
- `calculatePositionCurrent()` 仍维持“全部成功才提交 next state”的事务式行为。

## 5. 第二个可能同时出现的错误：调试断点导致 `dt_s > 0.020`

函数还会把两个 PID 的错误原样向上传递。位置和速度 PID 都要求：

```text
0.001 s <= dt_s <= 0.020 s
```

相关配置在 `main.cpp:134-135`，检查在 `lib/control/pid.c:102-105`。正常 5 ms 控制周期通常满足条件，但以下情况会让 `dt_s` 超过 20 ms：

- 在控制循环或 `calculatePositionCurrent()` 内打断点；
- 单步执行后再继续；
- 日志、CAN 或其他高优先级工作长时间阻塞线程；
- 无线 DAPLink 暂停 CPU 后再恢复。

此时第一个位置 PID 调用就会返回 `-ERANGE`，函数不会走到摩擦前馈。这与“零分母导致的末尾 `-ERANGE`”是两条不同路径。

### 如何区分

不要在运行中的电机控制环内长时间停断点。先断开电机功率，仅保留逻辑供电，然后观察错误发生位置：

| 返回位置 | 原因 |
| --- | --- |
| `main.cpp:220-222` 的第一个 `return ret` | 位置 PID；优先看 `dt_s` 是否超过 0.020 |
| `main.cpp:230-232` | slew limiter；看是否 reset、`dt_s` 是否大于 0 |
| `main.cpp:241-243` | 速度 PID；优先看 `dt_s` 和状态有限性 |
| `main.cpp:249-251` | 最终电流非有限；当前零分母/NaN 会到这里 |

推荐用日志或 VOFA 遥测观察 `dt_s` 和阶段编号，不用断点暂停闭环。若必须单步，只能把电机功率断开，并在恢复实时运行前复位控制器和周期时间基准。

不要简单把 `dt_max_s` 改成几秒来迁就断点。PID 用一个巨大的 `dt_s` 积分会产生不真实的状态跳变，恢复电机功率时不安全。

## 6. 其他错误出口核对

`calculatePositionCurrent()` 的接口为：

```cpp
int calculatePositionCurrent(PositionController &controller,
                             const skywalker::motor::Feedback &feedback,
                             float position_target_rad,
                             float dt_s,
                             PositionControlOutput &output);
```

参数与状态：

- `controller`：位置 PID、速度参考斜坡和速度 PID 的持久状态；成功才提交更新，失败保持原状态。
- `feedback`：位置和速度反馈。当前函数本身没有完整验证，但唯一调用方在进入前通过 `readFreshPositionFeedback()` 检查有效位、有限数和时间戳。
- `position_target_rad`：目标连续位置，必须是有限数。
- `dt_s`：本周期秒数；PID 要求在 1～20 ms 内，slew 要求大于零。
- `output`：只有成功时才覆盖；失败时调用方拿不到半成品输出。

错误映射：

| 错误码 | 可能来源 |
| --- | --- |
| `-EINVAL` | 目标/dt/反馈/控制状态含 NaN 或 Inf；PID 配置非法；指针非法 |
| `-ERANGE` | PID 的 dt 超界；PID 数学结果溢出；slew 结果非有限；最终电流为 NaN/Inf |
| `-EACCES` | slew state 未先调用 reset 初始化 |

当前调用顺序已经在 arm 之前执行 `resetController()`，因此正常启动下 `-EACCES` 的概率低。调用方也已经过滤非有限反馈。结合工作区 diff，三个零分母仍是“每次启动都失败”的最高置信度根因。

## 7. 当前改动中的附加安全风险

这不是 `calculatePositionCurrent()` 返回错误的直接原因，但当前工作区还同时做了以下改动：

```text
kVelocityAbsMaxRadS: 0.30 -> 5.0
kVelocityRampRate*: 4.0 -> 25.0
速度反馈 1.0 rad/s 安全停机检查被注释
```

修好 NaN 后，程序将不再立即停机，以上更激进的速度和斜率就会真正生效。因此硬件测试前必须：

1. 恢复速度安全检查；
2. 断开或悬空机构，准备物理断电；
3. 保持软件电流限制 `0.10 A`，不要同时提高；
4. 先让目标保持初始位置，确认电流、速度和反馈方向；
5. 再进行小幅位置阶跃，不要第一次就带载执行 3 rad。

## 8. 建议验证顺序

### 第一步：只做最小参数修正

把三个分母恢复为 `0.10f`、`0.05f`、`0.20f`，两个前馈电流继续保持零。

自检：搜索常量，确认没有任何启用中的计算以零为分母。

### 第二步：构建

在 Zephyr IDE 中重新构建当前 `dji_position_control` 配置，或在工作区环境中运行：

```bash
../.venv/bin/west build samples/motor/dji_position_control \
  --build-dir samples/motor/dji_position_control/build/rm_typec/stm32f407xx \
  --board rm_typec/stm32f407xx
```

预期构建成功，不出现 C++ 有限数或类型错误。

### 第三步：断开电机功率进行软件路径检查

不要靠断点停住实时闭环。临时记录或遥测：

```text
dt_s
velocity_reference_rad_s
friction_feedforward_a
overspeed_damping_a
current_command_a
ret
```

预期首周期 `friction_feedforward_a == 0`，`current_command_a` 有限，函数返回 0。

### 第四步：受控上电

恢复速度安全检查后，机构悬空、人员远离、物理断电可触达，再上电验证。若仍返回 `-ERANGE`，先看实际 `dt_s`；不要先改 PID 增益。

## 9. 最终检查清单

- [ ] 两个前馈电流为零时，三个混合区间仍为正数。
- [ ] 首周期没有 `0/0`、NaN 或 Inf。
- [ ] `calculateFrictionFeedforward()` 关闭分支时不执行无意义除法。
- [ ] `validateController()` 能拒绝启用分支的非正分母。
- [ ] 不在实时电机闭环中停断点。
- [ ] 正常运行的 `dt_s` 位于 0.001～0.020 s。
- [ ] 速度安全停机检查已经恢复。
- [ ] 电机功率测试前已准备物理断电并让机构悬空。

## 10. 本次诊断边界

- 已只读检查当前未提交的 `main.cpp` 修改、`calculatePositionCurrent()` 完整调用链、PID 和 slew limiter 的错误返回条件、初始化顺序及现有调参指南。
- 已对比提交版本，确认三个分母原值分别为 `0.10f`、`0.05f`、`0.20f`，当前工作区把它们全部改成了零。
- 未获得本次目标板的完整 `cause=` 日志、实际 `dt_s` 和断点使用情况，因此“是否还叠加断点导致 dt 超界”尚未现场验证。
- 按仓库古法编程规则，未构建、未测试、未烧录，也未修改业务源码；本指南是本次唯一新增文件。
