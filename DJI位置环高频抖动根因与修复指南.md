# DJI GM6020 位置环“手拨后高频抖动”根因与修复指南

## 1. 修订后的结论

用户已经实测确认：

```text
负电流 → 负速度 → 顺时针
正电流 → 正速度 → 逆时针
```

因此，当前代码的速度反馈方向是正确的：正速度时控制器输出负电流，确实是在制动。
**不要加入 `-1` 极性映射，也不要使用负 Kp。**

速度控制样例已经实测正常，因此可以进一步排除“DJI 电流换算、速度反馈符号、CAN
刷新或速度控制实现整体错误”。问题集中在**速度环被放进位置串级环后的参数和工作点**。

当前故障更符合以下三个因素叠加：

1. **静止时太弱。** `0.25 rad` 的位置误差只产生约 `1.5 mA`，远低于同台架约
   `50 mA` 的起转电流，所以电机不会主动到达目标。
2. **动起来后又太强。** 手拨后速度误差一旦超过 `2 rad/s`，内环立即撞到
   `±200 mA`。对空载、低惯量 GM6020 来说，这个反向制动力能在一个或少数控制拍内
   把速度推过零点；下一拍控制器又给相反的满幅电流，形成离散极限环。
3. **目标速度小于反馈分辨率。** DJI 速度反馈是整数 rpm，1 rpm 约为
   `0.1047 rad/s`；当前外环只请求 `0.015 rad/s`，仅约 `0.143 rpm`，速度反馈无法
   分辨这个目标。当前 Kp 下每跳 1 rpm，电流就跳约 `10.5 mA`。

在未饱和且前馈为 0 时，当前串级环可近似写成：

```text
电流 = Kp_vel × (Kp_pos × 位置误差 - 实际速度)
     = 0.006 × 位置误差 - 0.1 × 实际速度
```

这里的位置“刚度”只有 `0.006 A/rad`，速度“阻尼”却是 `0.1 A/(rad/s)`。对
`0.25 rad` 静态误差只给 `1.5 mA`，对 `2 rad/s` 手拨速度却给到 `200 mA`；后者是
前者的约 133 倍。这套参数更像**强力零速刹车**，不像把轴拉向目标位置的弹簧。

一句话概括：**控制器在静摩擦区推不动，一旦被手带出静摩擦区，位置恢复力仍很弱，
速度制动却立即打满并反复穿越零速，于是整体不转但高频抖动。**

## 2. 检查范围与明确假设

只读检查了：

- `samples/motor/dji_position_control/src/main.cpp`
- `samples/motor/dji_position_control/app.overlay`
- `samples/motor/dji_speed_control/src/main.cpp`
- `samples/motor/dji_unified/src/main.cpp`
- `drivers/motor/dji/dji_motor.cpp`
- `lib/control/pid.c`、`feedforward_pid.c`、`feedforward.c`
- 用户提供的 141 行运行日志

假设实物是 overlay 中的 GM6020、ID 7、直驱 `1:1`，输出轴近似空载。若实际上有减速
机构、机械限位或较大负载，下面的电流和增益只能作为低能量起点，不能直接当成最终值。

## 3. 为什么日志与这个判断吻合

### 3.1 目标出现后仍然不转

日志连续出现：

```text
target=250 pos=0 pos_err=250 vel_ref=15 vel=0 current=1 mA sat=0
```

源码对应的计算是：

```text
0.25 rad × 位置 Kp 0.06 = 0.015 rad/s
0.015 rad/s × 速度 Kp 0.1 = 0.0015 A ≈ 1.5 mA
```

这说明通信、目标位置和基本单位都在工作，只是电流小到克服不了静摩擦。

### 3.2 手拨后制动过猛并来回穿零

116 条 `target=250` 遥测中：

- 79 条速度内环饱和，饱和比例约 **68.1%**；
- 速度范围约 `-6.806 ... +6.597 rad/s`，约等于 `-65 ... +63 rpm`；
- 电流频繁在 `-200 mA` 与 `+200 mA` 间切换；
- feedback age 为 `0...2 ms`，明显小于 20 ms 超时阈值。

典型片段：

```text
vel=+6.178 rad/s current=-200 mA sat=1
vel=-4.921 rad/s current=+200 mA sat=1
vel=+6.073 rad/s current=-200 mA sat=1
vel=-5.654 rad/s current=+200 mA sat=1
```

电流符号是正确的制动方向，但速度每次都穿过 0 并反向，随后电流也跟着反向。这是
“制动增益/控制权过大 + 离散采样和执行延迟”的振荡，不是方向反了。

### 3.3 为什么 5 ms 的斜率限制没有阻止抖动

当前 `control_slew_rate_step()` 只限制**目标速度**变化，并不限制 PID 最终电流。
手拨后，测量速度可以在相邻控制拍中改变符号，最终电流仍能直接从 `-0.2 A` 跳到
`+0.2 A`，即一拍变化 `400 mA`。

### 3.4 目前可以暂时排除什么

- 不是明显的 CAN 反馈过期：日志 age 始终很小，也没有 state/fresh 错误。
- 没有发现编码器整圈跳变：驱动按 8192 tick 连续解圈，日志也没有 `2π rad` 跳变。
- 不是位置积分 wind-up：位置环虽然写了 `.ki = 0.1f`，但积分上下限都是 0，积分
  实际始终被钳为 0。
- 不应先加 D：原始速度只有整数 rpm 分辨率，未经滤波的 D 会进一步放大跳变。

### 3.5 为什么“速度控制正常”与位置控制抖动并不矛盾

两个样例并没有在相同工作点使用相同参数：

| 项目 | 速度样例 | 位置样例 |
| --- | ---: | ---: |
| 速度反馈 Kp | `0.04` | `0.10` |
| 请求速度量级 | 正弦峰值 `20 rad/s` | 初始仅 `0.015 rad/s` |
| 是否长期停留在零速附近 | 否，只是穿过 | 是，位置保持必然回到零速 |
| 软件电流限幅 | `3.0 A` | `0.2 A` |

速度样例正常证明电机和速度环在明显运动区能够工作；位置保持要求它长期工作在
“零速量化 + 静摩擦 + 换向”区，这是速度轨迹短暂穿零没有充分覆盖的工况。位置样例还
把已验证的速度 Kp 从 `0.04` 提高到了 `0.10`，因此不能把速度样例的稳定性直接外推。

## 4. 当前控制链路

```text
位置误差
  │
  ▼
位置 P ──> 目标速度 ──> 目标速度斜率限制 ──> 速度误差
                                                   │
                                                   ▼
                                              速度 P / FF
                                                   │
                                  只有 ±200 mA 幅值限制
                                  没有最终电流变化率限制
                                                   │
                                                   ▼
                                           GM6020 电流环
                                                   │
                                                   ▼
                                   整数 rpm 速度 + 编码器位置
```

真正需要拆开解决的是：

- 先复用已经实测正常的速度环 Kp，再用较低电流上限验证零速阻尼；
- 提高位置环相对于速度阻尼的作用，使目标速度进入可分辨范围；
- 用静摩擦前馈单独解决“起步推不动”；
- 不要用一个很大的速度 P 同时承担这两件互相冲突的工作。

## 5. 推荐的手动修改与验证顺序

### 第一步：保留现有极性，不做反转

文件：`samples/motor/dji_position_control/src/main.cpp`

最终电流仍保持：

```cpp
next_output.current_command_a = clampFloat(
    next_output.velocity.output,
    -kSoftwareCurrentAbsMaxA,
    kSoftwareCurrentAbsMaxA);
```

无需增加 `kCurrentCommandPolarity = -1`。如果为了表达安装方向而保留方向常量，本台架
应是 `+1.0f`。

自检：正速度时，在目标速度接近 0 的情况下，命令电流必须为负；负速度时命令电流
必须为正。当前日志已经符合这一点。

### 第二步：关闭位置外环，用已验证速度 Kp 检查零速阻尼

第一次验证暂时将位置 Kp 设为 0，让速度目标始终是 0：

```cpp
constexpr float kPositionKp = 0.0f;          // 仅内环验证阶段
constexpr float kSoftwareCurrentAbsMaxA = 0.03f;
constexpr float kInnerKp = 0.04f;             // 先复用速度样例的实测值
constexpr float kInnerKi = 0.0f;
```

这里保留速度样例已经实测正常的 `0.04 A/(rad/s)`，只把扰动时的最大控制权暂时限制为
`30 mA`。运行后轻拨输出轴：

- 正常：速度逐渐衰减，最多轻微回弹，不持续来回抖；
- 异常：速度反复跨零并保持振荡，说明控制权仍太大或存在额外延迟；
- 松手后停在新位置是正常的，因为位置环被故意关闭。

若稳定但阻尼太弱，先把上限从 `30 mA` 增到 `40 mA`，不要同时修改 Kp。若振荡仍在，
则保持 `30 mA` 上限，把 `kInnerKp` 依次降到 `0.03`、`0.02`、`0.01`，从而区分究竟
是饱和电流过大，还是零速附近反馈增益过大。每轮只改一项。

若在 `20 mA` 上限、`Kp=0.01` 时仍持续抖动，才检查：

- 控制周期实际 dt 是否稳定在约 5 ms；
- 当前反馈 timestamp 是否在一个控制拍内重复使用；
- 命令电流、反馈电流、速度是否在同一时刻采样；
- GM6020 是否还有上层速度/位置模式，而不是预期的电流模式。

### 第三步：再恢复小目标的位置外环

零速内环稳定后，先用 `50...100 mrad` 目标，而不是直接用 `250 mrad`。建议首轮起点：

```cpp
constexpr float kTargetOffsetRad = 0.05f;
constexpr float kPositionKp = 1.0f;
constexpr float kVelocityAbsMaxRadS = 0.30f;
constexpr float kSoftwareCurrentAbsMaxA = 0.05f;
constexpr float kInnerKp = 0.04f;              // 或第二步得到的稳定值
constexpr float kVelocityRampRateRadS2 = 0.5f;
```

位置 Kp 从 `0.06` 提到 `1.0` 的目的，是修正“恢复力/阻尼”比例：让 `50 mrad` 误差
产生 `0.05 rad/s` 请求，并让 `250 mrad` 误差产生约 `0.25 rad/s`（约 2.4 rpm）的
可分辨请求。速度上限仍保持较低，避免大误差时过快运动。

注意：即便这样，纯 P 仍可能因静摩擦停在目标之前。不要再靠增大内环 Kp 解决，进入
下一步单独补偿静摩擦。

### 第四步：用静摩擦前馈解决“推不动”

当前 `feedforward` 全部为 0。内环稳定后，可以从很小的静摩擦前馈开始：

```cpp
controller.velocity_config.feedforward = {
    .k_bias = 0.0f,
    .k_static = 0.01f,          // 先从 10 mA 开始
    .k_velocity = 0.0f,
    .k_acceleration = 0.0f,
    .k_gravity = 0.0f,
    .velocity_epsilon = 0.01f,
    .acceleration_epsilon = 0.1f,
    .gravity_model = CONTROL_GRAVITY_NONE,
};
```

正速度请求会得到正静摩擦前馈，负速度请求会得到负前馈，符合本台架已确认的方向。
每次只把 `k_static` 增加 `5 mA`，刚好能在正负两个方向可靠起转就停止。仓库记录的
`50 mA` 只能作为数量级参考，实物测试才是准值。

始终保证：

```text
|静摩擦前馈| + |正常速度反馈电流| < 软件电流上限
```

例如软件上限 `50 mA` 时，不应直接配置 `50 mA` 静摩擦前馈，因为这会完全吃掉反馈
控制余量。若实测确实需要约 `50 mA` 起转，可在内环稳定的前提下把总上限逐步增加到
`60...80 mA`，不要直接回到 `200 mA`。

### 第五步：若仍有电流硬翻转，再考虑最终电流变化率限制

这不是第一修复项。先降低 Kp 和电流上限；只有它们稳定后仍存在明显扭矩尖峰，才在
`PositionController` 中增加第二个 slew-rate limiter，用于最终电流命令。

需要新增的状态：

```cpp
control_slew_rate_config current_command_config{};
control_slew_rate_state current_command_state{};
```

配置可从 `0.5...1.0 A/s` 低能量试起；5 ms 一拍相当于每拍最多变化
`2.5...5 mA`。在 `resetController()` 中把它 reset 为 `0.0f`，在 PID 输出后调用：

```cpp
float limited_current_a = 0.0f;
float current_rate_a_s = 0.0f;
ret = control_slew_rate_step(
    &next_current_command_state,
    &controller.current_command_config,
    next_output.velocity.output,
    dt_s,
    &limited_current_a,
    &current_rate_a_s);
```

然后把 `limited_current_a` 作为 `setCurrent()` 的命令，并与其他 controller state 一起
事务式提交。配置验证失败返回 `-EINVAL`，未 reset 返回 `-EACCES`，dt 非法返回
`-ERANGE`；任何错误仍应走现有 `stopAfterFailure()`。

风险：电流变化率过小会让制动力在速度过零后保持旧方向太久，反而增加相位滞后。因此
它只能用于削尖峰，不能替代正确的低增益与低电流上限。

## 6. 必须同步清理的误导项

- 源码是 `kVelocityAbsMaxRadS = 5.0f`，注释却按 `0.5 rad/s` 计算。
- `app.overlay` 实际是 `current-limit-ma = <1500>`，源码注释却写允许 `±3.0 A`。
- `requestedPositionRad()` 实际等待 `5000 ms`，注释却写 `0.5 s`。
- `.ki = 0.1f` 但积分上下限都是 0，实际上没有位置积分；建议明确写为 0。
- `kPositionDeadbandRad = 0.002` 在 1:1 下约为 2.6 encoder tick，不是约 1 tick。
- overlay 中的 “M3508 gearbox ratio” 注释与当前 GM6020 直驱实例不符。

这些不一致不是抖动的直接根因，却会让限幅计算和调参判断持续出错。

## 7. 遥测应如何补充

当前每 40 个控制拍才打印一次，即约 200 ms；高频振动会被严重混叠。不要在 5 ms
控制线程逐拍 `printk()`，应写入固定长度环形缓冲区，停机后打印，或让低优先级线程
读取快照。

至少记录：

```text
cycle_index, dt_ms, feedback_timestamp, target_pos, measured_pos,
position_error, velocity_ref, measured_velocity, feedback_rpm,
pid_p_current, feedforward_current, unsaturated_current,
command_current, feedback_current, saturated
```

重点检查两件事：

1. 速度第一次穿过 0 时，上一拍的制动电流是否仍在继续作用；
2. 从负饱和到正饱和究竟用了几拍，反馈电流是否跟随命令电流。

## 8. 构建与硬件验证

用户手动修改后构建：

```bash
west build -b rm_typec/stm32f407xx \
  samples/motor/dji_position_control -p \
  --build-dir samples/motor/dji_position_control/build/rm_typec/stm32f407xx \
  -- -DBOARD_ROOT=$PWD
```

然后手动烧录：

```bash
west flash -d samples/motor/dji_position_control/build/rm_typec/stm32f407xx
```

建议把测试恢复成有限运行时间，并在正常结束时执行 `0 A -> flush -> Bus::stop()`，不要
继续用“自动 arm + 无限循环 + 只能断电退出”调闭环。

安全顺序：

1. 输出轴悬空并远离机械限位，手边准备物理断电开关。
2. 先以位置环关闭、`20...30 mA` 上限验证零速阻尼。
3. 再以 `50 mrad` 目标测试位置环。
4. 再逐步增加静摩擦前馈，每次只增加 `5 mA`。
5. 稳定后才扩大到 `250 mrad`；每次只改一个参数。
6. 一旦振幅增长、连续饱和或速度超过安全阈值，立即物理断电。

## 9. 最终验收清单

- [ ] 没有加入负极性映射；正电流仍对应正速度。
- [ ] 位置环关闭时，手拨后的速度能衰减且不持续跨零振荡。
- [ ] 已先复用速度样例实测正常的 `Kp=0.04`，并在零速扰动工况重新验证。
- [ ] 已从 `20...30 mA` 上限检查内环，而不是从 `200 mA` 开始。
- [ ] 静摩擦前馈从 10 mA 起逐级增加，正负两个方向分别验证。
- [ ] 正常运动时有电流余量，未长期饱和。
- [ ] 先通过 `50...100 mrad` 小目标，再测试 `250 mrad`。
- [ ] feedback age 小于 20 ms，dt 稳定在允许范围内。
- [ ] 正常结束、超时和故障路径都发送 0 A 并执行 stop。

## 10. 当前最值得先试的一组改动

只做诊断性首轮时：保持电流/速度同号定义不变，把位置环临时关掉，将软件电流限幅从
`200 mA` 降到 `30 mA`，速度 Kp 从位置样例的 `0.10` 改回速度样例已实测正常的
`0.04`，再轻拨观察速度是否衰减。若不再抖，说明问题来自位置样例的零速参数组合，
而不是速度控制实现。之后将位置 Kp 从 `0.06` 提到约 `1.0`，先试 `50 mrad` 小目标，
再逐步加入静摩擦前馈，分别解决“到位恢复力”和“起转电流”问题。
