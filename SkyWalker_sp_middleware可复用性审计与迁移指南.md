# SkyWalker 对 `sp_middleware` 的可复用性审计与迁移指南

> 目标：判断 `TongjiSuperPower/sp_middleware` 里哪些设计值得在 SkyWalker 中独立实现，哪些只能用于协议交叉核对，哪些不应引入。
>
> 本文遵守古法编程模式：只给结论、接口草案、伪代码、测试方法和实施顺序，没有修改任何业务源码。

## 0. 审计快照

- 参考仓库：`https://github.com/TongjiSuperPower/sp_middleware`
- 本次核查分支：远端默认分支 `main`
- 本次核查提交：`608b09e712fb53dd2cf86d5c8e622fc7e1006bb3`
- 提交时间：`2026-08-14T17:57:46+08:00`
- 仓库自述：同济大学 SuperPower 战队 25 赛季电控中间件
- 核查规模：约 119 个文件，C++/Markdown 约 10701 行
- 主要目录：`io/`、`motor/`、`referee/`、`tools/`

本次核查没有在参考仓库中找到：

- `LICENSE`、`LICENSE.*` 或 `COPYING` 文件。
- 自动化单元测试目录或测试文件。
- 能让本项目直接作为 Zephyr 模块使用的根构建配置。

这三个缺口会直接影响复用方式，不能把它当作“下载后编译一下就能用”的成熟依赖。

## 1. 先说最终结论

这个仓库有不少好思路，但对当前 SkyWalker 最有价值的不是 HAL 驱动，而是下面五类“控制行为”：

1. 舵轮零速保持、最短转向与反向驱动的处理方式。
2. 关节控制器的 `位置 → 速度 → 输出` 串级结构和控制模式状态机。
3. PID 的测量微分、角度误差、梯形积分、显式 reset 等设计点。
4. yaw 的惯量/阻尼前馈，以及上层目标携带速度、加速度的接口思路。
5. 串口流中处理拆包、粘包、CRC 失败后逐字节重新同步的解析思路。

建议按下面四档处理：

| 等级 | 模块 | 在 SkyWalker 中怎么用 |
|---|---|---|
| S：近期独立实现 | `tools/swerve`、`tools/joint`、`tools/math_tools` 中的角度展开、`io/vision` 的流式解析行为 | 吸收需求和测试用例，按本项目接口重写 |
| A：基础闭环稳定后实现 | `tools/yaw_feedward`、`tools/gimbal` 的坐标系分层、`tools/low_pass_filter`、`tools/linear_differentiator` | 只提取小而明确的算法，不搬整个类 |
| B：以后按需求核对 | `motor/rm_motor`、`motor/dm_motor`、`tools/crc`、`tools/slip_detect`、`io/dbus`、`referee` | 以官方手册为准，用参考仓库做第二来源交叉检查 |
| X：不要迁入 | `io/can`、`io/fdcan`、STM32 HAL 外设驱动、整个 `tools/gimbal` 大类、当前 `tools/fuzzy_pid` | 平台耦合、接口风险或实现质量不适合当前项目 |

最重要的一句话：

> 不建议把 `sp_middleware` 作为 git submodule 接入当前项目；建议记录审计提交，然后依据本文列出的行为，在 SkyWalker 中独立实现并自己补齐测试。

## 2. 许可证边界：目前不能直接复制代码

参考仓库的 `readme.md` 建议使用 git submodule，但本次审计提交没有许可证文件。README 中的“建议作为子模块使用”不等价于明确授予复制、修改、分发源码的许可。

GitHub 官方文档说明：仓库没有许可证时，默认版权规则仍然适用，其他人通常没有复制、分发或创作衍生作品的许可。这里不是法律意见，但工程上应采用保守做法。

在维护者增加许可证或给出明确书面授权之前，你应该：

- 可以阅读代码、理解数学关系、发现接口需求和风险。
- 可以依据电机官方协议、运动学公式和你自己的接口独立实现。
- 可以把“零速保持”“最短转向”“CRC 失败重同步”等行为写成自己的测试需求。
- 不要复制它的源文件、函数体、查表数据、注释或大段结构定义。
- 不要把该仓库作为子模块参与固件构建。
- 不要声称本项目的代码来自 MIT、Apache、GPL 等许可证，因为参考仓库目前没有声明这些许可证。

如果以后得到许可，再做下面五件事：

1. 保存许可证文本和授权记录。
2. 固定到明确 commit，不跟随浮动 `main`。
3. 在第三方清单中记录仓库、commit、许可证和本地修改。
4. 先在隔离分支验证 C++ 标准、平台依赖和测试。
5. 再判断是保留独立实现，还是引入经过裁剪的第三方模块。

参考：

- GitHub 仓库：`https://github.com/TongjiSuperPower/sp_middleware`
- GitHub 关于仓库许可证的说明：`https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/licensing-a-repository`

## 3. 它和当前项目的根本差异

不能直接搬代码，不只是许可证问题，还有技术栈差异。

| 项目 | `sp_middleware` | 当前 SkyWalker |
|---|---|---|
| 平台 | STM32Cube/HAL、CMSIS-RTOS、部分 USB CDC | Zephyr device model、Zephyr CAN/UART |
| 驱动绑定 | 构造函数直接拿 HAL handle | devicetree + `struct device` |
| 错误处理 | 多处忽略 HAL 返回值 | 应返回 `-EINVAL/-EIO/-ETIMEDOUT` 等明确错误 |
| 时间 | 固定 `dt`、DWT、`osKernelSysTick()` | 实际 `dt`、`k_uptime_get()`、Zephyr cycle API |
| 数据风格 | 很多 public mutable field | 建议 Config/State/Input/Output 分离 |
| C++ 标准 | 出现 C++17 写法 | 当前目标是 C++14 |
| 测试 | 本次未发现自动化测试 | 纯数学、协议编解码应当先写 host/ztest |
| 电机输出 | 类内直接发命令 | 控制计算和设备发送应分层 |

参考仓库中存在 `namespace sp::referee`、命名空间作用域 `inline static constexpr` 等 C++17 语法。即使暂时忽略平台依赖，也不能保证它按当前 C++14 设置直接编译。

## 4. 第一优先级：吸收舵轮行为，不移植 `Swerve` 类

### 4.1 值得借鉴的四个行为

`tools/swerve` 最值得留下的是：

- 逆运动学使用每个模块位置计算轮端速度向量。
- 轮端速度为零时，保持当前舵角，不调用无意义的 `atan2(0, 0)`。
- 在目标方向和目标方向加 180°之间，选择离当前舵角更近的一支。
- 舵轮尚未对准目标时，用夹角余弦降低驱动轮输出。

它还同时提供正运动学，这提醒我们正解和逆解应该共用同一套坐标约定、轮序和几何参数。

### 4.2 不要照搬的细节

参考实现有这些限制：

- 写死 LF/LR/RF/RR 四个字段，无法自然扩展到配置数组。
- 只支持矩形四舵轮，几何和轮序耦合在公式中。
- 只提供一个全局 `invert_pivot`，而实车需要每个模块独立的舵向符号、驱动符号和零点。
- 精确比较速度向量是否等于零，实际控制要用阈值。
- 没有检查轮径为零、非法几何、NaN/Inf。
- 没有四轮共同的轮速反饱和缩放。
- 输出舵角被归一化成单圈角，不足以直接驱动连续角位置环。
- “反转驱动”和“对齐余弦缩放”合并在一个 `cos()` 表达式中，调试时很难分别观察。
- 正运动学是写死的手推公式，符号约定和逆解不容易独立验证。

### 4.3 你应该实现的接口

继续沿用《SkyWalker 舵轮解算与大小 Yaw 双板协同古法实施框架》中的 `ModuleGeometry[4]`，再给优化结果补充可观察字段：

```cpp
struct ModuleGeometry {
    float x_m;
    float y_m;
    float wheel_radius_m;
    float steer_zero_rad;
    int8_t steer_sign;
    int8_t drive_sign;
};

struct ModuleTarget {
    float steer_continuous_rad;
    float wheel_rad_s;
    float alignment_scale;
    bool drive_reversed;
    bool angle_held;
};

struct SwerveConfig {
    ModuleGeometry modules[4];
    float hold_speed_m_s;
    float max_wheel_rad_s;
    bool enable_alignment_scale;
};

int swerve_inverse(
    const SwerveConfig &cfg,
    float vx_m_s,
    float vy_m_s,
    float wz_rad_s,
    const float steer_continuous_rad[4],
    const float last_target_rad[4],
    ModuleTarget out[4]);
```

返回 `int` 是为了让非法轮径、无效符号、NaN 输入和空指针有明确结果，而不是产生一帧危险目标。

### 4.4 最短转向要按这个顺序写

对每个模块：

1. 根据模块位置得到轮端线速度：

```text
v_ix = vx - wz * y_i
v_iy = vy + wz * x_i
speed_m_s = hypot(v_ix, v_iy)
```

2. 如果 `speed_m_s < hold_speed_m_s`：

```text
steer_target = last_target
wheel_speed = 0
angle_held = true
```

3. 否则计算基础方向 `theta = atan2(v_iy, v_ix)`。
4. 构造两个机械等价候选：

```text
候选 A：theta，驱动方向 +1
候选 B：theta + pi，驱动方向 -1
```

5. 分别把 A、B 展开到最接近当前连续舵角的圈数，选择绝对转角误差更小者。
6. 先得到明确的 `drive_reversed` 和带符号的轮速。
7. 再单独计算：

```text
alignment_scale = clamp(cos(chosen_angle_error), 0, 1)
```

8. 如果开启对齐缩放，再把轮速乘 `alignment_scale`。
9. 四个模块全部完成后，若任一轮超过最大轮速，用一个共同系数缩放所有轮速。

这里故意把反转和余弦缩放分开。调试时你能清楚看到：到底是选择了 180°等价解，还是因为舵角没对准而减小了驱动输出。

### 4.5 必须先写的舵轮测试

| 测试 | 预期 |
|---|---|
| `vx=1, vy=0, wz=0` | 四轮平行，线速度同号 |
| `vx=0, vy=1, wz=0` | 四轮指向横移方向 |
| `vx=0, vy=0, wz=1` | 四轮切向布置，轮速符合几何半径 |
| 三个命令全为零 | 保持各自上一拍舵角，轮速为零 |
| 当前角与基础目标相差 100° | 选择反向驱动和较短的 80°转向 |
| 当前连续角为 `4*pi+0.1` | 输出仍在相邻圈，不跳回单圈 `0.1` |
| 一个轮目标超上限 | 四轮按同一比例缩小 |
| 轮径为零、输入 NaN | 返回错误，不更新有效输出 |
| 逆解结果再送正解 | 在数值容差内还原 `vx/vy/wz` |

正运动学第一版建议使用由模块位置组成的最小二乘解，不复制参考仓库的四轮手写式。

## 5. 第二优先级：借用“关节控制器分层”，不要绑定具体电机类

### 5.1 为什么它适合你的大小 yaw 和 GM6020 舵向

`tools/joint` 把关节分成四种模式：失能、位置、速度、力矩，并在位置模式中使用：

```text
位置目标 → 位置 PID → 速度目标 → 速度 PID → 电机输出
```

这个结构可以复用于：

- 四个 GM6020 舵向电机。
- 大 yaw 关节。
- 小 yaw 关节。
- 以后可能出现的 pitch 关节。

它还提醒我们给关节统一处理：零点、方向、软限位、最大速度、反馈滤波和前馈。

### 5.2 参考实现为什么不能直接拿来

- 模板直接依赖 `DM_Motor` 和 `RM_Motor`，控制算法和设备类粘在一起。
- 只显式实例化了两种电机类型，扩展性只是表面上的。
- 控制器内部直接调用 `motor_.cmd()`，安全层无法在发送前统一裁剪命令。
- 模式切换和失能时没有清理两个 PID 的历史状态。
- 没有反馈有效位、时间戳、年龄或错误返回。
- 堵转判断只看瞬时转矩/速度阈值，没有持续时间、回差和反馈新鲜度。
- 大量 public 可变状态让多线程所有权不清楚。

### 5.3 你应该实现成纯控制器

```cpp
enum class JointMode : uint8_t {
    Disabled,
    Current,
    Velocity,
    Position,
};

struct JointFeedback {
    float position_rad;
    float velocity_rad_s;
    float current_a;
    uint32_t stamp_ms;
    bool position_valid;
    bool velocity_valid;
    bool current_valid;
};

struct JointCommand {
    JointMode mode;
    float position_rad;
    float velocity_rad_s;
    float current_ff_a;
};

struct JointOutput {
    float current_a;
    float position_error_rad;
    float velocity_target_rad_s;
    float velocity_error_rad_s;
    bool saturated;
    bool limited;
};

struct JointController;

int joint_controller_step(
    JointController *controller,
    const JointCommand &command,
    const JointFeedback &feedback,
    float dt_s,
    JointOutput *out);

void joint_controller_reset(
    JointController *controller,
    const JointFeedback *current_feedback);
```

这个接口只计算，不发送 CAN。调用关系应是：

```text
电机驱动读取反馈
  → 转成 JointFeedback
  → joint_controller_step()
  → 安全层检查离线/超时/总电流
  → A 到 raw 的型号适配
  → DJI TxGroup 统一发送
```

### 5.4 GM6020 电流模式的单位不要混

本项目的 GM6020 默认是电流控制。第一版关节控制器输出建议明确叫 `current_a`，不要叫 `torque_nm`，除非你已经标定了可靠的力矩常数。

发送适配层负责：

```text
current_a
  → 按允许电流限幅
  → 乘电机安装方向
  → 换算到 GM6020 raw 电流
  → 放进 0x1FE 或 0x2FE 的对应槽位
```

理论上线性换算可写成：

```text
raw = round(current_a / 3.0 A * 16384)
```

但实际还要按手册、实车安全上限和供电能力限制，不能一上电就允许满量程。

### 5.5 模式切换必须有统一行为

你实现时逐条满足：

- 进入 `Disabled`：输出立即为零，两个 PID reset。
- `Disabled → Position`：位置目标默认捕获当前反馈，防止突然跳到旧目标。
- `Disabled → Velocity`：速度目标默认从零开始。
- 任一必需反馈失效或超时：进入失能，不维持最后输出。
- 位置模式：位置外环输出必须限为允许速度。
- 速度内环：最终输出按电流上限饱和，并做 anti-windup。
- 软限位外仍允许往安全方向回退，禁止继续撞向限位。
- 正常停机、通信失联、驱动离线都调用同一个 reset 路径。

## 6. PID：只吸收四个特性，不替换成参考实现

`tools/pid` 里值得借鉴的特性：

- 普通 PID 默认对反馈做微分，降低设定值跳变带来的 derivative kick。
- 角度环使用最短角误差。
- 积分使用梯形积分。
- 提供 `clear()` 清空输出、积分和微分历史。
- 另一个重载允许上层直接提供 `set_dot/fdb_dot`。

这些内容已经和《SkyWalker 多型号电机驱动古法实施手册》中建议的 PID 方向一致，可以拿来补充测试，但不要把参考 PID 整类搬进来。

参考 PID 仍有这些问题：

- `dt` 在构造时固定，实际任务抖动无法反映。
- 微分滤波用固定 alpha，滤波实际截止频率会随采样频率改变。
- 没有检查 `dt<=0`、NaN 或 Inf。
- 只有积分项限幅，没有基于最终执行器饱和的完整 anti-windup。
- “P 项超过阈值就暂停积分”无法覆盖前馈导致的总输出饱和。
- 多个 `calc()` 重载复制大量逻辑，后续很容易修一处漏一处。

你自己的 PID 输入最好最终明确到：

```cpp
struct PidInput {
    float setpoint;
    float measurement;
    float dt_s;
    float setpoint_rate;
    float measurement_rate;
    float feedforward;
    bool rate_valid;
};
```

如果 `rate_valid=true`，D 项直接用测得/规划的速度；否则才对测量做带时间常数的微分滤波。

## 7. yaw 前馈：能借公式，不能把它误认为大小 yaw 协调器

### 7.1 参考实现实际做了什么

`tools/yaw_feedward` 的核心是：

```text
目标角加速度 ≈ 目标角速度的一阶差分
惯性补偿 = kff * 转动惯量 * 目标角加速度
阻尼补偿 = clamp(阻尼系数 * 目标角速度)
总前馈 = 惯性补偿 + 阻尼补偿
```

这是单轴执行器的前馈，不负责：

- 大 yaw 和小 yaw 怎么分配角度。
- 哪块板拥有世界目标。
- 两个 yaw 如何做软限位卸载。
- 两块板的时间戳和失联处理。

所以它应该放进单个 `JointController` 的输出合成，而不是替代 `YawCoordinator`。

### 7.2 第一版建议直接使用“电流单位前馈”

GM6020 使用电流模式，未完成力矩常数标定前，可以先做经验模型：

```cpp
struct YawCurrentFeedforwardConfig {
    float ka_a_per_rad_s2;
    float kv_a_per_rad_s;
    float ks_a;
    float max_current_a;
};

struct YawCurrentFeedforwardInput {
    float velocity_ref_rad_s;
    float acceleration_ref_rad_s2;
    bool acceleration_valid;
};

float yaw_current_feedforward(
    const YawCurrentFeedforwardConfig &cfg,
    const YawCurrentFeedforwardInput &input);
```

计算顺序：

```text
ff = ka * acceleration_ref
   + kv * velocity_ref
   + ks * sign_with_deadband(velocity_ref)

ff = clamp(ff, -max_current_a, +max_current_a)
```

其中：

- `ka` 补偿转动惯量。
- `kv` 补偿近似粘性阻尼。
- `ks` 补偿静摩擦，但速度在死区内时必须为零，避免静止抖动。
- 输出单位始终是 A，之后和速度 PID 的 A 输出相加。

如果以后标定出 `Kt`，可以改为先算 N·m，再通过 `current = torque/Kt` 转成 A，但接口和标定记录必须明确单位。

### 7.3 加速度目标从哪里来

优先级从高到低：

1. 轨迹规划器直接给出位置、速度、加速度目标。
2. 对经过斜坡/轨迹平滑后的速度目标求导。
3. 最后才对原始速度指令做差分。

不要直接对遥控器阶跃命令求导。阶跃经过 `(v[k]-v[k-1])/dt` 会产生很大的瞬时前馈。

参考仓库的实现使用固定 `dt` 和固定 alpha 低通，没有 reset、有限值检查、总输出限幅和斜率限制。你实现时必须补齐：

- 使用本拍实际 `dt_s`，并验证合理范围。
- 模式切换、失联和重新使能时 reset 导数/滤波历史。
- 对前馈单独限幅，对 `PID + FF` 的总输出再次限幅。
- 总输出饱和信息反馈给 PID anti-windup。
- 记录 `pid_current_a`、`ff_current_a`、`total_current_a` 三个遥测量。

### 7.4 大小 yaw 分别怎么用前馈

- 小 yaw：使用目标轨迹的速度/加速度前馈，负责快速稳像和高频目标变化。
- 大 yaw：使用经过低通或限速后的卸载目标，前馈更保守，负责慢速把小 yaw 拉回舒适区。
- 不要把同一份完整前馈同时加给两个轴，否则两轴会一起抢动作。
- 协调器输出的是两套关节目标；每个轴自己的关节控制器再计算各自 PID 和前馈。

## 8. `tools/gimbal`：只借坐标系观念，不搬 753 行大类

### 8.1 对大小 yaw 真正有用的内容

该模块包含单 IMU/双 IMU 两种姿态关系，值得借鉴的观念是：

- 世界姿态目标和电机关节目标是两种不同数据。
- 双 IMU 模式先求底层到世界、上层到世界，再求上下层相对姿态。
- 四元数适合组合多个非共轴三维旋转。
- 角度、角速度、角加速度应该作为一组目标传递。
- 角度跨越 `+-pi` 时需要独立的连续角展开状态。

如果以后加入 pitch/roll 或两个 yaw 并非完全同轴，可以使用概念关系：

```text
q_base_to_upper = conjugate(q_base_to_world) ⊗ q_upper_to_world
```

但你必须自己确定四元数是主动旋转还是被动换系、乘法顺序和轴定义，再用已知 90°旋转测试验证。

### 8.2 第一版仍然用标量 yaw 关系

在“大 yaw、小 yaw 近似同轴”的当前假设下，先使用：

```text
yaw_world ≈ yaw_chassis_world
          + yaw_big_relative
          + yaw_small_relative
          + calibrated_offset
```

对应分工继续沿用已有框架：

- 云台板持有最终世界 yaw 目标和小 yaw IMU。
- 云台板上的唯一协调器分配大、小 yaw 目标。
- 底盘板只闭环控制大 yaw，并把测量与状态回传。
- 小 yaw 快速跟踪世界目标。
- 大 yaw 低频跟随，将小 yaw 从软限位附近卸载回中间区。

### 8.3 参考大类的风险

- 单个类拥有大量 public 状态，初始化和更新顺序难以审计。
- 固定 `dt`、固定 alpha，没有时间戳和数据新鲜度。
- 多处欧拉角速度换算除以 `cos(pitch)`，接近万向锁时会放大或发散。
- 部分历史四元数和派生字段的初始状态不清楚。
- 没有跨板采样时间对齐。
- 大量功能一次性耦合，第一版很难定位坐标符号错误。

因此以后只新增小型 `frame_math` 函数，不新增一个对应的“大一统 Gimbal 类”。

## 9. 电机模块：只做协议交叉核对，官方文档仍是唯一主依据

### 9.1 `motor/rm_motor` 能帮你核对什么

参考实现覆盖 M2006、M3508、GM6020 电流和 GM6020 电压模式，可以用于核对：

- M2006/M3508 反馈 ID 是 `0x200 + motor_id`。
- GM6020 反馈 ID 是 `0x204 + motor_id`。
- GM6020 电流命令组使用 `0x1FE/0x2FE`。
- 四槽命令位置使用 `(motor_id - 1) % 4`。
- M2006、M3508、GM6020 电流命令 raw 上限分别按 10000、16384、16384 处理。

这些结论与当前《SkyWalker 多型号电机驱动古法实施手册》的方向一致。

### 9.2 `rm_motor` 里不要照搬的地方

- M2006 仍把反馈第 6 字节当温度；C610 官方反馈该位置是空字段。
- GM6020 电压 raw 上限写为 25000，而官方协议量程需要重新按手册核对；不能把未解释的安全裁剪当协议常量。
- 用近似力矩常数直接生成通用 `torque`，容易掩盖型号、温度和电调估算误差。
- 编码器减去固定 4095 作为中心，不适合成为公共协议层规则。
- 缺少 ID、DLC、指针、有限值和传动比验证。
- 命令分组所有权没有显式保护，多任务写同一组帧会有竞争风险。

### 9.3 `motor/dm_motor` 能帮你核对什么

- DM MIT 帧中 P/V/KP/KD/T 的位打包顺序。
- 使能、失能、清错等特殊帧的形状。
- 反馈中的状态/错误 nibble、位置、速度、力矩和温度字段。
- MIT 浮点与无符号整数之间的线性映射测试向量。

### 9.4 `dm_motor` 中一个尤其危险的接口

参考实现的一个“只写转矩”路径只改了帧的后半部分，没有完整初始化前面的 P/V 字段。MIT 协议中零位置、零速度通常映射到量程中点，并不等价于把字节清零；复用残留 buffer 还会带入上一帧内容。

你的 DM-J4310 实现必须：

- 每次从一个全新、完整定义的 8 字节帧开始打包。
- 对 P/V/KP/KD/T 每个输入先检查有限值并按型号 profile 限幅。
- 不提供会留下未初始化字节的“局部写帧”接口。
- 不依赖 `memcpy(float)` 传速度，除非协议明确规定 IEEE-754 和字节序。
- 继续以仓库 `motor_docs/DM-J4310-2EC_V1.1_User_Manual.pdf` 为主依据。

## 10. 双板/UART 通信：借解析状态机，不借 packed float 帧

### 10.1 `io/vision` 值得借的解析行为

它能接收任意长度的数据块并：

1. 追加到接收缓存。
2. 搜索两字节帧头。
3. 数据不足一整帧时保留并等待下一块。
4. 一帧有效后取出，并继续解析后面的粘包。
5. CRC 失败时只丢一个字节，重新搜索帧头。

这个行为很适合作为你双板 UART 协议解析器的测试需求。

### 10.2 不能复用它的帧布局

- 使用 packed struct 直接映射 float，依赖编译器 ABI、大小端和 float 格式。
- 帧没有通用的版本、payload 长度、消息类型和源时间。
- 对不同消息写死不同 struct 大小。
- 线性 buffer 频繁 `memmove`，数据量大时开销不稳定。
- “alive” 状态只在成功收帧时顺手计算，沉默期间不会自己变 false。
- 接收结构中的 float 没有 `isfinite` 检查。

已有《SkyWalker 舵轮解算与大小 Yaw 双板协同古法实施框架》中的逐字段编码方案仍然优先。

### 10.3 建议解析器接口

```cpp
struct BoardLinkParserStats {
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t version_errors;
    uint32_t dropped_bytes;
    uint32_t overflow_resets;
};

struct BoardLinkParser;

using FrameCallback = void (*)(
    uint8_t message_type,
    const uint8_t *payload,
    uint16_t payload_len,
    void *user);

int board_link_parser_push(
    BoardLinkParser *parser,
    const uint8_t *bytes,
    size_t len,
    FrameCallback callback,
    void *user);
```

通用帧头至少包含：

```text
magic[2]
protocol_version
message_type
payload_length_le
sequence_le
source_timestamp_ms_le
flags_le
payload...
crc16_le
```

所有多字节字段用 `read_le16/write_le16/read_le32/write_le32` 逐字段处理，不对接收字节做 struct 强转。

### 10.4 CRC 不能只说“CRC16”

`tools/crc` 的表可用于理解裁判系统 CRC，但在没有许可证时不复制表；更重要的是，不同 CRC16 可能有不同：

- polynomial
- init
- refin/refout
- xorout
- 结果字节序

你要先为板间协议固定这些参数，并写入协议文档，再实现和测试。CRC API 还要拒绝空指针和过短长度，避免 `len-1`、`len-2` 的无符号下溢。

### 10.5 必须覆盖的流式解析测试

- 一个完整帧一次送入。
- 每次只送一个字节。
- 帧头刚好跨两次输入。
- 两个完整帧粘在一起。
- 完整帧 + 半个下一帧。
- 帧前面有随机噪声。
- payload 中出现和帧头相同的字节。
- CRC 错误帧后立即跟一个正确帧。
- 声明长度超过最大 payload。
- 未支持的协议版本。
- sequence 重复、倒退和跳号。
- 超过接收缓存后的恢复。
- 收到 NaN/Inf 浮点 payload 后拒绝更新控制目标。
- 超时后，即使没有新字节进入，`is_alive(now_ms)` 仍能变成 false。

## 11. 数学小工具：值得自己写成可靠的小模块

### 11.1 `AngleUnwrapper`

参考仓库提醒了连续角的重要性，但它的 `limit_angle` 使用 while 循环；输入是 Inf 时可能永远循环，极大值也可能耗时异常。

建议接口：

```cpp
struct AngleUnwrapper {
    bool initialized;
    float last_wrapped_rad;
    float continuous_rad;
};

int angle_unwrap_reset(AngleUnwrapper *state, float wrapped_rad);
int angle_unwrap_update(AngleUnwrapper *state, float wrapped_rad, float *continuous_rad);
float angle_error_shortest(float target_rad, float feedback_rad);
```

实现要求：

- 先 `isfinite()`。
- 使用 `remainder`/`fmod` 的常数时间归一化，不使用无界 while。
- 每个传感器/关节持有自己的状态，不能共享静态变量。
- 第一次样本只初始化，不制造一圈跳变。
- 明确边界是 `[-pi, pi)` 还是 `(-pi, pi]`，全项目保持一致。

### 11.2 一阶低通

参考 `LowPassFilter` 是固定 alpha：

```text
y = alpha*x + (1-alpha)*y_last
```

你应该配置时间常数而不是裸 alpha：

```text
alpha = dt / (tau + dt)
```

这样控制周期改变时，滤波物理意义不变。接口要有 reset、有限值和 `dt` 合法性检查。

### 11.3 跟踪微分器

`tools/linear_differentiator` 可以作为“平滑目标并估计速度/加速度”的思路，但其显式 Euler 离散在 `r*dt` 太大时可能不稳定。

第一版先用明确限速、限加速度的轨迹生成器。只有当你能推导离散稳定范围并写完阶跃、斜坡、抖动 `dt` 测试后，再考虑二阶跟踪微分器。

### 11.4 浮点/整数协议映射

可以保留 `float_to_uint/uint_to_float` 这一类接口概念，但实现必须：

- 检查 `min < max`。
- 明确先 clamp 还是报错。
- 限制 bits 的合法范围。
- 使用无符号且位宽足够的移位常量。
- 用已知最小值、中点、最大值写黄金向量测试。

参考实现没有完整保护这些边界，不应直接使用。

## 12. 暂缓或放弃的模块

### 12.1 `tools/slip_detect`：以后先做遥测，不立即参与控制

它把轮端切向速度和底盘预测速度做比较，再降低疑似打滑轮的分配权重。思路可用于诊断，但第一版存在：

- 底盘速度若本身来自轮速里程计，比较并不完全独立。
- 声明的部分滑移变量没有完整更新。
- 重新归一化权重可能在限制一个轮后抬高其他轮功率。
- 几何、轮序和阈值写死。

正确顺序：先完成舵轮正解、IMU yaw rate 和时间同步；再把 slip ratio 只发到 VOFA 观察；采集实车数据后才允许它影响输出。

### 12.2 `tools/mahony`：当前项目已有 IMU/滤波层，不应替换

这个实现能作为 Mahony 算法阅读材料，但有这些风险：

- 没有处理加速度模长为零或异常值。
- `asin` 输入没有显式 clamp。
- 固定 `dt`。
- 没有磁力计时，绝对 yaw 本身不可观，仍会随陀螺偏置漂移。
- 部分声明输出没有在已核查更新路径中完整赋值。
- 几何 pitch 的首拍历史状态需要更严格初始化。
- 欧拉角速度在 `cos(pitch)` 接近零时奇异。

当前项目继续使用自己的 IMU 抽象和滤波实现；如果以后比较 Mahony，只做离线数据回放，不把它直接换进控制链。

### 12.3 `io/can`、`io/fdcan` 和 `tools/timer`：不要迁移

- 直接依赖 STM32 HAL handle、全局 `hcan1` 和 HAL 回调。
- 多处明确忽略 HAL 返回值。
- 发送接口共享 public 8 字节 buffer，不利于并发和错误隔离。
- DWT Timer 直接访问固定地址寄存器，并依赖 `SystemCoreClock`。
- `now_us()` 用 float 表示不断增长的绝对微秒，运行一段时间后会明显损失分辨率。

SkyWalker 应继续使用 Zephyr CAN API、device model 和 Zephyr 时间 API。

### 12.4 `io/dbus`：以后只移植纯协议解码行为

如果以后接 DR16，可以参考 18 字节位拆分和键位映射，但要重写 UART 传输。参考实现还存在：

- 在完整校验通过前就刷新 alive 时间。
- 无效开关值直接归为某个正常档位。
- 没有校验所有通道和保留位。
- HAL DMA/idle 接收逻辑不能用于 Zephyr。

正确拆分是 `dbus_decode_18_bytes()` 纯函数 + Zephyr UART transport + 独立超时状态。

### 12.5 `referee/`：只能按赛季重新核对

参考 README 标的是裁判系统串口协议 `V1.8.0（20250418）`，头文件里又局部加入了 2026 字段。这不等于完整通过 2026 协议核查。

另外，packed bit-field 的布局具有编译器和 ABI 依赖，不能把串口字节直接强转为这些结构。

如果以后实现裁判系统：

1. 先取得当赛季官方协议。
2. 给每个 command ID 建立明确 payload 长度表。
3. 用移位和掩码逐字段解码。
4. 每个结构加 `static_assert` 只用于本地数据大小，不把它当线上 ABI。
5. CRC、帧长、sequence 和超时全部验证。

### 12.6 `tools/fuzzy_pid`：当前不要用

- 七集合模糊规则会显著增加调参维度。
- 当前实现的隶属函数端点和分母边界可疑。
- D 项是反馈差分但没有除以 `dt`，参数会随周期变化。
- 缺少 reset、有限值检查和完整 anti-windup。
- 使用了当前项目 C++14 不应依赖的 C++17 写法。

先把普通串级 PID、轨迹生成、前馈、饱和和故障处理做扎实。没有实车数据证明普通控制器不足前，不引入模糊 PID。

### 12.7 其他模块

| 模块 | 处理建议 |
|---|---|
| `tools/motor_composer` | 只适合两个电机机械耦合驱动同一等效关节；大小 yaw 是串联的两个关节，不要混淆 |
| `tools/diff_drive`、`tools/mecanum` | 当前四舵轮路线不需要 |
| `tools/diff_gear` | 只有机械上确实存在差速耦合机构时再看 |
| `motor/cybergear_motor`、`ddt_motor`、`lk_motor` | 当前型号清单之外，暂缓；需要时先找官方协议 |
| `motor/super_cap`、`referee/pm02` | 等底盘基本运动、裁判功率链路和硬件型号确定后再设计 |
| `io/bmi088`、`io/icm42688`、`io/adc`、LED/蜂鸣器/舵机 | HAL 平台实现不复用；当前有需求时使用 Zephyr 对应子系统 |
| `io/plotter` | 当前项目已有 VOFA，不需要再引入另一套 HAL 串口打印层 |

## 13. 推荐实施顺序

下面按风险从低到高排列。每一步都由你亲手写，前一步测试通过后再进入下一步。

### 阶段 0：固定来源和复用边界

- 在你的开发笔记记录参考仓库 URL 和本次 commit。
- 记录“当前未发现许可证，不复制代码”。
- 后续若仓库变化，先看 diff 和新许可证，不直接跟随最新代码。

完成标准：团队成员都知道这是设计参考，不是已经批准引入的依赖。

### 阶段 1：数学基础

建议新增：

```text
include/control/math/angle.hpp
include/control/math/filters.hpp
lib/control/math/angle.cpp
lib/control/math/filters.cpp
tests/control_math/
```

先实现：

- shortest angle error
- continuous angle unwrap
- clamp/finite validation
- `tau + dt` 一阶低通

完成标准：跨 `+-pi`、首次初始化、NaN、Inf 和异常 `dt` 测试全部通过。

### 阶段 2：舵轮纯运动学

在已有框架建议的文件中实现：

```text
include/control/chassis_types.hpp
include/control/swerve_kinematics.hpp
lib/control/swerve_kinematics.cpp
tests/swerve_kinematics/
```

依次做：

1. 不带最短转向的逆解。
2. 零速保持。
3. 连续角最短转向与驱动反转。
4. 可开关的对齐余弦缩放。
5. 四轮共同反饱和。
6. 最小二乘正解。

完成标准：第 4.5 节测试全部通过；纯算法测试不依赖 Zephyr CAN 和真实电机。

### 阶段 3：通用关节控制器

建议新增：

```text
include/control/joint_controller.hpp
lib/control/joint_controller.cpp
tests/joint_controller/
```

先用假反馈和假执行器测试模式切换、软限位、离线、饱和和 reset。不要一边写控制器一边用真电机猜符号。

完成标准：输出单位始终是 A；没有 CAN 调用；所有失能路径输出零并清历史。

### 阶段 4：接一只 GM6020 舵向模块

- 完成 GM6020 电流模式驱动和 TxGroup 泛化。
- 一只舵向电机先做电流开环极小值确认方向。
- 再做速度内环。
- 最后做位置外环。
- 验证软限位和离线清零。

完成标准：一只舵轮能在架空状态安全追踪连续角目标，停机不跳角，掉线电流归零。

### 阶段 5：四模块底盘

- 接四套关节控制器和四个驱动轮速度 PI。
- 先只允许平移，再允许原地旋转，最后合成运动。
- 逐个标定零点、steer sign 和 drive sign，不在公式里塞临时负号。
- 加入共同轮速反饱和和对齐保护。

完成标准：四个基础运动方向正确；逆解/实测/正解的符号一致。

### 阶段 6：PID 前馈

- 上层轨迹先给速度和加速度目标。
- `ka` 从零开始，小步增加。
- 再增加 `kv`，最后才尝试带死区的 `ks`。
- 每次同时记录 PID、FF、总电流和饱和状态。
- 对比同一轨迹有无前馈的误差和峰值电流。

完成标准：前馈降低跟踪误差，且不增加静止抖动、不造成饱和恢复变慢。

### 阶段 7：双板流式协议

- 先只传 heartbeat 和固定整数测试数据。
- 再传大 yaw 命令/反馈。
- 最后传浮点目标和 fault flags。
- 用人为拆包、粘包、CRC 错误和断线测试状态机。

完成标准：通信断开时两板都进入设计好的安全状态；恢复后不沿用过期目标。

### 阶段 8：大小 yaw 协调

- 标量、同轴、低速开始。
- 小 yaw 单独稳定后，再让大 yaw 低频卸载。
- 加入大小 yaw 软限位和目标捕获。
- 最后再考虑双 IMU 四元数换系、延迟补偿和加速度前馈。

完成标准：协调器只有一份；任一板失联不会造成两个 yaw 互相追赶或继续保持危险输出。

### 阶段 9：后续增强

按实车数据决定是否加入：

- slip telemetry/traction limiting
- 裁判功率控制和超级电容
- DBUS
- 复杂三维 frame math

不因参考仓库“已经有这个文件”就提前引入。

## 14. 你现在最应该抄的是“验收表”，不是函数体

最终建议取舍表：

| 参考内容 | 是否进入近期计划 | 采用方式 |
|---|---:|---|
| 舵轮零速保持 | 是 | 写成需求和单测，独立实现 |
| 舵轮 180°等价解 | 是 | 用连续角候选选择重写 |
| 舵角余弦降速 | 是，可配置 | 和驱动反转分开实现 |
| 舵轮正解 | 是 | 使用通用最小二乘，不复制手写公式 |
| Joint 模式和串级结构 | 是 | 改为纯 Input/Output 控制器 |
| PID 测量微分/梯形积分/reset | 是 | 合并到当前 PID 重构方案 |
| yaw 惯量/阻尼前馈 | 是，后置 | 改为显式 A 单位并使用轨迹加速度 |
| Gimbal 双 IMU 换系 | 后续 | 只提取小型四元数函数和测试 |
| RM/DM motor | 仅核对 | 官方手册为主，绝不直接替代现有驱动设计 |
| Vision 流式解析 | 是 | 借状态机行为，不借 packed frame |
| CRC | 是 | 自己固定参数和黄金向量后实现 |
| Slip detect | 暂缓 | 先遥测，后控制 |
| Mahony | 否 | 保留当前 IMU/滤波架构 |
| CAN/FDCAN/Timer HAL | 否 | 使用 Zephyr 原生 API |
| Fuzzy PID | 否 | 普通 PID + FF 稳定前不引入 |
| Referee/UI/DBUS | 按需求 | 按当赛季官方协议和 Zephyr transport 重写 |

## 15. 与现有两份手册怎么配合阅读

不要把本文当成第三套互相竞争的架构。三份文档分工如下：

- 《SkyWalker 多型号电机驱动古法实施手册》：负责 M2006、M3508、GM6020、DM-J4310 的协议、设备驱动和发送分组。
- 《SkyWalker 舵轮解算与大小 Yaw 双板协同古法实施框架》：负责最终模块划分、双板所有权、板间消息和大小 yaw 目标关系。
- 本文：负责说明 `sp_middleware` 哪些行为能补强前两份方案，以及哪些参考实现不能进入当前项目。

发生冲突时使用下面的优先级：

```text
电机/裁判系统官方文档
  > 当前项目明确的安全约束和接口
  > 本项目三份实施手册
  > sp_middleware 参考实现
```

## 16. 尚未验证、必须由你补充的信息

- 大 yaw、小 yaw 的真实电机型号和传动比。
- 两个 yaw 是否严格同轴、IMU 安装在哪一级。
- GM6020 实车允许电流以及是否还有上层功率限制。
- 四个舵向电机能否连续旋转，还是有机械/线缆限位。
- 两块板最终使用 Classic CAN、CAN-FD 还是 UART。
- 板间链路是否和电机共用 CAN，以及完整 CAN ID 分配。
- 是否已从 `sp_middleware` 维护者取得复制/修改/分发授权。
- 2026 赛季裁判协议的正式版本和你的参赛组别。

在这些信息确定前，本文所有接口可以先做纯算法和假数据测试，但不要把最大电流、限位和跨板失联时间写成无法配置的常量。

