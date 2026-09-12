# 08 纯 C 控制算法库（`lib/control`）

`lib/control` 是整套框架的算法底座：**无动态分配、无设备依赖、纯函数式**，
输入 `config/state/input`，输出 `output`，失败时不修改任何状态。所有控制
对象（PID、前馈、复合控制、斜坡、角度、速度/位置内核）都在这里。

> 与 `drivers/pid`（早期 Zephyr 设备型 PID）**不是一回事**。
> IMU 温控、`motor_velocity/position`、`VelocityMotor/PositionMotor`
> 用的都是本库。

---

## 1. 通用约定

- 每个模块都有 `validate(config)` / `reset(state, ...)` / `step(...)` 三段式。
- **失败原子性**：`step()` 出错时 `state` 与 `output` 都不被修改；
  但 `step()` 前必须先 `reset()`，否则返回 `-EACCES`。
- 数值先做 `isfinite` 检查，异常返回 `-EINVAL`/`-ERANGE`。
- `dt_s` 越界（`< dt_min_s` 或 `> dt_max_s`）返回 `-ERANGE`。
- 编译选项 `-O3`，纯 C（`lib/control/CMakeLists.txt`）。

---

## 2. PID（`pid.h`）

```c
typedef struct {
    float kp, ki, kd;
    float derivative_tau_s;      // D 项一阶低通时间常数，0 = 关闭
    float integral_min, integral_max; // I 项输出限幅
    float output_min, output_max;     // 总输出限幅（可不对称）
    float deadband;              // 误差死区
    float dt_min_s, dt_max_s;
} control_pid_config;

typedef struct {
    float integral_output;
    float previous_measurement;
    float filtered_measurement_rate;
    bool initialized;
} control_pid_state;

typedef struct {
    float setpoint, measurement, dt_s;
    bool freeze_integrator;      // 条件积分抗饱和的外部开关
} control_pid_input;

typedef struct {
    float error, effective_error;
    float p, i, d;
    float feedback_unsaturated, total_unsaturated;
    float output;
    bool saturated;
} control_pid_result;

int control_pid_validate(const control_pid_config *config);
int control_pid_reset(control_pid_state *state, float current_measurement);
int control_pid_step(control_pid_state *state, const control_pid_config *config,
                     const control_pid_input *input, control_pid_result *result);
```

实现要点：

- **P**：`kp * effective_error`，`effective_error` 为死区内的 0。
- **D**：使用**测量值变化率**（不是误差变化率）：
  `rate = (measurement - prev)/dt`，一阶低通（`tau>0` 时 `alpha = dt/(tau+dt)`），
  再取 `d = -kd * filtered_rate`。可避免设定值阶跃造成微分冲击。
- **I**：累计 `ki * effective_error * dt`，并钳位到 `[integral_min, integral_max]`。
- **抗饱和**：当输出饱和且误差方向继续推高饱和时冻结积分；或由
  `freeze_integrator=true` 强制冻结（串级控制里位置环常用）。
- `total_unsaturated = p + i + d`（+ 前馈），`output` 为其限幅值；
  `saturated` 标志饱和。

`validate` 拒绝：负增益/负 tau/负死区、`integral_min > integral_max`、
`output_min > output_max`、`dt_min_s <= 0` 或 `dt_max_s < dt_min_s`、
非有限字段。

---

## 3. 前馈（`feedforward.h`）

```c
typedef enum { CONTROL_GRAVITY_NONE = 0, CONTROL_GRAVITY_SIN, CONTROL_GRAVITY_COS } control_gravity_model;

typedef struct {
    float k_bias, k_static, k_velocity, k_acceleration, k_gravity;
    float velocity_epsilon, acceleration_epsilon;
    control_gravity_model gravity_model;
} control_feedforward_config;

typedef struct { float position_ref_rad, velocity_ref, acceleration_ref; } control_feedforward_reference;

int control_feedforward_validate(const control_feedforward_config *config);
int control_feedforward_calculate(const control_feedforward_config *config,
                                  const control_feedforward_reference *reference, float *output);
```

输出 = `k_bias` + 静摩擦项 + `k_velocity·v_ref` + `k_acceleration·a_ref`
+ 重力项。静摩擦的方向由速度/加速度是否超过 `velocity_epsilon` /
`acceleration_epsilon` 判定；重力项用 `SIN`/`COS` 模型。

---

## 4. 复合控制器（`feedforward_pid.h`）

```c
typedef struct { control_pid_config feedback; control_feedforward_config feedforward; } control_feedforward_pid_config;
typedef struct { control_pid_state feedback; } control_feedforward_pid_state;

typedef struct {
    control_pid_input feedback;
    control_feedforward_reference reference;
} control_feedforward_pid_input;

typedef struct {
    control_pid_result feedback;
    float feedforward;
    float output;   // = 反馈输出 + 前馈
} control_feedforward_pid_result;
```

`step()` 内部先算前馈，再把前馈作为**加性项**注入 PID 内核
（`control_pid_step_core(..., additive_output, ...)`），因此抗饱和与限幅
是统一在总输出上判断的。`output_min/max` 限制的是**含前馈的总输出**。

---

## 5. 非对称斜坡限幅（`slew_rate_limiter.h`）

```c
typedef struct { float value; bool initialized; } control_slew_rate_state;
typedef struct { float rising_rate_per_s, falling_rate_per_s; } control_slew_rate_config;

int control_slew_rate_validate(const control_slew_rate_config *config);
int control_slew_rate_reset(control_slew_rate_state *state, float current_value);
int control_slew_rate_step(control_slew_rate_state *state, const control_slew_rate_config *config,
                           float requested_value, float dt_s, float *limited_value, float *limited_rate);
```

上升/下降速率可不同。输出 `limited_value` 与本次实际速率 `limited_rate`
（后者用于前馈的加速度参考）。

---

## 6. 角度工具（`angle.h`）

```c
typedef struct { float last_wrapped_rad, continuous_rad; bool initialized; } control_angle_unwrapper;

int control_angle_unwrap_reset(control_angle_unwrapper *state, float wrapped_rad);
int control_angle_unwrap_step(control_angle_unwrapper *state, float wrapped_rad, float *continuous_rad);
int control_shortest_angle_error(float target_rad, float measurement_rad, float *error_rad);
int control_angle_nearest_continuous_target(float requested_absolute_rad, float measured_absolute_rad,
                                            float measured_continuous_rad, float *continuous_target_rad);
```

规范区间为 **`[-π, π)`**：精确的 `+π` 会映射到 `-π`。

- `unwrap`：把单圈角度解包成连续角度（跨圈 ±2π）。
- `shortest_angle_error`：最短角差，结果在 `[-π, π)`。
- `nearest_continuous_target`：给定固定零点单圈目标，解析出离当前连续位置
  最近的等价连续目标（即最短路径）。

---

## 7. 速度内核（`motor_velocity.h`）

速度环的完整计算链，DJI 与 DM 统一复用：

```c
typedef struct {
    control_feedforward_pid_config regulator;
    control_slew_rate_config reference_slew;
    float measurement_filter_tau_s;         // 速度测量低通
    float soft_deadband_rad_s;              // 软死区（不切断误差，只是抬升）
    float requested_velocity_abs_max_rad_s; // 允许请求上限
    float effort_abs_max;                   // 执行器上限（A 或 N·m）
} control_motor_velocity_config;
```

`step()` 顺序：

1. 一阶低通滤波测量速度。
2. 斜坡限幅请求速度，得到 `velocity_reference` 与 `acceleration_reference`。
3. 计算 `velocity_error = reference - filtered_velocity`；
   施加软死区得到 `effective_error`；用 `reference - effective_error`
   作为 PID 的测量值（保留 PID 内部的误差语义）。
4. 走 `control_feedforward_pid_step`（前馈参考用位置/速度/加速度）。
5. `effort_command = clamp(regulator.output, ±effort_abs_max)`。

校验：`requested_velocity_abs_max_rad_s > 0`、`effort_abs_max >= 0`、
`measurement_filter_tau_s >= 0`、`soft_deadband >= 0`。请求速度越界返回
`-ERANGE`；未 reset 返回 `-EACCES`。

---

## 8. 位置内核（`motor_position.h`）

位置-速度**串级**：

```c
typedef struct { control_pid_config position; control_motor_velocity_config velocity; } control_motor_position_config;
typedef struct { control_pid_state position; control_motor_velocity_state velocity; } control_motor_position_state;
```

- 外环：位置 PID，输出速度参考（单位 rad/s）。
- 内环：上面的速度内核，输出 effort（A 或 N·m）。
- **位置积分默认冻结**：配置 `ki` 不会启用位置积分累积（`freeze_integrator`
  固定为 true 语义），避免串级积分双计。
- 输入可选 `position_reference_rad` / `has_position_reference`：当外环 PID
  使用局部/重基坐标时，前馈仍能拿到物理目标。
- `step()` 失败时，位置状态、内层速度状态与输出都不修改。

---

## 9. 使用示例

```c
static control_motor_velocity_state state;

control_motor_velocity_config cfg = {
    .regulator = {
        .feedback = { .kp=..., .ki=..., .kd=0, .derivative_tau_s=0,
                      .integral_min=-..., .integral_max=...,
                      .output_min=-..., .output_max=...,
                      .deadband=0, .dt_min_s=0.001f, .dt_max_s=0.020f },
        .feedforward = { ... },
    },
    .reference_slew = { .rising_rate_per_s=2.0f, .falling_rate_per_s=2.0f },
    .measurement_filter_tau_s = 0.02f,
    .soft_deadband_rad_s = 0.02f,
    .requested_velocity_abs_max_rad_s = 5.0f,
    .effort_abs_max = 0.5f,
};

control_motor_velocity_reset(&state, measured_v, 0.0f);

control_motor_velocity_input in = {
    .requested_velocity_rad_s = target,
    .measured_velocity_rad_s = measured_v,
    .position_reference_rad = 0.0f,
    .dt_s = dt,
    .freeze_integrator = false,
};
control_motor_velocity_output out;
int ret = control_motor_velocity_step(&state, &cfg, &in, &out);
```

---

## 10. 相关文档

- [02 架构与构建](02-architecture.md)（双 PID 陷阱）
- [09 统一速度/位置封装](09-motor-wrapper.md)
- 样例自检：`samples/control/`
