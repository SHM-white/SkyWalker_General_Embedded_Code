# PID、前馈、斜坡与角度工具：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

无I/O的纯控制计算，明确单位、采样间隔、积分限制与失败语义。

**接入状态：已有代码。** 纯算法已由MotorControl、IMU加热与台架样例复用；可接入上车控制。增益和限幅仍是设备/机构相关参数，现有样例数值不代表整车调参完成。

## 职责与关联

PID算反馈校正，前馈按参考运动估算需要的输出；斜坡限制目标变化率，角度工具解决±π跳变；C速度/位置环把这些算子串起来。硬件权限、反馈时效和CAN由上层封装处理。

输入 / 依赖：直接对接底层设备或算法。

消费者：[IMU：独立采集、姿态与加热](imu.md)、[VelocityMotor / PositionMotor 硬件闭环](motor-control.md)、[舵轮底盘与功率缩放](chassis.md)

## 接口契约

### 1. int control_pid_validate(const control_pid_config *config)

```cpp
int control_pid_validate(const control_pid_config *config)
```

验证有限参数、非负增益/死区/微分时间常数、积分/输出上下限与dt范围。积分上下限限制的是Ki乘过之后的I项输出。

| 参数 | 含义与边界 |
| --- | --- |
| `config` | kp/ki/kd；derivative_tau_s；integral_min/max；output_min/max；deadband；dt_min_s/max_s。 |

**返回 / 输出：** 0合法，-EINVAL不合法。

**线程 / 时序：** 无I/O、无共享状态，任意线程。

**错误 / 边界：** -EINVAL：空指针、非有限数、负增益或范围不合法；dt_min必须>0。

### 2. int control_pid_reset(control_pid_state *state, float current_measurement)

```cpp
int control_pid_reset(control_pid_state *state, float current_measurement)
```

把I项清0、测量历史设为当前值、微分速率清0；避免重新开始时旧积分和导数冲击。

| 参数 | 含义与边界 |
| --- | --- |
| `state` | 每个控制环独有的状态。 |
| `current_measurement` | 当前测量，与后续measurement同单位。 |

**返回 / 输出：** 0成功，-EINVAL失败。

**线程 / 时序：** 由同一算法所有者串行调用；不带锁。

**错误 / 边界：** 失败不写state；正积分下限的配置须自行保证初始I=0也落在范围内。

### 3. int control_pid_step(control_pid_state *state, const control_pid_config *config, const control_pid_input *input, control_pid_result *result)

```cpp
int control_pid_step(control_pid_state *state, const control_pid_config *config, const control_pid_input *input, control_pid_result *result)
```

一次PID。P使用误差，D使用负的测量变化率并可低通；输出饱和且误差继续推向饱和时冻结I，支持显式freeze_integrator。

| 参数 | 含义与边界 |
| --- | --- |
| `state` | 上次状态；首次未initialized会就地初始化测量历史。 |
| `input.setpoint / measurement` | 参考/测量，单位相同。 |
| `input.dt_s` | 真实采样间隔秒，必须落在配置范围。 |
| `input.freeze_integrator` | true保留I项，仍计算P/D。 |
| `result` | error/effective_error、p/i/d、未限幅输出、最终output与saturated。 |

**返回 / 输出：** 0成功；仅成功时更新state/result，失败保持调用者原值。

**线程 / 时序：** 单实例单所有者纯计算，无CAN/驱动操作。

**错误 / 边界：** -EINVAL：空指针/非有限输入或配置；-ERANGE：dt越界、积分状态越界或计算溢出。调用方必须处理失败并停止使用本周期旧result。

### 4. int control_feedforward_validate(const control_feedforward_config *config); int control_feedforward_calculate(const control_feedforward_config *config, const control_feedforward_reference *reference, float *output)

```cpp
int control_feedforward_validate(const control_feedforward_config *config); int control_feedforward_calculate(const control_feedforward_config *config, const control_feedforward_reference *reference, float *output)
```

计算偏置、静摩擦方向项、速度项、加速度项与可选sin/cos重力项的总和。优先由速度参考确定静摩擦方向，速度近零时再看加速度。

| 参数 | 含义与边界 |
| --- | --- |
| `config` | k_bias/static/velocity/acceleration/gravity，方向判定阈值velocity_epsilon/acceleration_epsilon，gravity_model。 |
| `reference` | position_ref_rad、velocity_ref、acceleration_ref。 |
| `output` | 输出物理单位由全部系数共同决定，需与PID输出一致。 |

**返回 / 输出：** 0成功，输出不自动限幅；失败不写output。

**线程 / 时序：** 无状态纯函数。

**错误 / 边界：** -EINVAL：指针/非有限值/负阈值/无效gravity_model；-ERANGE：计算溢出。

### 5. int control_feedforward_pid_validate(const control_feedforward_pid_config *config); int control_feedforward_pid_reset(control_feedforward_pid_state *state, float current_measurement); int control_feedforward_pid_step(control_feedforward_pid_state *state, const control_feedforward_pid_config *config, const control_feedforward_pid_input *input, control_feedforward_pid_result *result)

```cpp
int control_feedforward_pid_validate(const control_feedforward_pid_config *config); int control_feedforward_pid_reset(control_feedforward_pid_state *state, float current_measurement); int control_feedforward_pid_step(control_feedforward_pid_state *state, const control_feedforward_pid_config *config, const control_feedforward_pid_input *input, control_feedforward_pid_result *result)
```

先算前馈，再在PID内部合成和限幅；抗积分饱和考虑PID+前馈总量，而不是先限PID再另加前馈。

| 参数 | 含义与边界 |
| --- | --- |
| `config` | feedback PID配置与feedforward配置。 |
| `state` | PID历史。 |
| `input` | feedback测量/目标/dt + reference位置/速度/加速度。 |
| `result` | 反馈项细节、feedforward和总output。 |

**返回 / 输出：** 0成功；step失败不修改state/result。

**线程 / 时序：** 单所有者纯计算。

**错误 / 边界：** 透传PID/前馈的-EINVAL/-ERANGE；每种量纲的系数须按输出单位配套。

### 6. int control_slew_rate_validate(const control_slew_rate_config *config); int control_slew_rate_reset(control_slew_rate_state *state, float current_value); int control_slew_rate_step(control_slew_rate_state *state, const control_slew_rate_config *config, float requested_value, float dt_s, float *limited_value, float *limited_rate)

```cpp
int control_slew_rate_validate(const control_slew_rate_config *config); int control_slew_rate_reset(control_slew_rate_state *state, float current_value); int control_slew_rate_step(control_slew_rate_state *state, const control_slew_rate_config *config, float requested_value, float dt_s, float *limited_value, float *limited_rate)
```

斜坡限制每周期目标变化，输出限制后的值和真实变化率。上升/下降按数值方向选择，不是按速度绝对值变大/变小选择。

| 参数 | 含义与边界 |
| --- | --- |
| `config` | rising_rate_per_s / falling_rate_per_s为非负变化率。 |
| `state / current_value` | 上次限制目标；step之前须reset。 |
| `requested_value / dt_s` | 本周期目标和正的秒间隔。 |
| `limited_value / limited_rate` | 限制值和每秒实际变化。 |

**返回 / 输出：** 0成功；失败不修改状态/输出。

**线程 / 时序：** 每个目标一份独立state。

**错误 / 边界：** -EACCES：未reset；-EINVAL：空指针/非有限值/负配置；-ERANGE：dt≤0或计算溢出。率=0表示该方向不变化。

### 7. int control_angle_unwrap_reset(control_angle_unwrapper *state, float wrapped_rad); int control_angle_unwrap_step(control_angle_unwrapper *state, float wrapped_rad, float *continuous_rad)

```cpp
int control_angle_unwrap_reset(control_angle_unwrapper *state, float wrapped_rad); int control_angle_unwrap_step(control_angle_unwrapper *state, float wrapped_rad, float *continuous_rad)
```

把每次单圈角度展开成连续角度；增量选[-π,π)的最短方向，要求相邻真实运动少于半圈。

| 参数 | 含义与边界 |
| --- | --- |
| `state` | 独立展开历史，先reset。 |
| `wrapped_rad` | 单圈角度rad，函数规范化到[-π,π)。 |
| `continuous_rad` | 累计连续角度输出。 |

**返回 / 输出：** 0成功，失败不更新。

**线程 / 时序：** 单所有者，适用于按时间有序的角度样本。

**错误 / 边界：** -EACCES：未初始化；-EINVAL：指针/非有限数；-ERANGE：运算溢出。不能恢复丢帧期间未知圈数。

### 8. int control_shortest_angle_error(float target_rad, float measurement_rad, float *error_rad); int control_angle_nearest_continuous_target(float requested_absolute_rad, float measured_absolute_rad, float measured_continuous_rad, float *continuous_target_rad)

```cpp
int control_shortest_angle_error(float target_rad, float measurement_rad, float *error_rad); int control_angle_nearest_continuous_target(float requested_absolute_rad, float measured_absolute_rad, float measured_continuous_rad, float *continuous_target_rad)
```

前者算最短角度误差，后者把单圈绝对目标映射到当前连续坐标中最近的一圈。恰好+π会规范化成-π。

| 参数 | 含义与边界 |
| --- | --- |
| `target_rad / requested_absolute_rad` | 绝对单圈目标，rad。 |
| `measurement_rad / measured_absolute_rad` | 同一固定零点的单圈测量。 |
| `measured_continuous_rad` | 当前连续位置。 |
| `error_rad / continuous_target_rad` | 误差或最近的连续目标。 |

**返回 / 输出：** 0成功。

**线程 / 时序：** 无历史纯调用。

**错误 / 边界：** -EINVAL：非有限/空输出；-ERANGE：差值或加法溢出。绝对测量与连续测量必须来自同一电机参考。

### 9. int control_motor_velocity_validate(const control_motor_velocity_config *config); int control_motor_velocity_reset(control_motor_velocity_state *state, float measured_velocity_rad_s, float initial_reference_rad_s); int control_motor_velocity_step(control_motor_velocity_state *state, const control_motor_velocity_config *config, const control_motor_velocity_input *input, control_motor_velocity_output *output)

```cpp
int control_motor_velocity_validate(const control_motor_velocity_config *config); int control_motor_velocity_reset(control_motor_velocity_state *state, float measured_velocity_rad_s, float initial_reference_rad_s); int control_motor_velocity_step(control_motor_velocity_state *state, const control_motor_velocity_config *config, const control_motor_velocity_input *input, control_motor_velocity_output *output)
```

纯速度环串联测量低通、目标斜坡、软死区、前馈PID和执行量限幅。它不检查电机在线或发送CAN。

| 参数 | 含义与边界 |
| --- | --- |
| `config` | regulator、reference_slew、measurement_filter_tau_s、soft_deadband_rad_s、目标速度与执行量上限。 |
| `state` | 每个电机独立状态，先reset。 |
| `input` | 目标/测量rad/s、前馈位置rad、dt秒、积分冻结标志。 |
| `output` | 限制目标、加速度、滤波速度、误差、调节器分量与effort_command。 |

**返回 / 输出：** 0成功；失败保持state/output。

**线程 / 时序：** 控制所有者纯计算。

**错误 / 边界：** -EACCES：没reset；-EINVAL：配置/非有限值；-ERANGE：目标/dt超限或数值异常。effort单位由调用者决定。

### 10. int control_motor_position_validate(const control_motor_position_config *config); int control_motor_position_reset(control_motor_position_state *state, const control_motor_position_config *config, float measured_position_rad, float measured_velocity_rad_s); int control_motor_position_step(control_motor_position_state *state, const control_motor_position_config *config, const control_motor_position_input *input, control_motor_position_output *output)

```cpp
int control_motor_position_validate(const control_motor_position_config *config); int control_motor_position_reset(control_motor_position_state *state, const control_motor_position_config *config, float measured_position_rad, float measured_velocity_rad_s); int control_motor_position_step(control_motor_position_state *state, const control_motor_position_config *config, const control_motor_position_input *input, control_motor_position_output *output)
```

串级位置环：外环位置PID给速度参考，内环速度控制给执行量。当前位置环I项冻结；不会因配置ki而持续积累位置积分。

| 参数 | 含义与边界 |
| --- | --- |
| `config` | position外环PID + velocity内环；外环输出范围不能超过内环最大目标速度。 |
| `state` | 先reset的外环/内环历史。 |
| `input` | 连续目标/位置rad、测量rad/s、dt；has_position_reference可指定重力前馈使用的物理角度。 |
| `output` | 外环结果、内环结果与effort_command。 |

**返回 / 输出：** 0成功；失败不改外环、内环或output。

**线程 / 时序：** 单所有者纯计算；参考系与电机权限由PositionMotor负责。

**错误 / 边界：** 透传速度/PID错误；-ERANGE：外环输出范围与内环目标范围不一致。

## 调用示例

### 前馈 + PID 一次更新：检查错误，再使用总输出

```cpp
#include <control/feedforward_pid.h>

int calculateEffort(float measured_rad_s, float dt_s, float &ampere) {
  static control_feedforward_pid_state state{};
  static const control_feedforward_pid_config cfg{
    .feedback={.kp=0.02f,.ki=0.05f,.kd=0,.derivative_tau_s=0,
      .integral_min=-0.1f,.integral_max=0.1f,
      .output_min=-0.8f,.output_max=0.8f,.deadband=0,
      .dt_min_s=0.001f,.dt_max_s=0.020f},
    .feedforward={.k_bias=0,.k_static=0.005f,.k_velocity=0,
      .k_acceleration=0,.k_gravity=0,.velocity_epsilon=0,
      .acceleration_epsilon=0,.gravity_model=CONTROL_GRAVITY_NONE}};
  if (!state.feedback.initialized) {
    const int r=control_feedforward_pid_reset(&state,measured_rad_s);
    if (r<0) return r;
  }
  const control_feedforward_pid_input in{
    .feedback={.setpoint=2.0f,.measurement=measured_rad_s,
      .dt_s=dt_s,.freeze_integrator=false},
    .reference={.position_ref_rad=0,.velocity_ref=2.0f,.acceleration_ref=0}};
  control_feedforward_pid_result out{};
  const int r=control_feedforward_pid_step(&state,&cfg,&in,&out);
  if (!r) ampere=out.output;
  return r; // 调用者遇负值应撤销输出，不使用旧ampere
}
```

增益取自台架结构作为占位，输出按A配置；它只算数值，尚无电机反馈时效、权限和安全检查。多电机必须每电机一份state；需要硬件集成时优先用VelocityMotor/PositionMotor。开启SKYWALKER_LIB_CONTROL。

## 调用顺序

1. 为每个控制环设置物理单位、限幅、允许dt和独立state。
2. 启动/恢复时reset，避免复用旧积分与测量导数。
3. 按真实周期dt进行step；仅返回0时使用本周期结果。
4. 暂停、重使能或位置参考变更时，由硬件封装重置算法历史。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `CONFIG_SKYWALKER_LIB_CONTROL` | 编入纯C算法，无Zephyrdevice绑定；状态均由调用者持有。 |
| `dt_min_s / dt_max_s` | 每周期用真实经过时间；dt超限返回错误，不会静默裁剪。 |
| `integral_min/max / output_min/max` | 按执行量单位设置，可为不对称范围；前馈PID共享总输出限幅。 |
| `requested_velocity_abs_max_rad_s / effort_abs_max` | 纯环速度/执行量限制；还需与Motor硬件限幅与安全最大测量速度配套。 |

- 积分限幅是I项输出单位，不是误差积分原始量；不能把其他PID库参数直接复制。
- D作用于测量变化率，目标突变不会直接产生微分踢；这与误差微分算法不同。
- 纯算法不会检查传感器时间戳，返回0不等于电机允许输出。
- 位置外环I冻结，改大ki不会得到想象中的位置积分效果。
- 速度斜坡上升/下降按数值方向；负速度增大绝对值属于数值下降。

## 正文与源码

- [控制算法Markdown](../modules/control/algorithms.md)

- [PID API](../../include/control/pid.h)
- [前馈API](../../include/control/feedforward_pid.h)
- [角度API](../../include/control/angle.h)
- [斜坡API](../../include/control/slew_rate_limiter.h)
- [PID实现](../../lib/control/pid.c)
- [串级位置环](../../lib/control/motor_position.c)
- [纯算法样例](../../samples/control/src/main.c)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
