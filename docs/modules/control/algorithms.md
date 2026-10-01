# 08 纯 C 控制算法库

实现位置：`lib/control/`、`include/control/`。库无动态分配，面向固定周期、嵌入式线程和电机闭环。

## 1. 通用输入约定

- 所有浮点输入必须 finite。
- dt 必须在配置的 `[dt_min_s, dt_max_s]` 内，否则返回 `-ERANGE`。
- `reset()` 要在第一次 `step()` 前调用。
- 失败时控制器状态和输出保持不变，便于上层执行安全停机。
- 输出上下限支持不对称值，例如 `[-0.3, 0.3]`。

## 2. 模块

| 头文件 | 功能 |
|---|---|
| `pid.h` | 条件积分抗饱和、积分/输出限幅、死区、测量微分低通 |
| `feedforward.h` | bias、静摩擦、速度、加速度、重力项 |
| `feedforward_pid.h` | 前馈与 PID 的加性组合 |
| `slew_rate_limiter.h` | 非对称上升/下降速率限制 |
| `angle.h` | 最短角差和连续角度解包 |
| `motor_velocity.h` | 速度滤波、目标斜坡、软死区、复合控制器 |
| `motor_position.h` | 位置外环 + 速度内环，可选物理目标前馈 |

## 3. PID 的实际语义

PID 输入是 setpoint、measurement、dt。D 项对测量变化率计算并低通，不直接对 setpoint 求导，可减少目标阶跃导致的微分冲击。积分器支持 `freeze_integrator`，上层检测到输出饱和或安全状态时可以冻结积分。

建议配置顺序：

1. 先验证 `dt_min_s < dt_max_s`。
2. 先只开 P，确认方向和单位。
3. 再加 D 抑制响应，最后加 I 消除静差。
4. 让 PID 输出限幅不超过后端设备树限幅。

## 4. 最小 PID 调用

纯 C API 不负责创建线程或发送 CAN。配置、复位、单步计算的顺序如下；失败时不要把上一次 `result.output` 再交给执行器：

```c
#include <control/pid.h>

control_pid_config config = {
    .kp = 1.0f, .ki = 0.0f, .kd = 0.0f,
    .integral_min = -0.2f, .integral_max = 0.2f,
    .output_min = -0.3f, .output_max = 0.3f,
    .dt_min_s = 0.001f, .dt_max_s = 0.02f,
};
control_pid_state state = {0};
control_pid_result result = {0};
int ret = control_pid_validate(&config);
if (ret == 0) ret = control_pid_reset(&state, measured_speed_rad_s);
control_pid_input input = {
    .setpoint = target_speed_rad_s,
    .measurement = measured_speed_rad_s,
    .dt_s = measured_dt_s,
};
if (ret == 0) ret = control_pid_step(&state, &config, &input, &result);
if (ret == 0) {
    // result.output 仅是数值结果；应用还要检查安全许可与电机限幅。
}
```

多数电机应用直接用 [VelocityMotor / PositionMotor](motor-control.md)，它们负责把该数值内核接到反馈快照和电机命令；`CanBus::commit()` 仍由应用调用。[模块联动](../../applications/module-integration.md)展示完整上下游。

## 5. 前馈

前馈配置可以叠加：

```text
output = bias
       + static_friction(sign(velocity/acceleration))
       + k_velocity * velocity
       + k_acceleration * acceleration
       + gravity(position)
```

静摩擦方向由速度/加速度阈值判断；gravity 支持 SIN/COS 模型。前馈不是安全限幅，最终仍需经过电机后端和 runtime 的边界校验。

## 6. 速度与位置内核

`control_motor_velocity`：

- 对测量速度做可选一阶低通。
- 对请求目标施加上升/下降斜坡。
- 限制请求绝对速度。
- 运行复合前馈 PID，输出 effort。

`control_motor_position`：

- 位置 PID 生成速度目标。
- 速度内核生成 actuator effort。
- 支持测量历史、积分冻结和可选物理目标前馈。

输出 effort 的单位不由 C 内核决定：DJI wrapper 使用 A，DM MIT wrapper 使用 N·m。

## 7. 角度处理

`control_shortest_angle_error(target, actual, &error)` 将误差规约到 `[-π, π)`。连续角度解包适用于编码器跨越 ±π 的场景，但必须保证相邻采样间运动没有跨越不可判定的半圈。

## 8. 启用与样例

```conf
CONFIG_SKYWALKER_LIB_CONTROL=y
```

`CONFIG_SKYWALKER_LIB_MOTOR_CONTROL=y` 会自动选择它。`samples/control` 用断言/打印验证复合前馈 PID、斜坡和角度工具；电机样例展示如何把内核接到 DJI/DM backend。

## 9. 常见错误

- 把 rad 当 degree 传给位置目标。
- 位置外环输出速度超过 velocity 内核上限。
- PID 输出限幅高于设备树电流/力矩限幅，导致每次更新被后端拒绝。
- dt 在断点后突然变大，触发 `-ERANGE`；这不是控制器参数本身坏了。
- 位置模式切换参考坐标时不调用 reset，产生跳变或积分残留。
\n\n更多对象生命周期与完整调用顺序见[封装模块调用示例](../../../call-examples.md)。\n