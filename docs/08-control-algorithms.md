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

## 4. 前馈

前馈配置可以叠加：

```text
output = bias
       + static_friction(sign(velocity/acceleration))
       + k_velocity * velocity
       + k_acceleration * acceleration
       + gravity(position)
```

静摩擦方向由速度/加速度阈值判断；gravity 支持 SIN/COS 模型。前馈不是安全限幅，最终仍需经过电机后端和 runtime 的边界校验。

## 5. 速度与位置内核

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

## 6. 角度处理

`control_shortest_angle_error(target, actual, &error)` 将误差规约到 `[-π, π)`。连续角度解包适用于编码器跨越 ±π 的场景，但必须保证相邻采样间运动没有跨越不可判定的半圈。

## 7. 启用与样例

```conf
CONFIG_SKYWALKER_LIB_CONTROL=y
```

`CONFIG_SKYWALKER_LIB_MOTOR_CONTROL=y` 会自动选择它。`samples/control` 用断言/打印验证复合前馈 PID、斜坡和角度工具；电机样例展示如何把内核接到 DJI/DM backend。

## 8. 常见错误

- 把 rad 当 degree 传给位置目标。
- 位置外环输出速度超过 velocity 内核上限。
- PID 输出限幅高于设备树电流/力矩限幅，导致每次更新被后端拒绝。
- dt 在断点后突然变大，触发 `-ERANGE`；这不是控制器参数本身坏了。
- 位置模式切换参考坐标时不调用 reset，产生跳变或积分残留。
