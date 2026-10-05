# 云台单轴与本地执行：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

机械单轴保存目标；公开双轴 GimbalExecutor 独立计算两轴并由应用统一提交总线。

**接入状态：已有代码。** 正式云台已装配双轴与惯性适配框架，真实接线与安装确认默认关闭。

## 职责与关联

GimbalAxis 管机械范围与目标，PositionMotor 管本轴反馈计算；GimbalExecutor 管新鲜输入/许可和双轴调用，不持有 CAN。

输入 / 依赖：[VelocityMotor / PositionMotor 硬件闭环](motor-control.md)、[命令仲裁与后台服务](command.md)

消费者：[双主控应用与执行器](application.md)、[头部惯性云台适配](inertial.md)、[大 Yaw 回中与独立速度环](big-yaw.md)、[摩擦轮与拨盘发射执行](shooter.md)

## 接口契约

### 1. int GimbalAxis::validate() const; int GimbalAxis::begin(); Status GimbalAxis::poll(uint64_t now_ms)

```cpp
int GimbalAxis::validate() const; int GimbalAxis::begin(); Status GimbalAxis::poll(uint64_t now_ms)
```

begin 配置；poll 观测反馈/配置允许的参考，Status 仅 feedback_healthy/error，无 ready_for_enable。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 2. int GimbalAxis::reset(); int GimbalAxis::update(const AxisCommand &, SafetyAction, float dt_s); int updateRate(float rate_rad_s, float dt_s)

```cpp
int GimbalAxis::reset(); int GimbalAxis::update(const AxisCommand &, SafetyAction, float dt_s); int updateRate(float rate_rad_s, float dt_s)
```

明确 Hold/reset 获取本轴目标；Rate 时间轴不因驱动离线重置，机械限位保留。

| 参数 | 含义与边界 |
| --- | --- |
| `dt_s` | 实际 s |
| `action` | 业务授权动作；不能代替 Motor disable |

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 3. GimbalExecutor(Motor &yaw, Motor &pitch, Group &, const PositionMotor::Config &, const GimbalAxisConfig &, const PositionMotor::Config &, const GimbalAxisConfig &, const Config &); int begin()

```cpp
GimbalExecutor(Motor &yaw, Motor &pitch, Group &, const PositionMotor::Config &, const GimbalAxisConfig &, const PositionMotor::Config &, const GimbalAxisConfig &, const Config &); int begin()
```

注入长期存活两轴与配置；应用先 attach/start。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 4. RunStatus GimbalExecutor::update(const GimbalExecutionInputs &, core::TimeUs now_us); RunStatus suspend(core::TimeUs, WaitReason, int error = 0, bool blocked = false)

```cpp
RunStatus GimbalExecutor::update(const GimbalExecutionInputs &, core::TimeUs now_us); RunStatus suspend(core::TimeUs, WaitReason, int error = 0, bool blocked = false)
```

source_stamp/permission 与命令分别过期；yaw_output_valid/pitch_output_valid 分别声明数学输入是否足够。update 不 commit CAN。

| 参数 | 含义与边界 |
| --- | --- |
| `now_us` | 单调本地 µs |
| `inputs` | command/source_stamp/permission、transport_ready、两轴输出有效性与急停 |

**返回 / 输出：** RunStatus 值副本，生产 stamp、requested 与各轴统计。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

## 调用示例

### 两轴持续更新，共享总线统一发布

```cpp
const auto status = executor.update(inputs, now_us);
const auto published = bus.commit();
// 若两轴跨物理 CAN，另一总线仍独立 commit；状态用于诊断。
```

应用周期片段，构造与实物配置以链接源码为准。

## 调用顺序

1. 确定 Continuous/Limited 和可信机械参考。
2. 应用 attach/start，axis/executor.begin 配置。
3. 每周期从真实新鲜输入持续更新目标。
4. 本轴缺反馈只作废本轴 computed effort，其他轴继续。
5. 每物理 CAN 周期末一次 commit；真实停止由 suspend/disable 处理。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `GimbalAxisConfig` | Continuous/Limited、机械范围、max_rate、hold_on_zero_rate、Preserve/CalibratedFeedback。 |
| `GimbalExecutor::Config` | 来源/命令 100 ms、许可 300 ms、周期 20 ms；所有 stamp 保留原时刻。 |

- 世界姿态须先经惯性适配，不能直接送编码器角。
- reset 是业务目标/历史操作，不是电机恢复时重采原点。
- 软件停止报告不能替代机械停止测量。

## 正文与源码

- [executors.md](../modules/robotics/executors.md)
- [README.md](../../samples/robotics/gimbal_control/README.md)

- [gimbal_axis.hpp](../../include/robotics/gimbal/gimbal_axis.hpp)
- [gimbal_executor.hpp](../../include/robotics/gimbal/gimbal_executor.hpp)
- [gimbal_axis.cpp](../../lib/robotics/gimbal_axis.cpp)
- [gimbal_executor.cpp](../../lib/robotics/gimbal_executor.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
