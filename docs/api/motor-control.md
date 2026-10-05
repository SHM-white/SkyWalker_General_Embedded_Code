# VelocityMotor / PositionMotor 硬件闭环：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

VelocityMotor / PositionMotor 保存最新目标，各轴独立反馈计算与自动重置历史。

**接入状态：已有代码。** 现行 configure/update/telemetry 契约已实现，MotorSession、MotorSafety、ControlFailurePolicy 和 preflight 已移除。

## 职责与关联

纯 C 控制算法 + producer 身份与计算版本；不管理运行许可，不创建线程，不发送 CAN。

输入 / 依赖：[DJI 电机与共享 CAN 总线](motor-dji.md)、[达妙电机：MIT / 速度 / 位置速度](motor-dm.md)、[PID、前馈、斜坡与角度工具](pid.md)

消费者：[云台单轴与本地执行](gimbal.md)、[大 Yaw 回中与独立速度环](big-yaw.md)、[摩擦轮与拨盘发射执行](shooter.md)

## 接口契约

### 1. VelocityMotor(Motor &, const Config &); PositionMotor(Motor &, const Config &); int configure()

```cpp
VelocityMotor(Motor &, const Config &); PositionMotor(Motor &, const Config &); int configure()
```

configure 校验 PID、单位、输出限幅和反馈能力，绑定唯一 producer；按 attach/start/configure 顺序装配。

**返回 / 输出：** 0 配置成功，重复 configure -EALREADY。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法 -EINVAL，输出限幅 -ERANGE，模式/能力不支持 -ENOTSUP，producer 冲突返回绑定错误。

### 2. int VelocityMotor::update(float target_rad_s, float dt_s); int PositionMotor::update(double target_position_rad, float dt_s)

```cpp
int VelocityMotor::update(float target_rad_s, float dt_s); int PositionMotor::update(double target_position_rad, float dt_s)
```

合法目标已接受即返回 0，离线/参考等待时仍推进 target_sequence；output_valid 表示实际计算。恢复首可用周期重置本轴历史并提交带执行版本的零 effort。

| 参数 | 含义与边界 |
| --- | --- |
| `target` | 速度 rad/s 或目标位置 rad；PositionReference 决定参考 |
| `dt_s` | 实际周期 s；有限异常 dt 跳过本次积分，负数/非有限非法 |

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法/超范围/未配置/producer 等调用错误；普通等待 error=0，原因由 telemetry.issue 观察。

### 3. int VelocityMotor::reset(); int PositionMotor::reset()

```cpp
int VelocityMotor::reset(); int PositionMotor::reset()
```

清本轴控制历史、作废 computed effort；不能重定义 StartupRelative 原点。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 未配置 -EACCES；下次有效周期自动初始化。

### 4. VelocityMotor::Telemetry telemetry() const; PositionMotor::Telemetry telemetry() const

```cpp
VelocityMotor::Telemetry telemetry() const; PositionMotor::Telemetry telemetry() const
```

值副本含 target_valid、target_sequence、output_valid、issue/error、同周期 MotorSnapshot 和计算输出。

**返回 / 输出：** 一致遥测副本；目标被接受不表示已经产生 effort。

**线程 / 时序：** 允许独立读取线程；configure/reset/update 由一个 owner 串行执行。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 5. enum class PositionReference { StartupRelative, DriverContinuous, AbsoluteNearest }

```cpp
enum class PositionReference { StartupRelative, DriverContinuous, AbsoluteNearest }
```

StartupRelative 保留首次可信原点，DriverContinuous 要求可信多圈坐标，AbsoluteNearest 用单圈绝对角本地展开。

**返回 / 输出：** 参考策略；不是独立恢复授权。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 缺参考时本轴等待，不猜圈数、不重采启动零位。

## 调用示例

### 控制器每周期持续更新

```cpp
if (run_requested) {
    const int enabled = drive.enable();
    const int accepted = axis.update(target_rad_s, dt_s);
} else {
    (void)drive.disable();
}
const auto published = bus.commit();
const auto t = axis.telemetry();
// t.target_valid/target_sequence 是接收；t.output_valid 是本周期输出。
```

应用周期片段，构造与实物配置以链接源码为准。

## 调用顺序

1. 静态构造，attach/start/configure。
2. 运行意图有效时持续 enable/update，每物理 CAN commit 一次。
3. 本轴反馈暂不可用时作废 computed effort，目标继续更新。
4. enable/reference generation 变化后自动重置本轴控制历史。
5. 用户停止取消旧目标；StartupRelative 的首次可信原点保留。

## 遥测与恢复等待

`VelocityMotor::Telemetry` 与 `PositionMotor::Telemetry` 都包含 `motor`、`target_valid`、`target_sequence`、`output_valid`、`effort_command`、`error` 和 `issue`。速度遥测额外给出 `target_rad_s`；位置遥测给出 `requested_position_rad`、解析后的 `target_position_rad` 与 `position_rad`。角度单位 rad，速度 rad/s，周期 s；effort 按 `EffortUnit` 为 A 或 N·m。

| 结果 | 读取含义 | 周期策略 |
| --- | --- | --- |
| `update() == 0` 且 `output_valid == true` | 当前目标已接收并产生有效输出（恢复首周期可能为零输出） | 统一 commit |
| `update() == 0` 且 `output_valid == false` | 合法目标已接收，当前反馈、参考、执行状态或有限周期范围待满足 | 持续更新目标；不撤销其他轴 |
| `update() < 0` | 未配置、非法参数、能力 / producer 或本次计算错误 | 记录调用错误，处理本轴原因；其他轴继续 |

`dt_s` 负数或非有限值返回调用错误；有限值超出 PID 的 `dt_min_s` / `dt_max_s` 时接收目标、跳过计算并报告 `InvalidPeriod`。`target_sequence` 表示有效目标接收次数，不是 CAN TX 次数。

`VelocityMotor::motor()` / `PositionMotor::motor()` 返回底层 `motor::Motor&`；`PositionMotor::reference()` 返回配置的 `PositionReference`。它们不启停设备或建立新参考。`reset()` 重置控制历史，不代替 configure、不推进用户授权，也不能把 StartupRelative 重新置零。

完整声明：[velocity_motor.hpp](../../include/control/velocity_motor.hpp)、[position_motor.hpp](../../include/control/position_motor.hpp)；周期实现：[motor_control.cpp](../../lib/control/motor_control.cpp)。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `Config.loop / effort_unit` | PID、目标斜坡、请求目标/effort 限幅和 Ampere/NewtonMeter。 |
| `PositionReference` | 按机构选择可信坐标，恢复不改变首次 StartupRelative 原点。 |

- 没有 ready/clearFault/MotorSafety/preflight 准入。
- DJI=A，DM MIT=N·m；原生 Velocity 模式不用主控力矩封装。
- 同轴普通 setter 与封装 producer 不混用。
- 反馈计算结果携带计算时 enable_generation；commit 不续期。

## 正文与源码

- [MotorControl Markdown](../modules/control/motor-control.md)
- [调用顺序](../modules/call-examples.md)

- [速度封装API](../../include/control/velocity_motor.hpp)
- [位置封装API](../../include/control/position_motor.hpp)
- [单位与安全配置](../../include/control/motor_common.hpp)
- [闭环绑定实现](../../lib/control/motor_control.cpp)
- [速度闭环完整样例](../../samples/motor/dji_speed_control/src/main.cpp)
- [位置闭环样例](../../samples/motor/dji_position_control/src/main.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
