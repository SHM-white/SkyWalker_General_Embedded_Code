# 09 统一速度 / 位置电机控制

实现位置：`include/control/{velocity_motor,position_motor,motor_common}.hpp`、`lib/control/motor_control.cpp`；控制算法位于 `lib/control/motor_{velocity,position}.c`。`VelocityMotor` 和 `PositionMotor` 接收一个 `motor::Motor&`，由驱动的 `CanBus` 统一接收反馈、发送命令和处理安全停机。

## 1. 对象与配置

一台物理电机创建一个 `motor::Motor`。按型号从 `motor::dji::m3508/m2006/gm6020` 或 `motor::dm::j4310Mit/j4310Velocity/j4310PositionVelocity` 工厂取得配置，再把 Motor 挂到对应物理 CAN 的 `motor::CanBus`。控制器需要电机具备 effort 命令能力：DJI 选 `EffortUnit::Ampere`，DM MIT 选 `EffortUnit::NewtonMeter`。电机原生速度/位置模式可直接调用 `Motor::setVelocity()`、`setPositionVelocity()`。

`VelocityMotor::Config` 含 `control_motor_velocity_config loop`、`EffortUnit effort_unit`、`MotorSafety safety`；`PositionMotor::Config` 还含 `PositionReference reference`。`loop` 是现有 C PID、前馈、斜坡和时间范围的配置；完整赋值见下方样例。安全字段只有：

```cpp
control::MotorSafety safety{
    .velocity_abs_max_rad_s = 12.0f,
    .temperature_max_c = 60.0f,
};
```

速度限值必须是正的有限数。温度限值 `0` 表示不启用温度截止；正值要求电机声明温度反馈。DM 的通用温度是转子温度；启用温度保护时，使能前还要求原生 MOS 与转子温度有效并低于限值。`loop.effort_abs_max` 不得超过 `motor.info().current_limit_a` 或 `torque_limit_nm`，请求速度限幅不得超过安全速度限幅。`configure()` 会校验控制环、能力、单位与限幅，并把该控制器绑定为电机唯一的命令生产者，同时登记速度、温度和所需反馈条件；绑定后不要再直接对同一 Motor 调用 `setCurrent()` / `setTorque()` 或绑定第二个控制器。

## 2. 启动与每周期调用

`configure()` 要在 `CanBus::start()` 之后调用。控制器不负责打开板级电源、使能电机或提交 CAN；应用完成这些步骤。推荐顺序：

1. 构造 Motor、CanBus、控制器及需要的 Group；先挂接全部组成员（包括跨 CAN 成员），再启动各总线。有板级电源时，先安装 CAN 路由，再接通电机电源。Motor、CanBus 和控制器应保持到应用结束，供异步回调和 I/O 线程使用。
2. 调用 `axis.configure()`；等待 `drive.ready()`，在未使能状态调用 `axis.reset()`，检查反馈、速度、温度与位置参考。
3. 独立电机调用 `drive.enable()`，组内电机调用 `group.enable()`；驱动在进入 Enabling 前对已绑定控制器重新检查最新反馈、速度、温度和位置参考，因此 reset 后条件变化会使使能被拒绝。等待 `drive.active()` 或 `group.active()`；使能返回 0 只表示请求已受理。
4. 每个控制周期调用 `axis.update(target, dt_s)`，然后调用所在 `bus.commit()`。`dt_s` 由应用根据实际周期传入，必须落在配置的 PID 时间范围内。总线提交仅发布命令，发帧和回调在 I/O 线程异步完成。

```cpp
// 独立 drive 已 ready，axis 已 configure/reset，且 drive 已 active。
const float dt_s = 0.005f; // 示例值；实际应用应使用本周期真实间隔
int ret = axis.update(target_rad_s, dt_s);
if (ret == 0) {
    const motor::CommitResult sent = bus.commit();
    ret = sent.error;
}
if (ret < 0)
    (void)drive.disable();
const auto data = axis.telemetry();
```

`VelocityMotor::update(float target_rad_s, float dt_s)` 与 `PositionMotor::update(double target_position_rad, float dt_s)` 读取驱动快照、检查反馈/限幅并计算 C 控制环输出，最后将安培或牛·米命令写入 Motor。使能世代或位置参考世代改变后的第一次 `update()` 会重置控制历史并提交零 effort；随后周期才输出闭环结果。`update()` 不调用 `CanBus::commit()`，即使第一个周期是零命令，应用也应照常提交。

`telemetry()` 返回最近一次操作的快照，包括电机反馈、目标、实际 `dt_s`、控制输出、effort 单位、`valid` 和 `error`。更新失败后不要继续使用旧 effort；失败的 telemetry 会标记无效。控制期间发现反馈、时序或安全问题，驱动会撤销输出许可并进入相应安全/故障状态。

## 3. 位置参考

| `PositionReference` | 目标意义 | 适用场景 |
| --- | --- | --- |
| `StartupRelative` | 显式 `reset()` 取当前位置为零，并跨后续使能代次保持；未显式复位或位置参考世代失效时按最新反馈重建 | 普通相对位置 |
| `DriverContinuous` | 目标直接使用驱动连续位置坐标 | 多圈轴、已知机械原点 |
| `AbsoluteNearest` | 固定零点单圈目标，经最短角度误差投到连续坐标 | 具备绝对单圈反馈的 GM6020 yaw |

`PositionMotor::reset()` 要求新鲜位置、速度及有效的 `position_reference_valid`；`AbsoluteNearest` 还要求 `FeedbackAbsolutePosition`。显式复位后的 `StartupRelative` 原点不会仅因异步使能完成而漂移；控制环内部历史仍会在新使能代次重置。DM MIT 的原生位置反馈不等于带固定零点的单圈绝对位置能力。位置环在连续坐标移动较远后会平移内部计算原点，避免长行程时浮点精度下降。若电机位置参考因反馈中断而失效，应用须重新确认坐标；可在未使能且反馈新鲜时调用 `Motor::reseedPosition(known_position_rad)`，再 `reset()`。

## 4. 多轴、故障与恢复

多个电机可共享一个 `CanBus`，一个总线每周期调用一次 `commit()`；不同物理 CAN 各有一个总线对象。每台电机各建自己的控制器。需要共同使能和共同停机的成员创建 `motor::Group`，成员可以跨 CAN；独立 Group 的故障域彼此分离。组内电机只能用 `Group::enable()/disable()/clearFault()` 管理生命周期，不能直接调用成员的对应方法。任一成员故障会撤销整个组的输出许可；总线控制器故障会影响该 CAN 上的所有电机。

常见返回值：

| 返回值 | 常见原因 |
| --- | --- |
| `-EACCES` | 尚未配置或使能，或组/生产者权限不符 |
| `-EAGAIN` / `-ESTALE` | 安全准备中、反馈不够新鲜或运行中反馈过期 |
| `-ENODATA` / `-ENOTSUP` | 缺少所需反馈字段或电机不具备所选控制能力 |
| `-ERANGE` / `-EINVAL` | 目标、速度、温度、effort、`dt_s` 或配置不合法 |
| `-EBUSY` / `-EALREADY` | 在使能状态复位，或重复配置/重复请求 |

出错时检查 `drive.snapshot()`、`group.status()` 与 `bus.status()`，停止发送运动命令。`disable()` 立即撤销软件输出许可，但安全帧与 DM 失能确认是异步的，应观察 `snapshot().stop.progress`。对 `MotorState::Fault` 的独立电机调用 `clearFault()`，组内使用 `group.clearFault()`；等待重新 `ready()` 后复位控制器，并显式重新使能。清故障及 CAN 控制器恢复都不会自动恢复运动命令。

参考样例：[DJI 速度](../samples/motor/dji_speed_control/)、[DJI 位置](../samples/motor/dji_position_control/)、[DM MIT 速度](../samples/motor/dm_mit_velocity_control/)、[DM MIT 位置](../samples/motor/dm_mit_position_control/)、[共享总线与 Group](../samples/motor/mixed_topology/) 和 [恢复](../samples/motor/recovery/)。

## 完整工作链路与并发约定

见 [17 电机工作链路](17-motor-workflow.md) 的模块调用图、完整调用示例和故障时序，或在 [浏览器](architecture-browser/index.html#motor-workflow) 中逐步查看。业务线程数量不固定；每个控制器保持单写入方，共享 CAN 的命令发布需协调。发送候选把帧、批次与操作代次绑定，反馈间断先撤销旧许可，快速恢复重新建立稳定窗口，锁存 Fault 不因后续通信恢复而自动清除。公开 setter/update/commit 调用方式保持不变。
