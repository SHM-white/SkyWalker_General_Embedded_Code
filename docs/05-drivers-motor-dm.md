# 05 达妙 DM CAN 电机驱动

实现位置：`drivers/motor/{motor,can_bus,group}.cpp`、`drivers/motor/dm/dm_protocol.cpp`；公开接口位于 `include/drivers/motor/{dm_motor,dm_protocol,motor,can_bus,group}.hpp`。当前支持 J4310-2EC-V1.1，电机配置在 C++ 中构造，设备树只提供物理 CAN、电源控制等板级设备。

## 1. 控制模式与 C++ 配置

达妙电机有 MIT、位置-速度、速度三种持久化控制模式。固件中的模式必须与电机调试助手中的实际设置一致，分别用 `dm::j4310Mit()`、`dm::j4310PositionVelocity()`、`dm::j4310Velocity()` 创建 `dm::Config`：

```cpp
using namespace skywalker;

static motor::Motor drive{motor::dm::j4310Mit({
    .id = 1, .master_id = 0x11,
    .position_max_rad = 12.5f,
    .velocity_max_rad_s = 30.0f,
    .torque_max_nm = 10.0f,
    .torque_limit_nm = 1.0f,
    .timing = {50, 20, 50, 3000},
})};
static motor::CanBus can1{DEVICE_DT_GET(DT_NODELABEL(can1))};

int ret = can1.attach(drive);
if (ret == 0)
    ret = can1.start();
```

`id` 范围为 1–15，`master_id` 是标准 CAN 反馈 ID。`position_max_rad`、`velocity_max_rad_s`、`torque_max_nm` 必须取自电机实际 PMAX/VMAX/TMAX 设置：它们既参与 MIT 量化，也决定反馈解码。`torque_limit_nm` 是正的应用软件限幅，不能超过 TMAX。`dm::describe(config, descriptor)` 可查询模式、命令 ID、反馈 ID、ID 和限幅；物理 CAN 设备由 `CanBus` 持有。

在需要板级电源控制的板卡上，先 `attach()` / `start()` 安装 CAN 接收路由，再打开电机电源并等待反馈。`Motor::ready()` 只说明安全准备与反馈稳定，可按实物再次检查速度、MOS 和转子温度后调用 `enable()`。该调用只受理异步使能请求；驱动等待真实 `Enabled` 反馈并发送中性命令，`active()` 为真后才能写运动目标。

## 2. 命令帧与模式对应 API

| 模式 | 标准 CAN ID | DLC | 数据 | `Motor` 命令 |
| --- | ---: | ---: | --- | --- |
| MIT | `motor-id` | 8 | 位置 16 bit，速度/KP/KD/前馈力矩各 12 bit 打包 | `setTorque(N·m)` 或 `setMit(MitCommand)` |
| 位置-速度 | `0x100 + motor-id` | 8 | little-endian float 位置与速度上限 | `setPositionVelocity(rad, max_rad_s)` |
| 速度 | `0x200 + motor-id` | 4 | little-endian float 速度 | `setVelocity(rad_s)` |

`setTorque()` 只用于 MIT 模式，其余 MIT 字段置零；完整 MIT 位置、速度和增益命令用 `setMit()`。命令被写入缓存后，每个控制周期调用同一物理 CAN 的 `CanBus::commit()`。其 `CommitResult.error == 0` 表示命令已发布给异步 I/O 线程，`sequence` 是提交序号，不是总线发送确认。查看 `CanBus::status().last_tx`、`Motor::snapshot()` 与 CAN 抓包确认后续状态。

底层协议函数包括 `buildMitFrame()`、`buildPositionVelocityFrame()`、`buildVelocityFrame()`、`buildSpecialFrame()`、`decodeFeedback()`。它们在量化前校验有限数与协议边界。

| 特殊命令 | 值 | 用途 |
| --- | ---: | --- |
| ClearError | `0xFB` | 清理驱动故障 |
| Enable | `0xFC` | 请求使能 |
| Disable | `0xFD` | 请求失能 |
| SaveZero | `0xFE` | 保存位置零点；仅底层协议提供，须由应用单独评估持久化影响 |

## 3. 反馈路由与状态

反馈标准 CAN ID 等于 `master_id`，`data[0]` 的高 4 bit 是驱动状态、低 4 bit 是电机 ID。一个 `CanBus` 可以接入多个 DM 和 DJI 电机；共享同一 `master_id` 的 DM 电机按帧内电机 ID 分发。重复的 `(master_id, motor_id)`、冲突的命令 ID 或命令/反馈 ID 会在 `start()` 时被拒绝。不同 CAN 控制器各有独立的 `CanBus`，也可通过一个 `Group` 联动；跨 CAN 的全部组成员应先挂接，再启动任一总线。

`Motor::snapshot()` 提供工程单位反馈、`feedback.valid`、接收时间、`feedback_fresh` 和状态。`feedback.position_rad` 是连续坐标；`native_position_rad` 是本次解码的原生位置。通用 `feedback.temperature_c` 是转子温度，`native_mos_temperature_c`、`native_rotor_temperature_c` 提供两个原生温度，使用前检查 `native_temperatures_valid`。`native_drive_status` 要先检查 `native_drive_status_valid`。驱动识别 Disabled、Enabled、OverVoltage、UnderVoltage、OverCurrent、MOS/电机过温、CommunicationLost 和 Overload 等状态。

## 4. 安全停机与恢复

`disable()` 撤销软件输出许可并请求 Disable，实际 CAN 发送与电机应答异步进行。`snapshot().stop.progress` 区分 `Pending`、`TxComplete`、`DriveConfirmed` 和 `Unreachable`。DM 需收到 Disable 发送完成后的真实 Disabled 反馈，才能确认驱动失能并重新准备安全状态。反馈或命令超时、非法命令、驱动报告故障时，相关电机/Group 撤销输出；CAN 控制器故障则影响该物理总线上的所有电机。

对 `MotorState::Fault` 的独立电机调用 `clearFault()`；组内通过 `Group::clearFault()`。清故障请求可能经由 ClearError 和安全输出的异步流程，必须等待电机重新 `ready()`，确认驱动 Disabled 和反馈稳定后显式 `enable()` 或 `Group::enable()`。清故障不会自动发送运动目标。`CanBus` 自行尝试恢复 CAN 控制器，但恢复总线不等于重新使能电机。

## 5. 配置与上机

```conf
CONFIG_CAN=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DM=y
```

每台电机的 `Timing{feedback_timeout_ms, command_timeout_ms, recovery_stable_ms, enable_timeout_ms}` 由工厂选项配置；DM J4310 默认分别为 50、20、50、3000 ms。共享总线容量及 I/O 资源使用通用 `CONFIG_SKYWALKER_MOTOR_*` Kconfig 项。

首次上机须核对物理 CAN 口、终端电阻、电机 ID、Master ID、持久化控制模式和调试助手中的 PMAX/VMAX/TMAX。编码范围或模式不一致会造成拒收、解码错误、位置跳变或立即故障。先设小力矩/速度限幅，确认机构安全、温度和正方向，再逐步增大命令。失能后机构仍可能自由转动。

原生模式样例：[dm_mit_control](../samples/motor/dm_mit_control/)、[dm_velocity_control](../samples/motor/dm_velocity_control/)、[dm_position_control](../samples/motor/dm_position_control/)。MIT 软件闭环样例：[dm_mit_velocity_control](../samples/motor/dm_mit_velocity_control/)、[dm_mit_position_control](../samples/motor/dm_mit_position_control/)。共享反馈路由与跨 CAN 组见 [mixed_topology](../samples/motor/mixed_topology/)。

## 完整工作链路与并发约定

见 [17 电机工作链路](17-motor-workflow.md) 的模块调用图、完整调用示例和故障时序，或在 [浏览器](architecture-browser/index.html#motor-workflow) 中逐步查看。业务线程数量不固定；每个控制器保持单写入方，共享 CAN 的命令发布需协调。发送候选把帧、批次与操作代次绑定，反馈间断先撤销旧许可，快速恢复重新建立稳定窗口，锁存 Fault 不因后续通信恢复而自动清除。公开 setter/update/commit 调用方式保持不变。
