# 04 DJI CAN 电机驱动

实现位置：`drivers/motor/{motor,can_bus,group}.cpp`、`drivers/motor/dji/dji_protocol.cpp`，公开接口位于 `include/drivers/motor/{dji_motor,motor,can_bus,group}.hpp`。电机型号、ID、限幅和时序由 C++ 配置；设备树只提供物理 CAN 等板级设备。

## 1. 型号与 CAN ID

| 型号 | 反馈 ID | 命令 ID（ID 1–4 / 高 ID） | 电机 ID | 协议满幅电流 | 温度反馈 |
| --- | ---: | ---: | ---: | ---: | --- |
| M3508-C620 | `0x200 + id` | `0x200` / `0x1FF` | 1–8 | 20 A | 有效 |
| M2006-C610 | `0x200 + id` | `0x200` / `0x1FF` | 1–8 | 10 A | 无效 |
| GM6020 current | `0x204 + id` | `0x1FE` / `0x2FE` | 1–7 | 3 A | 有效 |

ID 1–4 使用低命令帧的 slot 0–3，ID 5 起使用高命令帧的 slot 0–3。每个 8 字节命令帧包含 4 个 big-endian `int16`。`dji::describe(config, descriptor)` 可查询反馈 ID、命令 ID、slot、协议满幅、配置限幅和减速比；CAN 设备由 `CanBus` 持有，应用不必拼帧或手动分配 slot。

## 2. C++ 配置与共享总线

通过 `dji::m3508(options)`、`dji::m2006(options)` 或 `dji::gm6020(options)` 生成 `dji::Config`，再构造 `motor::Motor`。同一物理 CAN 上的电机交给同一个 `motor::CanBus`，包括 DJI 与 DM 混接。下面的两个型号使用同一条 CAN 和 `0x200` 命令帧：

```cpp
using namespace skywalker;

static motor::Motor m3508{motor::dji::m3508({
    .id = 1, .current_limit_a = 0.3f,
    .gear_ratio = 3591.0f / 187.0f, .timing = {20, 20, 30, 100},
})};
static motor::Motor m2006{motor::dji::m2006({
    .id = 2, .current_limit_a = 0.3f,
    .gear_ratio = 36.0f, .timing = {20, 20, 30, 100},
})};
static motor::CanBus can1{DEVICE_DT_GET(DT_NODELABEL(can1))};

int ret = can1.attach(m3508, m2006);
if (ret == 0)
    ret = can1.start();
```

`gear_ratio` 是电机轴转数与输出轴转数之比，应按实物减速机构填写。`current_limit_a` 必须为正且不超过对应协议满幅。GM6020 配置还需填写 `encoder_zero_ticks`（0–8191）并在确认固件处于电流环后设置 `current_mode_confirmed = true`。样例记录的电流环固件版本为 `>= 1.0.11.2`；现场仍应核对实际固件手册和电机设置。

`CanBus::start()` 建立接收路由并启动异步 I/O 线程，先准备安全输出。等待 `Motor::ready()` 表示安全准备已完成且反馈新鲜、稳定；使能前仍需由应用检查机械状态、速度和温度。独立电机调用 `Motor::enable()`；若需联动，应在总线启动前创建 `motor::Group group{m3508, m2006}`，所有成员挂接完毕后再调用 `start()`，随后通过 `group.enable()` 使能。进入组后，必须通过组进行使能、停机和清故障。`enable()` 返回 0 只表示请求被接受，运动命令须等 `active()` 为真后写入。

`setCurrent(ampere)` 将目标写入电机命令缓存；`CanBus::commit()` 发布这一周期的命令，实际发帧由 I/O 线程完成。返回的 `CommitResult.sequence` 是提交序号，`error == 0` 不代表 CAN 已发送成功。查看 `CanBus::status().last_tx` 和 `Motor::snapshot()` 追踪发送、反馈与停机进度。多个独立 Group 可以共享一个 `CanBus`，即使其成员使用同一命令帧，也会按 slot 合成完整帧。

## 3. 反馈、参考点与故障

`Motor::snapshot()` 中的 `feedback.valid` 和 `timestamp_ms` 标明哪些字段可用及最后接收时间；`feedback_fresh` 会按配置的反馈超时实时计算。`position_rad` 是按减速比换算的连续输出轴位置，初次有效反馈以 0 为参考。`velocity_rad_s` 是输出轴速度；`current_a` 是解码后的实际电流。M2006 不声明 `FeedbackTemperature`。GM6020 的 1:1 固定零点配置还提供 `FeedbackAbsolutePosition`，`absolute_position_rad` 位于 `[-π, π)`。原始编码器、转速及电流字段可从 `native_dji_feedback` 读取，但先确认 `native_dji_feedback_valid`。

反馈中断会令连续坐标参考失效；在重新定位后，可于未使能且反馈新鲜时调用 `reseedPosition(known_position_rad)`。位置控制应检查 `position_reference_valid` 与 `reference_generation`，不能沿用失效前的坐标。

反馈过期、命令超时、控制器故障或命令无效时，驱动撤销输出许可并请求安全帧；共享物理 CAN 故障会影响该总线所有电机，单电机故障则牵连其所在 Group。`disable()` 也是异步安全请求，`snapshot().stop.progress` 可区分 Pending 与已完成发送。对处于 `MotorState::Fault` 的独立电机显式调用 `clearFault()`，组内电机调用 `group.clearFault()`；等待重新 `ready()` 后才可再次使能。清故障不会自动使能。暂时离线的电机需等待反馈和安全准备恢复，再检查是否可使能。

## 4. 帧格式

反馈帧要求标准 CAN、DLC=8：

```text
data[0..1]  encoder      big-endian uint16
data[2..3]  speed_rpm    big-endian int16
data[4..5]  current_raw  big-endian int16
data[6]     temperature  uint8
data[7]     保留
```

命令帧由 4 个 big-endian `int16` 组成。驱动按型号协议满幅将安培值量化；非有限值、超出软件限幅或型号协议限幅的命令会被拒绝。`dji_protocol.hpp` 中的 `decodeFeedback()` 和 `buildCommandFrame()` 是底层帧函数，通常由共享总线使用。

## 5. 配置与上机

```conf
CONFIG_CAN=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
```

`Timing{feedback_timeout_ms, command_timeout_ms, recovery_stable_ms, enable_timeout_ms}` 随每台电机配置；DJI 默认值分别为 20、10、20、100 ms。共享总线容量、接收队列和 I/O 线程参数由 `CONFIG_SKYWALKER_MOTOR_MAX_MOTORS_PER_BUS`、`CONFIG_SKYWALKER_MOTOR_MAX_BUSES`、`CONFIG_SKYWALKER_MOTOR_RX_QUEUE_DEPTH`、`CONFIG_SKYWALKER_MOTOR_IO_STACK_SIZE` 等通用 Kconfig 项控制。控制周期应明显小于命令超时；调试断点会让反馈和命令过期。

首次上机：

1. 先运行 [can_smoke](../samples/motor/can_smoke/) 确认 CAN 收帧、终端电阻和波特率。
2. 核对型号、CAN 口、电机 ID、减速比、机械方向与 `describe()` 给出的 slot。
3. GM6020 填入实际固定零点，并确认电流环；先设低电流限幅。
4. 等待安全准备和新鲜反馈，检查静止及温度后再使能。
5. 先发零输出与极小正负电流；停机后查看安全帧的异步完成状态。

相关样例：[dji_unified](../samples/motor/dji_unified/)、[dji_speed_control](../samples/motor/dji_speed_control/)、[dji_position_control](../samples/motor/dji_position_control/)、[m2006_speed_control](../samples/motor/m2006_speed_control/) 和 [mixed_topology](../samples/motor/mixed_topology/)。

## 完整工作链路与并发约定

见 [17 电机工作链路](17-motor-workflow.md) 的模块调用图、完整调用示例和故障时序，或在 [浏览器](architecture-browser/index.html#motor-workflow) 中逐步查看。业务线程数量不固定；每个控制器保持单写入方，共享 CAN 的命令发布需协调。发送候选把帧、批次与操作代次绑定，反馈间断先撤销旧许可，快速恢复重新建立稳定窗口，锁存 Fault 不因后续通信恢复而自动清除。公开 setter/update/commit 调用方式保持不变。
