# DJI CAN 电机驱动

实现位于 `drivers/motor/{motor,can_bus,group}.cpp` 和 `drivers/motor/dji/dji_protocol.cpp`；接口位于 `include/drivers/motor/`。型号、ID、限幅和时序由 C++ 配置，设备树提供物理 CAN 设备。

调用顺序为配置 → Motor → CanBus attach/start → 持续 enable 与 setter/update → commit。电机不存在或短暂离线时仍接收目标；恢复后自动执行最新有效目标。Group 是批量启停工具，每台电机独立执行和恢复。完整并发说明见[电机工作链路](../../guides/motor-workflow.md)。

## 型号、ID 和电流换算

| 型号 | 反馈 ID | 命令 ID：ID 1–4 / 高 ID | 电机 ID | 协议满幅电流 | 温度反馈 |
| --- | ---: | ---: | ---: | ---: | --- |
| M3508-C620 | `0x200 + id` | `0x200` / `0x1FF` | 1–8 | 20 A | 有效 |
| M2006-C610 | `0x200 + id` | `0x200` / `0x1FF` | 1–8 | 10 A | 无效 |
| GM6020 current | `0x204 + id` | `0x1FE` / `0x2FE` | 1–7 | 3 A | 有效 |

ID 1–4 映射到低命令帧的 slot 0–3，ID 5 起映射到高命令帧的 slot 0–3。一个 8 字节命令帧包含四个 big-endian `int16`。`dji::describe()` 提供反馈 ID、命令 ID、slot、协议满幅和减速比。

量化公式为 `round(目标安培 × 原始满幅 / 协议满幅安培)`。M2006 原始满幅为 10000，其他型号为 16384。3508 的 1 A 约为 819，字节为 `03 33`；−1 A 为 −819，16 位补码为 `FC CD`。ID3 使用 slot 2，即字节 4–5。一个电机离线时只有对应槽位为零，其他槽位正常控制。

## 配置与周期调用

```cpp
using namespace skywalker;
static motor::Motor drive{motor::dji::m3508({
    .id = 3,
    .current_limit_a = 0.3f,
    .gear_ratio = 3591.0f / 187.0f,
    .timing = {.feedback_timeout_ms = 20, .command_timeout_ms = 20,
               .enable_timeout_ms = 100, .retry_interval_ms = 100},
})};
static motor::CanBus bus{DEVICE_DT_GET(DT_NODELABEL(can1))};

int initialize() {
    const int error = bus.attach(drive);
    return error < 0 ? error : bus.start();
}

// run_requested 由用户操作和输入有效期决定，电机状态不改变它。
void tick(bool run_requested, float current_a) {
    if (run_requested) {
        recordCallError(drive.enable());
        recordCallError(drive.setCurrent(current_a));
    } else {
        recordCallError(drive.disable());
    }
    recordCallError(bus.commit().error);
}
```

`recordCallError()` 表示应用自己的错误记录入口。同一物理 CAN 上所有 DJI/DM 电机使用一个 CanBus，由统一发布者协调 setter/update 与每周期一次 commit。

`enable()` 保存持续运行意图，重复调用成功。setter 返回 0 表示目标已保存，不表示设备已经运动。`commit()` 返回 0 表示整条总线最新命令已发布，Recovering 期间也接受发布。命令过期不被重复 commit 续期。

非有限值、超出配置限幅或命令类型不匹配分别返回调用错误，不锁存电机故障。`current_limit_a` 必须为正且不超过协议满幅；`gear_ratio` 是电机轴转数除以输出轴转数。

GM6020 使用实际固定零点 `encoder_zero_ticks`，范围 0–8191，并在确认电流控制模式后填写 `current_mode_confirmed=true`。设备持久模式必须与代码匹配。

## 反馈与位置参考

快照包含 `enabled_requested`、实际 `state`、`feedback_fresh`、最近命令序号／写入时间和历史错误。`active()` 是观测实际执行的辅助查询，上层不得用它阻止目标提交。

`position_rad` 是连续输出轴位置，首帧建立初始坐标；`velocity_rad_s` 为输出轴速度，`current_a` 为实际电流。GM6020 的 1:1 固定零点配置提供 `absolute_position_rad`，范围 `[-π, π)`。检查 `feedback.valid` 后使用字段，检查 `native_dji_feedback_valid` 后使用原始反馈。

连续位置在反馈中断后可能失去可信参考。速度环继续恢复；连续位置轴等待可信坐标，可显式调用 `reseedPosition(known_position_rad)`。GM6020 的绝对角控制可从新的单圈反馈恢复，无需重新定义多圈零点。

反馈帧：

```text
data[0..1] encoder      big-endian uint16
data[2..3] speed_rpm    big-endian int16
data[4..5] current_raw  big-endian int16
data[6]    temperature uint8
data[7]    reserved
```

## 停止与自动恢复

`disable()` 在运行意图真→假时取消此前命令和协议操作；持续重复已停止的 disable 不推进版本，也不擦掉停止后新写入的目标。新目标仍须后续 enable 才执行。

反馈掉线影响本轴实际输出，意图和最新目标保持。新反馈到达后本轴自动恢复；软件 PID 重新建立本轴历史，使用恢复后的反馈计算输出。两参数 `setCurrent(a, sampled_enable_generation)` 用于反馈计算得到的电流，防止掉线前的计算结果直接跨恢复发送；直接电流命令用单参数接口。

CanBus 自行处理传输故障。真实总线故障会影响该物理总线上的输出，其他总线和上层目标生产继续。`BusStatus.last_recovery` 保存重启前错误；当前 Running 状态与历史错误分别理解。

`StopProgress` 表示软件停止帧的发送进度。软件返回成功、CAN TX 完成和机械停止是三个不同事实；失能后机构仍可能自由运动。

## 编译配置

```conf
CONFIG_CAN=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
```

Timing 字段依次为反馈超时、命令超时、协议超时、重试间隔。默认 20、10、100、100 ms；配置使用具名字段。控制周期应短于命令超时，断点暂停后上层恢复更新即可继续。

样例：[recovery](../../../samples/motor/recovery/)、[dji_speed_control](../../../samples/motor/dji_speed_control/)、[dji_position_control](../../../samples/motor/dji_position_control/)、[mixed_topology](../../../samples/motor/mixed_topology/)。
