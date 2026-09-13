# 04 DJI CAN 电机驱动

实现位置：`drivers/motor/dji/`、`include/drivers/motor/dji_*.hpp`、`dts/bindings/motor/dji-*.yaml`。

## 1. 支持的 profile

| profile | 反馈 ID | 命令 ID | 电机 ID | 协议满幅电流 | 温度 |
|---|---:|---:|---:|---:|---|
| M3508-C620 | `0x200 + id` | `0x200` / `0x1FF` | 1–8 | 20 A | 有效 |
| M2006-C610 | `0x200 + id` | `0x200` / `0x1FF` | 1–8 | 10 A | 无效 |
| GM6020 current | `0x204 + id` | `0x1FE` / `0x2FE` | 1–7 | 3 A | 有效 |

低 ID 使用低命令帧，高 ID 使用高命令帧；每个 8 字节命令帧有 4 个 `int16` slot。profile 由 compatible 决定，应用不应手动拼接帧 ID。

## 2. 设备树

公共属性：

| 属性 | 含义 |
|---|---|
| `can-bus` | 所属 Zephyr CAN/FDCAN 设备 |
| `motor-id` | 电机 ID |
| `current-limit-ma` | 软件电流边界，不能超过 profile 协议满幅 |
| `gear-ratio-num/den` | 电机轴到输出轴的减速比；反馈已换算到输出轴 |

GM6020 还要求：

```dts
compatible = "dji,gm6020-current";
current-loop-confirmed;
encoder-zero-ticks = <0>;
```

`current-loop-confirmed` 是显式安全门槛；`encoder-zero-ticks` 是固定零点对应的原始 0–8191 tick。GM6020 current profile 要求固件支持电流环，样例按 `>= 1.0.11.2` 记录，现场仍应以实际固件手册为准。

## 3. 公共 API 与状态

`include/drivers/motor/motor.hpp` 提供通用设备 API：

- `capabilities(dev)`：读取 `CommandCurrent`、`FeedbackPosition`、`FeedbackVelocity`、`FeedbackCurrent`、`FeedbackTemperature`、`FeedbackAbsolutePosition` 等能力。
- `setCurrent(dev, ampere)`：当前 DJI profile 的命令入口。
- `readFeedback(dev, Feedback&)`：读取缓存反馈，带 `valid` 和 `timestamp_ms`。
- `getState(dev)`：`Offline`、`Ready`、`Fault`。

反馈语义：

- `position_rad`：连续输出轴角度，经过减速比换算；参考点由驱动生命周期重置。
- `absolute_position_rad`：只有固定零点 profile 才有效，范围为单圈角度。
- `velocity_rad_s`：输出轴速度。
- `current_a`：实际解码电流。

`dji::describe()` 可取得 CAN、motor ID、feedback/command ID、slot、协议满幅、配置限幅和减速比。`readRawFeedback()` 用于只读诊断。

## 4. Bus 生命周期

```text
Bus::init(can)
  → attach(motor) × N
  → arm(report)       检查反馈，发送零帧，建立生命周期 epoch
  → setCurrent(...) × N
  → flush(report)     按命令 ID 分组发送完整帧
  → stop(report)      清除命令并回到 Safe
```

如果反馈过期、命令缓存超过 timeout、发送失败或 slot 无效，Bus 会进入 `Fault` 并尝试发送零帧。`recover()` 清理驱动缓存并回到 `Safe`；重新使用前仍必须再次 `arm()`。

同一物理 CAN 的多个 DJI 电机必须共用一个 `dji::Bus`，由一个线程统一 flush。每个电机单独创建 Bus 会产生互相覆盖或重复 CAN 回调的风险。`DjiMotorBackend` 是单电机独占 wrapper，不适合直接拼成多轴共享 Bus。

## 5. CAN 帧格式

反馈帧要求标准 CAN、DLC=8：

```text
data[0..1]  encoder      big-endian uint16
data[2..3]  speed_rpm    big-endian int16
data[4..5]  current_raw  big-endian int16
data[6]     temperature  uint8
data[7]     保留
```

命令帧为 4 个 big-endian `int16`。驱动将 A 按 profile 满幅映射到原始量程，并在设备树限幅和协议满幅两层校验；超限返回 `-ERANGE`。

## 6. Kconfig 与时序

```conf
CONFIG_CAN=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
```

可调参数：

- `CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS`，默认 20 ms。
- `CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS`，默认 10 ms。
- `CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS`，默认 12。
- `CONFIG_SKYWALKER_DJI_MAX_BUSES`，默认 3。
- `CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY`，默认 90。

控制周期必须明显小于命令 timeout；断点会让反馈/命令同时过期，不能以断点暂停代替真实调试。

## 7. 首次上机清单

1. 先运行 `samples/motor/can_smoke` 确认 CAN 收帧。
2. 核对 `motor-id`、命令 slot、终端电阻和电机方向。
3. GM6020 写入真实 `encoder-zero-ticks`，确认 current loop。
4. 将 `current-limit-ma` 和 wrapper `effort_abs_max` 都设为小值。
5. 只在收到新鲜反馈、状态 `Ready` 后 arm。
6. 先做零输出和极小正负电流，确认机械正方向。

相关样例：[dji_unified](../samples/motor/dji_unified/)、[dji_speed_control](../samples/motor/dji_speed_control/)、[dji_position_control](../samples/motor/dji_position_control/) 和 [m2006_speed_control](../samples/motor/m2006_speed_control/)。
