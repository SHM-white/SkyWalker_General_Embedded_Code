# 05 达妙 DM CAN 电机驱动

实现位置：`drivers/motor/dm/`、`include/drivers/motor/dm_*.hpp`、`dts/bindings/motor/dm-*.yaml`。

当前 profile 是 `dm,j4310-2ec-v1-1`，驱动支持三种持久化控制模式：MIT、位置-速度、速度。设备树模式必须与电机实际配置一致。

## 1. 设备树

```dts
dm_motor: motor {
    compatible = "dm,j4310-2ec-v1-1";
    status = "okay";
    can-bus = <&can1>;
    motor-id = <1>;
    master-id = <0x11>;
    control-mode = "mit";
    p-max-millirad = <12500>;
    v-max-millirad-s = <30000>;
    t-max-millinewton-meter = <10000>;
    torque-limit-millinewton-meter = <1000>;
};
```

`p-max`、`v-max`、`t-max` 必须从达妙调试助手/电机配置读取；它们既参与协议量化，也决定反馈解码范围。`torque-limit-millinewton-meter` 是软件边界，不能超过协议力矩满幅。

## 2. 控制帧

| 模式 | 标准 CAN ID | DLC | 数据 |
|---|---:|---:|---|
| MIT | `motor-id` | 8 | 位置 16 bit、速度/KP/KD/前馈力矩各 12 bit 打包 |
| 位置-速度 | `0x100 + motor-id` | 8 | little-endian float 位置 + 速度上限 |
| 速度 | `0x200 + motor-id` | 4 | little-endian float 速度 |

公共协议 API：`buildMitFrame()`、`buildPositionVelocityFrame()`、`buildVelocityFrame()`、`buildSpecialFrame()`、`decodeFeedback()`。量化前会检查 finite 和上下界，非法值返回 `-EINVAL` 或 `-ERANGE`。

特殊命令：

| 命令 | 值 | 用途 |
|---|---:|---|
| ClearError | `0xFB` | 清理驱动故障 |
| Enable | `0xFC` | 使能 |
| Disable | `0xFD` | 失能 |
| SaveZero | `0xFE` | 保存位置零点 |

## 3. 反馈与路由

DM 反馈帧由 `master-id` 路由；帧首字节低 4 bit 是 motor ID。反馈包含：

- 驱动状态：Disabled、Enabled、OverVoltage、UnderVoltage、OverCurrent、MOS/电机过温、CommunicationLost、Overload。
- 原始位置/速度/力矩。
- MOS 温度与转子温度。
- 按 `p-max/v-max/t-max` 还原后的工程单位。

一个 `dm::Bus` 可以挂多个电机，并为不同 `master-id` 建立接收过滤器；同一 `(master-id, motor-id)` 或 control ID 重复会被拒绝。`CONFIG_SKYWALKER_DM_MAX_MASTER_IDS_PER_BUS` 默认 4，`MAX_MOTORS_PER_BUS` 默认 8。

## 4. Bus 生命周期

```text
init(can) → attach(motor) → arm(report) → set command × N → flush(report)
                                      ↘ stop / recover
```

`arm()` 在需要时发送 Enable，等待真实反馈变为 Enabled，再发送 neutral frame；CAN 发送完成不等于电机已经应答。`flush()` 发现准备或发送失败会对所有已挂电机发送 Disable，并进入 `Fault`。

`recover()` 的第一阶段发送 ClearError + Disable，之后轮询反馈和状态，成功后回到 `Safe`；返回 `-EINPROGRESS` / `-EAGAIN` 是恢复仍在进行，不是可以立即发运动命令。

## 5. 通用 API 与 effort 单位

```cpp
motor::dm::setMitCommand(dev, command);
motor::dm::setPositionVelocity(dev, position_rad, velocity_limit_rad_s);
motor::dm::setVelocity(dev, velocity_rad_s);
motor::dm::readRawFeedback(dev, raw);
motor::dm::getDriveStatus(dev, status);
```

通用 `motor.hpp` 将 DM MIT 映射为 `CommandTorque`，`setTorque()` 的单位是 N·m；DM 不提供 DJI 意义的电流命令。统一 `DmMotorBackend` 当前只实现 MIT effort 后端，原生位置/速度模式请直接使用 DM API 或对应样例。

## 6. Kconfig 与样例

```conf
CONFIG_CAN=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DM=y
CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS=50
CONFIG_SKYWALKER_DM_COMMAND_TIMEOUT_MS=20
```

原生闭环样例：

- `samples/motor/dm_mit_control`：MIT 力矩。
- `samples/motor/dm_velocity_control`：电机内置速度模式。
- `samples/motor/dm_position_control`：电机内置位置-速度模式。

软件闭环样例：

- `samples/motor/dm_mit_velocity_control`：`VelocityMotor` + MIT 后端。
- `samples/motor/dm_mit_position_control`：`PositionMotor` + MIT 后端。

首次上机必须确认电机助手中的控制模式和 PMAX/VMAX/TMAX 与 overlay 完全一致；不一致可能表现为拒收、位置跳变或立即故障。
