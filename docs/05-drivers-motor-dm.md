# 05 达妙 DM 电机驱动（C++）

`drivers/motor/dm/` 实现达妙（Damiao）集成 CAN 电机驱动。当前 profile 为
**DM-J4310-2EC V1.1**（`dm,j4310-2ec-v1-1`），支持 MIT、位置-速度、速度
三种控制模式，其中 MIT 模式可被统一封装复用做软件力矩环。

> 与 DJI 的关键差异：DM 提供**力矩反馈**并在 MIT 模式支持**力矩指令**，
> 位置/速度/力矩都由协议量程 `PMAX/VMAX/TMAX` 归一化，而不是原始电流。

---

## 1. 设备树 binding

公共属性 `dm-motor-base.yaml`：

| 属性 | 类型 | 范围 | 说明 |
|---|---|---|---|
| `can-bus` | phandle | — | CAN 控制器节点 |
| `motor-id` | int | 1–15 | 电机 CAN ID（命令帧地址） |
| `master-id` | int | 0–2047 | 主机 ID，驱动据此注册反馈过滤器 |
| `control-mode` | string | `mit` / `position-velocity` / `velocity` | 与电机**持久化模式一致** |
| `p-max-millirad` | int | >0 | 位置协议量程 PMAX（mrad） |
| `v-max-millirad-s` | int | >0 | 速度协议量程 VMAX（mrad/s） |
| `t-max-millinewton-meter` | int | >0 | 力矩协议量程 TMAX（mN·m） |
| `torque-limit-millinewton-meter` | int | 0 < ≤ TMAX | **软件力矩上限** |

`dm,j4310-2ec-v1-1.yaml` 仅 `include: dm-motor-base.yaml`，未追加属性。

### 示例

```dts
/ {
    aliases { motor0 = &dm_j4310_1; };

    dm_j4310_1: motor-1 {
        compatible = "dm,j4310-2ec-v1-1";
        status = "okay";
        can-bus = <&can1>;
        motor-id = <1>;
        master-id = <0x11>;
        control-mode = "mit";

        /* 必须与达妙调试助手中读到的 PMAX/VMAX/TMAX 一致 */
        p-max-millirad = <12500>;
        v-max-millirad-s = <30000>;
        t-max-millinewton-meter = <10000>;
        torque-limit-millinewton-meter = <1000>;
    };
};
```

> **模式是电机的持久化属性**，必须在达妙调试助手中设置并与
> `control-mode` 一致，驱动不会替你切换模式。
> `torque-limit` 只是软件上限，`TMAX` 是协议编码量程，两者不同。

---

## 2. Kconfig

```conf
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DM=y
CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS=50
CONFIG_SKYWALKER_DM_COMMAND_TIMEOUT_MS=20
CONFIG_SKYWALKER_DM_MAX_MOTORS_PER_BUS=8
CONFIG_SKYWALKER_DM_MAX_MASTER_IDS_PER_BUS=4
```

---

## 3. 控制模式与命令帧 ID

`controlFrameId(mode, motor_id)`：

| 模式 | 偏移 | 命令帧 ID（motor-id=1） | 载荷 |
|---|---|---|---|
| `Mit` | `+0x000` | `0x001` | 8 字节打包：位置/速度/KP/KD/力矩 |
| `PositionVelocity` | `+0x100` | `0x101` | 两个小端 float：位置、速度上限 |
| `Velocity` | `+0x200` | `0x201` | 一个小端 float（DLC=4）：速度 |

`motor-id` 仅允许 1–15，超出返回 `-ERANGE`。

### MIT 打包格式（8 字节大端位域）

| 字段 | 位宽 | 取值范围 | 说明 |
|---|---|---|---|
| `position_rad` | 16 | `[-PMAX, +PMAX]` | 线性映射到 0–65535 |
| `velocity_rad_s` | 12 | `[-VMAX, +VMAX]` | 0–4095 |
| `kp` | 12 | `[0, 500]` | 0–4095 |
| `kd` | 12 | `[0, 5]` | 0–4095 |
| `torque_ff_nm` | 12 | `[-TMAX, +TMAX]` | 0–4095 |

拼接顺序（`buildMitFrame`）：
`pos(16) | vel(12) | kp(12) | kd(12) | torque(12)` 共 64 位，
按大端写入 `data[0..7]`。任一字段越界返回 `-ERANGE`。

### 反馈解码（`decodeFeedback`）

| 字节 | 内容 |
|---|---|
| 0 | 高 4 位 = 状态码，低 4 位 = `motor-id` |
| 1–2 | 位置原始值 `uint16` |
| 3–4 高 4 位 | 速度原始值 `uint16`（12 位有效） |
| 4 低 4 位–5 | 力矩原始值 `uint16`（12 位有效） |
| 6 | MOS 温度（°C） |
| 7 | 转子温度（°C） |

反解：位置 `[-PMAX, PMAX]`，速度 `[-VMAX, VMAX]`，力矩 `[-TMAX, TMAX]`。
帧必须 DLC=8 且不带 IDE/RTR/FDF，`motor-id` 必须匹配，否则报
`-EBADMSG` / `-ENOENT`。

### 状态码（`DriveStatus`）

| 值 | 含义 | 故障？ |
|---|---|---|
| 0x0 | Disabled | 否 |
| 0x1 | Enabled | 否 |
| 0x8 | OverVoltage | 是 |
| 0x9 | UnderVoltage | 是 |
| 0xA | OverCurrent | 是 |
| 0xB | MOS 过温 | 是 |
| 0xC | 电机过温 | 是 |
| 0xD | CommunicationLost | 是 |
| 0xE | Overload | 是 |
| 0xF | Unknown | 是 |

`isFaultStatus(status)`：非 Disabled/Enabled 即视为故障。收到故障反馈时驱动
会**立即锁存 fault、撤销 arm、作废命令**。

### 特殊命令

特殊命令复用当前模式的命令帧 ID，`data[0..6] = 0xFF`，`data[7]` 为：

| 值 | 命令 |
|---|---|
| 0xFB | ClearError |
| 0xFC | Enable |
| 0xFD | Disable |
| 0xFE | SaveZero |

---

## 4. 对外接口

### 4.1 通用抽象（`motor.hpp`）

- 能力位恒为
  `FeedbackPosition | FeedbackVelocity | FeedbackTorque | FeedbackTemperature`，
  **只有 MIT 模式**额外带 `CommandTorque`。
- 无 `CommandCurrent`、无 `FeedbackCurrent`；对 DM 调 `setCurrent`
  返回 `-ENOTSUP`。
- `setTorque(dev, nm)` 在 MIT 模式下等价于只填 `torque_ff_nm` 的 MIT 命令。

### 4.2 DM 专属（`include/drivers/motor/dm_motor.hpp`）

```cpp
enum class Model { J4310_2EC_V1_1 };
enum class ControlMode { Mit, PositionVelocity, Velocity };

struct Descriptor {
    Model model; const device *can; ControlMode mode;
    uint16_t motor_id, master_id, control_id;
    Limits limits; float torque_limit_nm;
};

int describe(const device *dev, Descriptor &out);
int setMitCommand(const device *dev, const MitCommand &command);
int setPositionVelocity(const device *dev, float position_rad, float velocity_limit_rad_s);
int setVelocity(const device *dev, float velocity_rad_s);
int readRawFeedback(const device *dev, RawFeedback &out);
int getDriveStatus(const device *dev, DriveStatus &out);
```

参数校验：

| 函数 | 校验 | 越界 |
|---|---|---|
| `setMitCommand` | `|torque_ff| ≤ torque_limit_nm` | `-ERANGE`（KP 0–500、KD 0–5 在编码层校验） |
| `setPositionVelocity` | `|position| ≤ PMAX`，`0 ≤ v_limit ≤ VMAX` | `-ERANGE` |
| `setVelocity` | `|v| ≤ VMAX` | `-ERANGE` |

以上函数都只在**已 arm 且反馈新鲜**时接受，否则 `-EACCES` / `-EHOSTDOWN`，
并且只**暂存**命令帧，真正发送由 Bus `flush()` 统一完成。

### 4.3 Bus（`include/drivers/motor/dm_bus.hpp`）

```cpp
enum class BusState { Uninitialized, Safe, Armed, Fault };

struct TxReport {
    int preparation_error, tx_error;
    uint16_t failed_motor_id;
    uint8_t frames_expected, frames_sent;
};

class Bus {
    int init(const device *can);
    int attach(const device *motor);
    int arm(TxReport &report);
    int flush(TxReport &report);
    int stop(TxReport &report);
    int recover(TxReport &report);
    int savePositionZero(const device *motor, TxReport &report);
    BusState state() const;
};
```

行为：

- `init` 启动 CAN；`attach` 按 `master-id` 注册反馈过滤器并 `claimMotor`
  （同一电机被第二个 Bus 抢占返回 `-EBUSY`）。
- `arm` 前会预检：反馈新鲜且状态为 Disabled 就发 `Enable`；
  已 Enabled 则直接 arm；其他状态返回 `-EHOSTDOWN`。
- `flush` 发送本周期暂存的命令帧；准备/发送失败会 `disableAll` 并进入 Fault。
- `stop` 发 `Disable`；`recover` 先 `ClearError` 再 `Disable`。
- `savePositionZero` 发 `SaveZero`，**只能在电机失能时有意调用**。

---

## 5. 样例支持代码（`samples/motor/dm_common/`）

`skywalker::samples::dm` 提供 MC02 专属支持：

```cpp
int enableMotorPower();                       // 打开 XT30_1
int prepare(Session &, const device *motor);  // 起 CAN + 过滤器
int arm(Session &);
int readSafeFeedback(...);                    // 使能前允许 Disabled 反馈
int flush(Session &);
int stop(Session &);
int stopAfterFailure(Session &, int original_error);
```

在 `dm_mc02` 上：**先启动 CAN 并注册过滤器，再开 XT30_1，等 1.5 s**
做反馈握手（`DmMotorBackend` 的 `power_on` 回调即指向 `enableMotorPower`）。
在 `rm_typec` 等无 XT30 的板上，这段代码不参与编译。

---

## 6. 错误与生命周期

| 返回 | 场景 |
|---|---|
| `-ENOTSUP` | 模式不匹配（如对 MIT 设备调 `setVelocity`）、设备非 DM |
| `-ERANGE` | 位置/速度/力矩超设备树量程 |
| `-EACCES` | 未 arm |
| `-EHOSTDOWN` | 反馈不新鲜或已故障 |
| `-ENODATA` | `readRawFeedback`/`getDriveStatus` 尚无反馈 |
| `-EBADMSG` / `-ENOENT` | 反馈帧格式错误 / motor-id 不匹配 |
| `-EBUSY` / `-EALREADY` | 电机已被其他 Bus 占用 / 重复 attach |

局限性：

- 未实现官方 **模式 4（混合控制）** 报文，故无对应样例。
- 连续位置解包假设固件在 **±PMAX 回绕**，且相邻反馈间运动 < PMAX；
  首次多圈测试必须实机确认。
- `torque-limit` 是软件上限；空载持续给力矩会因超速触发上层保护。

---

## 7. 相关文档

- [04 DJI 电机驱动](04-drivers-motor-dji.md)
- [09 统一速度/位置封装](09-motor-wrapper.md)
- 样例：`samples/motor/dm_mit_control/`、`dm_velocity_control/`、
  `dm_position_control/`、`dm_mit_velocity_control/`、`dm_mit_position_control/`
- 达妙手册与调试助手说明见 [docs/README.md](README.md) 的 PDF 列表
