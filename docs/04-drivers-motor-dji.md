# 04 DJI 电机驱动（C++）

`drivers/motor/dji/` 实现大疆 RoboMaster 系列 CAN 电机的统一驱动，覆盖
M3508-C620、M2006-C610、GM6020（电流环）三种 profile。驱动本身是 Zephyr
设备，对外同时提供 C 风格的 `struct device` 句柄和一组 C++ 助手。

---

## 1. 支持的型号与 profile

| 型号 | compatible | 反馈帧 | 命令帧（低/高） | 电流满幅 | 温度 | 位置传感器 |
|---|---|---|---|---|---|---|
| M3508-C620 | `dji,m3508-c620` | `0x200+id`（id 1–8） | `0x200` / `0x1FF` | 20 A | 有效 | 仅相对 |
| M2006-C610 | `dji,m2006-c610` | `0x200+id`（id 1–8） | `0x200` / `0x1FF` | 10 A | **无** | 仅相对 |
| GM6020（电流环） | `dji,gm6020-current` | `0x204+id`（id 1–7） | `0x1FE` / `0x2FE` | 3 A | 有效 | 固定零点单圈 |

命令槽位：`motor-id` 1–4 用低命令帧的 slot 0–3；5–8 用高命令帧的 slot 0–3。
`command_raw` 是 `int16`，最大幅值各型号不同（M3508/GM6020 为 16384，
M2006 为 10000）。

---

## 2. 设备树 binding

### 公共属性（`dji-motor-base.yaml`）

| 属性 | 类型 | 说明 |
|---|---|---|
| `can-bus` | phandle | CAN 控制器节点，如 `<&can1>` |
| `motor-id` | int | 电机 CAN ID |
| `current-limit-ma` | int | **软件电流钳位**（mA），驱动会校验不超过协议满幅 |
| `gear-ratio-num` / `gear-ratio-den` | int | 减速比分子/分母，用于把反馈换算到输出轴 |

### 各型号附加属性

- `dji,gm6020-current`：`motor-id` 限 1–7；`current-limit-ma` 上限 3000；
  **必须**声明 `current-loop-confirmed;`（布尔）证明固件已切到电流环；
  `encoder-zero-ticks`（0–8191）指定机械零位对应的编码器 tick。
- `dji,m2006-c610`：`motor-id` 限 1–8；`current-limit-ma` 上限 10000。

### 示例

```dts
/ {
    aliases { motor0 = &gm6020_7; };

    gm6020_7: motor-7 {
        compatible = "dji,gm6020-current";
        status = "okay";
        can-bus = <&can1>;
        motor-id = <4>;
        current-limit-ma = <1500>;   /* 台架限幅，实机请按负载调整 */
        gear-ratio-num = <1>;
        gear-ratio-den = <1>;
        encoder-zero-ticks = <0>;    /* 读 raw.encoder 后按机械零位填写 */
        current-loop-confirmed;
    };
};
```

> GM6020 电流环需要电机固件 **≥ 1.0.11.2**；缺少 `current-loop-confirmed`
> 时设备初始化会失败。

---

## 3. Kconfig

```conf
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y          # 默认 y
# 可选参数
CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS=20
CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS=10
CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS=12
CONFIG_SKYWALKER_DJI_MAX_BUSES=3
```

---

## 4. 对外接口

### 4.1 通用电机抽象（`include/drivers/motor/motor.hpp`）

```cpp
namespace skywalker::motor {

enum class State : uint8_t { Offline, Ready, Fault };

enum Capability : uint32_t {
    CommandCurrent          = 1u << 0,
    CommandTorque           = 1u << 1,   // DJI 不支持
    FeedbackPosition        = 1u << 8,
    FeedbackVelocity        = 1u << 9,
    FeedbackCurrent         = 1u << 10,
    FeedbackTorque          = 1u << 11,
    FeedbackTemperature     = 1u << 12,
    FeedbackAbsolutePosition= 1u << 13,
};

struct Feedback {
    float position_rad;            // 输出轴连续角，首帧为 0
    float absolute_position_rad;   // 固定零点单圈角 [-π, π)
    float velocity_rad_s;
    float current_a;
    float torque_nm;
    float temperature_c;
    uint32_t valid;                // 置位的能力位才有效
    uint64_t timestamp_ms;
};

// 内联助手：capabilities / setCurrent / setTorque / readFeedback / getState
} // namespace skywalker::motor
```

### 4.2 DJI 专属描述（`include/drivers/motor/dji_motor.hpp`）

```cpp
int describe(const device *dev, dji::Descriptor &out);
int readRawFeedback(const device *dev, dji::RawFeedback &out);
```

`Descriptor` 含 `model / can / motor_id / feedback_id / command_id /
command_slot / protocol_current_max_a / configured_current_limit_a /
gear_ratio / temperature_valid`，便于诊断 ID 映射。

### 4.3 Bus 生命周期（`include/drivers/motor/dji_bus.hpp`）

```cpp
class dji::Bus {
    int init(const device *can);
    int attach(const device *motor);
    int arm(FlushReport &report);
    int flush(FlushReport &report);
    int stop(FlushReport &report);
    int recover(FlushReport &report);
    BusState state() const;   // Uninitialized → Safe → Armed → Fault
};
```

推荐顺序：

```text
init(can) → attach(motor)…（可多个） → arm() → 周期 flush() → stop()/recover()
```

- `arm()` 前不接受控制输出；`flush()` 按组把缓存的命令整帧发出，失败自动清零并进入 `Fault`。
- `stop()` 发送组内零电流，**不代表机械制动/断电**。
- 多个电机在**同一 CAN** 时，务必用**同一个 Bus** 统一 flush；
  分别给每个电机建 Bus 会因覆盖同一命令帧的其它 slot 而互相打架。

---

## 5. CAN 协议细节

### 反馈帧（8 字节，大端）

| 字节 | 含义 |
|---|---|
| 0–1 | 编码器值 `uint16` |
| 2–3 | 转速 `int16`（RPM） |
| 4–5 | 实际电流 `int16`（原始值） |
| 6 | 温度 `uint8`（M2006 无效） |
| 7 | 保留 |

`decodeFeedback()` 会拒绝 DLC≠8 或带 IDE/RTR/FDF 标志的帧。

### 命令帧（8 字节，大端，4×int16）

| 字节 | slot 0 | slot 1 | slot 2 | slot 3 |
|---|---|---|---|---|
| 0–1 | 命令 0 | 命令 1 | 命令 2 | 命令 3 |
| 2–3 | … | | | |
| 4–5 | | | | |
| 6–7 | | | | |

### 反馈换算（`rxCallback`）

- 编码器按 **8192 tick/圈** 连续展开：`delta = 本次 − 上次`；
  `|delta| > 4096` 时补 ±8192，得到累计 tick。
- 输出轴角度：`rad = total_ticks × (2π / 8192) / gear_ratio`。
- 输出轴角速度：`rad_s = rpm × (2π / 60) / gear_ratio`。
- **反馈已换算到输出轴，不要再除一次减速比。**
- GM6020 在 `gear_ratio == 1` 时额外给出 `absolute_position_rad`，
  由 `encoder-zero-ticks` 决定零点。

### 能力位（`getCapabilities`）

- 恒有：`CommandCurrent | FeedbackPosition | FeedbackVelocity | FeedbackCurrent`
- `temperature_valid` 时增加 `FeedbackTemperature`（M2006 无）
- GM6020 且 `gear-ratio-num == gear-ratio-den` 时增加
  `FeedbackAbsolutePosition`
- **永不含 `CommandTorque`**：`setTorque` 返回 `-ENOTSUP`，DJI 只有电流模式。

---

## 6. 错误与状态语义

`setCurrent()` 的返回：

| 返回 | 触发条件 |
|---|---|
| `-EINVAL` | 空指针 / 电流非有限值 |
| `-ERANGE` | `|current_a| > current-limit-ma`（软件钳位） |
| `-EHOSTDOWN` | 反馈不新鲜（超 `FEEDBACK_TIMEOUT_MS`）或从未收到 |
| `-EACCES` | 未 arm，或已锁定 Fault |
| `-ENOTSUP` | 设备不支持该命令（如对 DJI 调 `setTorque`） |

- `State::Fault` 是锁存的，需 `Bus::recover()`（或底层 `clearMotorFault`）
  才能清除。
- `current-limit-ma` 在设备初始化时校验：超过协议满幅直接 `-ERANGE`，
  驱动不会静默裁剪错误配置。

---

## 7. 直接使用（不用统一封装）

```cpp
#include <drivers/motor/motor.hpp>
#include <drivers/motor/dji_bus.hpp>

static const device *motor = DEVICE_DT_GET(DT_ALIAS(motor0));

motor::dji::Bus bus;
motor::dji::FlushReport report;

bus.init(DEVICE_DT_GET(DT_NODELABEL(can1)));
bus.attach(motor);
bus.arm(report);

for (;;) {
    k_sleep(K_MSEC(5));
    motor::setCurrent(motor, 0.2f);      // A
    bus.flush(report);
}

bus.stop(report);
```

更推荐直接用统一封装（[09](09-motor-wrapper.md)）：它内置反馈检查、dt 校验、
超速/超温保护与故障停机。

---

## 8. 常见坑

1. **多电机同总线必须共享 Bus**，否则命令帧互相覆盖。
2. GM6020 未声明 `current-loop-confirmed` 或固件过旧会初始化失败。
3. M2006 没有温度反馈；若上层开启温度保护，会因缺能力无法启动。
4. 反馈 `position_rad` 首帧为 0，是**相对量**；要固定零点用 GM6020 的
   `absolute_position_rad`。
5. `stop()` 只清命令，不等于断动力；调试必须能物理断电。

---

## 9. 相关文档

- [05 达妙 DM 电机驱动](05-drivers-motor-dm.md)
- [09 统一速度/位置封装](09-motor-wrapper.md)
- 样例：`samples/motor/dji_unified/`、`dji_speed_control/`、
  `dji_position_control/`、`m2006_speed_control/`
- 大疆手册见 [docs/README.md](README.md) 的 PDF 列表
