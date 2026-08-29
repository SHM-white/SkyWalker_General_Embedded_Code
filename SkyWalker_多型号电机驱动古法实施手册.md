# SkyWalker 多型号电机驱动古法实施手册

> 目标：在现有 M3508/C620 驱动基础上，由你亲手增加 M2006/C610、GM6020 和 DM-J4310 驱动。
>
> 本文只提供设计、接口、示例和验证步骤，没有修改任何业务源码。

## 0. 已核查的本地文档与型号

本文已经直接核查仓库 `motor_docs/` 下的手册，不再只根据型号简称猜测：

- “2006”指 DJI RoboMaster M2006 P36 电机与 C610 电调。
- “6020”指 DJI RoboMaster GM6020 一体化电机。
- “4310”确定为达妙 `DM-J4310-2EC V1.1`，使用达妙 MIT/位置速度/速度协议。

补充核查：仓库内的 GM6020 PDF 是较早版本，只描述电压控制。DJI GM6020 v1.4 手册还提供电流控制模式。本项目按实际使用习惯，默认实现电流控制；电压控制只作为可选兼容模式。

本地 DM-J4310-2EC V1.1 手册还确认了：

- Classic CAN 标准帧，固定 1 Mbps。
- 额定电压 24 V。
- 减速比 10:1。
- 双磁编码器，输出轴单圈绝对位置，上电后不需要像 DJI 增量编码器那样从零累计。
- 额定力矩 3 N·m，峰值力矩 7 N·m。
- 额定转速 120 rpm，空载最高转速 200 rpm。

注意：MIT 的 `T_MAX` 是“协议线性映射量程”，峰值 7 N·m 是“该型号硬件规格”。它们不是一回事。

## 1. 一句话结论

不要把 `dji_m3508.cpp` 复制三份后只改名字。

正确拆法是：

```text
应用层
  │
  ├── 统一 motor API：enable / disable / setTorque / readFeedback / getState
  │
  ├── DJI 协议族
  │     ├── 公共 8 字节反馈解码
  │     ├── 公共 4 槽位分组发送器
  │     ├── M3508/C620：原始电流范围 ±16384
  │     ├── M2006/C610：原始电流范围 ±10000
  │     └── GM6020：默认原始电流范围 ±16384；可选原始电压范围 ±30000
  │
  └── 达妙协议族
        ├── MIT 浮点/整数映射与打包
        ├── 使能、失能、设零特殊帧
        └── DM-J4310 Zephyr device
```

M2006 和 GM6020 可以复用 DJI 的“反馈解析骨架”和“分组发送骨架”；DM-J4310 只能复用最上面的统一 `motor::Api`、CAN 发送方法、锁和心跳判断，不能复用 DJI 的协议与 `TxGroup`。

## 2. 协议差异总表

| 型号              | 控制方式           | 控制 CAN ID                            | 反馈 CAN ID         |                                                                                      命令原始范围 | 反馈温度                          | 默认传动比 |
| ----------------- | ------------------ | -------------------------------------- | ------------------- | ------------------------------------------------------------------------------------------------: | --------------------------------- | ---------: |
| M3508 + C620      | 4 台一组，转矩电流 | ID 1~4 用 `0x200`；5~8 用 `0x1FF` | `0x200 + 电调 ID` |                                                                                 `-16384..16384` | `DATA[6]`                       |         19 |
| M2006 + C610      | 4 台一组，转矩电流 | ID 1~4 用 `0x200`；5~8 用 `0x1FF` | `0x200 + 电调 ID` |                                                           `-10000..10000`，对应约 `-10A..10A` | 官方协议中`DATA[6..7]` 为 Null  |         36 |
| GM6020            | 4 台一组，默认转矩电流给定 | 电流：ID 1~4 用 `0x1FE`、5~7 用 `0x2FE`；可选电压：`0x1FF`/`0x2FF` | `0x204 + 电机 ID` | 电流 `-16384..16384`，约对应 `-3A..3A`；电压 `-30000..30000` | `DATA[6]` | 1 |
| DM-J4310-2EC V1.1 | 单电机帧，先做 MIT | 命令发到电机 CAN ID                    | 由 Master ID 决定   | P/V/T 映射量程由调试助手配置；官方 SDK 的普通 DM4310 profile 为 ±12.5 rad、±30 rad/s、±10 N·m | `DATA[6]` MOS、`DATA[7]` 转子 |       10:1 |

### 2.1 一个很危险的 CAN ID 冲突

DJI 和达妙电机不要因为“都是 1 Mbps CAN”就随便挂在同一条总线上。

- DJI 的 `0x201..0x208` 是 C610/C620 反馈。
- 达妙速度模式使用 `0x200 + motor_id`，例如 1 号电机命令也是 `0x201`。
- C610/C620 的 `0x1FF` 控制 ID 5~8。
- GM6020 电流模式的 `0x1FE` 控制 ID 1~4；电压模式才使用 `0x1FF`。
- GM6020 的 `0x205..0x20B` 反馈又会和一部分 C610/C620 反馈 ID 重叠。

最坏情况不是“收不到数据”，而是别的设备发出的反馈被另一台电机当成控制命令。

第一版实现请遵守：

1. DJI 电机放一条 CAN。
2. DM-J4310 放另一条 CAN。
3. 如果板上只有一条 CAN，先只接一种协议族。
4. 真要混接时，必须先做完整的“发送 ID、接收 ID、槽位”冲突表；只检查反馈 ID 不够。

## 3. 当前项目中能复用什么

下面的行号基于当前提交 `ce4e2d1`。

### 3.1 可以复用

- `include/drivers/motor/motor.hpp`
  - `State`、`FeedbackValid`、`Feedback`、`Api` 是所有型号的公共上层接口。
- `drivers/motor/dji/dji_protocol.cpp`
  - `readBe16()` 和大端 16 位解析逻辑可直接抽成 DJI 公共编解码。
  - `buildGroupCurrentFrame()` 的 4 槽位打包循环可以复用，但函数名应该改成不限定“电流”的名字。
- `drivers/motor/dji/dji_m3508.cpp`
  - `Config/Data` 分离。
  - `can_add_rx_filter()` 注册单电机反馈。
  - RX 回调里解析后一次性加锁提交。
  - `last_rx_ms` 心跳判定。
  - `DEVICE_DT_INST_DEFINE` 与 `DT_INST_FOREACH_STATUS_OKAY`。
- `drivers/motor/dji/dji_tx_group.cpp`
  - 先收集四个槽位，再一次 `can_send()` 的思路是正确的。
- `dts/bindings/motor/dji,m3508.yaml`
  - `can-bus`、`feedback-id`、`command-id`、`command-slot`、`gear-ratio` 的配置形状可作为第一版模板。
- `samples/motor/dji_m3508_driver`
  - “设备树实例 → 获取 device → 初始化 TxGroup → 周期发送 → 打印反馈”的完整调用链可以复用。

### 3.2 不能照搬

- M2006 的命令上限不是 16384，而是 10000。
- M2006 的 `DATA[6]` 不是温度，不能设置 `FeedbackTemperature`。
- GM6020 必须先区分电流模式与电压模式。本项目默认电流模式，接口叫 `setCurrentRaw()`；只有明确启用电压模式时才使用 `setVoltageRaw()`。
- GM6020 的反馈 ID 和命令 ID 规则与 C610/C620 不同。
- DM-J4310 不是四台共用一个 8 字节命令帧。
- DM-J4310 有真正的使能、失能、设零帧；DJI 的“enable”基本只是软件状态。
- 不要把未经换算的 raw current 直接填到 `torque_nm`。

## 4. 先修当前公共 API 的三个小坑

在增加新型号前，先由你修改 `include/drivers/motor/motor.hpp`。

当前 `disable()`、`readFeedback()` 和 `getState()` 都错误地检查了 `api->enable`。它们应该各自检查即将调用的函数指针。

你需要改成：

```cpp
inline int disable(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) {
        return -EINVAL;
    }

    const auto *api = static_cast<const Api *>(dev->api);
    if (api->disable == nullptr) {
        return -ENOSYS;
    }
    return api->disable(dev);
}

inline int readFeedback(const struct device *dev, Feedback &out)
{
    if (dev == nullptr || dev->api == nullptr) {
        return -EINVAL;
    }

    const auto *api = static_cast<const Api *>(dev->api);
    if (api->read_feedback == nullptr) {
        return -ENOSYS;
    }
    return api->read_feedback(dev, &out);
}

inline State getState(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) {
        return State::Offline;
    }

    const auto *api = static_cast<const Api *>(dev->api);
    if (api->get_state == nullptr) {
        return State::Offline;
    }
    return api->get_state(dev);
}
```

### 自检

- 临时把一个具体驱动的 `read_feedback` 设成 `nullptr` 时，`readFeedback()` 应返回 `-ENOSYS`，不能崩溃。
- 这个修改不改变 API 布局，不要求改已有驱动初始化器。

## 5. 文件布局

建议最终得到：

```text
include/drivers/motor/
├── motor.hpp
├── dji_protocol.hpp
├── dji_tx_group.hpp
├── dji_m3508.hpp
├── dji_m2006.hpp
├── dji_gm6020.hpp
├── dm_protocol.hpp
└── dm_j4310.hpp

drivers/motor/
├── Kconfig
├── CMakeLists.txt
├── dji/
│   ├── CMakeLists.txt
│   ├── dji_protocol.cpp
│   ├── dji_tx_group.cpp
│   ├── dji_m3508.cpp
│   ├── dji_m2006.cpp
│   └── dji_gm6020.cpp
└── dm/
    ├── CMakeLists.txt
    ├── dm_protocol.cpp
    └── dm_j4310.cpp

dts/bindings/motor/
├── dji,m3508.yaml
├── dji,m2006.yaml
├── dji,gm6020.yaml
└── damiao,dm-j4310.yaml

samples/motor/
├── dji_m2006_driver/
├── dji_gm6020_driver/
└── dm_j4310_driver/
```

先不要实现 `motor_can_router.cpp`。当前 Zephyr CAN filter 已经能把反馈帧分发到各 device；在没有测出 filter 数量不够之前，加“万能路由器”只会多一层状态和并发问题。

## 6. 把 DJI 协议层改成真正的公共层

### 6.1 重命名公共反馈

当前 `M3508Feedback` 的四个字段其实是 DJI 多款电机共用的字节形状。建议在 `dji_protocol.hpp` 中增加：

```cpp
struct DjiFeedbackRaw
{
    std::uint16_t encoder = 0;
    std::int16_t rpm = 0;
    std::int16_t effort_raw = 0;
    std::uint8_t temperature = 0;
};

bool decodeFeedback(
    const struct can_frame &frame,
    DjiFeedbackRaw &out);

void buildGroupCommandFrame(
    struct can_frame &frame,
    std::uint16_t command_id,
    const std::int16_t command[4]);
```

为什么叫 `effort_raw`：

- C610/C620 把它描述为实际输出转矩/转矩电流。
- GM6020 把它描述为实际转矩电流。
- 在没有可靠比例系数前，它不是 `N·m`。

为什么叫 `buildGroupCommandFrame()`：

- M3508/M2006 的四个槽位是电流。
- GM6020 的四个槽位是电压给定。
- 打包算法完全相同，物理含义不同。

### 6.2 保持 M3508 暂时兼容

不要在同一个提交里同时重写 M3508 全部调用方。可以先提供别名/薄包装：

```cpp
using M3508Feedback = DjiFeedbackRaw;

inline bool decodeM3508Feedback(
    const struct can_frame &frame,
    M3508Feedback &out)
{
    return decodeFeedback(frame, out);
}

inline void buildGroupCurrentFrame(
    struct can_frame &frame,
    std::uint16_t command_id,
    const std::int16_t current[4])
{
    buildGroupCommandFrame(frame, command_id, current);
}
```

确认 M3508 样例仍能编译后，再逐步把旧名字删掉。

### 6.3 协议函数必须是纯函数

`decodeFeedback()` 只做：

1. 检查 `frame.dlc == 8`。
2. 按大端读取 `DATA[0..5]`。
3. 保存 `DATA[6]` 到 raw 结构。
4. 返回成功/失败。

它不应该：

- 读设备树。
- 获取系统时间。
- 加锁。
- 做齿轮比换算。
- 判断某个型号有没有温度。

型号语义放到对应的 `dji_mxxxx.cpp`。

## 7. 把 TxGroup 从 M3508 专用改成 DJI 通用

当前 `TxGroup::bindMotor()` 和 `send()` 硬编码调用 `m3508::getCommandInfo()` 与 `m3508::getCurrentRaw()`。这是增加 M2006 时最先撞到的墙。

### 7.1 推荐的适配器接口

在 `dji_tx_group.hpp` 中定义一个小适配器：

```cpp
using ReadRawCommand =
    int (*)(const struct device *dev, std::int16_t &out);

using WriteRawCommand =
    int (*)(const struct device *dev, std::int16_t value);

struct GroupCommandSource
{
    const struct device *motor = nullptr;
    const struct device *can = nullptr;
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
    ReadRawCommand read_raw = nullptr;
    WriteRawCommand write_raw = nullptr;
};
```

每个具体型号实现一个 `makeGroupCommandSource()`：

```cpp
namespace skywalker::motor::dji::m2006
{
int makeGroupCommandSource(
    const struct device *dev,
    GroupCommandSource &out);
}
```

M2006 的实现只是把自己的 `can`、`command_id`、`command_slot`、`getCurrentRaw` 和 `setCurrentRaw` 填进去。GM6020 填 `getVoltageRaw` 和 `setVoltageRaw`。

这比让 `TxGroup` 判断 device 到底是哪种电机安全得多。

### 7.2 TxGroup 的行为契约

建议接口：

```cpp
class TxGroup
{
public:
    int init(
        const struct device *can,
        std::uint16_t command_id);

    int bind(const GroupCommandSource &source);
    int send();
    int clear();
    int clearAndSend();

private:
    const struct device *can_ = nullptr;
    std::uint16_t command_id_ = 0;
    GroupCommandSource sources_[4]{};
};
```

每个函数必须有以下行为：

#### `init()`

- `can == nullptr`：返回 `-EINVAL`。
- CAN device 未 ready：返回 `-ENODEV`。
- 保存 CAN 和 command ID。
- 清空四个槽位。
- 不自动发送。

#### `bind()`

- motor、can、read/write 任一为空：`-EINVAL`。
- source CAN 与 group CAN 不同：`-EXDEV` 或 `-EINVAL`，二选一并统一。
- command ID 不同：`-EINVAL`。
- slot 大于 3：`-ERANGE`。
- 槽位已被其他 motor 占用：`-EADDRINUSE`，不要静默覆盖。
- 成功才保存 source。

#### `send()`

- 先把四个 raw 值读入局部数组。
- 任意 getter 失败时，不发送半新半旧的帧，直接返回错误。
- 空槽位填 0。
- 调 `buildGroupCommandFrame()`。
- 调 `can_send()`。

#### `clear()`

- 对所有已绑定 source 调 `write_raw(motor, 0)`。
- 返回第一个错误。
- 只清软件缓存，不代表电调已经收到 0。

#### `clearAndSend()`

- 先 `clear()`。
- 再 `send()`。
- 这是 DJI 协议族真正的紧急停机入口。

### 7.3 为什么 DJI 的 `disable()` 不该假装成功

DJI 分组帧一次控制四个电调。单个 motor device 不掌握同组其他三个槽位，所以不能安全地自己发一整帧。

建议：

- M3508/M2006/GM6020 的 `enable()` 返回 `-ENOSYS`，或者明确只代表软件允许状态。
- `disable()` 至少把本机 raw 缓存置 0，但不要宣传成“硬件已经停止”。
- 真正停机由拥有整个分组的 `TxGroup::clearAndSend()` 完成。

## 8. M2006/C610：第一台应该新增的型号

先写 M2006，因为它和 M3508 最接近，最容易验证公共抽取是否正确。

### 8.1 新增头文件接口

文件：`include/drivers/motor/dji_m2006.hpp`

```cpp
#pragma once

#include <cstdint>
#include <zephyr/device.h>

#include <drivers/motor/dji_protocol.hpp>
#include <drivers/motor/dji_tx_group.hpp>

namespace skywalker::motor::dji::m2006
{
int setCurrentRaw(
    const struct device *dev,
    std::int16_t current_raw);

int getCurrentRaw(
    const struct device *dev,
    std::int16_t &out);

int readRawFeedback(
    const struct device *dev,
    DjiFeedbackRaw &out);

int makeGroupCommandSource(
    const struct device *dev,
    GroupCommandSource &out);
}
```

### 8.2 M2006 Config

第一版沿用 M3508 的设备树形状，减少同时修改的文件：

```cpp
struct M2006Config
{
    const struct device *can;
    std::uint16_t feedback_id;
    std::uint16_t command_id;
    std::uint8_t command_slot;
    float gear_ratio;
};
```

初始化时增加一致性检查：

- feedback ID 必须在 `0x201..0x208`。
- command ID 只能是 `0x200` 或 `0x1FF`。
- slot 必须小于 4。
- gear ratio 必须大于 0。
- 如果能从 feedback ID 算出电调 ID，检查 command ID 和 slot 是否匹配：
  - ID 1~4：command `0x200`，slot `ID - 1`。
  - ID 5~8：command `0x1FF`，slot `ID - 5`。

这样设备树写错时在启动阶段报错，而不是让另一台电机误转。

### 8.3 M2006 Data

```cpp
struct M2006Data
{
    skywalker::motor::Feedback feedback{};
    DjiFeedbackRaw raw_feedback{};

    std::int16_t command_raw = 0;
    std::uint64_t last_rx_ms = 0;

    std::uint16_t last_encoder = 0;
    std::int64_t total_encoder_ticks = 0;
    bool has_encoder = false;

    struct k_spinlock lock{};
    int rx_filter_id = -1;
};
```

新增三个编码器字段是为了得到相对多圈位置。只用单帧 `encoder / 8192` 无法知道 M2006 输出轴转了多少圈。

### 8.4 RX 回调应按这个顺序

1. 检查 `dev` 和 `frame`。
2. 调 `decodeFeedback()` 得到局部 raw。
3. 获取当前时间。
4. 加锁。
5. 第一次收到编码器时只记录基准，`total_encoder_ticks = 0`。
6. 后续计算差值并处理 8192 回绕。
7. 更新 raw、统一 feedback、时间戳。
8. 解锁。

回绕算法：

```cpp
std::int32_t delta =
    static_cast<std::int32_t>(raw.encoder) -
    static_cast<std::int32_t>(data->last_encoder);

if (delta > 4096) {
    delta -= 8192;
} else if (delta < -4096) {
    delta += 8192;
}

data->total_encoder_ticks += delta;
data->last_encoder = raw.encoder;
```

物理量换算：

```cpp
feedback.position_rad =
    static_cast<float>(data->total_encoder_ticks) *
    (TWO_PI / 8192.0f) /
    cfg->gear_ratio;

feedback.velocity_rad_s =
    static_cast<float>(raw.rpm) *
    (TWO_PI / 60.0f) /
    cfg->gear_ratio;
```

有效位：

- 位置成功累计后设置 `FeedbackPosition`。
- gear ratio 合法时设置 `FeedbackVelocity`。
- 不设置 `FeedbackTemperature`，因为 C610 官方报文的 `DATA[6..7]` 是 Null。
- 不设置 `FeedbackTorque`，除非你后来拿到并验证 raw 到 N·m 的换算。

### 8.5 setCurrentRaw()

行为必须与 M3508 的 16384 限幅不同：

```cpp
if (current_raw < -10000 || current_raw > 10000) {
    return -ERANGE;
}
```

不要自动把 12000 截成 10000。对驱动底层而言，返回 `-ERANGE` 更容易发现上层控制器参数写错。

### 8.6 设备树 binding

文件：`dts/bindings/motor/dji,m2006.yaml`

```yaml
description: DJI M2006 motor with C610 CAN ESC

compatible: "dji,m2006"

include: base.yaml

properties:
  can-bus:
    type: phandle
    required: true

  feedback-id:
    type: int
    required: true

  command-id:
    type: int
    required: true

  command-slot:
    type: int
    required: true

  gear-ratio:
    type: int
    default: 36
```

### 8.7 overlay 示例

```dts
/ {
    aliases {
        motor0 = &m2006_1;
    };

    m2006_1: motor-1 {
        compatible = "dji,m2006";
        status = "okay";
        can-bus = <&can1>;
        feedback-id = <0x201>;
        command-id = <0x200>;
        command-slot = <0>;
        gear-ratio = <36>;
    };
};
```

### 8.8 M2006 最小验证顺序

1. 只编译，不接电机。
2. 接 CAN 与电调，架空输出轴，不发送非零命令。
3. 确认收到 `0x201`、DLC 8、约 1 kHz 的反馈。
4. 用手慢慢转输出轴，确认 encoder 变化。
5. 周期发送全 0 的 `0x200`。
6. 只给 `100..300` 的小 raw 值，确认方向。
7. 立即回 0。
8. 最后再做速度 PID；raw 电流不是目标转速。

## 9. GM6020：复用框架，但物理语义不同

### 9.1 公共接口

文件：`include/drivers/motor/dji_gm6020.hpp`

```cpp
namespace skywalker::motor::dji::gm6020
{
int setVoltageRaw(
    const struct device *dev,
    std::int16_t voltage_raw);

int getVoltageRaw(
    const struct device *dev,
    std::int16_t &out);

int readRawFeedback(
    const struct device *dev,
    DjiFeedbackRaw &out);

int makeGroupCommandSource(
    const struct device *dev,
    GroupCommandSource &out);
}
```

不要为了统一名字把它叫 `setCurrentRaw()`。名字写错后，上层很容易套用 M3508/C620 的电流常量。

### 9.2 Config/Data

Config 可与 M2006 同形：

```cpp
struct Gm6020Config
{
    const struct device *can;
    std::uint16_t feedback_id;
    std::uint16_t command_id;
    std::uint8_t command_slot;
};
```

Data 可复用：

- 公共 `Feedback`。
- `DjiFeedbackRaw`。
- `voltage_raw`。
- `last_rx_ms`。
- 多圈 encoder 字段。
- spinlock。
- filter ID。

GM6020 是直驱，位置和速度不用除以减速比。

### 9.3 ID 规则

初始化时严格检查：

| 电机 ID | feedback ID | command ID | slot |
| ------: | ----------: | ---------: | ---: |
|       1 |   `0x205` |  `0x1FF` |    0 |
|       2 |   `0x206` |  `0x1FF` |    1 |
|       3 |   `0x207` |  `0x1FF` |    2 |
|       4 |   `0x208` |  `0x1FF` |    3 |
|       5 |   `0x209` |  `0x2FF` |    0 |
|       6 |   `0x20A` |  `0x2FF` |    1 |
|       7 |   `0x20B` |  `0x2FF` |    2 |

`0x2FF` 的第 4 个槽位必须保持 0。

### 9.4 RX 与统一反馈

反馈布局仍是：

- `DATA[0..1]`：0~8191 机械角度。
- `DATA[2..3]`：有符号 rpm。
- `DATA[4..5]`：实际转矩电流 raw。
- `DATA[6]`：温度。
- `DATA[7]`：Null。

因此：

- 设置 position、velocity、temperature 的 valid。
- 暂时不设置 torque valid。
- error/过温状态目前不能仅凭此反馈帧完整判断；心跳在线时先返回 Ready。

### 9.5 setVoltageRaw()

```cpp
if (voltage_raw < -30000 || voltage_raw > 30000) {
    return -ERANGE;
}
```

`motor::setTorque()` 第一版返回 `-ENOSYS`。电压给定不是力矩给定；想提供 N·m 接口，需要你在上层实现闭环并决定电流/温度/电压保护。

### 9.6 binding 与 overlay

`dts/bindings/motor/dji,gm6020.yaml` 可沿用 `can-bus`、`feedback-id`、`command-id`、`command-slot`，不需要 `gear-ratio`。

```dts
gm6020_1: motor-1 {
    compatible = "dji,gm6020";
    status = "okay";
    can-bus = <&can1>;
    feedback-id = <0x205>;
    command-id = <0x1FF>;
    command-slot = <0>;
};
```

### 9.7 GM6020 最小验证顺序

1. 拨码确认 ID 和终端电阻。
2. 只接收 `0x204 + ID`，不发送非零值。
3. 周期发送全 0 分组。
4. 先试 `100..300`，不要一上来试 30000。
5. 架空机构并准备物理断电。
6. 验证正负方向。
7. 验证停止时调用 `clearAndSend()`，总线上确实出现全 0。

## 10. DM-J4310-2EC V1.1：另起达妙协议族

本地手册明确规定标准 CAN 帧、固定 1 Mbps。第一版只做 MIT 模式；位置速度模式、速度模式和参数读写等 MIT 跑通后再加。

### 10.1 上位机先配置

在写代码前，用达妙调试工具确认并记录：

- 电机显示为 DM-J4310-2EC V1.1。
- CAN 为 Classic CAN 标准帧、1 Mbps。
- Motor/Slave ID。
- Master ID，也就是反馈报文 CAN ID。
- 控制模式为 MIT。
- P_MAX、V_MAX、T_MAX 与当前固件参数。

本地 V1.1 手册说明 P_MAX、V_MAX、T_MAX 可以由调试助手设置，但没有在协议页给出固定默认值。达妙官方 SDK 的普通 DM4310 profile 使用：

```cpp
constexpr float P_MAX = 12.5f;
constexpr float V_MAX = 30.0f;
constexpr float T_MAX = 10.0f;
constexpr float KP_MAX = 500.0f;
constexpr float KD_MAX = 5.0f;
```

把上面三个值理解成“当前建议的软件 profile”，不要理解成手册保证的不可变常量。如果调试助手显示不同值，以电机实际参数为准。编解码两端量程不一致会导致命令和反馈比例全部错误。

另外增加硬件安全常量：

```cpp
constexpr float RATED_TORQUE_NM = 3.0f;
constexpr float PEAK_TORQUE_NM = 7.0f;
constexpr float GEAR_RATIO = 10.0f;
```

协议可以用 ±10 N·m 做量化映射，不代表允许对硬件持续或瞬时命令 10 N·m。第一版 `setTorque()` 的安全限幅应取：

```text
min(调试助手中的 T_MAX, 7 N·m, 你为机构设定的更低上限)
```

初次上电测试还应远低于这个上限。

### 10.2 协议头文件

文件：`include/drivers/motor/dm_protocol.hpp`

```cpp
#pragma once

#include <cstdint>
#include <zephyr/drivers/can.h>

namespace skywalker::motor::dm
{
struct Limits
{
    float position_max_rad;
    float velocity_max_rad_s;
    float torque_max_nm;
};

struct SafetyLimits
{
    float command_torque_max_nm;
};

struct MitCommand
{
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque_nm = 0.0f;
};

struct FeedbackRaw
{
    std::uint8_t motor_id = 0;
    std::uint8_t error = 0;
    std::uint16_t position = 0;
    std::uint16_t velocity = 0;
    std::uint16_t torque = 0;
    std::uint8_t mos_temperature = 0;
    std::uint8_t rotor_temperature = 0;
};

struct FeedbackDecoded
{
    FeedbackRaw raw{};
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float torque_nm = 0.0f;
};

int buildMitCommandFrame(
    struct can_frame &frame,
    std::uint16_t motor_id,
    const Limits &limits,
    const MitCommand &command);

void buildEnableFrame(
    struct can_frame &frame,
    std::uint16_t motor_id);

void buildDisableFrame(
    struct can_frame &frame,
    std::uint16_t motor_id);

void buildSetZeroFrame(
    struct can_frame &frame,
    std::uint16_t motor_id);

bool decodeFeedback(
    const struct can_frame &frame,
    const Limits &limits,
    FeedbackDecoded &out);
}
```

### 10.3 浮点和无符号整数映射

实现两个内部纯函数：

```cpp
static std::uint32_t floatToUint(
    float value,
    float min_value,
    float max_value,
    std::uint8_t bits);

static float uintToFloat(
    std::uint32_t value,
    float min_value,
    float max_value,
    std::uint8_t bits);
```

公式：

```text
raw = (value - min) / (max - min) * ((1 << bits) - 1)

value = raw * (max - min) / ((1 << bits) - 1) + min
```

公共 `buildMitCommandFrame()` 在调用映射前检查范围，越界返回 `-ERANGE`。内部 helper 可以再做一次 clamp 防止浮点误差，但不能靠静默 clamp 掩盖上层错误。

`Limits::torque_max_nm` 决定 12 bit raw 如何解释；`SafetyLimits::command_torque_max_nm` 决定这个具体硬件/机构允许发多大的命令。不要用一个字段同时承担“协议量化”和“硬件保护”。

### 10.4 MIT 命令打包

先转换：

- position：16 bit，`-P_MAX..P_MAX`。
- velocity：12 bit，`-V_MAX..V_MAX`。
- kp：12 bit，`0..500`。
- kd：12 bit，`0..5`。
- torque：12 bit，`-T_MAX..T_MAX`。

再打包：

```cpp
frame.id = motor_id;
frame.flags = 0;
frame.dlc = 8;

frame.data[0] = (position_raw >> 8) & 0xFF;
frame.data[1] = position_raw & 0xFF;
frame.data[2] = (velocity_raw >> 4) & 0xFF;
frame.data[3] =
    ((velocity_raw & 0x0F) << 4) |
    ((kp_raw >> 8) & 0x0F);
frame.data[4] = kp_raw & 0xFF;
frame.data[5] = (kd_raw >> 4) & 0xFF;
frame.data[6] =
    ((kd_raw & 0x0F) << 4) |
    ((torque_raw >> 8) & 0x0F);
frame.data[7] = torque_raw & 0xFF;
```

不要把 12 bit 字段存进 `std::uint8_t`，否则高 4 bit 会在打包前丢失。

### 10.5 特殊命令

下面的 `FC/FD/FE` 来自达妙官方 SDK/例程；你放入的 V1.1 产品手册协议页没有列出这三帧。因此实现后必须先在当前固件上用调试助手或抓包确认，不能只凭产品手册宣称所有固件都支持。

使能：

```text
CAN ID = motor_id
DATA = FF FF FF FF FF FF FF FC
```

失能：

```text
CAN ID = motor_id
DATA = FF FF FF FF FF FF FF FD
```

设当前位置为零：

```text
CAN ID = motor_id
DATA = FF FF FF FF FF FF FF FE
```

三者都是标准帧、DLC 8。

不要在 `init()` 中自动使能或设零：

- init 发生在系统启动期，机构可能还未准备好。
- 设零会改变持久/运行语义。
- 使能后错误的首帧可能让关节突然运动。

### 10.6 反馈解码

MIT 反馈布局：

```cpp
out.raw.error = (frame.data[0] >> 4) & 0x0F;
out.raw.motor_id = frame.data[0] & 0x0F;

out.raw.position =
    (static_cast<std::uint16_t>(frame.data[1]) << 8) |
    frame.data[2];

out.raw.velocity =
    (static_cast<std::uint16_t>(frame.data[3]) << 4) |
    (frame.data[4] >> 4);

out.raw.torque =
    (static_cast<std::uint16_t>(frame.data[4] & 0x0F) << 8) |
    frame.data[5];

out.raw.mos_temperature = frame.data[6];
out.raw.rotor_temperature = frame.data[7];
```

然后按对应 Limits 做 `uintToFloat()`。

驱动 RX filter 应匹配配置的 Master ID，而不是 motor ID。回调中还应检查 `raw.motor_id` 是否与期望 motor ID 的低 4 bit 一致。

本地手册明确列出的故障码是：

| 高 4 bit | 含义         |
| -------: | ------------ |
|  `0x8` | 过压         |
|  `0x9` | 欠压         |
|  `0xA` | 过流         |
|  `0xB` | MOS 过温     |
|  `0xC` | 电机线圈过温 |
|  `0xD` | 通信丢失     |
|  `0xE` | 过载         |

不要把“高 4 bit 非 0”一律当故障。V1.1 手册只把 `0x8..0xE` 定义为故障；`0x0..0x7` 的其他状态不要自行命名，先作为原始状态码保留。

### 10.7 DM-J4310 对外接口

文件：`include/drivers/motor/dm_j4310.hpp`

```cpp
namespace skywalker::motor::dm::j4310
{
int setMitCommand(
    const struct device *dev,
    const MitCommand &command);

int setZeroPosition(
    const struct device *dev);

int readDecodedFeedback(
    const struct device *dev,
    FeedbackDecoded &out);
}
```

统一 API 的行为：

- `enable()`：发送 `...FC`；发送成功才返回 0。
- `disable()`：发送 `...FD`；发送成功才返回 0。
- `set_torque()`：
  - 仅在 MIT 模式可用。
  - 构造 `position=0, velocity=0, kp=0, kd=0, torque=目标值`。
  - 立即发送一帧。
  - 超过协议 ±T_MAX 返回 `-ERANGE`。
  - 超过该实例的安全力矩上限也返回 `-ERANGE`；DM-J4310-2EC V1.1 的上限不得高于手册峰值 7 N·m，实际机构通常应该更低。
- `read_feedback()`：返回最近一帧的位置、速度、力矩、建议用转子温度作为公共 temperature；MOS 温度保留在专用 raw/decoded API。
- `get_state()`：
  - 超过心跳时间未反馈：Offline。
  - 有新鲜反馈且状态码在 `0x8..0xE`：Fault。
  - 有新鲜反馈且不在上述故障范围：Ready；原始状态码仍通过专用反馈接口提供。

### 10.8 DM-J4310 Config/Data

```cpp
struct DmJ4310Config
{
    const struct device *can;
    std::uint16_t motor_id;
    std::uint16_t master_id;
    skywalker::motor::dm::Limits limits;
    skywalker::motor::dm::SafetyLimits safety_limits;
};

struct DmJ4310Data
{
    skywalker::motor::Feedback feedback{};
    skywalker::motor::dm::FeedbackDecoded decoded{};
    std::uint64_t last_rx_ms = 0;
    std::uint8_t last_error = 0;
    struct k_spinlock lock{};
    int rx_filter_id = -1;
};
```

J4310 的 position 已由协议解码成实际弧度。产品内部虽然是 10:1 减速器和双编码器，但本地手册说明它提供输出轴单圈绝对位置；不要再套 DJI 的 8192 编码器累计，也不要擅自把协议位置再除以 10。第一次 RX 测试时手动转输出轴一圈，用实测确认固件的零点和范围。

### 10.9 binding

文件：`dts/bindings/motor/damiao,dm-j4310.yaml`

第一版把“SDK profile 值”和“硬件安全值”集中放在该型号驱动中，设备树只放总线地址。不要把这些常量藏进通用 `dm_protocol.cpp`，因为其它达妙型号不同：

```yaml
description: Damiao DM-J4310 motor using Classic CAN MIT protocol

compatible: "damiao,dm-j4310"

include: base.yaml

properties:
  can-bus:
    type: phandle
    required: true

  motor-id:
    type: int
    required: true

  master-id:
    type: int
    required: true
```

如果当前 Zephyr 对未知 vendor prefix 报错，在项目的 devicetree vendor prefix 文件中登记 `damiao`；不要为了绕过校验把 compatible 冒充成别的厂商。

overlay：

```dts
dm4310_1: motor-1 {
    compatible = "damiao,dm-j4310";
    status = "okay";
    can-bus = <&can2>;
    motor-id = <0x01>;
    master-id = <0x11>;
};
```

这里的 `0x01/0x11` 必须与调试助手中的实际配置一致。

### 10.10 DM-J4310 上电验证顺序

1. 卸载连杆或把机构可靠架空。
2. 确认总线只有一台待测电机。
3. 只注册反馈 filter，先不使能。
4. 打印 Master ID、DLC 和原始 8 字节。
5. 确认解码位置/温度合理。
6. 上电后等待几秒，再由应用显式调用 `enable()`。
7. 首个 MIT 命令用 `kp=0, kd=0, torque=0`。
8. 需要阻尼测试时，只逐步增加很小的 kd；不要先加 kp。
9. 尝试极小正/负 torque。
10. 调 `disable()` 并抓 CAN，确认 `...FD` 已发出。
11. 设零只能在机械零位明确、机构静止时由人主动触发。

## 11. 构建系统怎么接

### 11.1 Kconfig

在 `drivers/motor/Kconfig` 保留 DJI，并新增达妙开关：

```kconfig
config SKYWALKER_MOTOR_DM
    bool "Damiao CAN motors"
    default n
    help
        Enable Damiao motor drivers such as DM-J4310.
```

### 11.2 motor/CMakeLists.txt

取消 DM 子目录的注释：

```cmake
add_subdirectory_ifdef(CONFIG_SKYWALKER_MOTOR_DJI dji)
add_subdirectory_ifdef(CONFIG_SKYWALKER_MOTOR_DM dm)
```

### 11.3 dji/CMakeLists.txt

```cmake
zephyr_library_sources(
    dji_tx_group.cpp
    dji_protocol.cpp
    dji_m3508.cpp
    dji_m2006.cpp
    dji_gm6020.cpp
)
```

### 11.4 dm/CMakeLists.txt

```cmake
zephyr_library()

zephyr_library_sources(
    dm_protocol.cpp
    dm_j4310.cpp
)
```

### 11.5 样例 prj.conf

DJI 样例：

```conf
CONFIG_CPP=y
CONFIG_CAN=y
CONFIG_LOG=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
CONFIG_SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS=100
```

达妙样例：

```conf
CONFIG_CPP=y
CONFIG_CAN=y
CONFIG_LOG=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DM=y
CONFIG_SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS=100
```

## 12. 纯协议测试先于真电机

最好新增一个不需要电机的 ztest。即使暂时不会写 ztest，也至少在样例里对固定数组做断言。

### 12.1 DJI 解码向量

输入：

```text
12 34 FF 9C 00 7B 37 00
```

预期：

- encoder = `0x1234`。
- rpm = `-100`。
- effort_raw = `123`。
- temperature = `55`。

### 12.2 DJI 分组打包向量

输入：

```text
command = [0x1234, -1, -16384, 0]
```

预期 DATA：

```text
12 34 FF FF C0 00 00 00
```

### 12.3 DM MIT 零命令向量

使用 DM4310 默认 limits，所有目标和 kp/kd 都为 0。按“截断到整数”的实现，中间值应接近：

```text
7F FF 7F F0 00 00 07 FF
```

由于奇数满量程映射的中点可能因“截断还是四舍五入”差 1 LSB，测试要统一你选择的规则；端点必须精确：

- `-P_MAX -> 0`。
- `+P_MAX -> 65535`。
- `-V_MAX -> 0`。
- `+V_MAX -> 4095`。

### 12.4 DM 反馈拆包向量

至少检查：

- error 来自 `DATA[0]` 高 4 bit。
- motor id 来自低 4 bit。
- position 是 16 bit。
- velocity/torque 各自是跨字节的 12 bit。
- 两个温度字节没有颠倒。

## 13. 推荐实现顺序

严格按小步提交：

1. 修 `motor.hpp` 的函数指针检查。
2. 给现有 DJI codec 写固定向量测试。
3. 引入 `DjiFeedbackRaw` 和 `buildGroupCommandFrame()`，保留 M3508 兼容包装。
4. 把 `TxGroup` 改成 `GroupCommandSource` 适配器，不改 M3508 行为。
5. 重新验证 M3508。
6. 新增 M2006 binding、头文件、driver、sample。
7. 先验证 M2006 RX，再验证全 0 TX，再验证小命令。
8. 新增 GM6020 binding、头文件、driver、sample。
9. 先验证 GM6020 RX，再验证全 0 TX，再验证小命令。
10. 新增 DM 协议纯函数与固定向量测试。
11. 新增 DM-J4310 binding、driver、sample。
12. 只做 MIT 模式的零命令、使能、反馈、失能。
13. 最后再考虑位置速度模式、速度模式、参数读写和统一闭环控制器。

每一步都应能单独编译。不要等三个型号全部写完才第一次上电。

## 14. 你可以运行的构建命令

这些命令会生成 build 目录，所以本文只列出，不代你执行。

```bash
west build \
  -b dm_mc02/stm32h723xx \
  samples/motor/dji_m2006_driver \
  -d build/dji_m2006_driver \
  -p always \
  -- -DBOARD_ROOT=$PWD
```

```bash
west build \
  -b dm_mc02/stm32h723xx \
  samples/motor/dji_gm6020_driver \
  -d build/dji_gm6020_driver \
  -p always \
  -- -DBOARD_ROOT=$PWD
```

```bash
west build \
  -b dm_mc02/stm32h723xx \
  samples/motor/dm_j4310_driver \
  -d build/dm_j4310_driver \
  -p always \
  -- -DBOARD_ROOT=$PWD
```

如果构建报 compatible/binding 问题，按顺序查：

1. YAML 文件名和 `compatible` 是否一致。
2. `DT_DRV_COMPAT` 是否把逗号和连字符转换成下划线。
3. overlay 节点是否 `status = "okay"`。
4. CMake 是否加入对应 cpp。
5. Kconfig symbol 是否开启。
6. `can-bus` phandle 指向的控制器是否启用。

## 15. 常见错误

### 电机只有最大速度，没有“按 raw 值调速”

DJI 的 raw 指令是电流/电压，不是目标 rpm。空载时很小的持续转矩也可能让电机加速到很高转速。要调速，必须在应用层做：

```text
目标速度 - 反馈速度
        │
        ▼
      PID
        │
        ▼
raw current / raw voltage
```

### encoder 在 8191 到 0 处位置跳变

你只换算了单圈角度，没有做 `±4096` 回绕判断和累计。

### M2006 温度一直是 0

这是正常的协议差异。C610 官方反馈中 `DATA[6..7]` 是 Null，不应把 0 标成“有效 0℃”。

### 多台 DJI 电机只有最后绑定的一台能控制

当前代码会静默覆盖同一 slot。通用 `bind()` 必须对已占用 slot 返回 `-EADDRINUSE`。

### 调用 disable 后电机仍有输出

你只把软件缓存清零，没有继续发送分组帧。调用 `clearAndSend()`，并确认 CAN 上真的发出了全 0。

### DM4310 位置/速度/力矩比例都不对

大概率是 P_MAX、V_MAX、T_MAX 与电机固件不一致，不要先改移位代码。

### DM4310 有反馈但驱动一直 Offline

检查 RX filter 用的是 Master ID，不是 motor/slave ID。

### 一接入第二种电机就乱转

立即断电，检查 `0x1FF`、`0x201..0x208`、DM 的 `0x100/0x200 + ID` 冲突；不要继续靠试数值排查。

## 16. 第二阶段再做的优化

第一版验证完后，可以把 DJI 设备树的四个冗余字段：

```text
feedback-id
command-id
command-slot
gear-ratio
```

收敛成：

```text
motor-id
```

然后由型号驱动根据 motor ID 推导其余值，gear ratio 使用型号默认值。这样不会出现“feedback ID 写 0x205、command slot 却写 0”的人工不一致。

但不要把这项重构和第一台 M2006 bring-up 混在一个提交里。

## 17. 完成检查清单

### 公共层

- [ ] `motor.hpp` 检查正确的函数指针。
- [ ] DJI codec 使用中立命名。
- [ ] 旧 M3508 有兼容包装或已完整迁移。
- [ ] TxGroup 不再依赖 `m3508::*`。
- [ ] 重复槽位会报错。
- [ ] 有 `clearAndSend()`。

### M2006

- [ ] raw 命令限幅 ±10000。
- [ ] 默认 gear ratio 36。
- [ ] 不声明温度有效。
- [ ] feedback/command/slot 规则已验证。
- [ ] 多圈位置做回绕。
- [ ] 全 0 分组和小命令均实测。

### GM6020

- [ ] 接口命名为 voltage，不叫 current。
- [ ] raw 限幅 ±30000。
- [ ] ID 1~4 使用 0x1FF，5~7 使用 0x2FF。
- [ ] feedback ID 为 0x204 + ID。
- [ ] 位置/速度不除齿轮比。
- [ ] 第 4 个 0x2FF 槽位保持 0。

### DM-J4310

- [ ] 实际型号已确认是 DM-J4310-2EC V1.1。
- [ ] 使用 Classic CAN 标准帧、1 Mbps。
- [ ] 确认 motor ID 与 Master ID。
- [ ] 确认 P_MAX/V_MAX/T_MAX。
- [ ] 协议映射 T_MAX 与硬件安全力矩上限分离。
- [ ] 安全力矩上限不高于产品手册峰值 7 N·m，初测值远低于此。
- [ ] MIT 打包固定向量通过。
- [ ] 特殊 FC/FD/FE 帧已在当前固件抓包/实测确认。
- [ ] RX filter 使用 Master ID。
- [ ] 仅手册定义的 0x8..0xE 状态码映射为 Fault。
- [ ] 位置不做 DJI 式累计，也不额外除以 10。
- [ ] 不在 init 自动使能或设零。
- [ ] 与 DJI 使用不同 CAN 总线或已完成冲突证明。

## 18. 权威资料

优先使用仓库内已经固定版本的文档：

- [C610 User Guide](motor_docs/C610_User_Guide.pdf)
- [C620 User Guide V1.01](motor_docs/C620_User_Guide_V1.01.pdf)
- [M2006 P36 User Guide](motor_docs/M2006_P36_User_Guide.pdf)
- [M2006 DEMO Guide](motor_docs/M2006_DEMO_Guide.pdf)
- [GM6020 User Guide](motor_docs/GM6020_User_Guide.pdf)
- [M3508 User Guide V1.0](motor_docs/M3508_User_Guide_V1.0.pdf)
- [DM-J4310-2EC V1.1 User Manual](motor_docs/DM-J4310-2EC_V1.1_User_Manual.pdf)
- [下载来源记录](motor_docs/DOWNLOAD_LINKS.txt)

外部原始来源与厂商 SDK：

- [DJI C610 无刷电机调速器使用说明](<https://cdn-hz.robomaster.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf>)
- [DJI M2006 动力系统 DEMO 示例程序说明](<https://cdn-hz.robomaster.com/tem/RoboMaster%20M2006%20%E5%8A%A8%E5%8A%9B%E7%B3%BB%E7%BB%9F%20DEMO%20%E7%A4%BA%E4%BE%8B%E7%A8%8B%E5%BA%8F.pdf>)
- [DJI GM6020 使用说明书](<https://rm-static.djicdn.com/tem/3724/RM%20GM6020%20%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6.pdf>)
- [达妙官方 dmBots 组织](https://github.com/dmBots)
- [达妙官方 motor-sdk 中的 DM_CAN.py](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)
- [达妙官方 DM4310 STM32 示例说明](<https://github.com/dmBots/motor-control-routine/blob/master/stm32%E4%BE%8B%E7%A8%8B/dm_ctrl%28f4%29-4310_v1.0%20%E8%A3%B8%E6%9C%BA/USAGE.md>)

## 19. 你现在第一步只做什么

不要今天同时写三台电机。

你现在只做以下四件事：

1. 修 `motor.hpp` 的三个函数指针检查。
2. 把 `M3508Feedback` 抽成 `DjiFeedbackRaw`，保留兼容别名。
3. 把 `TxGroup` 改成 `GroupCommandSource` 适配器。
4. 重新构建并实测现有 M3508。

现有 M3508 不回归通过，就不要开始 `dji_m2006.cpp`。公共层一旦稳定，M2006 才会是一次可控的小增量。

# 第二部分：电机驱动完成后接 PID 与前馈控制

## 20. 先分清三层，PID 不属于电机驱动

电机驱动负责“正确收发和换算”，控制器负责“根据目标和反馈决定发什么命令”，应用负责“什么时候运行控制器”。

不要在 `dji_m3508.cpp`、`dji_m2006.cpp`、`dji_gm6020.cpp` 或 `dm_j4310.cpp` 的 CAN RX 回调里直接跑 PID。

推荐的数据流：

```text
轨迹/遥控目标
  q_ref, v_ref, a_ref
          │
          ▼
  外环位置控制（可选）
          │ 输出 v_cmd
          ▼
  内环速度 PID / PI
          │
          ├──────────┐
          │          │
          ▼          ▼
      反馈项      前馈项
    P + I + D   kS/kV/kA/kG
          │          │
          └────┬─────┘
               ▼
        限幅 + 变化率限制
               ▼
       型号命令适配器
       ├─ M3508: current_raw
       ├─ M2006: current_raw
       ├─ GM6020: voltage_raw
       └─ DM4310: torque_nm / MIT
               ▼
          CAN 发送层
```

这三层的边界是：

- `drivers/motor/`：CAN、设备树、反馈缓存、状态、raw/物理单位转换。
- `drivers/pid/` 或以后迁移到 `lib/control/`：纯控制算法，不知道 CAN 和具体电机型号。
- `samples/` 或 `application/`：控制线程、目标生成、选择哪个 PID、把输出交给哪个电机。

## 21. 当前 PID 实现的真实行为

当前文件：

- `include/drivers/pid/pid.h`
- `drivers/pid/pid.c`
- `dts/bindings/pid/skywalker,pid.yaml`

当前公式是：

```text
error = setpoint - measurement
P = Kp * error
I = clamp(I + Ki * error * dt, ±max_i_out)
D = Kd * (error - last_error) / dt
output = clamp(P + I + D + feedforward, ±max_out)
```

它可以用于低风险实验，但接电机前必须知道以下问题。

### 21.1 它不是“增量式 PID”

`drivers/pid/pid.c` 的注释写“增量式计算”，但代码保存积分项并直接计算完整输出，所以它是位置式 PID。

真正的增量式 PID 通常计算 `Δu(k)`，再加到上一拍输出 `u(k-1)`；当前代码没有这样做。

先把注释改正确，不要急着切换算法。电机控制使用位置式 PID 完全可以。

### 21.2 dt 没有保护

当前 D 项直接除以 `dt`：

```c
data->d_out =
    config->Kd *
    (error - data->last_error) /
    dt;
```

当 `dt == 0`、负数、异常大或 NaN 时，输出可能变成 Inf/NaN。电机命令一旦从 float 转成整数，结果不可预测。

控制器必须拒绝：

- `dt <= 0`。
- 非有限数。
- 超过允许最大周期的 dt。

发生异常时应清 PID 状态并发送安全零命令，而不是继续算。

### 21.3 当前死区会吞掉前馈

当前误差进入 deadband 后直接返回 0，因此：

- 重力补偿会突然消失。
- 摩擦前馈会突然消失。
- 已有积分保留在内存里，离开死区后又突然回来。

更合适的语义是：

1. deadband 只把用于 P/I 的小误差视为 0。
2. D 仍按测量变化计算，或者按策略冻结。
3. 前馈照常保留。
4. 积分选择“保持”“缓慢泄放”或“继续校正”，不能隐式遗留。

### 21.4 D 对 error 求导会有设定值冲击

目标从 0 突然跳到 10，即使电机还没动，error 也瞬间变化 10，当前 D 项会产生一个巨大尖峰。

电机控制第一版建议使用“对测量值求导”：

```text
D = -Kd * d(measurement)/dt
```

这样目标阶跃不会直接制造 D 冲击。D 还必须加低通滤波，因为编码器速度和差分噪声会被放大。

### 21.5 只有 I 项钳位，不等于完整抗饱和

当前 `i_out` 会钳到 `max_i_out`，但总输出可能长期卡在 `max_out`。在此期间积分仍可能朝错误方向增加，解除饱和后产生大过冲。

第一版至少实现“条件积分”；进阶版再做“回算抗饱和”。

### 21.6 没有 reset API

以下时刻必须能清理控制器状态：

- 电机从 Offline 恢复。
- enable/disable。
- 目标模式切换。
- 位置环与速度环切换。
- 急停。
- dt 超界。
- 在线调参大幅修改 Ki/Kd。

只靠系统启动时的 `skywalker_pid_init()` 清零不够。

### 21.7 一个 PID device 只能服务一个环

`pid_data` 是可写状态。两台电机或位置/速度两个环如果共用同一个 `skywalker,pid` 节点，会互相覆盖 last_error 和积分。

必须满足：

```text
一台电机的一个控制环 = 一份独立 pid_data
```

两台电机各有速度环，就是两份；每台又有位置外环，就是四份。

### 21.8 设备树配置是 ROM，不能直接在线改

当前 `pid_config` 由 `static const` 生成。不要为了 VOFA 在线调参而把 `dev->config` 强转后写入。

正确做法：

1. 设备树保存开机默认值。
2. 应用启动时复制到 RAM 中的 runtime config。
3. 控制线程只读 runtime config。
4. UART 回调通过消息队列提交新参数。
5. 控制线程在周期边界应用参数。

## 22. 建议先补齐的 PID 接口

保留 C 风格，便于当前 C/C++ 混合项目调用。建议由你把 PID API 逐步扩展为以下形状。

### 22.1 配置、状态、输入和输出分开

```c
typedef struct {
    float kp;
    float ki;
    float kd;

    float beta;
    float derivative_tau;
    float anti_windup_gain;

    float integral_min;
    float integral_max;
    float output_min;
    float output_max;

    float deadband;
    float output_slew_rate;
} pid_config;

typedef struct {
    float integral;
    float previous_measurement;
    float filtered_measurement_rate;
    float previous_output;

    float p_out;
    float i_out;
    float d_out;
    float ff_out;
    float unsaturated_output;
    float output;

    bool initialized;
} pid_data;

typedef struct {
    float setpoint;
    float measurement;
    float feedforward;
    float dt;
    bool hold_integrator;
} pid_input;

typedef struct {
    float p;
    float i;
    float d;
    float feedforward;
    float unsaturated;
    float saturated;
} pid_result;
```

为什么要返回 `pid_result`：

- 调 PID 时必须看见 P/I/D/FF 各项。
- 只看最终 output 无法判断是增益错误、积分饱和还是前馈符号错误。
- VOFA 可以直接画这些字段。

### 22.2 建议函数

```c
int pid_step(
    pid_data *data,
    const pid_config *config,
    const pid_input *input,
    pid_result *result);

void pid_reset(
    pid_data *data,
    float current_measurement);
```

行为契约：

#### `pid_step()`

- 任一指针为空：`-EINVAL`。
- dt 非有限、`dt <= 0` 或超过上限：`-ERANGE`。
- 任一输入/配置为 NaN/Inf：`-EDOM`。
- 配置上下限颠倒：`-EINVAL`。
- 成功返回 0，并填 result。
- 失败时不更新 PID 状态。

#### `pid_reset()`

- integral 清零。
- P/I/D/FF/输出清零。
- `previous_measurement = current_measurement`。
- `initialized = true`，避免下一拍 D 项把当前测量当成从 0 突变。

进阶的无扰切换可以再增加 `pid_track_output()`，第一版先可靠清零。

## 23. 推荐的基础 PID 计算顺序

### 23.1 第一步：验证输入

先检查：

```c
isfinite(setpoint)
isfinite(measurement)
isfinite(feedforward)
isfinite(dt)
dt > 0
```

对 500 Hz 控制环，标称 dt 是 `0.002 s`。可以允许一定抖动，但如果突然变成 0.1 s，应进入故障恢复，不要拿这个 dt 积分。

### 23.2 第二步：误差和 deadband

```text
error = setpoint - measurement

if abs(error) < deadband:
    effective_error = 0
else:
    effective_error = error
```

不要在这里 return。后面的 feedforward、限幅和状态记录仍要执行。

### 23.3 第三步：P 项

普通 P：

```text
P = Kp * effective_error
```

两自由度 PID 的 P：

```text
P = Kp * (beta * setpoint - measurement)
```

`beta=1` 就是普通 P；`0<beta<1` 可以减少目标阶跃导致的攻击性，同时保留对扰动的纠正能力。

第一版使用 `beta=1`，把接口留好即可。

### 23.4 第四步：测量微分与低通滤波

原始测量变化率：

```text
measurement_rate =
    (measurement - previous_measurement) / dt
```

一阶低通：

```text
alpha = derivative_tau / (derivative_tau + dt)

filtered_rate =
    alpha * previous_filtered_rate
    + (1 - alpha) * measurement_rate
```

D 项：

```text
D = -Kd * filtered_rate
```

`derivative_tau` 越大，滤波越强但延迟越大。不要用一个“万能值”；从关闭 D 开始，看到确实需要阻尼再增加。

速度环的 measurement 已经是速度，D 项等价于对速度再求导，容易受噪声影响。大多数第一版速度环先用 PI，不用 D。

### 23.5 第五步：条件积分

先算候选积分：

```text
I_candidate =
    clamp(
        I + Ki * effective_error * dt,
        integral_min,
        integral_max)
```

先用候选值算未饱和输出：

```text
u_candidate = P + I_candidate + D + FF
u_saturated = clamp(u_candidate, output_min, output_max)
```

如果满足下面任一条件，接受 `I_candidate`：

- 输出没有饱和。
- 输出在上限，但 error 为负，积分会把输出拉回。
- 输出在下限，但 error 为正，积分会把输出拉回。

否则保持旧积分。

等价判断可以写成：

```text
如果 (u_candidate - u_saturated) * effective_error > 0
    当前误差正在把饱和推得更严重，冻结积分
否则
    接受积分
```

### 23.6 第六步：输出限幅和变化率限制

先做物理/安全限幅：

```text
u_limited = clamp(u_unsaturated, output_min, output_max)
```

再做每秒最大变化量：

```text
delta_max = output_slew_rate * dt

u_applied = clamp(
    u_limited,
    previous_output - delta_max,
    previous_output + delta_max)
```

变化率限制可以减小齿隙冲击和电流阶跃，但它也形成了第二种饱和。进阶抗饱和应该把实际 `u_applied` 反馈给积分回算。

### 23.7 回算抗饱和变种

条件积分跑通后，才考虑回算：

```text
I += [
    Ki * effective_error
    + Kaw * (u_applied - u_unsaturated)
] * dt
```

`Kaw` 让积分跟踪实际能施加的输出。太大会造成积分快速抖动，太小则恢复慢。

第一版不要同时实现条件积分和回算；选一个并写清楚测试。

## 24. “前馈 PID”到底是什么

前馈不是另一套误差 PID，而是利用“目标运动本身需要多少输出”的已知关系，提前给控制量：

```text
u = u_feedback + u_feedforward

u_feedback = P + I + D
```

反馈负责纠正未知误差；前馈负责承担可预测负载。前馈不看误差，所以模型错了也不会自己修正，仍需要反馈闭环。

项目当前 `pid_update(..., feedforward)` 已经留了一个标量入口，但前馈应由轨迹/机构模型在控制层计算，而不是写死在 PID 算法里。

## 25. 常用前馈变种

### 25.1 常量偏置前馈

```text
FF = k0
```

适用：

- 单向加热器的基础加热量。
- 已知近似恒定的负载。

项目 `imu_heat_control()` 的 `HEAT_OFFSET_NS` 就是这种前馈。

不适合直接用于双向电机，因为正反方向通常不同。

### 25.2 静摩擦前馈 kS

```text
FF_static = kS * sign(v_ref)
```

用于克服电机、减速箱和机构的静摩擦。

零速附近不要直接对噪声取 sign，否则输出会正负抖动。推荐：

```text
if abs(v_ref) > velocity_epsilon:
    direction = sign(v_ref)
else if abs(a_ref) > acceleration_epsilon:
    direction = sign(a_ref)
else:
    direction = 0
```

正反摩擦不同的机构可使用 `kS_positive` 和 `kS_negative`。

### 25.3 速度前馈 kV

```text
FF_velocity = kV * v_ref
```

它承担稳态速度所需的大部分输出。速度 PID 只需要补偿电池电压、温度、负载等偏差。

### 25.4 加速度前馈 kA

```text
FF_acceleration = kA * a_ref
```

它承担加速惯量需要的输出。

`a_ref` 必须来自轨迹生成器，不要对摇杆目标或阶跃目标直接差分；阶跃的理论加速度无限大，实际差分会制造巨大尖峰。

### 25.5 电机常用组合

```text
FF =
    kS * direction
    + kV * v_ref
    + kA * a_ref
```

系数单位取决于最终输出域。

以 M3508 raw current 为例：

- `kS`：raw。
- `kV`：raw / (rad/s)。
- `kA`：raw / (rad/s²)。

以 DM4310 torque 为例：

- `kS`：N·m。
- `kV`：N·m / (rad/s)。
- `kA`：N·m / (rad/s²)。

不能把 M3508 的 kV 数字直接复制到 GM6020 或 DM4310。

### 25.6 重力前馈 kG

单关节、零位定义合适时，常见近似：

```text
torque_gravity = kG * sin(position)
```

或者因为机械零位不同使用 cos：

```text
torque_gravity = kG * cos(position)
```

到底用 sin 还是 cos 由“零角度在哪里”决定，不能背公式。

更物理的单连杆模型：

```text
torque_gravity =
    mass * gravity * center_of_mass_length *
    sin(position_relative_to_vertical)
```

对于多关节机械臂，重力项会依赖多个关节角，不能给每个关节孤立套一个 kG 就宣称完整。

### 25.7 完整模型前馈

进阶轨迹控制：

```text
tau_ff =
    M(q) * qdd_ref
    + C(q, qd) * qd_ref
    + G(q)
    + friction(qd_ref)
```

这是计算力矩/逆动力学前馈。它最后仍与反馈控制叠加：

```text
tau_cmd =
    tau_ff
    + Kp * position_error
    + Kd * velocity_error
    + Ki * integral_error
```

第一版单电机速度控制不需要直接跳到这里。

## 26. 两自由度 PID 是另一种“前馈式变种”

两自由度 PID 对目标变化和外部扰动使用不同权重：

```text
P = Kp * (beta * r - y)
I = Ki * integral(r - y)
D = Kd * (gamma * dr/dt - filtered(dy/dt))
```

常用选择：

- `beta=1`。
- `gamma=0`，即 D 只对测量求导。

它不等于 kS/kV/kA 模型前馈，但目的相似：减少目标变化带来的冲击，同时保留扰动抑制。

建议实现顺序：

1. 普通 PI。
2. 测量微分 + 滤波。
3. kS/kV 前馈。
4. 需要时再开放 beta。
5. 最后才考虑 gamma 和设定值导数。

## 27. 位置—速度串级控制

对 M3508、M2006、GM6020，位置控制最好先做两环：

```text
位置目标 q_ref
      │
      ▼
位置外环 P/PD
      │ 输出速度目标 v_cmd
      ▼
速度内环 PI/PID
      │ 输出 raw current / raw voltage
      ▼
电机
```

### 27.1 外环

```text
position_error = q_ref - q

v_cmd =
    clamp(
        Kp_position * position_error
        + v_ref,
        -v_max,
        +v_max)
```

第一版外环只用 P：

- 不加 I，避免位置环和速度环同时积累。
- `v_ref` 是轨迹速度前馈。
- 如果有轨迹加速度，传给内环的 kA 前馈。

### 27.2 内环

```text
velocity_error = v_cmd - v

u_feedback =
    PI_or_PID(v_cmd, v)

u =
    u_feedback
    + kS * direction
    + kV * v_cmd
    + kA * a_ref
    + gravity_term_if_needed
```

### 27.3 两环频率

一个保守起点：

- 速度内环：500 Hz，即 2 ms。
- 位置外环：100 Hz，即每 5 次内环更新一次。

验证 CPU、CAN 带宽、反馈频率和抖动后，内环可提高到 1 kHz。

不要因为 C610/C620 默认反馈 1 kHz，就默认应用控制线程也已经稳定达到 1 kHz。

## 28. 连续角度与有限角度的误差

### 28.1 连续旋转

如果目标是“走最短角度”，用：

```text
error = wrap_to_pi(q_ref - q)
```

结果在 `[-π, π]`。

### 28.2 有机械限位

机械臂关节有硬限位时，不应无条件 wrap。必须：

1. 先把目标限制到软限位。
2. 使用同一连续坐标系计算误差。
3. 靠近硬限位降低速度/力矩上限。

### 28.3 多圈位置

M3508/M2006 的多圈相对位置应由驱动按 encoder 回绕累计后提供。PID 不应自己重新解析 raw encoder。

DM-J4310-2EC V1.1 是输出轴单圈绝对位置。是否 wrap 由机构需求决定，不能再除以 10 或做 DJI 式累计。

## 29. 不同电机怎么接 PID 输出

| 型号       | 推荐反馈                                        | PID 输出域           | 最终调用                                                       |                             最终硬限幅 |
| ---------- | ----------------------------------------------- | -------------------- | -------------------------------------------------------------- | -------------------------------------: |
| M3508/C620 | `velocity_rad_s`，位置环再用 `position_rad` | raw current float    | 四舍五入后`m3508::setCurrentRaw()`，最后 `TxGroup::send()` |     协议 ±16384，初测另设更小安全限幅 |
| M2006/C610 | 同上                                            | raw current float    | `m2006::setCurrentRaw()` + group send                        |     协议 ±10000，初测另设更小安全限幅 |
| GM6020     | 同上                                            | raw voltage float    | `gm6020::setVoltageRaw()` + group send                       |     协议 ±30000，初测另设更小安全限幅 |
| DM-J4310   | position/velocity/torque                        | N·m 或直接 MIT 参数 | `setMitCommand()`                                            | 协议 T_MAX 与硬件/机构安全上限取更小值 |

### 29.1 float 转 raw

不要直接 C 风格强转：

本项目根目录 `CMakeLists.txt` 当前指定的是 C++14，因此：

- 不要照搬 C++17 的 `std::clamp`；下面的 `clamp` 表示你自己实现的 `clampf(value, min, max)`，或项目中已经确认可用的等价函数。
- `std::isfinite` 和 `std::lround` 需要包含 `<cmath>`。
- 不要使用 C++20 的结构体指定初始化语法；本手册后续示例采用“先零初始化，再逐字段赋值”。

```cpp
const float limited =
    clamp(pid_output, -safe_raw, safe_raw);

const auto raw =
    static_cast<std::int16_t>(
        std::lround(limited));
```

顺序必须是：

1. 检查 finite。
2. float 安全限幅。
3. 四舍五入。
4. 转整数。
5. 调型号 API，由型号 API 再做协议硬限幅。

### 29.2 DJI 分组发送顺序

同一组四台电机每个周期只发送一次：

```text
读取 motor0..3 反馈
计算 motor0..3 PID
分别写 motor0..3 的 raw 缓存
调用一次 TxGroup::send()
```

不要每算完一台就 send 一次，否则一帧中的四个槽位来自不同控制时刻。

如果任意一台反馈失效，第一版安全策略是：

1. 所有同组命令清零。
2. `clearAndSend()`。
3. reset 同组 PID。
4. 记录故障，等待人工或明确恢复条件。

## 30. 控制线程怎么写

不要在以下上下文运行完整 PID：

- CAN RX callback。
- `k_timer` ISR callback。
- UART RX callback。

推荐：

```text
k_timer / 固定周期中断
          │
          ▼
      k_sem_give
          │
          ▼
专用控制线程 k_sem_take
          │
          ├─ 读取反馈快照
          ├─ 计算真实 dt
          ├─ PID/前馈
          ├─ 写命令
          └─ CAN send
```

### 30.1 为什么用线程

- `can_send()` 可能等待邮箱。
- 浮点、日志和多电机循环不应堵塞 ISR。
- 线程里更容易统一处理超时和安全状态。

### 30.2 dt 来源

使用单调时钟记录实际周期，不要永远把 dt 写死成 0.002：

```text
dt = (now - previous_control_time) in seconds
```

同时检查反馈自己的 `timestamp_ms`：

- timestamp 没变：没有新反馈，不更新积分和 D。
- 可以重发上一拍安全命令，但不能假装有新测量。
- feedback age 超过控制超时：立即清零。

现有 `timestamp_ms` 对 500 Hz 足够入门；做到 1 kHz 后，毫秒分辨率可能太粗，应考虑微秒或 cycle timestamp。

### 30.3 在线与可控不是同一个超时

`CONFIG_SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS=100` 可以用于“设备是否在线”，但 100 ms 对高速闭环太久。

控制层应另设更短的 `control_feedback_timeout`，例如 3~5 个控制周期。具体值根据实际反馈频率和系统抖动确定。

## 31. M3508 速度 PI 的应用伪代码

下面是数据流示例，不是要求你现在复制进源码。

```cpp
for (;;) {
    k_sem_take(&control_tick, K_FOREVER);

    Feedback feedback{};
    const int read_ret =
        motor::readFeedback(motor, feedback);

    if (read_ret < 0 ||
        motor::getState(motor) != State::Ready ||
        !(feedback.valid & FeedbackVelocity) ||
        feedback_is_too_old(feedback)) {
        tx_group.clearAndSend();
        pid_reset(&speed_pid, feedback.velocity_rad_s);
        continue;
    }

    if (feedback.timestamp_ms ==
        last_feedback_timestamp_ms) {
        tx_group.send();
        continue;
    }

    const float dt =
        feedback_dt_seconds(
            last_feedback_timestamp_ms,
            feedback.timestamp_ms);

    const float ff =
        ks * direction_from_reference(
            velocity_ref,
            acceleration_ref)
        + kv * velocity_ref
        + ka * acceleration_ref;

    pid_input input{};
    input.setpoint = velocity_ref;
    input.measurement = feedback.velocity_rad_s;
    input.feedforward = ff;
    input.dt = dt;
    input.hold_integrator = false;

    pid_result result{};
    const int pid_ret =
        pid_step(
            &speed_pid,
            &speed_config,
            &input,
            &result);

    if (pid_ret < 0 ||
        !std::isfinite(result.saturated)) {
        tx_group.clearAndSend();
        pid_reset(
            &speed_pid,
            feedback.velocity_rad_s);
        continue;
    }

    const std::int16_t raw =
        safe_float_to_current_raw(
            result.saturated);

    if (m3508::setCurrentRaw(motor, raw) < 0 ||
        tx_group.send() < 0) {
        tx_group.clearAndSend();
        pid_reset(
            &speed_pid,
            feedback.velocity_rad_s);
        continue;
    }

    last_feedback_timestamp_ms =
        feedback.timestamp_ms;
}
```

第一版速度环用 PI：

- `Kd=0`。
- `Ki=0` 起步，先调 Kp。
- 输出安全限幅远小于 16384。
- 电机架空。

## 32. DM-J4310 的三种控制选择

DM4310 已经有内部控制模式，不能不加区分地再套一层外部 PID。

### 32.1 直接 MIT 阻抗控制

发送：

```text
p_des = q_ref
v_des = v_ref
Kp = 位置刚度
Kd = 阻尼
t_ff = 重力/动力学前馈
```

这是最自然的“PD + torque feedforward”。

注意：

- 本地手册明确说位置控制时 Kd 不能为 0。
- Kp/Kd 先从很小值开始。
- t_ff 仍受安全力矩限制。
- 不需要再让外部 PID 输出另一个位置命令。

### 32.2 外部 PID 输出 torque

发送 MIT：

```text
p_des = 0
v_des = 0
Kp = 0
Kd = 0
t_ff = external_controller_output
```

此时外部控制器完全负责位置/速度反馈，电机只执行 torque feedforward 字段。

适合：

- 需要统一多关节控制。
- 需要完整重力/动力学补偿。
- 控制线程和通信已经足够可靠。

风险更高，通信超时、限幅和急停必须先完成。

### 32.3 使用电机内部位置速度/速度模式

应用只生成平滑目标，把内部环当作执行器。外部不再重复跑同类型高速 PID，只做更慢的任务/轨迹层修正。

### 32.4 最重要的禁止项

不要同时：

- 给 MIT 很大的 Kp/Kd；
- 又让外部位置 PID 输出很大的 t_ff；
- 还开电机内部位置模式。

除非你能写出完整的等效闭环，否则这叫重复控制，不叫更强控制。

## 33. 前馈参数怎么辨识

### 33.1 准备

必须记录：

- target。
- measurement。
- error。
- P/I/D。
- FF。
- unsaturated output。
- applied output。
- dt。
- feedback age。

先让所有 Ki/Kd/前馈为 0，仅保留很小 Kp 和安全输出限幅。

### 33.2 kS

正方向缓慢增加输出，记录刚开始稳定运动的输出；反方向重复。

得到：

```text
kS_positive
kS_negative
```

不要只测一个方向。

### 33.3 kV

在多个稳定速度点记录：

```text
steady_output - kS * direction
```

对速度做直线拟合，斜率近似 kV。

空载 kV 只能作为起点，装上机构后需要复核。

### 33.4 kA

先确定 kS/kV，再做受控的加速段：

```text
remaining_output =
    applied_output
    - kS * direction
    - kV * v_ref
```

用 remaining output 对 `a_ref` 拟合 kA。

### 33.5 kG

机械臂在多个安全角度静态保持，记录维持所需输出并拟合 sin/cos。

不要在没有制动/保护的情况下松手做重力辨识。

## 34. PID 调参顺序

### 34.1 速度环

1. 架空电机，设置很小输出上限。
2. Ki=0、Kd=0、FF=0。
3. 从很小 Kp 开始，增加到响应明显但不过度振荡。
4. 测 kS/kV，加入前馈。
5. 加少量 Ki 消除稳态误差。
6. 只有噪声和动态确实需要时才加 D，并先配置滤波。
7. 增加负载重新验证。

很多电机速度环最终是 `PI + kS + kV + kA`，不需要完整 PID。

### 34.2 位置串级

1. 先把速度内环调稳定。
2. 外环 Ki=0、Kd=0。
3. 从小 Kp_position 开始。
4. 限制 v_cmd。
5. 加轨迹 v_ref 前馈。
6. 需要时再加外环 D 或速度反馈阻尼。
7. 最后考虑重力前馈。

内环没稳定前，不允许调外环。

## 35. 目标必须经过轨迹生成

前馈需要 `v_ref` 和 `a_ref`。如果用户命令是阶跃或摇杆噪声，先经过：

- 速度斜坡限制。
- 梯形速度轨迹。
- S 曲线轨迹。
- 至少一个 slew-rate limiter。

最小斜坡：

```text
max_delta = max_acceleration * dt

v_ref =
    clamp(
        requested_velocity,
        previous_v_ref - max_delta,
        previous_v_ref + max_delta)

a_ref =
    (v_ref - previous_v_ref) / dt
```

这样 kA 才有可控输入。

## 36. PID 设备树实例怎么配

先保留现有 binding，每个环建立独立节点：

```dts
/ {
    motor0_speed_pid: motor0-speed-pid {
        compatible = "skywalker,pid";
        k-p = "0";
        k-i = "0";
        k-d = "0";
        i-max = "0";
        out-max = "0";
        deadband = "0";
    };

    motor0_position_pid: motor0-position-pid {
        compatible = "skywalker,pid";
        k-p = "0";
        k-i = "0";
        k-d = "0";
        i-max = "0";
        out-max = "0";
        deadband = "0";
    };
};
```

这里故意不提供“万能增益”。你需要根据：

- 输出单位是 raw 还是 N·m。
- 机构惯量。
- 减速比和齿隙。
- 控制周期。
- 安全输出上限。

逐台调试。

prj.conf：

```conf
CONFIG_SKYWALKER_DRIVER_PID=y
```

后续给 binding 增加 derivative filter、beta、anti-windup 等属性时，仍应把设备树值复制到 RAM runtime config，方便在线调参。

## 37. VOFA 在线调参注意事项

现有 `lib/vofa` 能解析 `kp=1.5` 形式的命令，但不要在 UART callback 里直接修改控制线程正在读取的多个 float。

推荐：

```text
UART callback
    │ 解析 key/value
    ▼
k_msgq_put(PidParamUpdate)
    │
    ▼
控制线程周期边界
    ├─ 应用新参数
    ├─ 校验上下限
    └─ 必要时 reset integral
```

建议命令：

```text
m0.speed.kp=...
m0.speed.ki=...
m0.speed.kd=...
m0.speed.ks=...
m0.speed.kv=...
m0.speed.ka=...
m0.speed.out_max=...
```

每次修改：

1. 检查 finite。
2. 检查允许范围。
3. 大幅修改 Ki/Kd 时 reset。
4. 修改 output limit 时立即重新 clamp 积分与上一输出。

### 37.1 当前 VOFA 发送缓冲风险

`vofa_send()` 使用 static buffer，然后异步 `uart_tx()`。如果上一次 DMA 尚未完成又调用，buffer 可能被覆盖。

高频记录 PID 前，应由你增加：

- TX busy 标志；或
- 双缓冲；或
- 消息队列 + 独立遥测线程。

不要在 500 Hz 控制线程里直接无节制打印日志。控制与遥测应解耦。

## 38. PID 纯算法测试

在接电机前至少完成这些固定测试。

### 38.1 P + FF

配置：

```text
Kp=2, Ki=0, Kd=0
setpoint=3
measurement=1
FF=0.5
```

未限幅输出应为：

```text
2 * (3 - 1) + 0.5 = 4.5
```

### 38.2 输出饱和

若 output_max=3，最终输出应为 3，但 result.unsaturated 仍应记录 4.5。

### 38.3 条件积分

输出已卡在正上限且 error 为正时，多运行 1000 拍，积分不应继续正向增长。

当 error 变为负时，积分应允许把输出拉回饱和区内。

### 38.4 无 D 冲击

measurement 不变，只把 setpoint 从 0 改到 10；使用测量微分时 D 应接近 0。

### 38.5 D 滤波

measurement 中加入一个单拍尖峰，filtered D 的峰值应小于未滤波值，并按设定时间常数衰减。

### 38.6 deadband 不吞前馈

error 在 deadband 内、FF=2 时，最终输出应保留 FF 及明确的积分策略，不能无条件返回 0。

### 38.7 reset

reset 后：

- integral=0。
- output=0。
- 下一拍 measurement 不变时 D=0。

### 38.8 非法 dt

`dt=0`、负数、NaN、超大值都应返回错误且不修改状态。

## 39. 硬件验证顺序

1. 电机驱动 RX/TX 已独立验证。
2. 电机架空，准备物理断电。
3. 控制器输出硬限制到很小值。
4. 只跑 P，不跑 I/D/FF。
5. 确认反馈正方向和命令正方向一致。
6. 确认目标为 0 时输出为安全值。
7. 拔掉 CAN，确认在几个控制周期内清零。
8. 制造一次 dt 超时，确认 reset + 清零。
9. 加 kS/kV。
10. 加少量 I。
11. 最后才增加负载、位置外环、kA/kG。

## 40. PID 接入完成检查清单

### 算法

- [ ] 注释明确为位置式 PID。
- [ ] dt 和所有 float 都做 finite/range 检查。
- [ ] D 对测量求导并低通。
- [ ] deadband 不会吞前馈。
- [ ] 有条件积分或回算抗饱和。
- [ ] 有 reset API。
- [ ] 记录 unsaturated 与 saturated 输出。
- [ ] 输出有安全限幅和变化率限制。

### 调度

- [ ] PID 在专用线程运行，不在 CAN/timer/UART callback 运行。
- [ ] 使用实际 dt。
- [ ] 只有新反馈才更新积分和 D。
- [ ] 控制超时短于设备在线心跳。
- [ ] CAN 发送失败会清零并 reset。
- [ ] 同一 DJI TxGroup 每周期只 send 一次。

### 单位

- [ ] 位置统一 rad。
- [ ] 速度统一 rad/s。
- [ ] dt 统一 s。
- [ ] M3508/M2006 输出明确为 raw current。
- [ ] GM6020 输出明确为 raw voltage。
- [ ] DM4310 输出明确为 N·m 或 MIT 参数。
- [ ] 协议硬限幅与机构安全限幅分开。

### 前馈

- [ ] kS 正反方向分别验证。
- [ ] kV 与当前输出域单位一致。
- [ ] kA 使用轨迹 a_ref，不差分阶跃目标。
- [ ] kG 的 sin/cos 与机械零位一致。
- [ ] feedforward 也经过最终安全限幅。
- [ ] PID 饱和时考虑前馈造成的饱和。

### 调参

- [ ] 每个电机、每个环有独立 pid_data。
- [ ] runtime config 在 RAM，不写 const dev->config。
- [ ] UART 参数通过消息队列交给控制线程。
- [ ] 遥测不会阻塞控制线程或覆盖 DMA buffer。

## 41. 你接下来只实现哪一步

完成所有电机驱动后，按这个顺序写控制层：

1. 给现有 PID 增加 `pid_reset()` 和 dt/finite 检查。
2. 把 deadband 改成 effective_error，不再提前 return。
3. 把 D 改成测量微分并增加一阶低通。
4. 加条件积分抗饱和。
5. 增加 `pid_result`，把 P/I/D/FF/饱和值送到 VOFA。
6. 先为一台架空 M3508 写 500 Hz 速度 PI 控制线程。
7. 验证断 CAN、dt 超时、send 失败都能清零。
8. 再增加 kS/kV。
9. 速度环稳定后再写位置外环。
10. 最后把同样的控制层通过“输出适配器”接到 M2006、GM6020 和 DM4310。

不要第一步就写“通用全电机位置速度力矩三环控制器”。先让一台电机的一条速度环可测、可停、可解释。
