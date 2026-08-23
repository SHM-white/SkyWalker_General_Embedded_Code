# SkyWalker C++ 电机驱动：0.1B 小模型执行手册

> 目标：在 `SkyWalker_General_Embedded_Code` 中，从零写出一套自己的电机驱动。
> 风格：**C++14 写协议和电机逻辑，Zephyr Driver Model 保持 C 风格胶水。**
> 原则：不照搬 Breeze 源码，只参考架构思路；协议内容以电机官方文档为准。
>
> 这份文档假设执行者很笨、上下文很短，所以：
>
> - 一次只做一件事；
> - 每一步都给出“完成标准”；
> - 前一步不过，不允许继续；
> - 遇到错误先看文档给的排查顺序；
> - 不允许为了“优雅”提前抽象。

---

# 0. 最终我们要得到什么

最后希望应用层能写成：

```cpp
const struct device *motor = DEVICE_DT_GET(DT_ALIAS(motor0));

skywalker::motor::Feedback fb{};

skywalker::motor::setTorque(motor, 0.3f);
skywalker::motor::readFeedback(motor, fb);
```

应用层不需要知道：

```text
这是 DJI
这是 DM
这是 LK
CAN ID 是多少
反馈第几字节是什么
MIT 怎么打包
DJI 四个电机为什么共用一帧
```

这些全部放在 driver 里面。

但是：

> **第一阶段不要从这个最终目标开始写。**

正确顺序：

```text
CAN 能用
↓
M3508 raw demo
↓
M3508 protocol
↓
Motor 公共 API
↓
M3508 Zephyr driver
↓
DJI group TX
↓
heartbeat
↓
DM
↓
LK
```

---

# 1. C++ 使用规则

整个 motor subsystem 使用：

```text
C++14
```

## 1.1 可以使用

```text
namespace
class
struct
enum class
constexpr
template（简单模板）
static_cast
RAII（简单锁封装）
引用
函数重载
默认成员初始化
```

## 1.2 暂时禁止

```text
new
delete
exceptions
throw
try/catch
RTTI
dynamic_cast
iostream
std::vector
std::map
std::unordered_map
复杂模板元编程
大量 virtual
```

原因不是这些东西一定不能用。

原因是：

> 我们现在写 MCU driver，越简单越容易定位问题。

## 1.3 错误仍然用 errno

例如：

```cpp
return 0;
return -EINVAL;
return -ENODEV;
return -ENOSYS;
return -ETIMEDOUT;
return -EIO;
```

不要设计：

```cpp
throw MotorError();
```

---

# 2. 推荐目录

最终目标：

```text
skywalker_code/
├── drivers/
│   └── motor/
│       ├── Kconfig
│       ├── CMakeLists.txt
│       │
│       ├── motor_api.cpp
│       ├── motor_can_router.cpp
│       │
│       ├── dji/
│       │   ├── CMakeLists.txt
│       │   ├── dji_protocol.cpp
│       │   ├── dji_m3508.cpp
│       │   └── dji_tx_group.cpp
│       │
│       ├── dm/
│       │   ├── CMakeLists.txt
│       │   ├── dm_protocol.cpp
│       │   └── dm_motor.cpp
│       │
│       └── lk/
│           ├── CMakeLists.txt
│           ├── lk_protocol.cpp
│           └── lk_motor.cpp
│
├── include/
│   └── skywalker/
│       └── motor/
│           ├── motor.hpp
│           ├── dji_protocol.hpp
│           ├── dm_protocol.hpp
│           └── lk_protocol.hpp
│
├── dts/
│   └── bindings/
│       └── motor/
│           ├── dji,m3508.yaml
│           ├── damiao,motor.yaml
│           └── lingkong,motor.yaml
│
└── samples/
    └── motor/
        ├── can_smoke/
        ├── dji_m3508_raw/
        ├── dji_m3508/
        ├── dm_mit/
        └── lk_basic/
```

但第一次只建：

```text
samples/motor/can_smoke
```

---

# 3. 第 1 阶段：CAN smoke test

## 目标

确认：

```text
dm_mc02
+
Zephyr
+
FDCAN
+
物理 CAN 总线
```

本身没有问题。

这一阶段：

```text
不写 Motor
不写 Protocol
不写 Class
不写 DTS motor binding
```

只测试 CAN。

---

# 4. 新建 CAN smoke sample

创建：

```bash
mkdir -p samples/motor/can_smoke/src
```

目录：

```text
samples/motor/can_smoke/
├── CMakeLists.txt
├── prj.conf
└── src/
    └── main.cpp
```

---

# 5. `CMakeLists.txt`

写：

```cmake
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(can_smoke)

target_sources(app PRIVATE src/main.cpp)
```

不要加其他东西。

---

# 6. `prj.conf`

写：

```ini
CONFIG_CPP=y

CONFIG_CAN=y

CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

CONFIG_ASSERT=y
```

---

# 7. `main.cpp`

第一版：

```cpp
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can_smoke, LOG_LEVEL_INF);

int main()
{
    const struct device *can = DEVICE_DT_GET(DT_NODELABEL(can1));

    if (!device_is_ready(can)) {
        LOG_ERR("CAN device not ready");
        return -ENODEV;
    }

    int ret = can_start(can);

    if (ret < 0 && ret != -EALREADY) {
        LOG_ERR("can_start failed: %d", ret);
        return ret;
    }

    LOG_INF("CAN ready");

    while (true) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
```

---

# 8. 编译

在：

```text
skywalker_code/
```

执行：

```bash
west build \
  -b dm_mc02/stm32h723xx \
  samples/motor/can_smoke \
  -p always \
  -- -DBOARD_ROOT=$PWD
```

## 成功标准

必须：

```text
0 error
0 Kconfig undefined warning
```

烧录后看到：

```text
CAN ready
```

## 如果失败

不要继续。

先修 CAN。

---

# 9. 第 2 阶段：接收任意 CAN 帧

现在给 CAN 加 filter。

先不要管 M3508。

---

# 10. RX callback

在 `main.cpp` 加：

```cpp
static void rxCallback(const struct device *dev,
                       struct can_frame *frame,
                       void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    printk("RX id=0x%03x dlc=%d\n",
           frame->id,
           frame->dlc);
}
```

然后：

```cpp
struct can_filter filter = {
    .id = 0,
    .mask = 0,
    .flags = 0,
};

int filter_id =
    can_add_rx_filter(can,
                      rxCallback,
                      nullptr,
                      &filter);
```

如果返回：

```text
>= 0
```

说明 filter 注册成功。

---

# 11. 一个重要规则：RX callback 里不要干重活

CAN callback 里暂时只允许：

```text
复制数据
更新时间
放 msgq
改几个变量
```

不要：

```text
k_sleep
复杂日志
PID
malloc
new
大量浮点计算
保存 Flash
```

以后正式 driver 会进一步缩短 callback。

---

# 12. 第 3 阶段：M3508 raw demo

现在开始写第一台真正电机。

创建：

```text
samples/motor/dji_m3508_raw/
├── CMakeLists.txt
├── prj.conf
└── src/
    ├── main.cpp
    ├── dji_m3508_protocol.cpp
    └── dji_m3508_protocol.hpp
```

---

# 13. 这一阶段不允许做的事

暂时不要：

```text
DEVICE_DT_INST_DEFINE
motor.hpp
统一 Motor API
DM
LK
CAN manager
TX manager
heartbeat workqueue
```

只做：

```text
官方协议
+
CAN
+
一台 M3508
```

---

# 14. 从官方 C620 文档抄什么

你已经有 C620 官方文档。

需要整理出一个自己的表。

例如：

```text
M3508 / C620

CAN bitrate:
1 Mbps

反馈：
ID = 0x200 + ESC ID

控制：
ID 0x200
用于某一组电机

反馈数据：
byte 0~1 encoder
byte 2~3 speed
byte 4~5 current
byte 6 temperature

编码器：
0~8191

控制值：
-16384 ~ +16384
对应约 -20A ~ +20A
```

注意：

> **协议事实可以使用。**

但是不要照着某个第三方 C 文件逐行翻译。

---

# 15. 写自己的 `dji_m3508_protocol.hpp`

建议：

```cpp
#pragma once

#include <cstdint>
#include <zephyr/drivers/can.h>

namespace skywalker::motor::dji {

struct M3508Feedback {
    std::uint16_t encoder = 0;
    std::int16_t rpm = 0;
    std::int16_t current_raw = 0;
    std::uint8_t temperature = 0;
};

bool decodeM3508Feedback(
    const struct can_frame &frame,
    M3508Feedback &out);

void buildGroupCurrentFrame(
    struct can_frame &frame,
    std::uint16_t command_id,
    const std::int16_t current[4]);

}
```

---

# 16. 为什么 protocol 用 namespace

因为这样不会满工程出现：

```text
decode
encode
feedback
current
```

这些名字。

调用时：

```cpp
skywalker::motor::dji::decodeM3508Feedback(...)
```

非常清楚。

---

# 17. 写 `dji_m3508_protocol.cpp`

第一件事写一个 helper：

```cpp
#include "dji_m3508_protocol.hpp"

namespace skywalker::motor::dji {

static std::uint16_t readBe16(const std::uint8_t *p)
{
    return
        (static_cast<std::uint16_t>(p[0]) << 8) |
        static_cast<std::uint16_t>(p[1]);
}

}
```

然后 decode：

```cpp
bool decodeM3508Feedback(
    const struct can_frame &frame,
    M3508Feedback &out)
{
    if (frame.dlc != 8) {
        return false;
    }

    out.encoder =
        readBe16(&frame.data[0]);

    out.rpm =
        static_cast<std::int16_t>(
            readBe16(&frame.data[2]));

    out.current_raw =
        static_cast<std::int16_t>(
            readBe16(&frame.data[4]));

    out.temperature =
        frame.data[6];

    return true;
}
```

---

# 18. 为什么单独写 `readBe16`

不要反复写：

```cpp
(frame.data[0] << 8) | frame.data[1]
```

因为：

```text
容易漏 cast
容易写错字节
以后所有协议都重复
```

helper 函数更容易检查。

---

# 19. TX frame pack

DJI 电机是典型的：

```text
一帧
包含多台电机 command
```

所以写：

```cpp
void buildGroupCurrentFrame(
    struct can_frame &frame,
    std::uint16_t command_id,
    const std::int16_t current[4])
{
    frame.id = command_id;
    frame.flags = 0;
    frame.dlc = 8;

    for (int i = 0; i < 4; ++i) {
        frame.data[i * 2] =
            static_cast<std::uint8_t>(
                (current[i] >> 8) & 0xFF);

        frame.data[i * 2 + 1] =
            static_cast<std::uint8_t>(
                current[i] & 0xFF);
    }
}
```

第一版不追求模板。

---

# 20. M3508 raw demo 的 runtime data

`main.cpp` 里：

```cpp
struct MotorRuntime {
    skywalker::motor::dji::M3508Feedback feedback{};
    std::int64_t last_rx_ms = 0;
};

static MotorRuntime motor;
```

不允许：

```cpp
new MotorRuntime
```

---

# 21. RX callback

```cpp
static void rxCallback(const struct device *,
                       struct can_frame *frame,
                       void *user_data)
{
    auto *runtime =
        static_cast<MotorRuntime *>(user_data);

    if (runtime == nullptr ||
        frame == nullptr) {
        return;
    }

    if (skywalker::motor::dji::
        decodeM3508Feedback(
            *frame,
            runtime->feedback)) {

        runtime->last_rx_ms =
            k_uptime_get();
    }
}
```

这里第一次看到：

```cpp
static_cast
```

它只是把：

```text
void *
```

安全、明确地转成：

```text
MotorRuntime *
```

---

# 22. 为什么 callback 不写成成员函数

你可能想写：

```cpp
class M3508 {
    void rxCallback(...);
};
```

但是 Zephyr 要的是：

```text
普通 C 函数指针
```

非 static 成员函数内部隐藏了：

```text
this
```

所以不能直接传。

正确模式以后是：

```cpp
static void callback(..., void *user)
{
    auto *self =
        static_cast<M3508 *>(user);

    self->handleFrame(...);
}
```

这个模式叫：

```text
trampoline
```

记住这个词就行。

---

# 23. 第一台 M3508 控制

第一版：

```cpp
std::int16_t current[4] = {
    300,
    0,
    0,
    0
};
```

不要一开始：

```text
5000
10000
16384
```

先很小。

每：

```text
20 ms
```

发送一次。

也就是：

```text
50 Hz
```

---

# 24. raw demo 验收

至少要看到：

```text
encoder 有变化
rpm 合理
temperature 合理
current 有变化
```

电机：

```text
轻微稳定转动
```

断掉 TX：

```text
电机停止输出
```

这一步不过：

> 不允许继续写正式 driver。

---

# 25. 第 4 阶段：正式建立 Motor 公共 API

现在建立：

```text
include/skywalker/motor/motor.hpp
```

---

# 26. Feedback

推荐：

```cpp
#pragma once

#include <cstdint>
#include <zephyr/device.h>

namespace skywalker::motor {

enum class State : std::uint8_t {
    Offline = 0,
    Ready,
    Fault,
};

enum FeedbackValid : std::uint32_t {
    FeedbackPosition    = 1u << 0,
    FeedbackVelocity    = 1u << 1,
    FeedbackTorque      = 1u << 2,
    FeedbackTemperature = 1u << 3,
};

struct Feedback {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;

    std::uint32_t valid = 0;
    std::uint64_t timestamp_ms = 0;
};

}
```

---

# 27. 为什么公共层统一成 SI 单位

统一：

```text
位置 rad
速度 rad/s
扭矩 N·m
温度 °C
```

不要让上层同时看到：

```text
rpm
encoder raw
0.01 dps
12-bit torque
```

raw 数据留在 vendor driver 内部。

---

# 28. Zephyr 风格 API 表

我们不用大量 virtual。

定义：

```cpp
namespace skywalker::motor {

struct Api {
    int (*enable)(
        const struct device *dev);

    int (*disable)(
        const struct device *dev);

    int (*set_torque)(
        const struct device *dev,
        float torque_nm);

    int (*read_feedback)(
        const struct device *dev,
        Feedback *out);

    State (*get_state)(
        const struct device *dev);
};

}
```

这本质上还是：

```text
C function pointer table
```

但数据类型可以用 C++。

这样最适合 Zephyr。

---

# 29. wrapper

```cpp
inline int setTorque(
    const struct device *dev,
    float torque_nm)
{
    if (dev == nullptr ||
        dev->api == nullptr) {
        return -EINVAL;
    }

    const auto *api =
        static_cast<const Api *>(dev->api);

    if (api->set_torque == nullptr) {
        return -ENOSYS;
    }

    return api->set_torque(
        dev,
        torque_nm);
}
```

以后应用只调用：

```cpp
skywalker::motor::setTorque(...)
```

---

# 30. 为什么不用 `virtual`

当然可以写：

```cpp
class Motor {
public:
    virtual int setTorque(float) = 0;
};
```

但第一版不建议。

因为会增加：

```text
vtable
对象构造
对象与 Zephyr device 如何绑定
生命周期
静态初始化顺序
```

现在没有必要。

我们要的是：

> C++ 类型系统 + Zephyr 原生 device model。

---

# 31. 第 5 阶段：建立 M3508 DTS binding

创建：

```text
dts/bindings/motor/dji,m3508.yaml
```

推荐：

```yaml
description: DJI M3508 motor with CAN ESC

compatible: "dji,m3508"

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
    required: true
```

---

# 32. 为什么有 `command-slot`

因为 DJI 不是：

```text
一个 motor
→
一个 CAN TX frame
```

而是：

```text
多个 motor
→
共享 group frame
```

所以每个 motor 必须知道：

```text
自己在 group frame 的第几个 16-bit slot
```

---

# 33. overlay 示例

第一台：

```dts
/ {
    aliases {
        motor0 = &m3508_1;
    };

    m3508_1: motor-1 {
        compatible = "dji,m3508";
        status = "okay";

        can-bus = <&can1>;

        feedback-id = <0x201>;
        command-id = <0x200>;

        command-slot = <0>;

        gear-ratio = <19>;
    };
};
```

---

# 34. 第 6 阶段：M3508 Zephyr driver

建立：

```text
drivers/motor/dji/
├── dji_m3508.cpp
├── dji_protocol.cpp
└── CMakeLists.txt
```

---

# 35. config / data 必须分开

## Config

永远不变。

```cpp
struct M3508Config {
    const struct device *can;

    std::uint16_t feedback_id;
    std::uint16_t command_id;

    std::uint8_t command_slot;

    float gear_ratio;
};
```

## Data

运行时变化：

```cpp
struct M3508Data {
    skywalker::motor::Feedback feedback{};

    std::int16_t command_raw = 0;

    std::uint64_t last_rx_ms = 0;

    struct k_spinlock lock{};

    int rx_filter_id = -1;
};
```

---

# 36. 永远记住

```text
config = 常量
data   = 状态
```

例如：

```text
CAN ID       config
gear ratio   config

当前速度     data
最后反馈时间 data
当前命令     data
```

---

# 37. driver callback trampoline

可以写：

```cpp
static void m3508RxCallback(
    const struct device *,
    struct can_frame *frame,
    void *user_data)
{
    auto *dev =
        static_cast<const struct device *>(
            user_data);

    if (dev == nullptr ||
        frame == nullptr) {
        return;
    }

    auto *data =
        static_cast<M3508Data *>(
            dev->data);

    const auto *cfg =
        static_cast<const M3508Config *>(
            dev->config);

    // parse
}
```

---

# 38. callback 里使用 spinlock

只保护：

```text
feedback copy
timestamp
```

例如：

```cpp
k_spinlock_key_t key =
    k_spin_lock(&data->lock);

data->feedback = new_feedback;
data->last_rx_ms = now;

k_spin_unlock(
    &data->lock,
    key);
```

不要拿着锁：

```cpp
LOG_INF(...)
can_send(...)
k_sleep(...)
```

---

# 39. readFeedback 应该复制

正确：

```cpp
static int readFeedback(
    const struct device *dev,
    skywalker::motor::Feedback *out)
{
    if (out == nullptr) {
        return -EINVAL;
    }

    auto *data =
        static_cast<M3508Data *>(
            dev->data);

    k_spinlock_key_t key =
        k_spin_lock(&data->lock);

    *out = data->feedback;

    k_spin_unlock(
        &data->lock,
        key);

    return 0;
}
```

不要：

```cpp
return &data->feedback;
```

原因：

```text
防止上层改内部状态
避免并发读一半
以后多核更安全
```

---

# 40. M3508 init 固定顺序

永远按：

```text
1. cfg/data 非空
2. CAN ready
3. can_start
4. 注册 filter
5. command = 0
6. feedback 清零
7. timestamp = 0
8. return 0
```

init 时绝对不要：

```text
给电机非零电流
转动电机
写参数
```

---

# 41. `DEVICE_DT_INST_DEFINE`

最底下会有 Zephyr 宏。

例如概念上：

```cpp
#define DT_DRV_COMPAT dji_m3508
```

然后：

```cpp
#define M3508_DEFINE(inst) \
    static M3508Data data_##inst; \
    static const M3508Config config_##inst = { \
        ... \
    }; \
    DEVICE_DT_INST_DEFINE( \
        inst, \
        m3508Init, \
        nullptr, \
        &data_##inst, \
        &config_##inst, \
        POST_KERNEL, \
        CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY, \
        &m3508_api);

DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)
```

这里宏很丑。

正常。

Zephyr 就是这样。

不要试图用复杂 C++ template 把它藏掉。

---

# 42. 第 7 阶段：解决 DJI group TX

这个阶段非常重要。

## 错误设计

```text
Motor1
自己发送 0x200

Motor2
自己发送 0x200

Motor3
自己发送 0x200
```

这会：

```text
互相覆盖
```

---

# 43. 正确设计

建立：

```cpp
class DjiTxGroup
```

它维护：

```cpp
std::int16_t command_[4];
```

每个 motor 的：

```cpp
setTorque()
```

只更新一个 slot。

真正的：

```cpp
can_send()
```

由 group 统一完成。

---

# 44. `DjiTxGroup` 第一版

可以：

```cpp
class DjiTxGroup {
public:
    int setSlot(
        std::uint8_t slot,
        std::int16_t value);

    int send();

private:
    const struct device *can_ = nullptr;

    std::uint16_t command_id_ = 0;

    std::int16_t command_[4] = {
        0, 0, 0, 0
    };

    struct k_spinlock lock_{};
};
```

第一版：

```text
不要线程
不要 workqueue
```

先由 sample：

```text
每 10~20 ms
调用 send()
```

跑通。

---

# 45. 第二版再做自动周期发送

只有前面通过以后，再考虑：

```text
k_work_delayable
或者
一个 TX thread
```

不要：

```text
一台 motor 一个 thread
```

推荐：

```text
一个 CAN bus
一个 TX scheduler
```

---

# 46. 第 8 阶段：heartbeat

第一版不要 workqueue。

RX callback：

```cpp
data->last_rx_ms =
    k_uptime_get();
```

查询：

```cpp
State getState(...)
{
    const auto now =
        k_uptime_get();

    if (data->last_rx_ms == 0) {
        return State::Offline;
    }

    if ((now - data->last_rx_ms) >
        CONFIG_SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS) {

        return State::Offline;
    }

    return State::Ready;
}
```

就够了。

---

# 47. Kconfig

新增：

```text
drivers/motor/Kconfig
```

建议：

```Kconfig
menuconfig SKYWALKER_MOTOR
    bool "SkyWalker motor subsystem"
    depends on CAN

if SKYWALKER_MOTOR

config SKYWALKER_MOTOR_DJI
    bool "DJI CAN motor support"
    default y

config SKYWALKER_MOTOR_DM
    bool "Damiao CAN motor support"

config SKYWALKER_MOTOR_LK
    bool "Lingkong CAN motor support"

config SKYWALKER_MOTOR_INIT_PRIORITY
    int "Motor driver init priority"
    default 90
    range 0 99

config SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS
    int "Motor heartbeat timeout"
    default 100
    range 10 5000

endif
```

---

# 48. drivers/Kconfig 接入

SkyWalker 原本：

```Kconfig
menu "Skywalker Device Drivers"

...
```

加：

```Kconfig
rsource "./motor/Kconfig"
```

---

# 49. CMake 接入

`drivers/CMakeLists.txt`：

```cmake
add_subdirectory_ifdef(
    CONFIG_SKYWALKER_MOTOR
    motor
)
```

`drivers/motor/CMakeLists.txt`：

```cmake
zephyr_library()

add_subdirectory_ifdef(
    CONFIG_SKYWALKER_MOTOR_DJI
    dji
)

add_subdirectory_ifdef(
    CONFIG_SKYWALKER_MOTOR_DM
    dm
)

add_subdirectory_ifdef(
    CONFIG_SKYWALKER_MOTOR_LK
    lk
)
```

---

# 50. 第 9 阶段：DM

M3508 跑稳定后才能开始 DM。

---

# 51. DM protocol 和 driver 必须分开

目录：

```text
drivers/motor/dm/
├── dm_protocol.cpp
├── dm_motor.cpp
└── CMakeLists.txt
```

header：

```text
include/skywalker/motor/dm_protocol.hpp
```

---

# 52. `DmProtocol` 只做数据转换

推荐：

```cpp
class DmProtocol {
public:
    struct Limits {
        float position_min;
        float position_max;

        float velocity_min;
        float velocity_max;

        float torque_min;
        float torque_max;

        float kp_min;
        float kp_max;

        float kd_min;
        float kd_max;
    };

    struct MitCommand {
        float position;
        float velocity;
        float kp;
        float kd;
        float torque;
    };

    static bool encodeMit(
        const MitCommand &cmd,
        const Limits &limits,
        struct can_frame &frame);

    static bool decodeFeedback(
        const struct can_frame &frame,
        const Limits &limits,
        skywalker::motor::Feedback &out);
};
```

---

# 53. DM 为什么很适合 C++

因为 MIT command 是：

```text
position
velocity
kp
kd
torque
```

如果用 C API：

```cpp
dm_mit(dev,
       0,
       0,
       1,
       0.2,
       0.5);
```

你很容易忘掉第几个参数是什么。

C++：

```cpp
DmProtocol::MitCommand cmd{
    .position = 0.0f,
    .velocity = 0.0f,
    .kp = 1.0f,
    .kd = 0.2f,
    .torque = 0.5f,
};
```

注意：

C++14 不支持所有 C++20 风格 designated initializer。

所以实际可以：

```cpp
DmProtocol::MitCommand cmd{};

cmd.position = 0.0f;
cmd.velocity = 0.0f;
cmd.kp = 1.0f;
cmd.kd = 0.2f;
cmd.torque = 0.5f;
```

---

# 54. DM 浮点 → uint helper

自己写：

```cpp
static std::uint16_t floatToUint(
    float x,
    float min,
    float max,
    unsigned bits)
{
    if (x < min) {
        x = min;
    }

    if (x > max) {
        x = max;
    }

    const float span =
        max - min;

    const auto max_value =
        (1u << bits) - 1u;

    const float normalized =
        (x - min) / span;

    return static_cast<std::uint16_t>(
        normalized *
        static_cast<float>(max_value));
}
```

不要复制第三方实现。

---

# 55. DM DTS 参数

DTS 不适合直接 float。

推荐：

```dts
position-limit-millirad = <12500>;
velocity-limit-millirad-s = <45000>;
torque-limit-millinewton-m = <18000>;
```

driver：

```cpp
float position_max =
    DT_INST_PROP(
        inst,
        position_limit_millirad)
    / 1000.0f;
```

一定把单位写进属性名。

---

# 56. DM vendor-specific API

通用：

```cpp
setTorque(...)
```

不够表达 MIT。

所以额外：

```cpp
namespace skywalker::motor::dm {

int setMit(
    const struct device *dev,
    const MitCommand &cmd);

int enable(
    const struct device *dev);

int disable(
    const struct device *dev);

}
```

这很正常。

不要强迫所有协议都塞进：

```text
Motor API
```

---

# 57. 第 10 阶段：LK

LK 最后做。

因为命令很多。

第一版只实现：

```text
enable
disable
torque
speed
basic feedback
```

不要做：

```text
PID 参数写入
编码器零点
各种位置模式
所有寄存器
```

---

# 58. LK 也拆 protocol / driver

```text
lk_protocol.cpp
    ↓
纯字节打包/解析

lk_motor.cpp
    ↓
device / CAN / heartbeat / state
```

以后增加命令只改：

```text
protocol
vendor API
```

公共 motor API 不需要经常变。

---

# 59. 什么时候需要 CAN RX router

第一版：

```text
每个 motor 自己 can_add_rx_filter()
```

完全可以。

等出现：

```text
filter 数量多
CAN bus 上设备很多
想统一统计负载
想统一 RX thread
```

再做：

```text
motor_can_router.cpp
```

不要提前写。

---

# 60. CAN RX router 的最终模型

```text
FDCAN ISR
   ↓
最小 callback
   ↓
k_msgq
   ↓
RX worker thread
   ↓
listeners
   ├── DJI
   ├── DM
   └── LK
```

---

# 61. router API 可以很简单

```cpp
using Listener =
    void (*)(
        const struct can_frame &frame,
        void *user_data);

int subscribe(
    std::uint32_t id,
    std::uint32_t mask,
    Listener listener,
    void *user_data);
```

内部：

```text
固定数组
```

不要：

```text
std::vector
std::map
动态分配
```

---

# 62. 数据结构选择规则

MCU driver 优先：

```text
固定数组
struct
class
enum
```

最后才考虑：

```text
STL dynamic container
```

比如：

```cpp
Listener listeners[32];
```

比：

```cpp
std::vector<Listener>
```

更适合第一版。

---

# 63. 日志规则

## 可以

初始化：

```cpp
LOG_INF("M3508 ready");
```

掉线：

```cpp
LOG_WRN("motor offline");
```

CAN 错误：

```cpp
LOG_ERR("CAN send failed: %d", ret);
```

## 不要

每帧：

```cpp
LOG_INF(
    "encoder=%d speed=%d ...");
```

1kHz log 会把系统搞得很难看。

实时数据：

```text
100ms
200ms
500ms
```

打印一次即可。

---

# 64. 不要使用 `std::cout`

不要：

```cpp
std::cout << ...
```

继续用：

```text
LOG_INF
LOG_DBG
printk
```

和 Zephyr 调试体系保持一致。

---

# 65. C++ static object 的坑

第一版不要：

```cpp
static DjiMotor motor(can, ...);
```

如果 constructor 里面：

```text
访问 Zephyr device
start CAN
注册 callback
```

可能遇到：

```text
C++ static constructor
和
Zephyr device initialization
谁先执行
```

的问题。

正确：

```text
对象数据放 dev->data
初始化放 DEVICE init function
```

---

# 66. 可以有 class，但 constructor 要简单

例如：

```cpp
class DmProtocol {
public:
    DmProtocol() = default;
};
```

没问题。

不要 constructor：

```cpp
DmMotor::DmMotor()
{
    can_start(...);
}
```

硬件初始化交给：

```text
Zephyr init
```

---

# 67. 如果以后想用 RAII lock

可以自己做非常小的：

```cpp
class SpinLockGuard {
public:
    explicit SpinLockGuard(
        k_spinlock &lock)
        : lock_(lock),
          key_(k_spin_lock(&lock_))
    {
    }

    ~SpinLockGuard()
    {
        k_spin_unlock(
            &lock_,
            key_);
    }

private:
    k_spinlock &lock_;
    k_spinlock_key_t key_;
};
```

使用：

```cpp
{
    SpinLockGuard guard(data->lock);

    data->feedback = fb;
}
```

作用域结束自动解锁。

这个属于：

```text
适合嵌入式的 C++
```

但第一版可以先不用。

---

# 68. 编译错误排查顺序

如果：

```text
CONFIG_SKYWALKER_MOTOR undefined
```

查：

```text
drivers/Kconfig
是否 rsource motor/Kconfig
```

---

如果：

```text
源文件没参与编译
```

查：

```text
drivers/CMakeLists.txt
drivers/motor/CMakeLists.txt
```

---

如果：

```text
compatible 找不到 binding
```

查：

```text
dts/bindings/motor/
yaml 文件
compatible 拼写
```

---

如果：

```text
__device_dts_ord_xxx
```

查：

```text
引用的 device 有没有 status = "okay"
driver 有没有 DEVICE_DT_INST_DEFINE
依赖设备是否 ready
```

---

如果：

```text
头文件找不到
```

优先 include：

```cpp
#include <skywalker/motor/motor.hpp>
```

然后确认 SkyWalker module 已经：

```cmake
zephyr_include_directories(include)
```

---

# 69. 运行错误排查顺序

电机不动：

```text
1. 电机供电
2. CANH/CANL
3. 共地
4. 终端电阻
5. CAN bitrate
6. 电机 CAN ID
7. TX CAN ID
8. 帧 DLC
9. 字节序
10. command 是否太小/为零
```

不要一开始怀疑：

```text
Zephyr scheduler
C++ compiler
模板
```

大多数时候是：

```text
接线 / ID / 协议
```

---

# 70. 如果有反馈但控制不了

说明：

```text
RX 通
TX 或协议有问题
```

查：

```text
command ID
command slot
大端/小端
控制范围
电调模式
电机是否 enable
```

---

# 71. 如果控制正常但反馈乱

查：

```text
反馈 ID
字节序
signed / unsigned
frame DLC
单位转换
```

尤其：

```cpp
std::uint16_t
std::int16_t
```

不要混。

---

# 72. signed 转换注意

正确：

```cpp
std::uint16_t raw =
    readBe16(...);

std::int16_t speed =
    static_cast<std::int16_t>(raw);
```

不要：

```cpp
int speed =
    frame.data[2] << 8 |
    frame.data[3];
```

然后忘了符号。

---

# 73. 最重要的测试策略

每加一个功能：

```text
compile
↓
flash
↓
只测这个功能
↓
git commit
```

不要：

```text
M3508
+
DM
+
router
+
heartbeat
+
shell
+
PID
```

一起写完再第一次编译。

---

# 74. 推荐 Git commit 顺序

```text
motor: add CAN smoke sample

motor: add standalone M3508 codec

motor: add raw M3508 sample

motor: add generic motor API

motor: add M3508 devicetree binding

motor: add M3508 Zephyr device driver

motor: add DJI grouped TX support

motor: add motor heartbeat

motor: add DM MIT protocol

motor: add DM device driver

motor: add LK basic protocol

motor: add LK device driver

motor: add shared CAN RX router
```

---

# 75. 第一版 Motor API 不要太多

只要：

```text
enable
disable
setTorque
readFeedback
getState
```

够了。

以后需要再加：

```text
setVelocity
setPosition
stop
clearFault
```

---

# 76. 第一版 Feedback 不要太多

只要：

```text
position
velocity
torque
temperature
timestamp
valid mask
```

不要一开始：

```text
bus voltage
phase A
phase B
phase C
MOS temp
raw encoder
fault subcode
multi-turn raw
```

vendor-specific 需要时再扩展。

---

# 77. 什么时候可以把 raw 数据暴露出去

如果调试需要：

可以额外：

```cpp
namespace dji {

struct RawFeedback {
    ...
};

int readRawFeedback(...);
}
```

而不是污染通用：

```text
Feedback
```

---

# 78. 你的官方文档分别怎么用

## C620

用于：

```text
M3508
CAN ID
控制分组
电流范围
反馈字段
编码器
温度
```

这是 DJI 第一阶段最重要资料。

## C610 + M2006

等 M3508 driver 完成以后：

```text
复用 DJI protocol 框架
添加 M2006
```

不要先写。

## GM6020

单独处理：

```text
控制 group ID
反馈 ID
控制范围
```

不要假设与 M3508 完全相同。

## DM J4310

用于：

```text
MIT
enable
disable
zero
feedback decode
position-velocity
velocity mode
```

非常适合第二个 vendor backend。

## LK

当前还需要：

```text
官方协议文档
```

再正式写。

---

# 79. 你现在要执行的唯一任务

现在不要写正式 motor driver。

只做：

```text
samples/motor/can_smoke
```

完成标准：

```text
能 build
能 flash
CAN ready
能收到一帧
```

然后做：

```text
samples/motor/dji_m3508_raw
```

完成标准：

```text
M3508 有反馈
M3508 小电流转动
```

只有这两个完成之后：

```text
开始 drivers/motor/
```

---

# 80. 给本地小模型的强制规则

如果你把这份文档交给本地模型，可以直接附加下面这段：

```text
你是一个执行能力有限的小模型。

严格遵守以下规则：

1. 一次只完成当前阶段，不提前实现下一阶段。
2. 不要重构用户现有 SkyWalker 代码。
3. 不复制 Breeze 源码。
4. 电机协议字段只从官方文档提取。
5. 新代码使用 C++14。
6. 禁止 exceptions、RTTI、new/delete、iostream、复杂 STL。
7. Zephyr device glue 保持普通函数 + DEVICE_DT_INST_DEFINE。
8. 所有 runtime object 静态分配。
9. CAN RX callback 内不 sleep、不做 PID、不进行长时间日志。
10. 每一步完成后先保证 west build 通过。
11. 如果 build 失败，只修第一处真实错误。
12. 不要为了“优雅”提前添加 manager、factory、template framework。
13. M3508 raw demo 没跑通前，不允许创建正式 motor driver。
14. M3508 正式 driver 没跑通前，不允许开始 DM。
15. DM 没跑通前，不允许开始 LK。
```

---

# 81. 一句话总结这套风格

我们不是写：

```text
桌面软件式 C++
```

也不是写：

```text
纯 C 驱动
```

而是写：

```text
Zephyr C Driver Model
+
C++14 类型系统
+
简单 class / namespace / struct
+
静态内存
+
errno
+
官方协议
```

也就是：

> **C with classes，但保留真正有价值的 C++ 类型安全和代码组织能力。**

这最适合当前 SkyWalker 电机子系统。
