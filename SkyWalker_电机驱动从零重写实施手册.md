# SkyWalker M3508 C++ 驱动：逐步完整代码手册 v3

> 适用仓库：`SHM-white/SkyWalker_General_Embedded_Code`
> 目标平台：Zephyr 4.4.x / `dm_mc02/stm32h723xx`
> 风格：C++14 + Zephyr Device Model
> 本文按你当前 GitHub `main` 分支实际状态编写。
>
> **执行规则：一次只做一个步骤。每一步都给完整文件内容。不要同时修改后面的步骤。**

---

# 0. 你现在实际做到哪里了

当前仓库已经有：

```text
drivers/motor/
├── CMakeLists.txt
├── Kconfig
├── motor_api.cpp
├── motor_can_router.cpp
└── dji/
    ├── CMakeLists.txt
    ├── dji_m3508.cpp
    ├── dji_protocol.cpp
    └── dji_tx_group.cpp

include/drivers/motor/
├── motor.hpp
├── dji_protocol.hpp
└── dji_tx_group.hpp

dts/bindings/motor/
└── dji,m3508.yaml
```

并且 `samples/motor/can_smoke`、`samples/motor/dji_m3508_raw` 已经存在。`dji_m3508_raw` 已经完成 CAN start、RX、M3508 feedback decode、0x200 group frame encode 和 `can_send()`，所以从现在开始不要再改 raw demo，把它当“协议基准样例”。

---

# 1. 当前正式 driver 有哪些问题

你现在的 `dji_m3508.cpp` 还不能作为正式驱动使用，主要有：

```text
1. DEVICE_DT_INST_DEFINE(...) 还是占位符
2. m3508RxCallback() 没有解析 CAN frame
3. callback 反而调用了 m3508ReadFeedback()
4. m3508Init() 没注册 CAN RX filter
5. include 路径有错误/不统一
6. motor Kconfig 没真正接入 drivers/Kconfig
7. CONFIG_SKYWALKER_DRIVER_MOTOR 没有定义
8. Kconfig 中写了 config CONFIG_xxx
9. Kconfig 没有 uint64_t 类型
10. motor.hpp 只有 setTorque wrapper
11. dji_tx_group.hpp/.cpp 还是空文件
```

所以不要在当前 `dji_m3508.cpp` 上继续零碎补洞，按下面顺序整理。

---

# 2. 第一步：先修 Kconfig 接线

## 2.1 完整替换 `drivers/Kconfig`

```Kconfig
menu "Skywalker Device Drivers"

config SKYWALKER_DRIVER_PID
    bool "Skywalker PID Controller"
    help
        Enable the PID Controller driver.

config SKYWALKER_DRIVER_KALMAN_FILTER
    bool "Skywalker Kalman Filter"
    help
        Enable the Kalman Filter driver.

config SKYWALKER_DRIVER_IMU
    bool "Skywalker IMU Driver"
    select CMSIS_DSP_FASTMATH
    help
        Enable the IMU attitude estimation driver.
        Select estimation method via DT property "estimator".

config SKYWALKER_DRIVER_MOTOR
    bool "Skywalker Motor Subsystem"
    depends on CAN
    help
        Enable Skywalker motor drivers.

if SKYWALKER_DRIVER_MOTOR

rsource "motor/Kconfig"

endif

endmenu
```

你的 `drivers/CMakeLists.txt` 已经写了：

```cmake
add_subdirectory_ifdef(CONFIG_SKYWALKER_DRIVER_MOTOR motor)
```

所以必须存在 `SKYWALKER_DRIVER_MOTOR` 这个 Kconfig symbol。

注意：Kconfig 中写 `config SKYWALKER_DRIVER_MOTOR`，代码/`prj.conf` 中才写 `CONFIG_SKYWALKER_DRIVER_MOTOR`。

---

# 3. 第二步：完整替换 `drivers/motor/Kconfig`

```Kconfig
config SKYWALKER_MOTOR_DJI
    bool "DJI CAN motors"
    default y
    help
        Enable DJI CAN motor drivers such as M3508/C620.

config SKYWALKER_MOTOR_INIT_PRIORITY
    int "Motor driver init priority"
    default 90
    range 0 99

config SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS
    int "Motor feedback timeout in milliseconds"
    default 100
    range 10 5000
```

现在先不要写 DM/LK/CAN router。

---

# 4. 第三步：修 CMake

`drivers/CMakeLists.txt` 中这行已经正确，不改：

```cmake
add_subdirectory_ifdef(CONFIG_SKYWALKER_DRIVER_MOTOR motor)
```

完整替换 `drivers/motor/CMakeLists.txt`：

```cmake
add_subdirectory_ifdef(CONFIG_SKYWALKER_MOTOR_DJI dji)
```

暂时完整替换 `drivers/motor/dji/CMakeLists.txt`：

```cmake
zephyr_library()

zephyr_library_sources(
    dji_protocol.cpp
    dji_m3508.cpp
)
```

现在先不编译 `dji_tx_group.cpp`，等 RX driver 通过再加。

---

# 5. 第四步：补完公共 `motor.hpp`

文件：`include/drivers/motor/motor.hpp`

完整替换为：

```cpp
#pragma once

#include <cstdint>
#include <errno.h>
#include <zephyr/device.h>

namespace skywalker::motor
{

enum class State : std::uint8_t
{
    Offline = 0,
    Ready,
    Fault,
};

enum FeedbackValid : std::uint32_t
{
    FeedbackPosition    = 1u << 0,
    FeedbackVelocity    = 1u << 1,
    FeedbackTorque      = 1u << 2,
    FeedbackTemperature = 1u << 3,
};

struct Feedback
{
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;

    std::uint32_t valid = 0;
    std::uint64_t timestamp_ms = 0;
};

struct Api
{
    int (*enable)(const struct device *dev);
    int (*disable)(const struct device *dev);
    int (*set_torque)(const struct device *dev, float torque_nm);
    int (*read_feedback)(const struct device *dev, Feedback *out);
    State (*get_state)(const struct device *dev);
};

inline int enable(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const auto *api = static_cast<const Api *>(dev->api);
    if (api->enable == nullptr) return -ENOSYS;
    return api->enable(dev);
}

inline int disable(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const auto *api = static_cast<const Api *>(dev->api);
    if (api->disable == nullptr) return -ENOSYS;
    return api->disable(dev);
}

inline int setTorque(const struct device *dev, float torque_nm)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const auto *api = static_cast<const Api *>(dev->api);
    if (api->set_torque == nullptr) return -ENOSYS;
    return api->set_torque(dev, torque_nm);
}

inline int readFeedback(const struct device *dev, Feedback &out)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const auto *api = static_cast<const Api *>(dev->api);
    if (api->read_feedback == nullptr) return -ENOSYS;
    return api->read_feedback(dev, &out);
}

inline State getState(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) return State::Offline;
    const auto *api = static_cast<const Api *>(dev->api);
    if (api->get_state == nullptr) return State::Offline;
    return api->get_state(dev);
}

} // namespace skywalker::motor
```

它只负责：应用层 → 统一 motor API → 具体 driver。这里不应该出现 CAN 协议。

---

# 6. 第五步：确认 DJI protocol

## 6.1 完整 `include/drivers/motor/dji_protocol.hpp`

```cpp
#pragma once

#include <cstdint>
#include <zephyr/drivers/can.h>

namespace skywalker::motor::dji
{

struct M3508Feedback
{
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

} // namespace skywalker::motor::dji
```

## 6.2 完整 `drivers/motor/dji/dji_protocol.cpp`

```cpp
#include <drivers/motor/dji_protocol.hpp>

namespace skywalker::motor::dji
{

static std::uint16_t readBe16(const std::uint8_t *p)
{
    return
        (static_cast<std::uint16_t>(p[0]) << 8) |
        static_cast<std::uint16_t>(p[1]);
}

bool decodeM3508Feedback(
    const struct can_frame &frame,
    M3508Feedback &out)
{
    if (frame.dlc != 8) return false;

    out.encoder = readBe16(&frame.data[0]);
    out.rpm = static_cast<std::int16_t>(readBe16(&frame.data[2]));
    out.current_raw = static_cast<std::int16_t>(readBe16(&frame.data[4]));
    out.temperature = frame.data[6];

    return true;
}

void buildGroupCurrentFrame(
    struct can_frame &frame,
    std::uint16_t command_id,
    const std::int16_t current[4])
{
    frame.id = command_id;
    frame.flags = 0;
    frame.dlc = 8;

    for (int i = 0; i < 4; ++i)
    {
        frame.data[i * 2] =
            static_cast<std::uint8_t>((current[i] >> 8) & 0xFF);

        frame.data[i * 2 + 1] =
            static_cast<std::uint8_t>(current[i] & 0xFF);
    }
}

} // namespace skywalker::motor::dji
```

以后 `CAN frame ↔ M3508 原始协议` 全部只在这里实现。

---

# 7. 第六步：确认 DTS binding

文件：`dts/bindings/motor/dji,m3508.yaml`

```yaml
description: DJI M3508 motor with C620 CAN ESC

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

# 8. 第七步：理解 DTS → driver

假设 overlay：

```dts
/ {
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

`compatible = "dji,m3508"` 对应：

```cpp
#define DT_DRV_COMPAT dji_m3508
```

`DT_INST_PROP(inst, feedback_id)` 读取 `feedback-id`；`DT_INST_PHANDLE(inst, can_bus)` 读取 `can-bus`。

`DEVICE_DT_INST_DEFINE()` 最后把：

```text
DTS 节点
+ M3508Config
+ M3508Data
+ m3508Init()
+ m3508_api
```

合成一个 Zephyr `struct device`。

---

# 9. 第八步：完整实现 M3508 RX driver

文件：`drivers/motor/dji/dji_m3508.cpp`

完整替换：

```cpp
#include <cstdint>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <drivers/motor/motor.hpp>
#include <drivers/motor/dji_protocol.hpp>

#define DT_DRV_COMPAT dji_m3508

namespace
{

constexpr float TWO_PI = 6.28318530717958647692f;

struct M3508Config
{
    const struct device *can;
    std::uint16_t feedback_id;
    std::uint16_t command_id;
    std::uint8_t command_slot;
    float gear_ratio;
};

struct M3508Data
{
    skywalker::motor::Feedback feedback{};
    skywalker::motor::dji::M3508Feedback raw_feedback{};
    std::int16_t command_raw = 0;
    std::uint64_t last_rx_ms = 0;
    struct k_spinlock lock{};
    int rx_filter_id = -1;
};

static void m3508RxCallback(
    const struct device *can_dev,
    struct can_frame *frame,
    void *user_data)
{
    ARG_UNUSED(can_dev);

    if (frame == nullptr || user_data == nullptr) return;

    const auto *dev =
        static_cast<const struct device *>(user_data);

    auto *data =
        static_cast<M3508Data *>(dev->data);

    const auto *cfg =
        static_cast<const M3508Config *>(dev->config);

    skywalker::motor::dji::M3508Feedback raw{};

    if (!skywalker::motor::dji::decodeM3508Feedback(*frame, raw)) {
        return;
    }

    skywalker::motor::Feedback feedback{};

    if (cfg->gear_ratio > 0.0f)
    {
        feedback.velocity_rad_s =
            static_cast<float>(raw.rpm) *
            (TWO_PI / 60.0f) /
            cfg->gear_ratio;

        feedback.valid |=
            skywalker::motor::FeedbackVelocity;
    }

    feedback.temperature_c =
        static_cast<float>(raw.temperature);

    feedback.valid |=
        skywalker::motor::FeedbackTemperature;

    const auto now =
        static_cast<std::uint64_t>(k_uptime_get());

    feedback.timestamp_ms = now;

    k_spinlock_key_t key =
        k_spin_lock(&data->lock);

    data->raw_feedback = raw;
    data->feedback = feedback;
    data->last_rx_ms = now;

    k_spin_unlock(&data->lock, key);
}

static int m3508Init(const struct device *dev)
{
    if (dev == nullptr) return -EINVAL;

    auto *data =
        static_cast<M3508Data *>(dev->data);

    const auto *cfg =
        static_cast<const M3508Config *>(dev->config);

    if (data == nullptr || cfg == nullptr || cfg->can == nullptr) {
        return -EINVAL;
    }

    if (!device_is_ready(cfg->can)) {
        return -ENODEV;
    }

    int ret = can_start(cfg->can);

    if (ret < 0 && ret != -EALREADY) {
        return ret;
    }

    struct can_filter filter{};
    filter.id = cfg->feedback_id;
    filter.mask = CAN_STD_ID_MASK;
    filter.flags = 0;

    ret = can_add_rx_filter(
        cfg->can,
        m3508RxCallback,
        const_cast<struct device *>(dev),
        &filter);

    if (ret < 0) {
        return ret;
    }

    data->rx_filter_id = ret;
    data->command_raw = 0;
    data->last_rx_ms = 0;
    data->feedback = {};
    data->raw_feedback = {};

    return 0;
}

static int m3508Enable(const struct device *dev)
{
    if (dev == nullptr) return -EINVAL;
    return 0;
}

static int m3508Disable(const struct device *dev)
{
    if (dev == nullptr) return -EINVAL;

    auto *data =
        static_cast<M3508Data *>(dev->data);

    k_spinlock_key_t key =
        k_spin_lock(&data->lock);

    data->command_raw = 0;

    k_spin_unlock(&data->lock, key);
    return 0;
}

static int m3508SetTorque(
    const struct device *dev,
    float torque_nm)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(torque_nm);

    // 暂不伪造 N*m -> raw current 换算
    return -ENOSYS;
}

static int m3508ReadFeedback(
    const struct device *dev,
    skywalker::motor::Feedback *out)
{
    if (dev == nullptr || out == nullptr) {
        return -EINVAL;
    }

    auto *data =
        static_cast<M3508Data *>(dev->data);

    k_spinlock_key_t key =
        k_spin_lock(&data->lock);

    *out = data->feedback;

    k_spin_unlock(&data->lock, key);
    return 0;
}

static skywalker::motor::State
m3508GetState(const struct device *dev)
{
    if (dev == nullptr) {
        return skywalker::motor::State::Offline;
    }

    auto *data =
        static_cast<M3508Data *>(dev->data);

    std::uint64_t last_rx_ms = 0;

    k_spinlock_key_t key =
        k_spin_lock(&data->lock);

    last_rx_ms = data->last_rx_ms;

    k_spin_unlock(&data->lock, key);

    if (last_rx_ms == 0) {
        return skywalker::motor::State::Offline;
    }

    const auto now =
        static_cast<std::uint64_t>(k_uptime_get());

    if ((now - last_rx_ms) >
        CONFIG_SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS) {
        return skywalker::motor::State::Offline;
    }

    return skywalker::motor::State::Ready;
}

static const skywalker::motor::Api m3508_api = {
    m3508Enable,
    m3508Disable,
    m3508SetTorque,
    m3508ReadFeedback,
    m3508GetState,
};

} // namespace

#define M3508_DEFINE(inst)                                          static M3508Data m3508_data_##inst;                                                                                           static const M3508Config m3508_config_##inst = {                   DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                  static_cast<std::uint16_t>(DT_INST_PROP(inst, feedback_id)),         static_cast<std::uint16_t>(DT_INST_PROP(inst, command_id)),          static_cast<std::uint8_t>(DT_INST_PROP(inst, command_slot)),         static_cast<float>(DT_INST_PROP(inst, gear_ratio)),         };                                                                                                                            DEVICE_DT_INST_DEFINE(                                             inst,                                                          m3508Init,                                                     nullptr,                                                       &m3508_data_##inst,                                            &m3508_config_##inst,                                          POST_KERNEL,                                                   CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY,                          &m3508_api);

DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)
```

这一版先只做 RX。`position_rad` 暂时不标有效，因为单圈 encoder 还没做多圈跟踪；`torque_nm` 暂时不标有效，因为还没有可靠 Nm 模型。

---

# 10. 三个最难的宏

```cpp
#define DT_DRV_COMPAT dji_m3508
```

表示当前 `.cpp` 服务 `compatible = "dji,m3508"`。

```cpp
#define M3508_DEFINE(inst)
```

表示“造一台 M3508”的模板。

```cpp
DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)
```

概念上等价于：

```cpp
M3508_DEFINE(0)
M3508_DEFINE(1)
...
```

每次都会生成：一份 `M3508Config`、一份 `M3508Data`、一个 Zephyr `struct device`。

---

# 11. 第九步：RX-only 正式 driver 测试 sample

新建：

```text
samples/motor/dji_m3508_driver/
├── CMakeLists.txt
├── prj.conf
├── app.overlay
└── src/main.cpp
```

## `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dji_m3508_driver)

target_sources(app PRIVATE src/main.cpp)
```

## `prj.conf`

```ini
CONFIG_CPP=y
CONFIG_CAN=y

CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
CONFIG_SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS=100
```

## `app.overlay`

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

这里假设 C620 ID=1。

## `src/main.cpp`

```cpp
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/motor.hpp>

LOG_MODULE_REGISTER(m3508_driver_test, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

int main()
{
    const struct device *motor =
        DEVICE_DT_GET(MOTOR0_NODE);

    if (!device_is_ready(motor)) {
        LOG_ERR("M3508 device not ready");
        return -ENODEV;
    }

    LOG_INF("M3508 device ready");

    while (true)
    {
        skywalker::motor::Feedback feedback{};

        int ret =
            skywalker::motor::readFeedback(motor, feedback);

        if (ret == 0)
        {
            const auto state =
                skywalker::motor::getState(motor);

            const int velocity_mrad_s =
                static_cast<int>(
                    feedback.velocity_rad_s * 1000.0f);

            const int temp_c =
                static_cast<int>(
                    feedback.temperature_c);

            printk(
                "state=%d vel=%d mrad/s temp=%d C ts=%llu\n",
                static_cast<int>(state),
                velocity_mrad_s,
                temp_c,
                static_cast<unsigned long long>(
                    feedback.timestamp_ms));
        }

        k_sleep(K_MSEC(500));
    }

    return 0;
}
```

---

# 12. 第十步：第一次正式 driver 编译

```bash
west build   -b dm_mc02/stm32h723xx   samples/motor/dji_m3508_driver   -p always   -- -DBOARD_ROOT=$PWD
```

目标只是 `BUILD SUCCESS`，还没要求电机转。

烧录后目标：

```text
M3508 device ready
state=1
timestamp 持续变化
temperature/velocity 有合理数据
```

如果一直 `state=0 timestamp=0`，说明 device 创建成功但 RX filter 没收到帧，此时只查反馈 ID/CAN 接线/C620 ID/can1，不要查 TX。

---

# 13. 第十一步：增加 M3508 专用 raw-current API

新建 `include/drivers/motor/dji_m3508.hpp`：

```cpp
#pragma once

#include <cstdint>
#include <zephyr/device.h>

#include <drivers/motor/dji_protocol.hpp>

namespace skywalker::motor::dji::m3508
{

int setCurrentRaw(
    const struct device *dev,
    std::int16_t current_raw);

int getCurrentRaw(
    const struct device *dev,
    std::int16_t &out);

int readRawFeedback(
    const struct device *dev,
    skywalker::motor::dji::M3508Feedback &out);

int getCommandInfo(
    const struct device *dev,
    std::uint16_t &command_id,
    std::uint8_t &command_slot);

} // namespace skywalker::motor::dji::m3508
```

在 `dji_m3508.cpp` 顶部增加：

```cpp
#include <drivers/motor/dji_m3508.hpp>
```

然后在匿名 namespace 结束之后、`M3508_DEFINE` 之前加入：

```cpp
namespace skywalker::motor::dji::m3508
{

int setCurrentRaw(
    const struct device *dev,
    std::int16_t current_raw)
{
    if (dev == nullptr) return -EINVAL;

    if (current_raw < -16384 ||
        current_raw > 16384) {
        return -ERANGE;
    }

    auto *data =
        static_cast<M3508Data *>(dev->data);

    k_spinlock_key_t key =
        k_spin_lock(&data->lock);

    data->command_raw = current_raw;

    k_spin_unlock(&data->lock, key);
    return 0;
}

int getCurrentRaw(
    const struct device *dev,
    std::int16_t &out)
{
    if (dev == nullptr) return -EINVAL;

    auto *data =
        static_cast<M3508Data *>(dev->data);

    k_spinlock_key_t key =
        k_spin_lock(&data->lock);

    out = data->command_raw;

    k_spin_unlock(&data->lock, key);
    return 0;
}

int readRawFeedback(
    const struct device *dev,
    skywalker::motor::dji::M3508Feedback &out)
{
    if (dev == nullptr) return -EINVAL;

    auto *data =
        static_cast<M3508Data *>(dev->data);

    k_spinlock_key_t key =
        k_spin_lock(&data->lock);

    out = data->raw_feedback;

    k_spin_unlock(&data->lock, key);
    return 0;
}

int getCommandInfo(
    const struct device *dev,
    std::uint16_t &command_id,
    std::uint8_t &command_slot)
{
    if (dev == nullptr) return -EINVAL;

    const auto *cfg =
        static_cast<const M3508Config *>(dev->config);

    command_id = cfg->command_id;
    command_slot = cfg->command_slot;

    return 0;
}

} // namespace skywalker::motor::dji::m3508
```

---

# 14. 第十二步：实现 DJI TxGroup

## 完整 `include/drivers/motor/dji_tx_group.hpp`

```cpp
#pragma once

#include <cstdint>
#include <zephyr/device.h>

namespace skywalker::motor::dji
{

class TxGroup
{
public:
    int init(
        const struct device *can,
        std::uint16_t command_id);

    int bindMotor(
        const struct device *motor);

    int send();

    void clear();

private:
    const struct device *can_ = nullptr;
    std::uint16_t command_id_ = 0;

    const struct device *motors_[4] = {
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    };
};

} // namespace skywalker::motor::dji
```

## 完整 `drivers/motor/dji/dji_tx_group.cpp`

```cpp
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#include <drivers/motor/dji_protocol.hpp>
#include <drivers/motor/dji_m3508.hpp>
#include <drivers/motor/dji_tx_group.hpp>

namespace skywalker::motor::dji
{

int TxGroup::init(
    const struct device *can,
    std::uint16_t command_id)
{
    if (can == nullptr) return -EINVAL;
    if (!device_is_ready(can)) return -ENODEV;

    can_ = can;
    command_id_ = command_id;

    for (auto &motor : motors_) {
        motor = nullptr;
    }

    return 0;
}

int TxGroup::bindMotor(
    const struct device *motor)
{
    if (motor == nullptr) return -EINVAL;

    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;

    int ret =
        skywalker::motor::dji::m3508::
            getCommandInfo(
                motor,
                command_id,
                command_slot);

    if (ret < 0) return ret;
    if (command_id != command_id_) return -EINVAL;
    if (command_slot >= 4) return -EINVAL;

    motors_[command_slot] = motor;
    return 0;
}

int TxGroup::send()
{
    if (can_ == nullptr) return -ENODEV;

    std::int16_t command[4] = {
        0, 0, 0, 0
    };

    for (int i = 0; i < 4; ++i)
    {
        if (motors_[i] == nullptr) continue;

        int ret =
            skywalker::motor::dji::m3508::
                getCurrentRaw(
                    motors_[i],
                    command[i]);

        if (ret < 0) return ret;
    }

    struct can_frame frame{};

    buildGroupCurrentFrame(
        frame,
        command_id_,
        command);

    return can_send(
        can_,
        &frame,
        K_MSEC(1),
        nullptr,
        nullptr);
}

void TxGroup::clear()
{
    for (auto *motor : motors_)
    {
        if (motor == nullptr) continue;

        skywalker::motor::dji::m3508::
            setCurrentRaw(motor, 0);
    }
}

} // namespace skywalker::motor::dji
```

然后把 `drivers/motor/dji/CMakeLists.txt` 改成：

```cmake
zephyr_library()

zephyr_library_sources(
    dji_protocol.cpp
    dji_m3508.cpp
    dji_tx_group.cpp
)
```

---

# 15. 第十三步：正式 driver sample 开始发送

把测试 sample 的 `src/main.cpp` 改成：

```cpp
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/motor.hpp>
#include <drivers/motor/dji_m3508.hpp>
#include <drivers/motor/dji_tx_group.hpp>

LOG_MODULE_REGISTER(m3508_driver_test, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

static skywalker::motor::dji::TxGroup tx_group;

int main()
{
    const struct device *motor =
        DEVICE_DT_GET(MOTOR0_NODE);

    const struct device *can =
        DEVICE_DT_GET(DT_NODELABEL(can1));

    if (!device_is_ready(motor)) {
        LOG_ERR("M3508 device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(can)) {
        LOG_ERR("CAN device not ready");
        return -ENODEV;
    }

    int ret = tx_group.init(can, 0x200);

    if (ret < 0) {
        LOG_ERR("TxGroup init failed: %d", ret);
        return ret;
    }

    ret = tx_group.bindMotor(motor);

    if (ret < 0) {
        LOG_ERR("TxGroup bind failed: %d", ret);
        return ret;
    }

    ret =
        skywalker::motor::dji::m3508::
            setCurrentRaw(motor, 300);

    if (ret < 0) {
        LOG_ERR("setCurrentRaw failed: %d", ret);
        return ret;
    }

    LOG_INF("M3508 driver ready");

    int print_counter = 0;

    while (true)
    {
        ret = tx_group.send();

        if (ret < 0) {
            LOG_ERR("CAN send failed: %d", ret);
        }

        ++print_counter;

        if (print_counter >= 25)
        {
            print_counter = 0;

            skywalker::motor::dji::M3508Feedback raw{};

            skywalker::motor::dji::m3508::
                readRawFeedback(motor, raw);

            const auto state =
                skywalker::motor::getState(motor);

            printk(
                "state=%d encoder=%u rpm=%d current=%d temp=%u\n",
                static_cast<int>(state),
                raw.encoder,
                raw.rpm,
                raw.current_raw,
                raw.temperature);
        }

        k_sleep(K_MSEC(20));
    }

    return 0;
}
```

TX 数据流：

```text
m3508::setCurrentRaw(motor, 300)
→ M3508Data.command_raw = 300
→ tx_group.send()
→ 读取四个 slot 的 command_raw
→ buildGroupCurrentFrame()
→ can_send()
```

这样多个 M3508 共享 `0x200` 时不会互相覆盖。

---

# 16. 第二台 M3508

overlay：

```dts
/ {
    aliases {
        motor0 = &m3508_1;
        motor1 = &m3508_2;
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

    m3508_2: motor-2 {
        compatible = "dji,m3508";
        status = "okay";
        can-bus = <&can1>;
        feedback-id = <0x202>;
        command-id = <0x200>;
        command-slot = <1>;
        gear-ratio = <19>;
    };
};
```

应用：

```cpp
const struct device *motor0 =
    DEVICE_DT_GET(DT_ALIAS(motor0));

const struct device *motor1 =
    DEVICE_DT_GET(DT_ALIAS(motor1));

tx_group.bindMotor(motor0);
tx_group.bindMotor(motor1);

skywalker::motor::dji::m3508::
    setCurrentRaw(motor0, 300);

skywalker::motor::dji::m3508::
    setCurrentRaw(motor1, -300);
```

之后仍然只调用一次：

```cpp
tx_group.send();
```

---

# 17. 为什么 `m3508SetTorque()` 故意返回 `-ENOSYS`

通用 API 参数叫 `torque_nm`，这意味着单位必须真的是 N·m。当前可靠的是 C620 `raw command ↔ current`，不是完整的输出轴 `N·m ↔ current` 模型。

所以现在：

```text
通用 setTorque()
→ 暂不支持

M3508 专用 setCurrentRaw()
→ 支持
```

等后面明确 `Kt`、减速比、效率和扭矩定义后再实现 Nm API。

---

# 18. 文件职责

`motor.hpp`：

```text
统一 motor API
```

`dji_protocol.hpp/.cpp`：

```text
CAN 帧编码/解码
```

`dji_m3508.cpp`：

```text
Config/Data
init
RX filter
RX callback
heartbeat
generic feedback
raw-current 状态
DEVICE_DT_INST_DEFINE
```

`dji_m3508.hpp`：

```text
M3508 专用 API
```

`dji_tx_group.hpp/.cpp`：

```text
最多四台 M3508 command
→ 一个 DJI group CAN frame
```

`dji,m3508.yaml`：

```text
描述每一台实际 M3508
```

---

# 19. 当前暂时不要写

```text
motor_can_router
万能 CAN manager
万能 motor factory
DM
LK
自动 TX thread
动态注册
std::vector
new/delete
复杂 template
PID 控制
多圈位置
```

等 M3508 的 1 台 RX、1 台 TX、2 台 group TX、heartbeat 全部验证以后再继续。

---

# 20. 给本地小模型的当前提示词

```text
你正在实现 SkyWalker 的 DJI M3508 Zephyr driver。

当前只允许修改：

drivers/Kconfig
drivers/motor/Kconfig
drivers/motor/CMakeLists.txt
drivers/motor/dji/CMakeLists.txt
drivers/motor/dji/dji_m3508.cpp
drivers/motor/dji/dji_protocol.cpp
include/drivers/motor/motor.hpp
include/drivers/motor/dji_protocol.hpp
dts/bindings/motor/dji,m3508.yaml
samples/motor/dji_m3508_driver/

要求：

1. C++14。
2. 禁止 new/delete。
3. 禁止 exception/RTTI。
4. 不复制 Breeze 源码。
5. protocol.cpp 只做协议帧转换。
6. dji_m3508.cpp 负责 Zephyr device、CAN RX 和状态。
7. CAN callback 不能 sleep。
8. M3508Config/Data 必须保持 cpp 私有。
9. 不实现 DM/LK。
10. 不实现 CAN router。
11. 不猜 torque Nm 转换。
12. 先完成 RX-only driver 并编译。
13. RX-only 成功以后才实现 TxGroup。
14. 每次只处理第一处编译错误。
```

---

# 21. 你现在立刻应该做什么

你当前 GitHub 已经到了 `dji_m3508.cpp` 骨架，所以现在按顺序只做：

```text
第 2 步：Kconfig 接线
第 3 步：motor Kconfig
第 4 步：CMake
第 5 步：motor.hpp
第 6 步：protocol
第 9 步：完整 RX driver
第 11 步：创建正式 driver sample
```

第一次目标只有三个：

```text
1. west build 成功
2. device_is_ready(motor) == true
3. feedback.timestamp_ms 能持续变化
```

三个都成功后，再继续 TX。
