# SkyWalker 古法编程线性实施指南

> 适用仓库：当前 `skywalker_code` 工作区
> Zephyr 基线：`west.yml` 固定提交 `6085aadeb337f27ee7411fa2ecbe8cf49164e360`，本地版本 `4.4.99`
> 目标平台：`dm_mc02/stm32h723xx`
> 语言边界：C11 + C++14
> 文档角色：这是唯一施工主线；它合并并取代原先分散的电机、PID、舵轮、双板、裁判系统和 `sp_middleware` 审计文档。

> 本轮架构裁决：DJI 电机接口执行一次性破坏性重构，不保留旧 M3508 API、类型别名、转发包装、旧 sample 或旧 devicetree 写法。本文出现“迁移”时，含义都是“所有调用方在同一次改动中切到新接口并删除旧接口”，不是长期双轨兼容。

这份指南把源码当作教材，不替你修改业务代码。你每次只做当前步骤，完成自检并保存证据后才进入下一步。DJI 重构已经改成“代码先行”：先给目标文件、操作方式和可完整粘贴的内容，再用短句解释关键边界。后面的整车章节仍保留背景说明，但当前只需完成紧接着的 DJI 施工区。

## 现在就从这里施工：DJI 统一接口逐文件一条龙

> 这一节是可直接照做的施工主线。先不要读后面的架构解释；从“施工 0”依次做到“施工 13”。每个代码块上方都写了目标文件和操作方式。写着“整文件替换”的，必须删掉该文件原内容后完整粘贴，不能把新旧两版拼在一起。

### 你当前工作树到底处于什么状态

我按当前文件只读核对后，起点是：

- 你已经删除旧 `dji_m3508.cpp/.hpp`、空的 `dji_gm6020` 文件、旧 binding 和两个旧 sample；这些删除正是本次破坏性重构的一部分，不要恢复。
- `motor.hpp` 已经改了一半，但缺少一致的 capability 检查和 `errno` 头；继续零碎补容易留下两套错误语义。
- `dji_protocol.hpp` 已经换了类型，`dji_protocol.cpp` 却只剩一个未使用的 `readBe16()`，现在没有可调用的 decoder/encoder。
- `dji_tx_group.cpp` 仍然 include 已删除的 `dji_m3508.hpp`，而 `drivers/motor/dji/CMakeLists.txt` 仍然编译已删除的 `dji_m3508.cpp`；因此当前工作树本来就不能完成 DJI driver 构建。
- 新文件的真实名字是 `include/drivers/motor/dji_motor.hpp<U+200B>`，末尾多了零宽字符。编辑器看起来正常，编译器却找不到 `<drivers/motor/dji_motor.hpp>`。

先接受一个简单目标：上层只调用 `motor::setCurrent(motor, 安培)` 和 `motor::readFeedback()`；所有 DJI 型号差异放在 profile；每条物理 CAN 只由一个 `dji::Bus` 组帧发送。下面不给旧 API 留任何 wrapper。

### 施工 0：先把文件名和旧残骸处理干净

在仓库根目录执行。第一条只查看，后面三条才改变文件：

```bash
git status --short

# 这个 $'...\u200b' 精确表示当前文件名末尾的零宽字符。
mv $'include/drivers/motor/dji_motor.hpp\u200b' \
   include/drivers/motor/dji_motor.hpp

# TxGroup 写死 m3508::*，统一 Bus 会完整取代它。
rm include/drivers/motor/dji_tx_group.hpp
rm drivers/motor/dji/dji_tx_group.cpp

mkdir -p samples/motor/dji_unified/src
mkdir -p tests/motor/dji_protocol/src
```

如果 `mv` 报“找不到文件”，先执行下面命令。输出若已经是没有隐藏字符的 `dji_motor.hpp`，说明你已经改过名，可以跳过 `mv`：

```bash
python3 -c 'import os; [print(repr(x)) for x in os.listdir("include/drivers/motor")]'
```

做完后，`include/drivers/motor/` 暂时只应有：

```text
motor.hpp
dji_protocol.hpp
dji_motor.hpp
```

`dji_bus.hpp` 会在施工 4 新建。此时不要构建，因为 CMake 要到施工 11 才切换完成。

### 施工 1：整文件替换通用 motor API

目标文件：`include/drivers/motor/motor.hpp`

操作：删除原内容，完整粘贴以下内容。

```cpp
#pragma once

#include <cstdint>
#include <errno.h>
#include <zephyr/device.h>

namespace skywalker {
namespace motor {

enum class State : std::uint8_t {
    Offline = 0,
    Ready,
    Fault,
};

enum Capability : std::uint32_t {
    CommandCurrent      = 1u << 0,
    CommandTorque       = 1u << 1,
    FeedbackPosition    = 1u << 8,
    FeedbackVelocity    = 1u << 9,
    FeedbackCurrent     = 1u << 10,
    FeedbackTorque      = 1u << 11,
    FeedbackTemperature = 1u << 12,
};

struct Feedback {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float current_a = 0.0f;
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;
    std::uint32_t valid = 0;
    std::uint64_t timestamp_ms = 0;
};

struct Api {
    std::uint32_t (*get_capabilities)(const struct device *dev);
    int (*set_current)(const struct device *dev, float current_a);
    int (*set_torque)(const struct device *dev, float torque_nm);
    int (*read_feedback)(const struct device *dev, Feedback *out);
    State (*get_state)(const struct device *dev);
};

inline std::uint32_t capabilities(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) return 0u;
    const Api *api = static_cast<const Api *>(dev->api);
    return api->get_capabilities == nullptr
        ? 0u
        : api->get_capabilities(dev);
}

inline int setCurrent(const struct device *dev, float current_a)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const Api *api = static_cast<const Api *>(dev->api);
    if ((capabilities(dev) & CommandCurrent) == 0u ||
        api->set_current == nullptr) {
        return -ENOTSUP;
    }
    return api->set_current(dev, current_a);
}

inline int setTorque(const struct device *dev, float torque_nm)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const Api *api = static_cast<const Api *>(dev->api);
    if ((capabilities(dev) & CommandTorque) == 0u ||
        api->set_torque == nullptr) {
        return -ENOTSUP;
    }
    return api->set_torque(dev, torque_nm);
}

inline int readFeedback(const struct device *dev, Feedback &out)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const Api *api = static_cast<const Api *>(dev->api);
    if (api->read_feedback == nullptr) return -ENOTSUP;
    return api->read_feedback(dev, &out);
}

inline State getState(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) return State::Offline;
    const Api *api = static_cast<const Api *>(dev->api);
    return api->get_state == nullptr
        ? State::Offline
        : api->get_state(dev);
}

} // namespace motor
} // namespace skywalker
```

为什么这里使用两层老式 namespace：仓库根 `CMakeLists.txt` 固定 `CMAKE_CXX_STANDARD 14`，`namespace skywalker::motor` 是 C++17 写法。后面的所有新文件也统一使用 C++14 写法。

本步结束检查：

```bash
rg -n 'enable|disable|setCurrentRaw|getCurrentRaw' \
  include/drivers/motor/motor.hpp
```

预期：零命中。这里故意不提供单电机 `enable/disable`；DJI 的启停属于共享 CAN Bus。

### 施工 2：整文件替换纯协议头和实现

目标文件：`include/drivers/motor/dji_protocol.hpp`

操作：整文件替换。

```cpp
#pragma once

#include <cstdint>
#include <zephyr/drivers/can.h>

namespace skywalker {
namespace motor {
namespace dji {

struct RawFeedback {
    std::uint16_t encoder = 0;
    std::int16_t speed_rpm = 0;
    std::int16_t current_raw = 0;
    std::uint8_t temperature_raw = 0;
    std::uint64_t timestamp_ms = 0;
};

bool decodeFeedback(const struct can_frame &frame, RawFeedback &out);

int buildCommandFrame(struct can_frame &frame,
                      std::uint16_t command_id,
                      const std::int16_t command_raw[4]);

} // namespace dji
} // namespace motor
} // namespace skywalker
```

目标文件：`drivers/motor/dji/dji_protocol.cpp`

操作：整文件替换。

```cpp
#include <errno.h>
#include <cstddef>

#include <drivers/motor/dji_protocol.hpp>

namespace skywalker {
namespace motor {
namespace dji {
namespace {

std::uint16_t readBe16(const std::uint8_t *p)
{
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(p[0]) << 8) |
        static_cast<std::uint16_t>(p[1]));
}

std::int16_t signedFromBits(std::uint16_t bits)
{
    std::int32_t value = static_cast<std::int32_t>(bits);
    if (value >= 0x8000) value -= 0x10000;
    return static_cast<std::int16_t>(value);
}

} // namespace

bool decodeFeedback(const struct can_frame &frame, RawFeedback &out)
{
    if (frame.dlc != 8u) return false;
    if ((frame.flags &
         (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0u) {
        return false;
    }

    RawFeedback next{};
    next.encoder = readBe16(&frame.data[0]);
    next.speed_rpm = signedFromBits(readBe16(&frame.data[2]));
    next.current_raw = signedFromBits(readBe16(&frame.data[4]));
    next.temperature_raw = frame.data[6];
    out = next;
    return true;
}

int buildCommandFrame(struct can_frame &frame,
                      std::uint16_t command_id,
                      const std::int16_t command_raw[4])
{
    if (command_raw == nullptr) return -EINVAL;
    if (command_id > CAN_STD_ID_MASK) return -ERANGE;

    struct can_frame next{};
    next.id = command_id;
    next.flags = 0u;
    next.dlc = 8u;

    for (std::size_t i = 0; i < 4u; ++i) {
        const std::uint16_t bits =
            static_cast<std::uint16_t>(command_raw[i]);
        next.data[i * 2u] =
            static_cast<std::uint8_t>((bits >> 8) & 0xFFu);
        next.data[i * 2u + 1u] =
            static_cast<std::uint8_t>(bits & 0xFFu);
    }

    frame = next;
    return 0;
}

} // namespace dji
} // namespace motor
} // namespace skywalker
```

不要用旧代码的 `current[i] >> 8` 直接右移负数；上面先转成 `uint16_t` 位模式，C++14 行为明确。

### 施工 3：新建公开的 DJI 设备头

目标文件：`include/drivers/motor/dji_motor.hpp`

操作：整文件替换刚刚改名后的空文件。

```cpp
#pragma once

#include <cstdint>
#include <zephyr/device.h>

#include <drivers/motor/dji_protocol.hpp>

namespace skywalker {
namespace motor {
namespace dji {

enum class Model : std::uint8_t {
    M3508C620 = 0,
    M2006C610,
    GM6020Current,
};

struct Descriptor {
    Model model = Model::M3508C620;
    const struct device *can = nullptr;
    std::uint8_t motor_id = 0;
    std::uint16_t feedback_id = 0;
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
    float protocol_current_max_a = 0.0f;
    float configured_current_limit_a = 0.0f;
    float gear_ratio = 1.0f;
    bool temperature_valid = false;
};

int describe(const struct device *dev, Descriptor &out);
int readRawFeedback(const struct device *dev, RawFeedback &out);

} // namespace dji
} // namespace motor
} // namespace skywalker
```

这个头只负责“查看某个统一 DJI device 是什么”。正常控制仍然 include `motor.hpp` 并调用 `setCurrent(A)`，不应在这里重新增加型号 namespace 或 raw setter。

### 施工 4：新建唯一 Bus 的公开头

目标文件：`include/drivers/motor/dji_bus.hpp`

操作：新建并完整粘贴。

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <zephyr/device.h>

#include <drivers/motor/dji_motor.hpp>

namespace skywalker {
namespace motor {
namespace dji {

enum class BusState : std::uint8_t {
    Uninitialized = 0,
    Safe,
    Armed,
    Fault,
};

struct FlushReport {
    int preparation_error = 0;
    int command_tx_error = 0;
    int zero_tx_error = 0;
    std::uint16_t target_failed_command_id = 0;
    std::uint16_t zero_failed_command_id = 0;
    std::uint8_t preparation_failed_slot = 0xFFu;
    std::uint8_t zero_groups_expected = 0;
    std::uint8_t zero_groups_attempted = 0;
    std::uint8_t zero_groups_succeeded = 0;
    bool zero_sent = false;
};

class Bus {
public:
    Bus() = default;
    Bus(const Bus &) = delete;
    Bus &operator=(const Bus &) = delete;
    Bus(Bus &&) = delete;
    Bus &operator=(Bus &&) = delete;

    int init(const struct device *can);
    int attach(const struct device *motor);
    int arm(FlushReport &report);
    int flush(FlushReport &report);
    int stop(FlushReport &report);
    int recover(FlushReport &report);
    BusState state() const;

private:
    int groupIndex(std::uint16_t command_id) const;
    int sendZeros(FlushReport &report);
    int enterFaultAndZero(FlushReport &report);

    const struct device *can_ = nullptr;
    const struct device *
        motors_[CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS]{};
    Descriptor
        descriptors_[CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS]{};
    std::uint16_t
        group_ids_[CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS]{};
    std::size_t motor_count_ = 0;
    std::size_t group_count_ = 0;
    std::uint64_t lifecycle_epoch_ = 0;
    BusState state_ = BusState::Uninitialized;
};

} // namespace dji
} // namespace motor
} // namespace skywalker
```

这里把固定容量数组直接放进 `Bus`，是为了让第一版没有 `new/delete` 和悬空对象。`Bus` 必须定义成 static/global 对象，不能定义成会提前析构的局部临时对象。

### 施工 5：新建 driver 私有头

目标文件：`drivers/motor/dji/dji_internal.hpp`

操作：新建并完整粘贴。应用和 sample 不得 include 这个文件。

```cpp
#pragma once

#include <cstdint>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/spinlock.h>

#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>

namespace skywalker {
namespace motor {
namespace dji {
namespace internal {

struct Profile {
    Model model;
    std::uint8_t max_motor_id;
    std::uint16_t feedback_base;
    std::uint16_t low_command_id;
    std::uint16_t high_command_id;
    std::int16_t command_raw_max;
    float protocol_current_max_a;
    bool temperature_valid;
};

struct Endpoint {
    std::uint16_t feedback_id = 0;
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
};

struct CommandSnapshot {
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
    std::int16_t command_raw = 0;
};

struct DjiConfig {
    const struct device *can;
    const Profile *profile;
    std::uint32_t motor_id;
    std::uint32_t current_limit_ma;
    std::uint32_t gear_ratio_num;
    std::uint32_t gear_ratio_den;
};

struct DjiData {
    Feedback feedback{};
    RawFeedback raw_feedback{};
    Endpoint endpoint{};
    float current_limit_a = 0.0f;
    float gear_ratio = 0.0f;

    std::int16_t command_raw = 0;
    std::uint64_t command_stamp_ms = 0;
    std::uint64_t command_generation = 0;
    std::uint64_t command_epoch = 0;
    std::uint64_t active_epoch = 0;
    bool armed = false;
    bool fault_latched = false;

    std::uint16_t last_encoder = 0;
    std::int64_t total_encoder_ticks = 0;
    bool has_encoder = false;
    std::uint64_t last_rx_ms = 0;

    struct k_spinlock lock{};
    int rx_filter_id = -1;
};

extern const Profile kM3508C620Profile;
extern const Profile kM2006C610Profile;
extern const Profile kGM6020CurrentProfile;
extern const skywalker::motor::Api dji_motor_api;

int resolveEndpoint(const Profile &profile,
                    std::uint8_t motor_id,
                    Endpoint &out);
int currentToRaw(const Profile &profile,
                 float current_a,
                 std::int16_t &out);
int rawToCurrent(const Profile &profile,
                 std::int16_t raw,
                 float &out);

int djiMotorInit(const struct device *dev);
bool isDjiMotor(const struct device *dev);
bool feedbackReady(const struct device *dev);
int armMotor(const struct device *dev,
             std::uint64_t epoch,
             std::uint64_t now_ms);
void prepareMotorStop(const struct device *dev, bool latch_fault);
void clearMotorFault(const struct device *dev);
int snapshotCommand(const struct device *dev,
                    std::uint64_t expected_epoch,
                    std::uint64_t now_ms,
                    CommandSnapshot &out);

} // namespace internal
} // namespace dji
} // namespace motor
} // namespace skywalker

#define DJI_MOTOR_DEFINE(inst, profile_symbol)                              \
    static skywalker::motor::dji::internal::DjiData                        \
        dji_data_##inst;                                                    \
    static const skywalker::motor::dji::internal::DjiConfig                \
        dji_config_##inst = {                                               \
            DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                  \
            &(profile_symbol),                                              \
            DT_INST_PROP(inst, motor_id),                                   \
            DT_INST_PROP(inst, current_limit_ma),                           \
            DT_INST_PROP(inst, gear_ratio_num),                             \
            DT_INST_PROP(inst, gear_ratio_den),                             \
        };                                                                  \
    DEVICE_DT_INST_DEFINE(                                                  \
        inst,                                                               \
        skywalker::motor::dji::internal::djiMotorInit,                      \
        nullptr,                                                            \
        &dji_data_##inst,                                                   \
        &dji_config_##inst,                                                 \
        POST_KERNEL,                                                        \
        CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY,                               \
        &skywalker::motor::dji::internal::dji_motor_api);
```

不要把 `current_limit_ma / gear_ratio_num / gear_ratio_den` 提前转成 float。先保留设备树整数，`djiMotorInit()` 校验分母以后再相除，才能真正挡住除零。

### 施工 6：新建三型号 profile 和换算

目标文件：`drivers/motor/dji/dji_profiles.cpp`

操作：新建并完整粘贴。

```cpp
#include <cmath>
#include <errno.h>

#include "dji_internal.hpp"

namespace skywalker {
namespace motor {
namespace dji {
namespace internal {

const Profile kM3508C620Profile = {
    Model::M3508C620,
    8u,
    0x200u,
    0x200u,
    0x1FFu,
    16384,
    20.0f,
    true,
};

const Profile kM2006C610Profile = {
    Model::M2006C610,
    8u,
    0x200u,
    0x200u,
    0x1FFu,
    10000,
    10.0f,
    false,
};

const Profile kGM6020CurrentProfile = {
    Model::GM6020Current,
    7u,
    0x204u,
    0x1FEu,
    0x2FEu,
    16384,
    3.0f,
    true,
};

int resolveEndpoint(const Profile &profile,
                    std::uint8_t motor_id,
                    Endpoint &out)
{
    if (motor_id == 0u || motor_id > profile.max_motor_id) {
        return -ERANGE;
    }

    Endpoint next{};
    next.feedback_id =
        static_cast<std::uint16_t>(profile.feedback_base + motor_id);

    if (motor_id <= 4u) {
        next.command_id = profile.low_command_id;
        next.command_slot =
            static_cast<std::uint8_t>(motor_id - 1u);
    } else {
        next.command_id = profile.high_command_id;
        next.command_slot =
            static_cast<std::uint8_t>(motor_id - 5u);
    }

    if (next.command_slot >= 4u) return -ERANGE;
    out = next;
    return 0;
}

int currentToRaw(const Profile &profile,
                 float current_a,
                 std::int16_t &out)
{
    if (!std::isfinite(current_a)) return -EINVAL;
    if (profile.command_raw_max <= 0 ||
        !std::isfinite(profile.protocol_current_max_a) ||
        profile.protocol_current_max_a <= 0.0f) {
        return -EINVAL;
    }
    if (std::fabs(current_a) >
        profile.protocol_current_max_a) {
        return -ERANGE;
    }

    const float scaled =
        current_a *
        static_cast<float>(profile.command_raw_max) /
        profile.protocol_current_max_a;
    const long rounded = std::lround(scaled);
    if (rounded < -profile.command_raw_max ||
        rounded > profile.command_raw_max) {
        return -ERANGE;
    }

    out = static_cast<std::int16_t>(rounded);
    return 0;
}

int rawToCurrent(const Profile &profile,
                 std::int16_t raw,
                 float &out)
{
    if (profile.command_raw_max <= 0 ||
        !std::isfinite(profile.protocol_current_max_a) ||
        profile.protocol_current_max_a <= 0.0f) {
        return -EINVAL;
    }
    if (raw < -profile.command_raw_max ||
        raw > profile.command_raw_max) {
        return -ERANGE;
    }

    out =
        static_cast<float>(raw) *
        profile.protocol_current_max_a /
        static_cast<float>(profile.command_raw_max);
    return 0;
}

} // namespace internal
} // namespace dji
} // namespace motor
} // namespace skywalker
```

你现在只需要记住三个换算结果：

```text
C620:   20 A ↔ 16384 raw
C610:   10 A ↔ 10000 raw
GM6020:  3 A ↔ 16384 raw
```

应用层永远不写右边的 raw；右边只存在于 profile、codec 和测试。

### 施工 7：新建统一 device core

目标文件：`drivers/motor/dji/dji_motor.cpp`

操作：新建并完整粘贴。这个文件比较长，但只做三件事：初始化设备、接收反馈、缓存安培命令。

```cpp
#include <cmath>
#include <errno.h>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include "dji_internal.hpp"

namespace skywalker {
namespace motor {
namespace dji {
namespace internal {
namespace {

constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kEncoderTicksPerTurn = 8192.0f;

DjiData *dataOf(const struct device *dev)
{
    return dev == nullptr
        ? nullptr
        : static_cast<DjiData *>(dev->data);
}

const DjiConfig *configOf(const struct device *dev)
{
    return dev == nullptr
        ? nullptr
        : static_cast<const DjiConfig *>(dev->config);
}

std::uint32_t getCapabilities(const struct device *dev)
{
    const DjiConfig *cfg = configOf(dev);
    if (cfg == nullptr || cfg->profile == nullptr) return 0u;

    std::uint32_t caps =
        CommandCurrent |
        FeedbackPosition |
        FeedbackVelocity |
        FeedbackCurrent;
    if (cfg->profile->temperature_valid) {
        caps |= FeedbackTemperature;
    }
    return caps;
}

State getStateImpl(const struct device *dev)
{
    DjiData *data = dataOf(dev);
    if (data == nullptr) return State::Offline;

    bool fault = false;
    std::uint64_t last_rx_ms = 0;
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    fault = data->fault_latched;
    last_rx_ms = data->last_rx_ms;
    k_spin_unlock(&data->lock, key);

    if (fault) return State::Fault;
    if (last_rx_ms == 0u) return State::Offline;

    const std::uint64_t now_ms =
        static_cast<std::uint64_t>(k_uptime_get());
    if (now_ms < last_rx_ms ||
        now_ms - last_rx_ms >
            CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS) {
        return State::Offline;
    }
    return State::Ready;
}

int setCurrentImpl(const struct device *dev, float current_a)
{
    DjiData *data = dataOf(dev);
    const DjiConfig *cfg = configOf(dev);
    if (data == nullptr || cfg == nullptr ||
        cfg->profile == nullptr) {
        return -EINVAL;
    }
    if (!std::isfinite(current_a)) return -EINVAL;
    if (std::fabs(current_a) > data->current_limit_a) {
        return -ERANGE;
    }
    if (getStateImpl(dev) != State::Ready) {
        return -EHOSTDOWN;
    }

    std::int16_t raw = 0;
    const int convert_ret =
        currentToRaw(*cfg->profile, current_a, raw);
    if (convert_ret < 0) return convert_ret;

    const std::uint64_t now_ms =
        static_cast<std::uint64_t>(k_uptime_get());
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    if (!data->armed || data->fault_latched ||
        data->active_epoch == 0u) {
        k_spin_unlock(&data->lock, key);
        return -EACCES;
    }

    data->command_raw = raw;
    data->command_stamp_ms = now_ms;
    ++data->command_generation;
    data->command_epoch = data->active_epoch;
    k_spin_unlock(&data->lock, key);
    return 0;
}

int readFeedbackImpl(const struct device *dev, Feedback *out)
{
    DjiData *data = dataOf(dev);
    if (data == nullptr || out == nullptr) return -EINVAL;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    *out = data->feedback;
    k_spin_unlock(&data->lock, key);
    return 0;
}

void rxCallback(const struct device *,
                struct can_frame *frame,
                void *user_data)
{
    const struct device *dev =
        static_cast<const struct device *>(user_data);
    DjiData *data = dataOf(dev);
    const DjiConfig *cfg = configOf(dev);
    if (dev == nullptr || data == nullptr ||
        cfg == nullptr || cfg->profile == nullptr ||
        frame == nullptr) {
        return;
    }

    RawFeedback raw{};
    if (!decodeFeedback(*frame, raw)) return;

    const std::uint64_t now_ms =
        static_cast<std::uint64_t>(k_uptime_get());
    raw.timestamp_ms = now_ms;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);

    if (!data->has_encoder) {
        data->last_encoder = raw.encoder;
        data->total_encoder_ticks = 0;
        data->has_encoder = true;
    } else {
        std::int32_t delta =
            static_cast<std::int32_t>(raw.encoder) -
            static_cast<std::int32_t>(data->last_encoder);
        if (delta > 4096) delta -= 8192;
        if (delta < -4096) delta += 8192;
        data->total_encoder_ticks += delta;
        data->last_encoder = raw.encoder;
    }

    Feedback next{};
    next.position_rad =
        static_cast<float>(data->total_encoder_ticks) *
        (kTwoPi / kEncoderTicksPerTurn) /
        data->gear_ratio;
    next.valid |= FeedbackPosition;

    next.velocity_rad_s =
        static_cast<float>(raw.speed_rpm) *
        (kTwoPi / 60.0f) /
        data->gear_ratio;
    next.valid |= FeedbackVelocity;

    float feedback_current_a = 0.0f;
    if (rawToCurrent(*cfg->profile,
                     raw.current_raw,
                     feedback_current_a) == 0) {
        next.current_a = feedback_current_a;
        next.valid |= FeedbackCurrent;
    }

    if (cfg->profile->temperature_valid) {
        next.temperature_c =
            static_cast<float>(raw.temperature_raw);
        next.valid |= FeedbackTemperature;
    }

    next.timestamp_ms = now_ms;
    data->raw_feedback = raw;
    data->feedback = next;
    data->last_rx_ms = now_ms;

    k_spin_unlock(&data->lock, key);
}

} // namespace

const skywalker::motor::Api dji_motor_api = {
    getCapabilities,
    setCurrentImpl,
    nullptr,
    readFeedbackImpl,
    getStateImpl,
};

int djiMotorInit(const struct device *dev)
{
    DjiData *data = dataOf(dev);
    const DjiConfig *cfg = configOf(dev);
    if (dev == nullptr || data == nullptr ||
        cfg == nullptr || cfg->can == nullptr ||
        cfg->profile == nullptr) {
        return -EINVAL;
    }
    if (!device_is_ready(cfg->can)) return -ENODEV;
    if (cfg->motor_id == 0u ||
        cfg->motor_id > cfg->profile->max_motor_id ||
        cfg->motor_id > 255u) {
        return -ERANGE;
    }
    if (cfg->gear_ratio_num == 0u ||
        cfg->gear_ratio_den == 0u) {
        return -EINVAL;
    }

    const float current_limit_a =
        static_cast<float>(cfg->current_limit_ma) / 1000.0f;
    const float gear_ratio =
        static_cast<float>(cfg->gear_ratio_num) /
        static_cast<float>(cfg->gear_ratio_den);
    if (!std::isfinite(current_limit_a) ||
        current_limit_a < 0.0f ||
        current_limit_a >
            cfg->profile->protocol_current_max_a) {
        return -ERANGE;
    }
    if (!std::isfinite(gear_ratio) ||
        gear_ratio <= 0.0f) {
        return -ERANGE;
    }

    Endpoint endpoint{};
    const int endpoint_ret =
        resolveEndpoint(*cfg->profile,
                        static_cast<std::uint8_t>(cfg->motor_id),
                        endpoint);
    if (endpoint_ret < 0) return endpoint_ret;

    data->feedback = {};
    data->raw_feedback = {};
    data->endpoint = endpoint;
    data->current_limit_a = current_limit_a;
    data->gear_ratio = gear_ratio;
    data->command_raw = 0;
    data->command_stamp_ms = 0;
    data->command_generation = 0;
    data->command_epoch = 0;
    data->active_epoch = 0;
    data->armed = false;
    data->fault_latched = false;
    data->last_encoder = 0;
    data->total_encoder_ticks = 0;
    data->has_encoder = false;
    data->last_rx_ms = 0;
    data->rx_filter_id = -1;

    struct can_filter filter{};
    filter.id = endpoint.feedback_id;
    filter.mask = CAN_STD_ID_MASK;
    filter.flags = 0u;

    const int filter_id =
        can_add_rx_filter(cfg->can,
                          rxCallback,
                          const_cast<struct device *>(dev),
                          &filter);
    if (filter_id < 0) return filter_id;
    data->rx_filter_id = filter_id;
    return 0;
}

bool isDjiMotor(const struct device *dev)
{
    return dev != nullptr &&
           dev->api == &dji_motor_api;
}

bool feedbackReady(const struct device *dev)
{
    return getStateImpl(dev) == State::Ready;
}

int armMotor(const struct device *dev,
             std::uint64_t epoch,
             std::uint64_t now_ms)
{
    DjiData *data = dataOf(dev);
    if (data == nullptr || epoch == 0u) return -EINVAL;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->fault_latched = false;
    data->command_raw = 0;
    data->command_stamp_ms = now_ms;
    ++data->command_generation;
    data->command_epoch = epoch;
    data->active_epoch = epoch;
    data->armed = true;
    k_spin_unlock(&data->lock, key);
    return 0;
}

void prepareMotorStop(const struct device *dev, bool latch_fault)
{
    DjiData *data = dataOf(dev);
    if (data == nullptr) return;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->armed = false;
    data->active_epoch = 0;
    data->command_epoch = 0;
    data->command_raw = 0;
    data->command_stamp_ms = 0;
    ++data->command_generation;
    if (latch_fault) data->fault_latched = true;
    k_spin_unlock(&data->lock, key);
}

void clearMotorFault(const struct device *dev)
{
    DjiData *data = dataOf(dev);
    if (data == nullptr) return;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->fault_latched = false;
    data->armed = false;
    k_spin_unlock(&data->lock, key);
}

int snapshotCommand(const struct device *dev,
                    std::uint64_t expected_epoch,
                    std::uint64_t now_ms,
                    CommandSnapshot &out)
{
    DjiData *data = dataOf(dev);
    if (data == nullptr || expected_epoch == 0u) {
        return -EINVAL;
    }

    CommandSnapshot next{};
    int ret = 0;
    const k_spinlock_key_t key = k_spin_lock(&data->lock);

    if (!data->armed || data->fault_latched) {
        ret = -EACCES;
    } else if (data->active_epoch != expected_epoch ||
               data->command_epoch != expected_epoch) {
        ret = -ESTALE;
    } else if (data->last_rx_ms == 0u ||
               now_ms < data->last_rx_ms ||
               now_ms - data->last_rx_ms >
                   CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS) {
        ret = -EHOSTDOWN;
    } else if (data->command_stamp_ms == 0u ||
               now_ms < data->command_stamp_ms ||
               now_ms - data->command_stamp_ms >
                   CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS) {
        ret = -ESTALE;
    } else {
        next.command_id = data->endpoint.command_id;
        next.command_slot = data->endpoint.command_slot;
        next.command_raw = data->command_raw;
    }

    k_spin_unlock(&data->lock, key);
    if (ret < 0) return ret;
    out = next;
    return 0;
}

} // namespace internal

int describe(const struct device *dev, Descriptor &out)
{
    if (!internal::isDjiMotor(dev)) return -ENOTSUP;
    const internal::DjiConfig *cfg =
        static_cast<const internal::DjiConfig *>(dev->config);
    internal::DjiData *data =
        static_cast<internal::DjiData *>(dev->data);
    if (cfg == nullptr || cfg->profile == nullptr ||
        data == nullptr) {
        return -EINVAL;
    }

    Descriptor next{};
    next.model = cfg->profile->model;
    next.can = cfg->can;
    next.motor_id =
        static_cast<std::uint8_t>(cfg->motor_id);
    next.feedback_id = data->endpoint.feedback_id;
    next.command_id = data->endpoint.command_id;
    next.command_slot = data->endpoint.command_slot;
    next.protocol_current_max_a =
        cfg->profile->protocol_current_max_a;
    next.configured_current_limit_a =
        data->current_limit_a;
    next.gear_ratio = data->gear_ratio;
    next.temperature_valid =
        cfg->profile->temperature_valid;
    out = next;
    return 0;
}

int readRawFeedback(const struct device *dev, RawFeedback &out)
{
    if (!internal::isDjiMotor(dev)) return -ENOTSUP;
    internal::DjiData *data =
        static_cast<internal::DjiData *>(dev->data);
    if (data == nullptr) return -EINVAL;

    RawFeedback next{};
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    next = data->raw_feedback;
    k_spin_unlock(&data->lock, key);

    if (next.timestamp_ms == 0u) return -ENODATA;
    out = next;
    return 0;
}

} // namespace dji
} // namespace motor
} // namespace skywalker
```

粘贴后只核对三个位置，不要自己发挥：

1. `djiMotorInit()` 末尾才注册 RX filter；所有 RAM 字段必须先初始化。
2. `setCurrentImpl()` 只写缓存，不调用 `can_send()`。
3. `prepareMotorStop()` 先关闭 `armed`，再清 raw；真正的零帧由下一步 Bus 发送。

### 施工 8：新建唯一 Bus 实现

目标文件：`drivers/motor/dji/dji_bus.cpp`

操作：新建并完整粘贴。第一版规定 `init/attach/arm/setCurrent/flush/stop/recover` 全部由同一个控制线程串行调用；RX callback 是唯一例外，它只更新反馈快照。

```cpp
#include <errno.h>
#include <limits>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <drivers/motor/dji_bus.hpp>

#include "dji_internal.hpp"

namespace skywalker {
namespace motor {
namespace dji {
namespace {

struct OwnerEntry {
    const struct device *can = nullptr;
    Bus *owner = nullptr;
};

OwnerEntry owner_registry[CONFIG_SKYWALKER_DJI_MAX_BUSES]{};
struct k_spinlock owner_registry_lock{};

int claimCan(const struct device *can, Bus *owner)
{
    const k_spinlock_key_t key =
        k_spin_lock(&owner_registry_lock);
    int empty_index = -1;

    for (std::size_t i = 0;
         i < CONFIG_SKYWALKER_DJI_MAX_BUSES;
         ++i) {
        if (owner_registry[i].can == can) {
            const int ret =
                owner_registry[i].owner == owner
                    ? -EALREADY
                    : -EBUSY;
            k_spin_unlock(&owner_registry_lock, key);
            return ret;
        }
        if (owner_registry[i].can == nullptr &&
            empty_index < 0) {
            empty_index = static_cast<int>(i);
        }
    }

    if (empty_index < 0) {
        k_spin_unlock(&owner_registry_lock, key);
        return -ENOSPC;
    }

    owner_registry[empty_index].can = can;
    owner_registry[empty_index].owner = owner;
    k_spin_unlock(&owner_registry_lock, key);
    return 0;
}

void releaseCan(const struct device *can, Bus *owner)
{
    const k_spinlock_key_t key =
        k_spin_lock(&owner_registry_lock);
    for (std::size_t i = 0;
         i < CONFIG_SKYWALKER_DJI_MAX_BUSES;
         ++i) {
        if (owner_registry[i].can == can &&
            owner_registry[i].owner == owner) {
            owner_registry[i] = {};
            break;
        }
    }
    k_spin_unlock(&owner_registry_lock, key);
}

int reportError(const FlushReport &report)
{
    if (report.zero_tx_error < 0) {
        return report.zero_tx_error;
    }
    if (report.command_tx_error < 0) {
        return report.command_tx_error;
    }
    return report.preparation_error;
}

} // namespace

int Bus::init(const struct device *can)
{
    if (can == nullptr) return -EINVAL;
    if (state_ != BusState::Uninitialized) return -EALREADY;
    if (!device_is_ready(can)) return -ENODEV;

    int ret = claimCan(can, this);
    if (ret < 0) return ret;

    ret = can_start(can);
    if (ret < 0 && ret != -EALREADY) {
        releaseCan(can, this);
        return ret;
    }

    can_ = can;
    state_ = BusState::Safe;
    return 0;
}

int Bus::groupIndex(std::uint16_t command_id) const
{
    for (std::size_t i = 0; i < group_count_; ++i) {
        if (group_ids_[i] == command_id) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int Bus::attach(const struct device *motor)
{
    if (state_ != BusState::Safe) return -EACCES;
    if (!internal::isDjiMotor(motor)) return -ENOTSUP;
    if (motor_count_ >=
        CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS) {
        return -ENOSPC;
    }

    Descriptor descriptor{};
    int ret = describe(motor, descriptor);
    if (ret < 0) return ret;
    if (descriptor.can != can_) return -EXDEV;

    for (std::size_t i = 0; i < motor_count_; ++i) {
        if (motors_[i] == motor) return -EALREADY;
        if (descriptors_[i].feedback_id ==
            descriptor.feedback_id) {
            return -EADDRINUSE;
        }
        if (descriptors_[i].command_id ==
                descriptor.command_id &&
            descriptors_[i].command_slot ==
                descriptor.command_slot) {
            return -EADDRINUSE;
        }
    }

    if (groupIndex(descriptor.command_id) < 0) {
        if (group_count_ >=
            CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS) {
            return -ENOSPC;
        }
        group_ids_[group_count_++] =
            descriptor.command_id;
    }

    motors_[motor_count_] = motor;
    descriptors_[motor_count_] = descriptor;
    ++motor_count_;
    return 0;
}

int Bus::sendZeros(FlushReport &report)
{
    report.zero_tx_error = 0;
    report.zero_failed_command_id = 0;
    report.zero_groups_expected =
        static_cast<std::uint8_t>(group_count_);
    report.zero_groups_attempted = 0;
    report.zero_groups_succeeded = 0;
    report.zero_sent = false;

    const std::int16_t zeros[4] = {0, 0, 0, 0};
    for (std::size_t i = 0; i < group_count_; ++i) {
        struct can_frame frame{};
        int ret =
            buildCommandFrame(frame, group_ids_[i], zeros);
        if (ret == 0) {
            ++report.zero_groups_attempted;
            ret = can_send(can_,
                           &frame,
                           K_MSEC(2),
                           nullptr,
                           nullptr);
        }

        if (ret < 0) {
            if (report.zero_tx_error == 0) {
                report.zero_tx_error = ret;
                report.zero_failed_command_id =
                    group_ids_[i];
            }
        } else {
            ++report.zero_groups_succeeded;
        }
    }

    report.zero_sent =
        report.zero_groups_expected > 0u &&
        report.zero_groups_attempted ==
            report.zero_groups_expected &&
        report.zero_groups_succeeded ==
            report.zero_groups_expected;
    return report.zero_tx_error;
}

int Bus::enterFaultAndZero(FlushReport &report)
{
    for (std::size_t i = 0; i < motor_count_; ++i) {
        internal::prepareMotorStop(motors_[i], true);
    }
    state_ = BusState::Fault;
    sendZeros(report);
    return reportError(report);
}

int Bus::arm(FlushReport &report)
{
    report = {};
    if (state_ != BusState::Safe) {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }
    if (motor_count_ == 0u) {
        report.preparation_error = -ENODEV;
        return report.preparation_error;
    }

    for (std::size_t i = 0; i < motor_count_; ++i) {
        if (!internal::feedbackReady(motors_[i])) {
            report.preparation_error = -EHOSTDOWN;
            report.preparation_failed_slot =
                descriptors_[i].command_slot;
            return enterFaultAndZero(report);
        }
    }

    if (sendZeros(report) < 0 || !report.zero_sent) {
        for (std::size_t i = 0; i < motor_count_; ++i) {
            internal::prepareMotorStop(motors_[i], true);
        }
        state_ = BusState::Fault;
        return reportError(report);
    }

    if (lifecycle_epoch_ ==
        std::numeric_limits<std::uint64_t>::max()) {
        report.preparation_error = -EOVERFLOW;
        return enterFaultAndZero(report);
    }
    ++lifecycle_epoch_;
    if (lifecycle_epoch_ == 0u) ++lifecycle_epoch_;

    const std::uint64_t now_ms =
        static_cast<std::uint64_t>(k_uptime_get());
    for (std::size_t i = 0; i < motor_count_; ++i) {
        const int ret =
            internal::armMotor(motors_[i],
                               lifecycle_epoch_,
                               now_ms);
        if (ret < 0) {
            report.preparation_error = ret;
            report.preparation_failed_slot =
                descriptors_[i].command_slot;
            return enterFaultAndZero(report);
        }
    }

    state_ = BusState::Armed;
    return 0;
}

int Bus::flush(FlushReport &report)
{
    report = {};
    if (state_ != BusState::Armed) {
        report.preparation_error = -EACCES;
        if (state_ == BusState::Uninitialized) {
            return report.preparation_error;
        }
        return enterFaultAndZero(report);
    }

    std::int16_t
        commands[CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS][4]{};
    const std::uint64_t now_ms =
        static_cast<std::uint64_t>(k_uptime_get());

    for (std::size_t i = 0; i < motor_count_; ++i) {
        internal::CommandSnapshot snapshot{};
        const int ret =
            internal::snapshotCommand(motors_[i],
                                      lifecycle_epoch_,
                                      now_ms,
                                      snapshot);
        if (ret < 0) {
            report.preparation_error = ret;
            report.preparation_failed_slot =
                descriptors_[i].command_slot;
            return enterFaultAndZero(report);
        }

        const int group = groupIndex(snapshot.command_id);
        if (group < 0 || snapshot.command_slot >= 4u) {
            report.preparation_error = -EFAULT;
            report.preparation_failed_slot =
                snapshot.command_slot;
            return enterFaultAndZero(report);
        }
        commands[group][snapshot.command_slot] =
            snapshot.command_raw;
    }

    for (std::size_t i = 0; i < group_count_; ++i) {
        struct can_frame frame{};
        int ret =
            buildCommandFrame(frame,
                              group_ids_[i],
                              commands[i]);
        if (ret == 0) {
            ret = can_send(can_,
                           &frame,
                           K_MSEC(2),
                           nullptr,
                           nullptr);
        }
        if (ret < 0) {
            report.command_tx_error = ret;
            report.target_failed_command_id =
                group_ids_[i];
            return enterFaultAndZero(report);
        }
    }
    return 0;
}

int Bus::stop(FlushReport &report)
{
    report = {};
    if (state_ == BusState::Uninitialized) {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }

    const bool was_fault = state_ == BusState::Fault;
    for (std::size_t i = 0; i < motor_count_; ++i) {
        internal::prepareMotorStop(motors_[i], was_fault);
    }

    if (sendZeros(report) < 0 || !report.zero_sent) {
        for (std::size_t i = 0; i < motor_count_; ++i) {
            internal::prepareMotorStop(motors_[i], true);
        }
        state_ = BusState::Fault;
        return reportError(report);
    }

    if (was_fault) {
        state_ = BusState::Fault;
    } else {
        for (std::size_t i = 0; i < motor_count_; ++i) {
            internal::clearMotorFault(motors_[i]);
        }
        state_ = BusState::Safe;
    }
    return 0;
}

int Bus::recover(FlushReport &report)
{
    report = {};
    if (state_ != BusState::Fault) {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }

    for (std::size_t i = 0; i < motor_count_; ++i) {
        internal::prepareMotorStop(motors_[i], true);
    }
    if (sendZeros(report) < 0 || !report.zero_sent) {
        state_ = BusState::Fault;
        return reportError(report);
    }

    for (std::size_t i = 0; i < motor_count_; ++i) {
        internal::clearMotorFault(motors_[i]);
    }
    state_ = BusState::Safe;
    return 0;
}

BusState Bus::state() const
{
    return state_;
}

} // namespace dji
} // namespace motor
} // namespace skywalker
```

这份第一版 Bus 的调用顺序必须是：

```text
init(CAN)
  → attach(这条 CAN 上的每台 DJI 电机)
  → 等所有反馈 Ready
  → 收到一次新的人工解锁动作
  → arm()，它先发送所有分组全零
  → 每拍先给所有电机 setCurrent(A)
  → 每条 Bus 每拍只 flush() 一次
  → 停机调用 stop()
```

`flush()` 失败后 Bus 已经是 Fault，并且原 `FlushReport` 记录了目标帧和零帧错误。此时不要立即调用 `recover()` 覆盖证据；先处理故障和物理断电。`recover()` 只能放在明确的人工恢复分支。

### 施工 9：新建三个 Zephyr 实例文件

目标文件：`drivers/motor/dji/dji_m3508_instance.cpp`

操作：新建并完整粘贴。

```cpp
#define DT_DRV_COMPAT dji_m3508_c620

#include "dji_internal.hpp"

#define M3508_DEFINE(inst)                                             \
    DJI_MOTOR_DEFINE(                                                  \
        inst,                                                          \
        skywalker::motor::dji::internal::kM3508C620Profile)

DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)
```

目标文件：`drivers/motor/dji/dji_m2006_instance.cpp`

操作：新建并完整粘贴。

```cpp
#define DT_DRV_COMPAT dji_m2006_c610

#include "dji_internal.hpp"

#define M2006_DEFINE(inst)                                             \
    DJI_MOTOR_DEFINE(                                                  \
        inst,                                                          \
        skywalker::motor::dji::internal::kM2006C610Profile)

DT_INST_FOREACH_STATUS_OKAY(M2006_DEFINE)
```

目标文件：`drivers/motor/dji/dji_gm6020_instance.cpp`

操作：新建并完整粘贴。

```cpp
#define DT_DRV_COMPAT dji_gm6020_current

#include "dji_internal.hpp"

#define GM6020_DEFINE(inst)                                            \
    DJI_MOTOR_DEFINE(                                                  \
        inst,                                                          \
        skywalker::motor::dji::internal::kGM6020CurrentProfile)

DT_INST_FOREACH_STATUS_OKAY(GM6020_DEFINE)
```

这三个文件只能不同在 `DT_DRV_COMPAT` 和 profile symbol。不要把 `setCurrent`、decoder 或 CAN ID 逻辑复制进来。

### 施工 10：新建四个 devicetree binding

目标文件：`dts/bindings/motor/dji-motor-base.yaml`

操作：新建并完整粘贴。

```yaml
description: Common properties for a DJI CAN motor

include: base.yaml

properties:
  can-bus:
    type: phandle
    required: true

  motor-id:
    type: int
    required: true
    minimum: 1

  current-limit-ma:
    type: int
    required: true
    minimum: 0

  gear-ratio-num:
    type: int
    required: true
    minimum: 1

  gear-ratio-den:
    type: int
    required: true
    minimum: 1
```

目标文件：`dts/bindings/motor/dji,m3508-c620.yaml`

```yaml
description: DJI M3508 motor driven by a C620 in CAN current mode

compatible: "dji,m3508-c620"

include: dji-motor-base.yaml

properties:
  motor-id:
    type: int
    required: true
    minimum: 1
    maximum: 8

  current-limit-ma:
    type: int
    required: true
    minimum: 0
    maximum: 20000
```

目标文件：`dts/bindings/motor/dji,m2006-c610.yaml`

```yaml
description: DJI M2006 motor driven by a C610 in CAN current mode

compatible: "dji,m2006-c610"

include: dji-motor-base.yaml

properties:
  motor-id:
    type: int
    required: true
    minimum: 1
    maximum: 8

  current-limit-ma:
    type: int
    required: true
    minimum: 0
    maximum: 10000
```

目标文件：`dts/bindings/motor/dji,gm6020-current.yaml`

```yaml
description: DJI GM6020 with current loop explicitly enabled

compatible: "dji,gm6020-current"

include: dji-motor-base.yaml

properties:
  motor-id:
    type: int
    required: true
    minimum: 1
    maximum: 7

  current-limit-ma:
    type: int
    required: true
    minimum: 0
    maximum: 3000

  current-loop-confirmed:
    type: boolean
    required: true
```

设备树现在只填写不能推导的事实。不要再写 `feedback-id / command-id / command-slot`，它们由 profile 和 `motor-id` 唯一推导。

### 施工 11：整文件替换 Kconfig 和 DJI CMake

目标文件：`drivers/motor/Kconfig`

操作：整文件替换。

```kconfig
config SKYWALKER_MOTOR_DJI
    bool "Unified DJI CAN motor family"
    default y
    depends on CAN

config SKYWALKER_MOTOR_INIT_PRIORITY
    int "Motor driver init priority"
    default 90
    range 0 99

config SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS
    int "DJI feedback freshness timeout"
    default 20
    range 2 1000

config SKYWALKER_DJI_COMMAND_TIMEOUT_MS
    int "DJI command cache timeout"
    default 10
    range 1 1000

config SKYWALKER_DJI_MAX_MOTORS_PER_BUS
    int "Maximum DJI motors owned by one Bus"
    default 12
    range 1 32

config SKYWALKER_DJI_MAX_BUSES
    int "Maximum physical CAN buses with a DJI Bus owner"
    default 3
    range 1 8
```

目标文件：`drivers/motor/dji/CMakeLists.txt`

操作：整文件替换。

```cmake
zephyr_library()

zephyr_library_sources(
    dji_protocol.cpp
    dji_profiles.cpp
    dji_motor.cpp
    dji_bus.cpp
    dji_m3508_instance.cpp
    dji_m2006_instance.cpp
    dji_gm6020_instance.cpp
)
```

现在 CMake 不得再出现 `dji_tx_group.cpp` 或 `dji_m3508.cpp`。`drivers/motor/CMakeLists.txt` 当前已有下面这一行，保留不动：

```cmake
add_subdirectory_ifdef(CONFIG_SKYWALKER_MOTOR_DJI dji)
```

施工 0～11 全部做完后，新的目标文件树应是：

```text
include/drivers/motor/
├── motor.hpp
├── dji_protocol.hpp
├── dji_motor.hpp
└── dji_bus.hpp

drivers/motor/dji/
├── CMakeLists.txt
├── dji_internal.hpp
├── dji_protocol.cpp
├── dji_profiles.cpp
├── dji_motor.cpp
├── dji_bus.cpp
├── dji_m3508_instance.cpp
├── dji_m2006_instance.cpp
└── dji_gm6020_instance.cpp

dts/bindings/motor/
├── dji-motor-base.yaml
├── dji,m3508-c620.yaml
├── dji,m2006-c610.yaml
└── dji,gm6020-current.yaml
```

### 施工 12：新建默认只能发送 0A 的统一 sample

目标文件：`samples/motor/dji_unified/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dji_unified)

target_sources(app PRIVATE src/main.cpp)
```

目标文件：`samples/motor/dji_unified/prj.conf`

```ini
CONFIG_CPP=y
CONFIG_CAN=y

CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

CONFIG_SERIAL=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_CONSOLE_SUBSYS=y
CONFIG_CONSOLE_GETCHAR=y

CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS=20
CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS=20
```

目标文件：`samples/motor/dji_unified/app.overlay`

这个 overlay 只放一台 M3508，电流上限固定为 0A。没有完成后面的硬件检查前，不要改 `current-limit-ma`。

```dts
/ {
    aliases {
        motor0 = &m3508_1;
    };

    m3508_1: motor-1 {
        compatible = "dji,m3508-c620";
        status = "okay";
        can-bus = <&can1>;
        motor-id = <1>;
        current-limit-ma = <0>;
        gear-ratio-num = <3591>;
        gear-ratio-den = <187>;
    };
};
```

目标文件：`samples/motor/dji_unified/src/main.cpp`

这个 sample 启动后只收反馈；必须在串口输入一次 `a`，才会 arm 并周期发送 0A。

```cpp
#include <errno.h>
#include <cstdint>

#include <zephyr/console/console.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>

LOG_MODULE_REGISTER(dji_unified, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

static skywalker::motor::dji::Bus dji_bus;

static int waitForFreshFeedback(const struct device *motor)
{
    const std::int64_t deadline = k_uptime_get() + 2000;
    while (skywalker::motor::getState(motor) !=
           skywalker::motor::State::Ready) {
        if (k_uptime_get() >= deadline) return -ETIMEDOUT;
        k_sleep(K_MSEC(5));
    }
    return 0;
}

static int waitForManualArmToken()
{
    const int init_ret = console_init();
    if (init_ret < 0) return init_ret;

    printk("保持电机架空且 current-limit-ma=0；输入 a 后只发送 0A：\n");
    for (;;) {
        const int ch = console_getchar();
        if (ch < 0) return ch;
        if (ch == 'a' || ch == 'A') return 0;
    }
}

int main()
{
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    if (!device_is_ready(motor)) {
        LOG_ERR("motor device not ready");
        return -ENODEV;
    }

    skywalker::motor::dji::Descriptor descriptor{};
    int ret =
        skywalker::motor::dji::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr ||
        !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }

    ret = dji_bus.init(descriptor.can);
    if (ret < 0) {
        LOG_ERR("Bus init failed: %d", ret);
        return ret;
    }

    ret = dji_bus.attach(motor);
    if (ret < 0) {
        LOG_ERR("Bus attach failed: %d", ret);
        return ret;
    }

    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("No fresh feedback: %d", ret);
        return ret;
    }

    ret = waitForManualArmToken();
    if (ret < 0) {
        LOG_ERR("Manual arm input failed: %d", ret);
        return ret;
    }

    skywalker::motor::dji::FlushReport arm_report{};
    ret = dji_bus.arm(arm_report);
    if (ret < 0 || !arm_report.zero_sent) {
        LOG_ERR("Arm/zero failed: ret=%d zero=%d",
                ret,
                arm_report.zero_sent ? 1 : 0);
        return ret < 0 ? ret : -EIO;
    }

    std::uint32_t print_divider = 0;
    for (;;) {
        ret = skywalker::motor::setCurrent(motor, 0.0f);
        if (ret < 0) {
            skywalker::motor::dji::FlushReport stop_report{};
            dji_bus.stop(stop_report);
            LOG_ERR("setCurrent(0A) failed: %d", ret);
            return ret;
        }

        skywalker::motor::dji::FlushReport report{};
        ret = dji_bus.flush(report);
        if (ret < 0) {
            LOG_ERR("flush failed: ret=%d zero=%d zero_err=%d",
                    ret,
                    report.zero_sent ? 1 : 0,
                    report.zero_tx_error);
            return ret;
        }

        if (++print_divider >= 100u) {
            print_divider = 0;
            skywalker::motor::Feedback feedback{};
            skywalker::motor::dji::RawFeedback raw{};
            const int feedback_ret =
                skywalker::motor::readFeedback(motor, feedback);
            const int raw_ret =
                skywalker::motor::dji::readRawFeedback(motor, raw);

            printk("state=%d feedback_ret=%d raw_ret=%d "
                   "encoder=%u rpm=%d current_raw=%d ts=%llu\n",
                   static_cast<int>(
                       skywalker::motor::getState(motor)),
                   feedback_ret,
                   raw_ret,
                   raw.encoder,
                   raw.speed_rpm,
                   raw.current_raw,
                   static_cast<unsigned long long>(
                       feedback.timestamp_ms));
        }

        k_sleep(K_MSEC(5));
    }
}
```

如果串口输入和日志共用同一个 UART，`console_getchar()` 会独占 pull-style 输入；这个 sample 不同时启用 shell。实车 application 以后应把“输入 a”替换为遥控器/按键产生的单次边沿 token，不能替换成恒 `true`。

### 施工 13：先做协议测试，再构建 sample

目标文件：`tests/motor/dji_protocol/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dji_protocol_test)

target_sources(app PRIVATE
    src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../../drivers/motor/dji/dji_protocol.cpp
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../../../include
)
```

目标文件：`tests/motor/dji_protocol/prj.conf`

```ini
CONFIG_CPP=y
CONFIG_CAN=y
CONFIG_ZTEST=y
```

目标文件：`tests/motor/dji_protocol/tests.yaml`

```yaml
tests:
  skywalker.motor.dji_protocol:
    platform_allow:
      - native_sim
      - native_sim/native/64
```

目标文件：`tests/motor/dji_protocol/src/main.cpp`

```cpp
#include <zephyr/ztest.h>

#include <drivers/motor/dji_protocol.hpp>

namespace dji = skywalker::motor::dji;

ZTEST(dji_protocol, test_decode_signed_golden_vector)
{
    struct can_frame frame{};
    frame.id = 0x201u;
    frame.dlc = 8u;
    frame.data[0] = 0x12u;
    frame.data[1] = 0x34u;
    frame.data[2] = 0xFFu;
    frame.data[3] = 0x9Cu;
    frame.data[4] = 0x00u;
    frame.data[5] = 0x7Bu;
    frame.data[6] = 55u;

    dji::RawFeedback out{};
    zassert_true(dji::decodeFeedback(frame, out));
    zassert_equal(out.encoder, 0x1234u);
    zassert_equal(out.speed_rpm, -100);
    zassert_equal(out.current_raw, 123);
    zassert_equal(out.temperature_raw, 55u);
}

ZTEST(dji_protocol, test_invalid_frame_does_not_change_output)
{
    struct can_frame frame{};
    frame.dlc = 7u;

    dji::RawFeedback out{};
    out.encoder = 77u;
    zassert_false(dji::decodeFeedback(frame, out));
    zassert_equal(out.encoder, 77u);
}

ZTEST(dji_protocol, test_encode_signed_golden_vector)
{
    const std::int16_t command[4] = {
        0x1234,
        -1,
        -16384,
        0,
    };
    struct can_frame frame{};
    zassert_ok(
        dji::buildCommandFrame(frame, 0x200u, command));

    const std::uint8_t expected[8] = {
        0x12u, 0x34u,
        0xFFu, 0xFFu,
        0xC0u, 0x00u,
        0x00u, 0x00u,
    };
    zassert_mem_equal(frame.data, expected, sizeof(expected));
}

ZTEST_SUITE(dji_protocol, nullptr, nullptr, nullptr, nullptr, nullptr);
```

施工完成后只按这个顺序执行：

```bash
# 1. 先确认旧接口真的删干净。
rg -n 'm3508::|M3508Feedback|TxGroup|setCurrentRaw|getCurrentRaw' \
  include drivers samples application tests

rg -n 'dji_m3508[.]cpp|dji_tx_group[.]cpp' \
  drivers/motor/dji/CMakeLists.txt

# 上面两条预期都是零命中。

# 2. 只跑无硬件协议测试。
west twister -T tests/motor/dji_protocol \
  -p native_sim \
  -O build/twister/dji_protocol \
  -v

# 3. 再构建默认 0A sample。
west build -b dm_mc02/stm32h723xx \
  samples/motor/dji_unified \
  -d build/dji_unified \
  -p always \
  -- -DBOARD_ROOT=$PWD
```

不要一看到编译报错就继续写 application。按第一条错误定位：

| 第一条错误 | 先检查 |
|---|---|
| 找不到 `drivers/motor/dji_motor.hpp` | 施工 0 的零宽字符是否真的去掉 |
| CMake 找不到 `dji_m3508.cpp` | 施工 11 是否整文件替换 DJI CMake |
| 找不到 `dji_m3508.hpp` | `dji_tx_group.cpp` 是否仍在树中或 CMake 中 |
| `CONFIG_SKYWALKER_DJI_*` 未定义 | `drivers/motor/Kconfig` 是否整文件替换，sample 是否打开 motor 配置 |
| devicetree 报旧 `dji,m3508` | overlay 是否改成 `dji,m3508-c620`，旧 binding/overlay 是否仍残留 |
| undefined reference: `decodeFeedback` | `dji_protocol.cpp` 是否在 DJI CMake 中 |
| device not ready | 先看 build 生成的 `zephyr.dts` 和 `.config`，不要改 CAN 字节 |

### 第一次上硬件只做这五件事

1. 电机架空，C620 动力侧有随手可断的物理电源；确认 CAN 模式、ID=1、1Mbps 和终端。
2. 保持 `current-limit-ma = <0>`，烧录 sample。此时只应收到反馈，不会自动 arm。
3. 日志出现新鲜 encoder/rpm 后，串口输入一次 `a`。抓包应看到 `0x200` 的 8 个数据字节全为 0。
4. 运行几十秒，确认 `setCurrent(0A)` 和 `flush()` 持续成功。拔物理 CAN 前先断动力；物理断线时零帧同样到不了电调，不能拿非零输出做这个试验。
5. 只有物理急停、零帧抓包、反馈方向和错误升级都确认后，才把 `current-limit-ma` 改成一个经你批准的极小值，并另写限时方向测试；不要直接改这个 0A sample 去持续开环旋转。

完成这一节的标准不是“所有架构都想明白”，而是：

- 旧 M3508 专用接口和 TxGroup 零命中；
- 三型号使用同一 `motor::Api`、同一 `dji_motor.cpp`、同一 `dji::Bus`；
- 型号差异只在 `dji_profiles.cpp` 和 compatible；
- 协议测试通过；
- 统一 sample 能构建，并在人工输入 `a` 后只发送 0A；
- 业务 application 尚未迁移前，不允许加入任何兼容 wrapper。

## 怎么使用这份文档

每一步都按同一种节奏阅读。正文先说明本步解决什么问题、哪些文件负责什么、错误时应该停在哪里；“落到代码”才给示例；“本步自检”是出口。示例中的参数不是实车默认值，尤其是电流、力矩、限位、PID 和通信超时，必须用第 0 步的实测表替换。

后文采用以下回指规则：

```text
“Zephyr 设备接线见第 3 步”
    = 复用 Kconfig → CMake → binding → overlay → DEVICE_DT_INST_DEFINE → sample 的完整链

“纯协议测试见第 2 步”
    = 复用 native_sim + ztest 的完整目录和断言写法

“控制线程见第 8 步”
    = 复用 k_timer 只发信号、线程执行浮点控制和 can_send 的写法

“异步 UART 见第 13 步”
    = 复用双 RX buffer、offset、ring buffer、worker 和 TX 所有权的写法
```

如果本文、参考仓库和手册冲突，优先级始终是：

```text
当前型号/当前赛季官方资料
  > 实车安全约束和实测参数
  > 本文的模块边界与阶段依赖
  > 仓库旧实现和第三方参考实现
```

## 历史起点和最终数据流（施工时以最前面的逐文件区为准）

下面记录的是本轮重构开始前为何要删除旧链，供排错时理解背景；你当前已经开始删除旧文件，真实施工状态以文档最前面的“你当前工作树到底处于什么状态”为准。旧仓库只有一条未完成的 M3508 试验链：Zephyr device、CAN RX、raw 电流缓存、M3508 专用发送组和两个 sample。它不能作为新架构的行为基线：`dji_tx_group.cpp` 写死 `m3508::*`，重复槽会静默覆盖，`clear()`/`disable()` 只清 RAM 不发零帧，发送不受反馈 freshness 和命令 TTL 门控；raw sample 给 RX callback 传空上下文，实际上不会记录反馈；两个 sample 还会默认写入非零 raw `300`。

所以旧 M3508 只提供三类可复用证据：已经验证过的 Zephyr 设备注册形状、CAN 字节布局、现有硬件接线线索。旧公开符号、旧状态语义、旧 sample 行为都不是兼容目标。新的唯一边界是：上层通过公共 `motor` API 用安培写命令、读 SI 单位反馈；DJI 型号差异只存在于 profile；每条物理 CAN 只有一个 `dji::Bus` 拥有所有 DJI 分组发送权。

整个项目只按下面一条线施工：

```text
第 0 步  固定硬件参数和安全边界
   ↓
第 1 步  定死统一 DJI API 和破坏性删除边界
   ↓
第 2 步  先写纯 profile、ID 映射和 codec 测试
   ↓
第 3 步  实现统一 device core 与唯一 CAN Bus，再迁移 M3508
   ↓
第 4 步  启用 M2006/C610 binding，并做集成验证
   ↓
第 5 步  启用 GM6020 电流 binding，并做实物前提验证
   ↓
第 6 步  DM-J4310 MIT 模式（没有当前使用者可暂缓）
   ↓
第 7 步  PID、角度、滤波和轨迹基础
   ↓
第 8 步  Zephyr 控制线程与数据所有权
   ↓
第 9 步  三条单轴闭环
   ↓
第 10 步 纯舵轮数学
   ↓
第 11 步 单模块，再到四模块和底盘状态机
   ↓
第 12 步 两个 application 和纯板间协议
   ↓
第 13 步 Zephyr 异步 UART 传输
   ↓
第 14 步 大小 Yaw 协调
   ↓
第 15 步 裁判 RX、decoder 和 store
   ↓
第 16 步 功率策略
   ↓
第 17 步 UI 与发射机构
   ↓
第 18 步 后期增强、总验收和故障索引
```

---

## 第 0 步：先把代码不能替你决定的事实写下来

电机方向、舵轮几何、Yaw 限位和裁判版本不是软件可以可靠猜出的常量。若这些事实没有单独记录，后续出现方向错误时，人会不断往公式里塞临时负号；出现撞限位时，又会误以为是 PID 参数问题。第 0 步不写控制代码，只建立一张唯一参数表，并约定 raw、A、N·m、rpm、rad/s、m/s 绝不混用。

第一次电机上电必须架空或拆负载，先证明“持续发送全 0”真的能停，再允许极小非零命令。旧 DJI `disable()` 只清某台电机的软件缓存，正是本轮要删除的误导接口；新设计只有本次 Bus 操作返回的 `FlushReport.zero_sent == true`，且它表示所有已知 command group 的零帧都成功，才有软件停机证据。任何舵向、Yaw 和发射机构测试都要有伸手可及、且已经实测有效的物理断电路径。

把下面内容复制到自己的实验记录，而不是悄悄写成源码默认值：

```text
项目与证据
  当前 Git commit：___
  Zephyr commit：6085aadeb337f27ee7411fa2ecbe8cf49164e360
  首次构建日期、工具链：___

电机与总线
  M3508 数量 / 电调 ID / CAN / 安装方向：___
  M2006 数量 / 电调 ID / CAN / 安装方向：___
  GM6020 数量 / ID / CAN / 电流模式 / 零点 / 限位：___
  DM-J4310 Motor ID / Master ID / CAN / P_MAX / V_MAX / T_MAX：___
  DJI 与 DM 发送 ID、接收 ID 冲突表：___

底盘
  模块顺序：FR / FL / RL / RR
  四模块 (x, y)：___ m
  轮半径：___ m
  驱动减速比：___
  四个 drive_sign：___
  四个 steer_sign：___
  四个 steer_zero：___ rad

Yaw 与双板
  大 Yaw 型号 / 方向 / 零点 / 软硬限位：___
  小 Yaw 型号 / 方向 / 中心 / 软硬限位：___
  两轴是否近似同轴：___
  IMU 安装层级：___
  板间链路与引脚：___

裁判与发射
  赛项 / 兵种 / 弹丸类型：___
  比赛前最终确认的官方通信协议版本：___
  常规链路 UART：___，115200 8N1
  图传链路 UART：___，921600 8N1
  物理挡弹、护目、急停负责人：___
```

本步自检：你必须能明确指出每种电机在哪条 CAN、发送哪些 ID、反馈哪些 ID，能说清软件零输出和物理断电两条路径。任何未知项可以暂时标 `UNKNOWN`，但对应后续步骤不得通过出口检查。

---

## 第 1 步：先定死统一 DJI API，再写任何驱动

这一步不是“给旧 M3508 套一层新名字”，而是决定以后所有 DJI 电机只有一条公开调用路径。统一的是命令单位、反馈快照、生命周期、错误语义和总线所有权；型号差异仍然必须真实存在，但只能藏在只读 profile 中。M3508/C620、M2006/C610 和 GM6020 共享 8 字节反馈外形与四槽大端发送形式，不共享量程、ID、温度有效性、减速比和固件前提。

本项目的破坏性迁移规则如下：

- 不保留旧头文件转发，不写 `using` 别名，不写 `deprecated` wrapper，不同时维护新旧 sample。
- 所有调用方在同一次改动中迁移；编译错误就是漏改清单，不靠兼容层消音。
- 旧 M3508 实现只用于核对 CAN 字节和 Zephyr 接线，不能作为新 API 的行为 oracle。
- 正常控制只暴露安培；raw 只用于协议解码、日志和纯测试。上层 PID 不再输出“某型号 raw”。
- DJI 没有一条“使能帧”。软件 arm、缓存清零和总线实际收到全零帧是三件不同的事。
- 每条物理 CAN 只允许一个 `dji::Bus` 发送 DJI 命令。应用不能另行 `can_send()` 同一命令 ID。
- GM6020 本项目只支持已开启电流环的电流模式，不实现电压模式，也不提供含糊的 `setRaw()`。

### 先画清唯一数据流

```text
控制线程
  → motor::setCurrent(device, current_a)
  → 统一 DjiMotorCore：检查 finite / arm / 机构电流上限
  → profile：A → 当前型号 raw
  → 带 generation 和 timestamp 的命令缓存
  → 每条 CAN 唯一 dji::Bus::flush()
  → 校验所有反馈 freshness、命令 TTL、设备归属和组状态
  → 按 command ID 汇成若干个四槽帧
  → can_send()

CAN RX ISR
  → 按本设备 feedback ID 过滤
  → dji::decodeFeedback()
  → profile：raw → A、rpm → rad/s、encoder 回绕累计
  → 一次短 spinlock 提交 Feedback + RawFeedback + timestamp
  → 控制线程只读快照
```

下面这些层不得倒置：

| 层                   | 负责                                                   | 不负责                     |
| -------------------- | ------------------------------------------------------ | -------------------------- |
| `motor.hpp`        | 通用能力位、A/N·m/SI 反馈、状态和安全 dispatch        | DJI ID、四槽打包           |
| `dji_motor.hpp`    | DJI 型号描述与只读 raw 诊断                            | 暴露型号专用 set 函数      |
| 内部 profile         | ID 映射、raw 量程、A 比例、温度能力、减速比默认事实    | PID、机构安全电流          |
| `dji_protocol.cpp` | 纯 8 字节大端 codec                                    | device、锁、时间、CAN 发送 |
| 统一 motor core      | RX、转换、回绕、命令缓存、状态                         | 拥有共享发送帧             |
| `dji::Bus`         | 唯一总线所有者、冲突检查、arm/stop、分组发送、故障锁存 | 控制算法                   |
| application          | 控制目标、机构限位、状态机                             | 手写 DJI CAN 字节          |

### 一次性删除和替换清单

| 旧入口                                                                | 本轮处理         | 唯一新入口                                                         |
| --------------------------------------------------------------------- | ---------------- | ------------------------------------------------------------------ |
| `include/drivers/motor/dji_m3508.hpp`                               | 删除，不留转发头 | `include/drivers/motor/dji_motor.hpp`                            |
| 空的`dji_gm6020.hpp`                                                | 删除             | 同上                                                               |
| `M3508Feedback`、`DjiFeedbackRaw` 双结构                          | 全部删除         | `dji::RawFeedback`                                               |
| `decodeM3508Feedback()`                                             | 删除             | `decodeFeedback()`                                               |
| `buildGroupCurrentFrame()`                                          | 删除             | `buildCommandFrame()`                                            |
| `m3508::setCurrentRaw/getCurrentRaw/readRawFeedback/getCommandInfo` | 删除             | `motor::setCurrent()`、`dji::readRawFeedback()`、内部 snapshot |
| `GroupCommandSource`、`TxGroup`、`bindMotor()`                  | 删除             | `dji::Bus::attach()`                                             |
| `dji,m3508` + 手填三个 CAN ID 属性                                  | 删除             | 型号明确的 compatible +`motor-id` 推导                           |
| `samples/motor/dji_m3508_raw`                                       | 删除             | 纯 ztest 黄金向量                                                  |
| `samples/motor/dji_m3508_driver`                                    | 替换或重命名     | `samples/motor/dji_unified`，默认只发零                          |
| 业务里的`m3508::*`                                                  | 同次全部迁移     | `motor::*` + 每条 CAN 一个 `dji::Bus`                          |

目标文件树只保留一套公开表面：

```text
include/drivers/motor/
├── motor.hpp
├── dji_motor.hpp
├── dji_protocol.hpp
└── dji_bus.hpp

drivers/motor/dji/
├── dji_motor.cpp
├── dji_profiles.cpp
├── dji_protocol.cpp
├── dji_bus.cpp
├── dji_internal.hpp          # 仅 driver 实现和白盒 tests 可 include
├── dji_m3508_instance.cpp    # 只有 compatible/实例宏，没有公开 API
├── dji_m2006_instance.cpp
└── dji_gm6020_instance.cpp

dts/bindings/motor/
├── dji-motor-base.yaml
├── dji,m3508-c620.yaml
├── dji,m2006-c610.yaml
└── dji,gm6020-current.yaml

tests/motor/
├── dji_protocol/
├── dji_motor/
└── dji_bus/

samples/motor/
└── dji_unified/
```

三个 `*_instance.cpp` 只是 Zephyr 为不同 compatible 生成 device 的薄胶水，不得定义公开类型、协议函数或型号专用 setter；它们全部调用同一个 core。型号名可以出现在 profile 和实例文件名中，但不能重新长成三套调用接口。

### 公共 motor API 只谈能力和物理量

旧 `motor::Api` 只有 `set_torque`，而 M3508 实际又从专用 namespace 写 raw，这正是“看似统一、实际分裂”的根源。现在直接做破坏性修改：加入电流命令和电流反馈能力，不重排后又偷偷保留旧路径。

```cpp
#pragma once

#include <cstdint>
#include <errno.h>
#include <zephyr/device.h>

namespace skywalker {
namespace motor {

enum class State : std::uint8_t {
    Offline = 0,
    Ready,
    Fault,
};

enum Capability : std::uint32_t {
    CommandCurrent      = 1u << 0,
    CommandTorque       = 1u << 1,
    FeedbackPosition    = 1u << 8,
    FeedbackVelocity    = 1u << 9,
    FeedbackCurrent     = 1u << 10,
    FeedbackTorque      = 1u << 11,
    FeedbackTemperature = 1u << 12,
};

struct Feedback {
    float position_rad = 0.0f;       // 本文 DJI core 定义为启动基准后的连续输出轴位置
    float velocity_rad_s = 0.0f;     // 输出轴
    float current_a = 0.0f;          // 实际转矩电流
    float torque_nm = 0.0f;          // 只有可靠换算的驱动才置 valid
    float temperature_c = 0.0f;
    std::uint32_t valid = 0;
    std::uint64_t timestamp_ms = 0;
};

struct Api {
    std::uint32_t (*get_capabilities)(const struct device *dev);
    int (*set_current)(const struct device *dev, float current_a);
    int (*set_torque)(const struct device *dev, float torque_nm);
    int (*read_feedback)(const struct device *dev, Feedback *out);
    State (*get_state)(const struct device *dev);
};

inline std::uint32_t capabilities(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) return 0;
    const Api *api = static_cast<const Api *>(dev->api);
    return api->get_capabilities == nullptr
        ? 0u
        : api->get_capabilities(dev);
}

inline int setCurrent(const struct device *dev, float current_a)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const Api *api = static_cast<const Api *>(dev->api);
    const std::uint32_t caps = api->get_capabilities == nullptr
        ? 0u
        : api->get_capabilities(dev);
    if ((caps & CommandCurrent) == 0u ||
        api->set_current == nullptr) {
        return -ENOTSUP;
    }
    return api->set_current(dev, current_a);
}

/* setTorque/readFeedback/getState 采用相同的“先检查对应能力/指针”写法。 */

} // namespace motor
} // namespace skywalker
```

能力位必须通过 `get_capabilities(dev)` 按设备查询，不能是 API 表里的常量：三个型号共用唯一 `dji_motor_api`，但 C610 没有有效温度反馈。wrapper 的前置条件是 `dev` 确实来自 motor binding；任意 Zephyr device 的 `api` 布局没有通用运行时标签，不能把 UART/CAN device 传进来再期待安全识别。DJI 的 `Bus::attach()` 则必须先比较 `dev->api` 是否为统一 DJI core 的 API 表，再读取内部描述，错误型号返回 `-ENOTSUP`。

这里故意不把 `enable/disable` 放回单电机公共 API。DJI 是共享帧协议，单台 `disable()` 无法保证总线已经送出零值；DM-J4310 又有真实的使能帧，两者语义不同。生命周期由各协议族的 owner 管理：DJI 用 `dji::Bus::arm/stop`，DM 后续用自己的显式协议 API。不要为了函数名整齐而制造一个说谎的抽象。

`motor::setCurrent()` 的统一契约：

| 条件                                          | 结果                                                        |
| --------------------------------------------- | ----------------------------------------------------------- |
| `dev == nullptr` 或 `dev->api == nullptr` | `-EINVAL`                                                 |
| 调用者违反前置条件，传入非 motor device       | 程序错误；不能靠强转后的字段猜测                            |
| 驱动不支持电流命令                            | `-ENOTSUP`                                                |
| NaN 或 ±Inf                                  | `-EINVAL`                                                 |
| 超过本节点`current-limit-ma`                | `-ERANGE`，不改旧缓存                                     |
| DJI bus 尚未 arm 或已 Fault                   | `-EACCES`，不改旧缓存                                     |
| 反馈 Offline                                  | `-EHOSTDOWN`，不改旧缓存                                  |
| 成功                                          | 转换并写入完整 raw、递增 generation、记录单调时间；仍未发送 |

成功返回只表示“新命令已安全缓存”，不表示电机已经收到。只有 `Bus::flush()` 成功报告目标帧已发，才算完成一次输出提交。

### DJI 公开头不再出现型号 namespace

```cpp
#pragma once

#include <cstdint>
#include <zephyr/device.h>

namespace skywalker {
namespace motor {
namespace dji {

enum class Model : std::uint8_t {
    M3508C620 = 0,
    M2006C610,
    GM6020Current,
};

struct RawFeedback {
    std::uint16_t encoder = 0;
    std::int16_t speed_rpm = 0;
    std::int16_t current_raw = 0;
    std::uint8_t temperature_raw = 0;
    std::uint64_t timestamp_ms = 0;
};

struct Descriptor {
    Model model = Model::M3508C620;
    const struct device *can = nullptr;
    std::uint8_t motor_id = 0;
    std::uint16_t feedback_id = 0;
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
    float protocol_current_max_a = 0.0f;
    float configured_current_limit_a = 0.0f;
    float gear_ratio = 1.0f;
    bool temperature_valid = false;
};

int describe(const struct device *dev, Descriptor &out);
int readRawFeedback(const struct device *dev, RawFeedback &out);

} // namespace dji
} // namespace motor
} // namespace skywalker
```

`Descriptor` 是只读诊断结果，不允许应用修改后再喂回驱动。`describe()` 成功只复制配置，不改状态；空参数返回 `-EINVAL`，非统一 DJI device 返回 `-ENOTSUP`。`readRawFeedback()` 在首帧前返回 `-ENODATA` 且不改输出；之后即使已经 Offline，仍可返回最后快照和原 timestamp，调用者必须同时检查 `getState()`/age。两者都只复制短快照，可由普通线程调用；不提供 ISR 控制用途。

正常控制甚至不需要 include `dji_motor.hpp`；只有总线装配、诊断和测试需要它。公开层没有 `setCurrentRaw()`，因此控制器不可能绕过安培单位和节点安全上限。

### 型号 profile 是唯一差异表

按仓库固定版官方手册记录下面事实；实施时在 profile 单元测试中逐项锁死：

| 型号         |   ID |    feedback ID | current command ID |       slot | raw 满量程 | 对应电流 | 温度 |
| ------------ | ---: | -------------: | -----------------: | ---------: | ---------: | -------: | ---- |
| M3508 + C620 | 1..4 | `0x200 + ID` |          `0x200` | `ID - 1` |    ±16384 |   ±20 A | 有效 |
| M3508 + C620 | 5..8 | `0x200 + ID` |          `0x1FF` | `ID - 5` |    ±16384 |   ±20 A | 有效 |
| M2006 + C610 | 1..4 | `0x200 + ID` |          `0x200` | `ID - 1` |    ±10000 |   ±10 A | 无   |
| M2006 + C610 | 5..8 | `0x200 + ID` |          `0x1FF` | `ID - 5` |    ±10000 |   ±10 A | 无   |
| GM6020 电流  | 1..4 | `0x204 + ID` |          `0x1FE` | `ID - 1` |    ±16384 |    ±3 A | 有效 |
| GM6020 电流  | 5..7 | `0x204 + ID` |          `0x2FE` | `ID - 5` |    ±16384 |    ±3 A | 有效 |

M3508/C620 依据 `motor_docs/C620_User_Guide_V1.01.pdf` 的 CAN 章节；M2006/C610 依据 `motor_docs/C610_User_Guide.pdf` 第 7～8 页；GM6020 电流模式依据 `motor_docs/RoboMaster GM6020直流无刷电机使用说明20231013.pdf` 第 6～7 页。GM6020 电流模式还要求电机固件 `>= 1.0.11.2`、RoboMaster Assistant `>= 2.7`，并在参数中打开电流环。软件不能从普通反馈帧证明这个开关已经打开，所以第 0 步必须保存固件截图和设置证据。

旧 GM6020 手册只描述电压模式；2023 v1.4 手册中的电压给定范围是 `±25000`，不是旧稿误写的 `±30000`。本项目不实现 `0x1FF/0x2FF` 电压命令。若电流环前提未被人工确认，`dji,gm6020-current` 节点不得设为 `okay`。

还要看到一个容易被“统一格式”掩盖的冲突：GM6020 ID 1 的反馈 `0x205` 与 C620/C610 ID 5 的反馈相同。每条 `dji::Bus` 必须拒绝重复 feedback ID，即使两台电机属于不同 command group；不能只检查发送槽。

本步自检：

- 你能解释为什么统一单位选 A，而不是 raw 或未经验证的 N·m。
- 你能指出软件缓存为零、总线发出零帧和物理断电三者的区别。
- 目标公开头里没有 `m3508`、`m2006`、`gm6020` namespace。
- 一条 CAN 只有一个 owner；应用不能自己拼 DJI 命令帧。
- 删除清单已进入你的提交计划，而不是被“以后再清”留成双轨。

---

## 第 2 步：先写纯 profile、ID 映射和 codec 测试

这一层不 include devicetree、`struct device`、锁、时间或 `can_send()`。它只回答四个问题：某型号/ID 对应哪个地址；安培如何变成 raw；八字节如何解码；四个有符号 raw 如何按大端打包。先让这些纯函数在 `native_sim` 通过，再接 Zephyr device。

### profile 与地址解析

```cpp
struct Profile {
    Model model;
    std::uint8_t max_motor_id;
    std::uint16_t feedback_base;
    std::uint16_t low_command_id;
    std::uint16_t high_command_id;
    std::int16_t command_raw_max;
    float current_max_a;
    bool temperature_valid;
};

struct Endpoint {
    std::uint16_t feedback_id = 0;
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
};

int resolveEndpoint(const Profile &profile,
                    std::uint8_t motor_id,
                    Endpoint &out);

int currentToRaw(const Profile &profile,
                 float current_a,
                 std::int16_t &out);

int rawToCurrent(const Profile &profile,
                 std::int16_t raw,
                 float &out);
```

`resolveEndpoint()` 必须先构造局部 `next`，全部校验成功后才赋给 `out`。失败不能留下半新的 command ID 和旧 slot。ID 0、M3508/M2006 的 ID 9、GM6020 的 ID 8 均返回 `-ERANGE`。

A/raw 转换使用明确的舍入和边界。先检查 `abs(current_a) <= current_max_a`，再做乘法和 `lround`；不能先让巨大但 finite 的输入溢出后才判范围。`rawToCurrent()` 同样先检查 profile 和 raw 协议范围，失败返回 errno 且不改 `out`：

```text
前置：
  current_a finite
  raw_max > 0
  current_max_a finite 且 > 0
  abs(current_a) <= current_max_a

通过前置后：
  scaled = current_a * raw_max / current_max_a
  raw = lround(scaled)

越过协议满量程：
  返回 -ERANGE
  不 clamp，不修改 out

例：
  C620 +10 A  → +8192
  C620 -20 A  → -16384
  C610 +5 A   → +5000
  GM6020 -1.5 A → -8192
```

机构安全上限在 device core 的 `current-limit-ma` 检查；profile 满量程是协议硬边界。两层不能合并，否则“协议允许 20A”会被误当成机构允许 20A。

### 唯一 codec

```cpp
bool decodeFeedback(const struct can_frame &frame,
                    RawFeedback &out);

void buildCommandFrame(struct can_frame &frame,
                       std::uint16_t command_id,
                       const std::int16_t command[4]);
```

`decodeFeedback()` 至少拒绝：

- `dlc != 8`；
- 扩展帧、RTR 帧或 CAN FD 帧；
- 空输出指针版本如果你选择指针 API。

解码必须先写局部对象，成功后一次赋给 `out`。`DATA[0..1]` 是 encoder，`[2..3]` 是有符号 rpm，`[4..5]` 是有符号实际转矩电流，`[6]` 只是原始字节；是否为有效温度由 profile 决定，codec 不猜型号。

C++14 中把大于 `INT16_MAX` 的 `uint16_t` 直接 cast 成 `int16_t` 是实现定义行为。按二补码位模式显式还原：

```cpp
static std::int16_t signedFromBits(std::uint16_t bits)
{
    const std::int32_t value =
        (bits <= 0x7FFFu)
            ? static_cast<std::int32_t>(bits)
            : static_cast<std::int32_t>(bits) - 0x10000;
    return static_cast<std::int16_t>(value); // 此时已保证在 int16_t 范围
}
```

编码负数时不要对负的 `int16_t` 直接右移。先保持 16 位位模式：

```cpp
for (std::size_t i = 0; i < 4; ++i) {
    const std::uint16_t bits =
        static_cast<std::uint16_t>(command[i]);
    frame.data[i * 2] =
        static_cast<std::uint8_t>((bits >> 8) & 0xFFu);
    frame.data[i * 2 + 1] =
        static_cast<std::uint8_t>(bits & 0xFFu);
}
```

`buildCommandFrame()` 每次从全新的 `can_frame{}` 生成完整标准数据帧，显式设置 `id`、`flags = 0`、`dlc = 8` 和全部八字节，不能残留上一帧 flags/data。

### 纯测试目录和矩阵

```text
tests/motor/dji_protocol/
├── CMakeLists.txt
├── prj.conf
├── tests.yaml
└── src/main.cpp
```

`prj.conf` 至少启用 `CONFIG_ZTEST=y`；`tests.yaml` 同时列出实际要跑的 `native_sim`/64 位平台，`CMakeLists.txt` 只编译本测试和 profile/codec 源文件，并显式把 driver private 目录作为白盒测试 include。沿用当前 Zephyr 提交支持的 ztest 入口。最低测试矩阵：

| 类别            | 输入                            | 预期                                                              |
| --------------- | ------------------------------- | ----------------------------------------------------------------- |
| 解码黄金向量    | `12 34 FF 9C 00 7B 37 00`     | encoder`0x1234`、rpm `-100`、current `123`、raw temp `55` |
| 符号端点        | `80 00`、`FF FF`、`7F FF` | `-32768`、`-1`、`32767`                                     |
| 非法帧          | DLC 0/7/9、RTR、IDE、FDF        | false，out 保持原值                                               |
| 编码黄金向量    | `0x1234,-1,-16384,0`          | `12 34 FF FF C0 00 00 00`                                       |
| M3508/C620 地址 | ID 1/4/5/8                      | 两个 group 和四个 slot 都正确                                     |
| M2006/C610 地址 | ID 1/8                          | 与 C620 地址相同、量程不同                                        |
| GM6020 地址     | ID 1/4/5/7                      | `0x1FE/0x2FE` 与 `0x205..0x20B`                               |
| 非法 ID         | 0、9、GM 8                      | `-ERANGE`，out 不变                                             |
| A/raw           | 每型号 0、半量程、正负满量程    | 精确端点和对称性                                                  |
| 越界/NaN/Inf    | 任一型号                        | `-EINVAL/-ERANGE`，out 不变                                     |
| 温度能力        | 三个 profile                    | C610 false，其余 true                                             |
| 地址冲突        | GM ID1..4 + C620/C610 ID5..8    | 留到第 3 步 Bus attach 矩阵，逐对拒绝`0x205..0x208`             |

测试中不要 include 旧头，更不能通过 wrapper 间接覆盖旧符号。建议由你运行：

```bash
west twister -T tests/motor/dji_protocol -p native_sim -v
```

本步自检：所有纯函数不引用设备或全局状态；旧 `M3508Feedback`、`decodeM3508Feedback`、`buildGroupCurrentFrame` 已从目标设计消失；所有型号的 profile 表和官方手册逐项一致；`±16384` 等协议端点只在纯测试中验证，不在硬件台架上发送。

---

## 第 3 步：实现统一 device core 和唯一 dji::Bus，再迁移 M3508

现在才接 Zephyr。先完成公共 core、三个 binding 形状和 Bus 状态机，再把 M3508 作为第一个 profile 接入。不要先把旧 `dji_m3508.cpp` 改到“勉强能编”，然后继续围着它补抽象。

### 先把构建接线改成最终形状

仓库根 `Kconfig → drivers/Kconfig → drivers/motor/Kconfig` 和根 `CMakeLists.txt → drivers/motor` 的外层链已经存在，不要复制一套。电机目录最终只保留统一开关和安全时序参数：

```kconfig
config SKYWALKER_MOTOR_DJI
    bool "Unified DJI CAN motor family"
    default y

config SKYWALKER_MOTOR_INIT_PRIORITY
    int "Motor driver init priority"
    default 90
    range 0 99

config SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS
    int "DJI feedback freshness timeout"
    default 20
    range 2 1000

config SKYWALKER_DJI_COMMAND_TIMEOUT_MS
    int "DJI command cache timeout"
    default 10
    range 1 1000

config SKYWALKER_DJI_MAX_MOTORS_PER_BUS
    int "Maximum DJI motors owned by one Bus"
    default 12
    range 1 32
```

```cmake
# drivers/motor/dji/CMakeLists.txt
zephyr_library()
zephyr_library_sources(
    dji_protocol.cpp
    dji_profiles.cpp
    dji_motor.cpp
    dji_bus.cpp
    dji_m3508_instance.cpp
    dji_m2006_instance.cpp
    dji_gm6020_instance.cpp
)
```

第 3 步初次施工时，后两种 instance 可以先存在但没有 `okay` 节点；不要为每个型号再加一套 Kconfig。统一 sample 的配置入口：

```ini
# samples/motor/dji_unified/prj.conf
CONFIG_CPP=y
CONFIG_CAN=y
CONFIG_LOG=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS=20
CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS=10
```

```cmake
# samples/motor/dji_unified/CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dji_unified)
target_sources(app PRIVATE src/main.cpp)
```

不要在头文件定义普通非模板函数来省一个 `.cpp`；当前半迁移头中的两个 wrapper 正是 ODR 风险来源。

### devicetree 只填写独立事实

旧 binding 同时要求 `feedback-id`、`command-id` 和 `command-slot`，三个值其实都由“型号 + 电调 ID”决定，允许手填只会制造自相矛盾配置。新公共 binding 只保留无法推导的事实：

```yaml
# dts/bindings/motor/dji-motor-base.yaml
include: base.yaml

properties:
  can-bus:
    type: phandle
    required: true

  motor-id:
    type: int
    required: true
    minimum: 1

  current-limit-ma:
    type: int
    required: true
    minimum: 0

  gear-ratio-num:
    type: int
    required: true
    minimum: 1

  gear-ratio-den:
    type: int
    required: true
    minimum: 1
```

三个具体 binding 必须继续收紧范围：M3508/C620 与 M2006/C610 的 `motor-id maximum: 8`，GM6020 的 `maximum: 7`；`current-limit-ma` 的 maximum 分别为 20000、10000、3000。每个具体文件都 `include: dji-motor-base.yaml`。`dji,gm6020-current.yaml` 还要把 `current-loop-confirmed` 声明为 required boolean；它不是运行时探测，只是迫使配置者留下人工确认。不要用一个可随意拼写的 `model = "..."` 字符串代替具体 compatible。schema 是第一道门，runtime 仍要重验，不能依赖强转或浮点除法替坏 DTS“兜底”。

```dts
m3508_1: motor-1 {
    compatible = "dji,m3508-c620";
    status = "okay";
    can-bus = <&can1>;
    motor-id = <1>;
    current-limit-ma = <0>;       /* 首次台架前必须保持 0。 */
    gear-ratio-num = <3591>;
    gear-ratio-den = <187>;
};

gm6020_1: motor-2 {
    compatible = "dji,gm6020-current";
    status = "disabled";          /* 固件/电流环证据完成后才改 okay。 */
    can-bus = <&can2>;
    motor-id = <1>;
    current-limit-ma = <0>;
    gear-ratio-num = <1>;
    gear-ratio-den = <1>;
    current-loop-confirmed;
};
```

如果 YAML 不允许 `current-limit-ma = 0` 通过正式范围校验，就让节点保持 `disabled`，或在“台架锁定”Kconfig 下强制 driver 拒绝非零；不要为了通过 schema 填一个未经验证的危险值。M3508 官方手册参数表给出减速比 `3591/187 ≈ 19.2032`，所以标准减速箱直接用这个有理数；若当前实物换过减速箱，则改为实物证据值，不要在源码散落 `19.0f`。

初始化时按顺序检查：

```text
dev/config/data 非空
  → CAN device ready
  → motor-id 对当前 profile 合法
  → current-limit-ma >= 0 且不超过协议满量程
  → gear-ratio-num > 0 且 gear-ratio-den > 0
  → resolveEndpoint(profile, motor-id)
  → 在 RAM 派生 finite 且 > 0 的 current_limit_a / gear_ratio
  → 完整初始化 endpoint、反馈、命令、epoch、Offline/Fault 状态
  → CAN 为 1Mbps 的板级配置已确认
  → 最后注册精确标准帧 filter；注册成功后不再清 RAM
```

任一步失败都让 `device_is_ready()` 为 false，不带错配置继续跑。

### core 数据结构和 ISR 边界

```cpp
struct DjiConfig {
    const struct device *can;
    const Profile *profile;
    std::uint32_t motor_id;
    std::uint32_t current_limit_ma;
    std::uint32_t gear_ratio_num;
    std::uint32_t gear_ratio_den;
};

struct DjiData {
    skywalker::motor::Feedback feedback{};
    skywalker::motor::dji::RawFeedback raw_feedback{};
    Endpoint endpoint{};  // init 时由 profile + motor_id 一次性推导
    float current_limit_a = 0.0f;
    float gear_ratio = 0.0f;

    std::int16_t command_raw = 0;
    std::uint64_t command_stamp_ms = 0;
    std::uint64_t command_generation = 0;
    std::uint64_t command_epoch = 0;
    bool armed = false;
    bool fault_latched = false;

    std::uint16_t last_encoder = 0;
    std::int64_t total_encoder_ticks = 0;
    bool has_encoder = false;
    std::uint64_t last_rx_ms = 0;

    struct k_spinlock lock{};
    int rx_filter_id = -1;
};
```

`dev->config` 是只读常量，所以原始整数事实留在 `DjiConfig`，推导出的 route、A 上限和 gear ratio 放在 `DjiData`，并且必须在注册 RX filter 前完成。这样不会在校验前除零或把 257 静默截成 motor ID 1。否则 CAN 已启动时 callback 可能先写数据，随后又被 init 的无锁清零覆盖。`dji_internal.hpp` 必须以 `extern const` 声明三个 profile symbol 和唯一 `dji_motor_api`，各自在一个 `.cpp` 中单一定义，避免 namespace-scope `const` 变成跨 TU 不可见。三个实例文件只展开同一个宏：

```cpp
#define DJI_MOTOR_DEFINE(inst, profile_symbol)                              \
    static DjiData dji_data_##inst;                                         \
    static const DjiConfig dji_config_##inst = {                            \
        DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                      \
        &(profile_symbol),                                                  \
        DT_INST_PROP(inst, motor_id),                                        \
        DT_INST_PROP(inst, current_limit_ma),                                \
        DT_INST_PROP(inst, gear_ratio_num),                                  \
        DT_INST_PROP(inst, gear_ratio_den),                                  \
    };                                                                      \
    DEVICE_DT_INST_DEFINE(inst,                                             \
                          djiMotorInit, nullptr,                             \
                          &dji_data_##inst, &dji_config_##inst,             \
                          POST_KERNEL,                                      \
                          CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY,             \
                          &dji_motor_api);
```

```cpp
// dji_m3508_instance.cpp
#define DT_DRV_COMPAT dji_m3508_c620
#include "dji_internal.hpp"

#define M3508_DEFINE(inst) DJI_MOTOR_DEFINE(inst, kM3508C620Profile)
DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)
```

M2006/GM6020 实例文件只换 `DT_DRV_COMPAT` 和 profile symbol。三个宏都明确以分号结束并保持完全同形。

统一 `getState()` 的优先级固定为：`fault_latched` 为真则 `Fault`；否则从未收到反馈或反馈 age 超限则 `Offline`；否则 `Ready`。Bus 进入 Fault 时给所有已 attach motor 锁存 fault；只有成功 `stop/recover` 回到 Safe 才清除。查询状态本身不修改缓存或恢复设备。

RX callback 只做固定时间工作：

```text
检查 frame/user_data
  → decodeFeedback 到局部 raw
  → 根据 profile 生成局部 Feedback
  → encoder 差值用 int32_t 计算
  → delta > 4096 则减 8192
  → delta < -4096 则加 8192
  → rpm × 2π/60 ÷ gear_ratio
  → current_raw × current_max_a/raw_max
  → 只有 profile.temperature_valid 才设置温度 valid
  → 取单调 timestamp
  → spinlock 内一次提交所有字段
```

第一次 encoder 只建立基准，`position_rad = 0`，随后报告相对启动点的连续输出轴位置。若舵向需要跨重启机械绝对零点，在关节配置层保存 `zero_encoder` 并做校准；不要把某台机器的零点写进通用 DJI driver。

callback 运行在 CAN 接收上下文，不能 sleep、打印 1kHz 日志、跑 PID、调用 `can_send()` 或拿可阻塞 mutex。`readFeedback()` 和 `readRawFeedback()` 只复制快照；它们不清状态、不延长 freshness。

### 命令 generation 与 TTL

`motor::setCurrent()` 在 core 中必须同时满足：

```text
bus 已 armed
反馈仍 Ready
current_a finite
abs(current_a) <= current_limit_a
A/raw 转换成功
```

然后在一个短 spinlock 中写 `command_raw`、`command_stamp_ms`、递增 `command_generation`，并复制当前 Bus 的 `lifecycle_epoch` 到 `command_epoch`。不要用 `generation > arm_generation`：回绕和 post-arm 零命令都会让这个比较含糊。`arm()` 的零帧成功后才递增非零 epoch；随后在锁内给每台写 `raw=0`、新 timestamp/generation/epoch，最后统一打开 Bus/device armed gate。`flush()` 只接受 `command_epoch == lifecycle_epoch` 的快照。`stop/Fault` 的第一步关闭所有 gate、使 epoch 失效并清缓存，然后才尝试零帧；epoch 递增溢出时直接锁 Fault，不允许回绕复用。

命令 TTL 与反馈 heartbeat 使用本步开头两个独立 Kconfig。实际控制周期若是 2ms，默认值仍要靠台架统计抖动后裁决。只要 owner 线程仍在周期调用 `flush()`，`command_stamp` 过期时即使反馈仍在 1kHz 更新，也必须进入全零故障路径。TTL 不能保护 owner 线程本身完全卡死：这种失效只能由独立硬件 watchdog、已经验证的动力切断或电调明确文档化的无帧超时兜底；本文不得把它宣传成软件已发零帧。

### dji::Bus 是共享帧的唯一 owner

```cpp
enum class BusState : std::uint8_t {
    Uninitialized = 0,
    Safe,
    Armed,
    Fault,
};

struct FlushReport {
    int preparation_error = 0;  // stale/offline/ownership/read 等第一个错误
    int command_tx_error = 0;   // 非零或目标帧发送错误
    int zero_tx_error = 0;      // 故障后零帧发送错误
    std::uint16_t target_failed_command_id = 0;
    std::uint16_t zero_failed_command_id = 0;
    std::uint8_t preparation_failed_slot = 0xFF;
    std::uint8_t zero_groups_expected = 0;
    std::uint8_t zero_groups_attempted = 0;
    std::uint8_t zero_groups_succeeded = 0;
    bool zero_sent = false;
};

class Bus {
public:
    Bus() = default;
    Bus(const Bus &) = delete;
    Bus &operator=(const Bus &) = delete;
    Bus(Bus &&) = delete;
    Bus &operator=(Bus &&) = delete;

    int init(const struct device *can);
    int attach(const struct device *motor);
    int arm(FlushReport &report);
    int flush(FlushReport &report);
    int stop(FlushReport &report);
    int recover(FlushReport &report);
    BusState state() const;

private:
    /* 固定容量数组；对象必须是静态/进程生命周期，不在运行期 new/delete。 */
};
```

完整契约：

- `init(can)`：CAN 必须 ready；加锁、原子认领全局 owner registry，同一 CAN 已被另一 `Bus` 认领则返回 `-EBUSY`；由这个唯一 owner 调 `can_start(can)`，把 `-EALREADY` 当成功，其余失败回滚 registry。`stop()` 只停电机输出，不 `can_stop()` 共享控制器。`init` 只允许一次。
- `attach(motor)`：只在 Safe 状态；拒绝不同 CAN（`-EXDEV`）、重复 device（`-EALREADY`）、重复 feedback ID 或同 command ID/slot（`-EADDRINUSE`）、容量耗尽（`-ENOSPC`）。
- `arm(report)`：只接受 Safe；要求至少一个 motor、全部反馈 Ready，并且上层刚消费了本次启动后新产生的人工 arm token；先向所有已知 command group 发送全零，成功后才按上述 epoch 顺序设置 Armed。失败先锁 Fault，再保留原 report。
- `flush(report)`：只允许 owner 控制线程调用。先在局部数组快照所有绑定电机，再构造全部 group 帧；任何 motor disarmed、Offline、命令过期、epoch 非法或 snapshot 失败，都不发送目标帧，先关闭 gate/锁存 Fault，再给该 Bus 的所有 command group 尝试全零。
- `stop(report)`：Uninitialized 返回 `-EACCES`；Armed 先关 gate/失效 epoch/清缓存，再发全零，成功到 Safe、失败到 Fault；Safe 幂等地再发全零并保持 Safe；Fault 可以再尝试零帧但无论成功与否都保持 Fault，不能借 `stop` 绕过人工恢复。
- `recover(report)`：只接受 Fault，并且必须发生在根因排除和新的人工恢复动作之后；再次清缓存并发送全零，成功只回到 Safe，绝不自动 Armed。之后仍需新的 arm token 和 `arm()`。
- `state()`：只读快照，不能隐式恢复。
- 一个 `flush()` 周期中，每个 command ID 最多发送一次。四台同组电机必须先全部计算、全部缓存，再 flush。

`init/attach/arm/flush/stop/recover` 以及 attached motors 的 `setCurrent()` 必须由同一 owner 线程串行调用，ISR 不得调用；`state()` 通过 atomic/短锁返回快照。这个约束让 lifecycle 和命令提交之间没有“停机途中又写入非零”的窗口；若未来改成独立 TX service，必须重新设计批量提交协议，不能只把 `flush()` 随手搬到另一个线程。

返回值优先级必须固定：零帧发送失败最优先返回，因为此时物理输出未知；其次是目标帧发送错误；最后是 preparation error。`FlushReport` 同时保留三类错误，不能用单个 `int` 丢掉“原故障 + 零帧也失败”的证据。发送某个零分组失败后仍继续尝试其余分组；`zero_sent` 严格定义为 `expected > 0 && attempted == expected && succeeded == expected`。

`flush()` 的故障伪代码：

```text
report = {}
if state != Armed:
    report.preparation_error = -EACCES
    先关闭全部 armed gate、使 epoch 失效并锁存 Fault
    尝试所有 group 全零
    return 按优先级选错误

复制所有 command snapshot 到局部数组
if 任一 snapshot 非法/过期/offline:
    report 记录第一个坏 slot
    先关闭全部 armed gate、使 epoch 失效并锁存 Fault
    尝试所有 group 全零
    return

为每个 command ID 从 [0,0,0,0] 构造完整帧
依次发送目标帧
if 任一 can_send 失败:
    记录 command_tx_error
    先关闭全部 armed gate、使 epoch 失效并锁存 Fault
    尝试所有 group 全零
    return

return 0
```

“尝试全零”不等于已经停机。只有 `report.zero_sent == true` 才有软件停机证据；`zero_tx_error < 0` 时立即走物理断电，不在循环中继续刷日志或重试非零。

### M3508 迁移顺序

按下面文件级顺序手工施工，每完成一项就做对应静态检查：

1. 修改 `motor.hpp`：加入 current capability/feedback，删除误导性的单电机 DJI enable/disable 依赖。
2. 将当前半迁移的 `dji_protocol.hpp/.cpp` 一次性改成唯一 `RawFeedback/decodeFeedback/buildCommandFrame`；所有非模板头文件实现移到 `.cpp`。
3. 新增 profile 纯函数和第 2 步测试。
4. 新增统一 `dji_motor.hpp/.cpp` 与具体 bindings；先只启用 M3508/C620 profile。
5. 新增 `dji_bus.hpp/.cpp` 和 fake-CAN 测试。
6. 将 `drivers/motor/dji/CMakeLists.txt` 只列新源文件。
7. 把统一 sample 默认锁在 current limit 0、RX-only/周期零帧。
8. 迁移所有 application/sample 调用方。
9. 删除旧头、旧 TxGroup、旧 raw sample 和旧 binding。
10. 做 clean build；不允许旧 build 目录掩盖漏删依赖。

静态负向验收应全部为零命中，注释和文档迁移表可按文件类型排除：

```bash
rg -n 'm3508::|M3508Feedback|decodeM3508Feedback|buildGroupCurrentFrame' \
  include drivers samples application tests

rg -n 'GroupCommandSource|TxGroup|bindMotor|dji_m3508[.]hpp|DjiFeedbackRaw' \
  include drivers samples application tests

rg -n 'buildGroupCommandFrame|getCommandInfo|SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS' \
  include drivers samples application tests

rg -n 'feedback-id|command-id|command-slot|compatible = "dji,m3508"' \
  dts samples application

test ! -e include/drivers/motor/dji_m3508.hpp
test ! -e samples/motor/dji_m3508_raw
test ! -e samples/motor/dji_m3508_driver
```

### 测试和第一次硬件回归

`dji_motor` 纯/半纯测试至少覆盖 encoder `8191→0`、`0→8191`、首次基准、gear ratio、C610 温度 invalid、Offline timeout、NaN/超限不改缓存、默认 disarmed、重新 arm 不复活旧 generation、command TTL。

`dji_bus` 用 fake CAN 覆盖：

- 同 CAN 第二个 owner `-EBUSY`；
- Bus 不可复制/移动，init 失败会回滚 owner registry；
- 跨 CAN attach `-EXDEV`；
- 重复 feedback ID、重复 command slot；
- M3508 与 M2006 可在同 command group 的不同槽各自按 A/raw 比例打包；
- GM6020 形成 `0x1FE/0x2FE`；
- 空槽恒为零；
- 任一 offline/stale/read 失败导致所有 group 全零并 Fault；
- 目标发送失败后会尝试零帧；
- 零帧失败在 report 中可见且错误优先；
- stop 后不能 flush 非零；
- Safe/Armed/Fault/Uninitialized 下 stop/recover 的完整状态矩阵；
- post-arm zero、stop→recover→arm、epoch 失配/溢出均不能复活旧非零；
- 启动后没有新的人工 arm token 时永不 Armed；
- TTL 的 `==`/`>` 边界用 fake clock 测；owner 线程完全停止时不得伪报零帧；
- recover 后仍需新的人工动作、重新 arm 和新 generation。

由你运行，本文不生成 build artifact：

```bash
west twister -T tests/motor -p native_sim -v \
  -O build/twister/motor

west build -b dm_mc02/stm32h723xx \
  samples/motor/dji_unified \
  -d build/dji_unified -p always \
  -- -DBOARD_ROOT=$PWD
```

第一次 M3508 台架只按以下顺序：

```text
电机/轮子架空，独立物理断电可触达
  → 确认 24V 供电来源、CAN_H/L、两端 120Ω、CAN 1Mbps
  → 只收反馈，核对唯一 ID 和 1kHz 时间戳
  → current-limit 保持 0，Bus arm 只发送所有 group 全零
  → 停止更新命令但保持 owner 循环 flush，验证 command TTL 触发全零 + Fault
  → fake CAN 或只抑制 RX、保持 TX 链路，验证 stale 触发全零 + Fault
  → current-limit=0 或动力级隔离时再拔物理 CAN；预期零帧不可达并升级物理断电
  → 模拟 can_send 失败，确认软件报告物理状态未知
  → 保存抓包后，人工批准单电机、限时、极小非零方向脉冲
  → 立即 stop、确认零帧、物理断电
```

注意 DM-MC02 板级 DTS 的 CAN1/2/3 配置为 1Mbps，但板上 `power1/power2` 是 boot-off。必须先确认 C620 使用外部 24V 还是板载受控 XT30 供电，不能从 sample 能编译推断电调已经安全供电。C620/C610 的校准过程会主动持续转动；必须卸载并按官方手册单独执行。CAN 与 PWM/串口调参线的切换也必须断电。

本步自检：统一 sample 能 clean build；旧公开符号在业务树零命中；M3508 只通过 `motor::setCurrent(A)` 和 `dji::Bus` 工作；默认上电没有任何非零；重复 ID/槽在 arm 前被拒绝；任一 stale 路径有全零抓包；全零发送失败时操作员知道必须物理断电，而不是继续调 PID。

---

## 第 4 步：启用 M2006/C610 binding，证明统一 core 不用分叉

三型号 profile 已在第 2 步一次性冻结；这里不再“新增”第二份协议真值，而是启用 M2006/C610 binding、补集成/硬件向量。不得新建 `dji_m2006.hpp`、`m2006::setCurrentRaw()` 或复制一份 driver；公共 `motor::setCurrent()`、`dji::RawFeedback`、device core、`dji::Bus` 与 sample 调用一行都不变。若接入第二种型号时必须修改 application 控制流程，说明第 1～3 步仍没有真正统一。

### 只允许出现的型号差异

| 属性            | M2006/C610                                |
| --------------- | ----------------------------------------- |
| motor ID        | 1..8                                      |
| feedback        | `0x200 + ID`                            |
| current command | ID 1..4 为`0x200`，ID 5..8 为 `0x1FF` |
| slot            | ID 1..4 为`ID-1`，ID 5..8 为 `ID-5`   |
| raw 满量程      | `±10000`                               |
| 转矩电流满量程  | `±10 A`                                |
| 默认减速比证据  | `36:1`                                  |
| `DATA[6..7]`  | Null；不得设置温度 valid                  |
| 反馈频率        | 默认 1kHz，可由 Assistant 修改            |

C610 手册把 `DATA[4..5]` 称为实际输出转矩电流，因此统一 feedback 可以给出 `current_a`；`DATA[6..7]` 明确为空，不能把零字节解释成 0℃。`FeedbackTemperature` capability 和 valid 位都必须关闭。

增加 profile 后，先补这些纯测试：

- ID 1/4/5/8 的 route；
- `-10A/0/+10A → -10000/0/+10000`；
- `±10001` raw 只在内部非法向量测试中拒绝；
- encoder 正反跨零；
- `rpm × 2π/60 ÷ 36`；
- temperature byte 即使非零也不进入通用 Feedback；
- 与同 CAN 上 C620 相同 feedback ID 被 `Bus::attach()` 拒绝；
- C620 ID1 与 C610 ID2 可共享 `0x200` 不同 slot，各自按不同 A/raw 比例打包。

`dji,m2006-c610.yaml` 仍只让用户填写 `can-bus`、`motor-id`、机构 `current-limit-ma` 和 gear ratio num/den。不要把 profile 已经知道的 `10000`、`10A` 或三个路由字段复制进 DTS。

硬件验证沿用第 3 步同一 sample：先 RX-only，再整 Bus 周期全零，再人工批准单电机极小限时电流。M2006 空载时，任意能克服损耗的持续电流都可能让速度继续升高；“命令很小”不等于“速度自动很小”。速度控制必须等第 7～9 步闭环，不能用开环电流长时间代替。

本步自检：仓库没有新增 M2006 公开 API；第 3 步 `dji_unified` sample 不需型号分支就能换 alias 测 M2006；C610 温度无效；route/量程/减速比测试通过；同总线反馈冲突在 arm 前失败；全零、TTL、受控 RX 抑制和 Fault 路径与 M3508 完全相同。

---

## 第 5 步：启用 GM6020 电流 binding，不引入第二套接口

GM6020 profile 已在第 2 步冻结；这里启用 binding 并核实实物前提。它继续走同一个 `motor::setCurrent(A) → dji::Bus`，但不是“把 C620 的 ID 改一下”。本项目第一版只支持 2023 v1.4 手册定义的电流模式；不提供 `setVoltageRaw()`、`setRaw()` 或运行时 mode 参数。

### 实物前提先于代码

GM6020 必须同时满足：

1. 电机固件版本 `>= 1.0.11.2`；
2. RoboMaster Assistant 版本 `>= 2.7`；
3. 参数设置中已经打开电流环；
4. 保存截图、日期、设备 ID，并在第 0 步登记；
5. 节点才允许从 `disabled` 改为 `okay`，并写 `current-loop-confirmed`。

普通 CAN feedback 不携带上述固件/开关证明。driver 只能校验“配置者做过确认”，不能自动断言实物一定处于电流模式。电流模式下若电机收到电压指令，v1.4 手册说明指示灯会橙色常亮；看到该现象应断电检查，不靠试更多 ID。

### 唯一 profile 表

```text
motor ID    feedback ID    current command ID    slot
1           0x205          0x1FE                 0
2           0x206          0x1FE                 1
3           0x207          0x1FE                 2
4           0x208          0x1FE                 3
5           0x209          0x2FE                 0
6           0x20A          0x2FE                 1
7           0x20B          0x2FE                 2

0x2FE 的 slot 3 永远为 0
raw ±16384 ↔ 转矩电流 ±3 A
gear ratio = 1
temperature 有效
```

`dji::Bus` 仍按完整 command ID 拥有四槽，`0x2FE` 的保留 slot 必须由 profile 标成不可绑定并始终编码为零。不要因为数组有四格就允许虚构 ID 8。

旧版 GM6020 手册与 2023 v1.4 手册对电压模式量程存在版本差异：旧资料写 `±30000`，v1.4 写 `±25000`。这进一步说明不应保留跨固件电压 wrapper。若未来确实需要电压模式，应另开任务，先锁定实物固件和对应手册，再设计带明确单位/版本的独立接口；本轮不实现。

纯测试至少增加：

- ID 1/4/5/7 与非法 0/8；
- `-3A/-1.5A/0/+1.5A/+3A` 的 raw；
- `0x2FE` slot 3 恒零且不可 attach；
- temperature/current/velocity/position capability；
- GM ID1 的 `0x205` 与 C620/C610 ID5 feedback 冲突；
- 同一 Bus 同时含 `0x1FE` 和 `0x2FE` 时每周期各发送一次；
- 任一 GM stale 时，Bus 的所有 DJI command group 全零并锁存 Fault；
- 未确认 current loop 的配置不能生成 ready device。

控制器内部继续只输出 A；GM6020 的 `3A` 是协议峰值，不是当前舵向机构安全值。`current-limit-ma` 必须小于等于协议上限并由台架确定。软限位外只允许向安全方向返回；零点和线缆多圈限制属于关节配置，不进入通用 profile。

本步自检：没有 GM6020 专用 setter；命令只出现 `0x1FE/0x2FE`；固件和电流环证据齐全前节点保持 disabled；ID、保留槽、反馈冲突和 A/raw 测试通过；全零抓包成功后才做单电机极小方向测试；电压模式在代码、sample 和业务调用中均不存在。

---

## 第 6 步：DM-J4310 另起协议族，第一版只做 MIT

DM-J4310 不是四槽 DJI 协议，不能通过改几个 ID 塞进 `dji::Bus`。它每台电机一帧，反馈 filter 匹配调试助手配置的 Master ID，命令发往 Motor ID。第一版只做完整 MIT 帧、反馈、显式 enable/disable，以及当前固件确认支持后才开放的设零；不要同时实现 MIT、位置速度、速度和参数读写大全。

写代码前先用达妙调试工具记录 P_MAX、V_MAX、T_MAX。它们决定协议量化比例，不等于硬件安全上限。本地 V1.1 手册记录硬件峰值 7N·m，但机构允许值通常更小，所以协议 Limits 和 SafetyLimits 必须分开。产品手册说明输出轴单圈绝对位置；不要再做 DJI 8192 累计，也不要擅自再除以 10。

构建开关、子目录和 device 实例参考第 3 步；纯映射测试复用第 2 步。

### 落到代码

```cpp
struct Limits {
    float position_max_rad;
    float velocity_max_rad_s;
    float torque_max_nm;       // 协议量化范围
};

struct SafetyLimits {
    float command_torque_max_nm; // 当前机构允许值
};

struct MitCommand {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque_nm = 0.0f;
};
```

```text
raw = (value - min) / (max - min) * ((1 << bits) - 1)
value = raw * (max - min) / ((1 << bits) - 1) + min

position  16 bit，-P_MAX..P_MAX
velocity  12 bit，-V_MAX..V_MAX
kp        12 bit，0..500
kd        12 bit，0..5
torque    12 bit，-T_MAX..T_MAX
```

```cpp
frame.id = motor_id;
frame.flags = 0;
frame.dlc = 8;
frame.data[0] = (p_raw >> 8) & 0xFF;
frame.data[1] = p_raw & 0xFF;
frame.data[2] = (v_raw >> 4) & 0xFF;
frame.data[3] = ((v_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F);
frame.data[4] = kp_raw & 0xFF;
frame.data[5] = (kd_raw >> 4) & 0xFF;
frame.data[6] = ((kd_raw & 0x0F) << 4) | ((t_raw >> 8) & 0x0F);
frame.data[7] = t_raw & 0xFF;
```

```text
反馈：
error       = DATA[0] 高 4 bit
motor_id    = DATA[0] 低 4 bit
position    = DATA[1..2] 16 bit
velocity    = DATA[3] + DATA[4] 高 4 bit，共 12 bit
torque      = DATA[4] 低 4 bit + DATA[5]，共 12 bit
MOS 温度    = DATA[6]
转子温度    = DATA[7]
```

本地 V1.1 手册把高 4 bit 的 `0x8..0xE` 分别定义为过压、欠压、过流、MOS 过温、电机线圈过温、通信丢失和过载；只有这些已定义值直接映射为 `State::Fault`。`0x0..0x7` 的其他状态先保留 raw，不自行命名。超过反馈心跳是 Offline，有新鲜反馈且命中已知故障码才是 Fault，其余是 Ready。

达妙 SDK/例程常见特殊帧如下，但旧稿已明确指出本地 V1.1 产品手册协议页没有列出它们，所以当前固件必须先抓包或用调试助手确认：

```text
enable   FF FF FF FF FF FF FF FC
disable  FF FF FF FF FF FF FF FD
set zero FF FF FF FF FF FF FF FE
```

`init()` 绝不自动 enable 或 set zero。上电先只收反馈，应用显式 enable，首个 MIT 命令必须是 `kp=0, kd=0, torque=0`。需要阻尼时从极小 kd 开始，不先加 kp。

本步自检：Master ID filter 正确；P/V/T 端点与中点黄金向量通过；每帧从全新 8 字节对象完整打包，不残留上一帧；安全力矩小于协议量程和硬件峰值两者；失能帧抓包通过；DM 与 DJI 不存在发送 ID 冲突。当前整车不用 DM-J4310 时，可以把本步标为“暂缓”，但不能标为完成。

---

## 第 7 步：先把 PID 变成可验证的纯算法，再谈调参

当前 `drivers/pid/pid.c` 实际是位置式 PID，不是注释所说的增量式 PID。它还会直接用 `dt` 做除数、在 deadband 内提前返回并吞掉前馈、对 error 求导制造设定值冲击、只钳积分却没有完整 anti-windup，也没有 reset。若直接接电机，线程抖动、掉线恢复和目标切换都可能把历史积分或微分尖峰带回执行器。

PID 不属于电机驱动。驱动只负责正确收发、单位换算和反馈快照；控制器只根据输入计算输出；应用线程决定什么时候算、哪个实例属于哪台电机，以及输出失败时怎样清零。每一台电机的每一个环都要有独立 `pid_data`，位置外环与速度内环不能共用状态。

第一版按固定顺序重构：先 reset 和所有有限值/`dt` 检查，再把 deadband 改成 effective error，D 改为测量微分并按时间常数滤波，然后做条件积分，最后增加总输出限幅、变化率限制和完整遥测。不要同时换算法、改数据类型、接在线调参和上真电机。

### 落到代码：建议的纯 PID 接口

```c
typedef struct {
    float kp;
    float ki;
    float kd;
    float derivative_tau;
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

int pid_step(pid_data *data,
             const pid_config *config,
             const pid_input *input,
             pid_result *result);

void pid_reset(pid_data *data, float current_measurement);
```

完整计算顺序应保持一条直线，失败时不修改旧状态：

```text
1. 验证所有指针、上下限、finite、dt > 0 且 dt 不超控制器允许上限
2. error = setpoint - measurement
3. abs(error) < deadband 时 effective_error = 0，但不提前 return
4. P = kp * effective_error
5. measurement_rate = (measurement - previous_measurement) / dt
6. alpha = derivative_tau / (derivative_tau + dt)
7. filtered_rate = alpha * previous_rate + (1-alpha) * measurement_rate
8. D = -kd * filtered_rate
9. I_candidate = clamp(I + ki * effective_error * dt)
10. candidate = P + I_candidate + D + FF
11. 若 candidate 饱和且 error 正把饱和推得更严重，冻结 I；否则接受 I_candidate
12. unsaturated = P + I + D + FF
13. 先做物理输出限幅，再按 output_slew_rate * dt 限制本拍变化
14. 一次性提交新状态和 pid_result
```

条件积分的核心判断可以写成伪代码：

```c
float candidate = p + i_candidate + d + ff;
float limited = clampf(candidate, cfg->output_min, cfg->output_max);

bool pushes_further_into_saturation =
    (candidate - limited) * effective_error > 0.0f;

if (!input->hold_integrator &&
    !pushes_further_into_saturation) {
    next.integral = i_candidate;
}
```

前馈不是另一种 PID。它承担目标运动可预测的输出，反馈项只纠正偏差。电机常用第一版是：

```text
FF = kS * direction(v_ref, a_ref)
   + kV * v_ref
   + kA * a_ref
   + kG * sin_or_cos(q_ref)   // 只有重力关节才有
```

`a_ref` 必须来自轨迹生成器或经过斜坡后的速度目标，不能直接差分摇杆阶跃。所有 DJI 电流控制器的输出域统一为 A，所以 `kS/kV/kA` 也是 A 系列；DM-J4310 若输出 N·m，则是力矩系列。统一单位不等于统一参数，不同型号和机构的系数绝不互相复制。

### 落到代码：角度、滤波与最小轨迹工具

```cpp
struct AngleUnwrapper {
    bool initialized = false;
    float last_wrapped_rad = 0.0f;
    float continuous_rad = 0.0f;
};

int angleUnwrapReset(AngleUnwrapper &state, float wrapped_rad);
int angleUnwrapUpdate(AngleUnwrapper &state,
                      float wrapped_rad,
                      float &continuous_rad);
float shortestAngleError(float target_rad, float feedback_rad);
```

实现必须先检查 finite，使用 `remainder`/`fmod` 做常数时间归一化，不用可能被 Inf 卡死的无界 while。每个关节独占一份 unwrap state。

最小速度轨迹只需要限加速度：

```text
max_delta = max_acceleration * dt
v_ref = clamp(requested_velocity,
              previous_v_ref - max_delta,
              previous_v_ref + max_delta)
a_ref = (v_ref - previous_v_ref) / dt
```

一阶低通用时间常数，不用脱离周期的裸 alpha：

```text
alpha = dt / (tau + dt)
y = y_last + alpha * (x - y_last)
```

纯测试沿用第 2 步的 ztest 目录，至少覆盖：`P+FF=4.5`、输出饱和仍记录 unsaturated、正饱和时积分不继续增大、setpoint 阶跃时 D 接近 0、deadband 内保留 FF、reset 后 D 为 0，以及 `dt=0/负数/NaN/过大` 返回错误且不改状态。角度测试必须跨 `±π`、首次初始化、NaN 和 Inf。

在线调参时，设备树生成的 `dev->config` 是只读开机默认值，不能强转后写。启动时把默认配置复制到 RAM；UART callback 只把 `key/value` 放进 `k_msgq`；控制线程在周期边界校验 finite、范围和上下限后应用，大幅修改 Ki/Kd 时 reset。当前 `lib/vofa/vofa.c` 的 TX 使用一个 static buffer 交给异步 `uart_tx()`，高频连续发送可能覆盖仍由 DMA 使用的内容；在拿它记录 500Hz PID 前，先按第 13 步增加 TX busy/独立 frame queue 或专用遥测线程。控制线程本身不做无节制打印。

本步自检：PID 算法不 include CAN/device；所有错误路径不更新状态；reset 能捕获当前 measurement；D 使用测量微分；有 anti-windup；每项输出可遥测；目标经过限速/限加速度；一个环一份状态。

---

## 第 8 步：第一次完整写出 Zephyr 定时控制线程

CAN RX callback 只提交反馈；`k_timer` expiry callback 也只发信号；浮点控制、`motor::setCurrent()` 和 `dji::Bus::flush()` 都在唯一 owner 控制线程。真实 dt 来自单调时间，反馈 timestamp 不变时不更新 I/D。命令 TTL 只保证“owner 循环仍在 flush、但上层没有提交新命令”时旧电流会失效；owner 线程完全卡死必须由硬件 watchdog/动力切断处理，软件不能伪造 `zero_sent` 证据。

与旧稿最大的不同是：控制线程不认识 M3508 namespace，不输出 raw，也不拥有某个 command ID 的 `TxGroup`。它只认一台 `motor device` 和这条物理 CAN 的唯一 `dji::Bus`。

### 落到代码：统一接口最小控制线程

示例的安全电流仍为 0；只有第 3 步全零、TTL、Offline 和发送失败证据完成后，才由人临时填写一个经过批准的极小值。节点 `current-limit-ma` 是第二层上限，两层都必须通过。示例中的人工 arm token 默认永远无效，物理断电 hook 也没有板级通用实现；两者都必须按当前硬件接线完成并实测，才允许把上限改成非零。

```cpp
#include <cmath>
#include <cstdint>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/motor.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dji_bus.hpp>
#include <drivers/pid/pid.h>

LOG_MODULE_REGISTER(dji_speed_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

static constexpr std::int64_t CONTROL_PERIOD_US = 2000;
static constexpr std::uint64_t CONTROL_FEEDBACK_TIMEOUT_MS = 10;
static constexpr float SAFE_CURRENT_LIMIT_A = 0.0f;

K_SEM_DEFINE(control_tick, 0, 1);
static skywalker::motor::dji::Bus dji_bus;
static pid_data speed_pid{};
static pid_config speed_cfg{};

static void controlTimerExpiry(struct k_timer *)
{
    k_sem_give(&control_tick);
}

K_TIMER_DEFINE(control_timer, controlTimerExpiry, nullptr);

static float safeCurrent(float value)
{
    if (!std::isfinite(value)) return 0.0f;
    if (value > SAFE_CURRENT_LIMIT_A) return SAFE_CURRENT_LIMIT_A;
    if (value < -SAFE_CURRENT_LIMIT_A) return -SAFE_CURRENT_LIMIT_A;
    return value;
}

static bool consumeFreshManualArmToken()
{
    /*
     * 安全默认：永不 arm。实机改为“本次启动后”的按钮边沿或
     * 有会话号的操作员 token；复位即失效，禁止改成恒 true。
     */
    return false;
}

[[noreturn]] static void emergencyStopRequired(
    const skywalker::motor::dji::FlushReport &report)
{
    LOG_ERR("NO ZERO-FRAME EVIDENCE: zero_err=%d target_err=%d",
            report.zero_tx_error, report.command_tx_error);
    LOG_PANIC();
    /*
     * 在允许非零前，这里必须接入并实测板级动力切断。
     * 若板上没有软件可控电源，只能锁存等待操作员按物理急停；
     * 停止本线程本身不会让 C620 自动归零。
     */
    for (;;) {
        k_sleep(K_SECONDS(1));
    }
}

static void stopFromArmed(float measurement)
{
    skywalker::motor::dji::FlushReport report{};
    const int ret = dji_bus.stop(report);
    pid_reset(&speed_pid, measurement);
    if (!report.zero_sent) emergencyStopRequired(report);
    if (ret < 0) LOG_ERR("Bus remains Fault after stop: %d", ret);
}

static void retainFlushFault(
    float measurement,
    const skywalker::motor::dji::FlushReport &original_report)
{
    pid_reset(&speed_pid, measurement);
    if (!original_report.zero_sent) {
        emergencyStopRequired(original_report);
    }
    /* 保持 Fault 和原报告；这里只退出，绝不自动 recover/arm。 */
}

static void controlThread(void *, void *, void *)
{
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    skywalker::motor::dji::Descriptor descriptor{};

    if (!device_is_ready(motor) ||
        skywalker::motor::dji::describe(motor, descriptor) < 0 ||
        descriptor.can == nullptr ||
        !device_is_ready(descriptor.can)) {
        LOG_ERR("motor or described CAN not ready");
        return;
    }

    int ret = dji_bus.init(descriptor.can);
    if (ret < 0) return;

    ret = dji_bus.attach(motor);
    if (ret < 0) return;

    /* 有界等待首帧反馈；超时就留在 Safe，绝不 arm。 */
    const std::int64_t wait_deadline = k_uptime_get() + 1000;
    while (skywalker::motor::getState(motor) !=
           skywalker::motor::State::Ready) {
        if (k_uptime_get() >= wait_deadline) {
            LOG_ERR("no fresh DJI feedback");
            return;
        }
        k_sleep(K_MSEC(5));
    }

    if (!consumeFreshManualArmToken()) {
        LOG_INF("Safe: waiting for a new manual arm token");
        return;
    }

    skywalker::motor::dji::FlushReport arm_report{};
    ret = dji_bus.arm(arm_report);  // 先发全零，成功后才 Armed
    if (ret < 0) {
        retainFlushFault(0.0f, arm_report);
        return;
    }

    std::int64_t previous_us =
        k_ticks_to_us_floor64(k_uptime_ticks());
    std::uint64_t last_feedback_stamp = 0;
    float velocity_ref = 0.0f;

    pid_reset(&speed_pid, 0.0f);
    k_timer_start(&control_timer,
                  K_USEC(CONTROL_PERIOD_US),
                  K_USEC(CONTROL_PERIOD_US));

    for (;;) {
        k_sem_take(&control_tick, K_FOREVER);

        const std::int64_t now_us =
            k_ticks_to_us_floor64(k_uptime_ticks());
        const float dt =
            static_cast<float>(now_us - previous_us) * 1.0e-6f;
        previous_us = now_us;

        skywalker::motor::Feedback feedback{};
        ret = skywalker::motor::readFeedback(motor, feedback);
        const std::uint64_t now_ms =
            static_cast<std::uint64_t>(k_uptime_get());

        const bool velocity_valid =
            (feedback.valid &
             skywalker::motor::FeedbackVelocity) != 0u;
        const bool fresh =
            feedback.timestamp_ms != 0 &&
            now_ms >= feedback.timestamp_ms &&
            (now_ms - feedback.timestamp_ms) <=
                CONTROL_FEEDBACK_TIMEOUT_MS;

        if (ret < 0 ||
            skywalker::motor::getState(motor) !=
                skywalker::motor::State::Ready ||
            !velocity_valid || !fresh ||
            !(dt > 0.0f) || dt > 0.01f ||
            !std::isfinite(dt)) {
            stopFromArmed(feedback.velocity_rad_s);
            break;  // 只能人工 recover → arm，禁止自动续跑
        }

        if (feedback.timestamp_ms == last_feedback_stamp) {
            /* 不更新 PID；旧命令只在独立 command TTL 内可重发。 */
            skywalker::motor::dji::FlushReport report{};
            if (dji_bus.flush(report) < 0) {
                retainFlushFault(feedback.velocity_rad_s, report);
                break;
            }
            continue;
        }

        pid_input input{};
        input.setpoint = velocity_ref;
        input.measurement = feedback.velocity_rad_s;
        input.feedforward = 0.0f;
        input.dt = dt;
        input.hold_integrator = false;

        pid_result result{};
        ret = pid_step(&speed_pid, &speed_cfg, &input, &result);
        if (ret < 0 || !std::isfinite(result.saturated)) {
            stopFromArmed(feedback.velocity_rad_s);
            break;
        }

        const float current_a = safeCurrent(result.saturated);
        ret = skywalker::motor::setCurrent(motor, current_a);
        if (ret < 0) {
            stopFromArmed(feedback.velocity_rad_s);
            break;
        }

        skywalker::motor::dji::FlushReport report{};
        ret = dji_bus.flush(report);
        if (ret < 0) {
            retainFlushFault(feedback.velocity_rad_s, report);
            break;
        }

        last_feedback_stamp = feedback.timestamp_ms;
    }

    k_timer_stop(&control_timer);
}

K_THREAD_DEFINE(control_thread_id,
                2048,
                controlThread,
                nullptr, nullptr, nullptr,
                2,
                0,
                0);

int main()
{
    return 0;
}
```

四台同组乃至同一 CAN 上多个 command group 都遵守同一个提交顺序：

```text
复制所有反馈快照
  → 一次性验证全部 freshness/finite/state
  → 计算所有控制器，输出单位全部为 A
  → 对每台调用 motor::setCurrent()，只更新缓存
  → 调一次 dji::Bus::flush()
  → Bus 内每个 command ID 恰好发一帧
```

不要算完一台就 flush，也不要让两个线程分别负责 `0x200` 和 `0x1FE`。任一必要电机失败时，本项目的保守策略是该 Bus 所有 DJI group 全零并 Fault；若未来确有降级控制需求，必须另写安全分析和显式模式，不能悄悄放宽低层语义。

本步自检：timer callback 只有 `k_sem_give`；PID 和 CAN 发送只在线程；使用实际 dt；新反馈才更新控制器；输出单位为 A；反馈 timeout 与命令 TTL 独立；所有故障通过 Bus 全零并锁存；零帧失败会触发物理断电；旧命令不会在 recover/arm 后自动复活；控制线程没有 1kHz 日志。

---

## 第 9 步：先完成三条单轴闭环，不写万能关节类

第一条是 M3508 速度 PI：目标速度先经第 7 步斜坡，PI 输出 A，再由第 8 步线程写入统一接口并由 Bus 提交。先 `Ki=Kd=FF=0` 调很小 Kp，方向正确后才加少量 Ki；多数速度环最终是 `PI + kS/kV/kA`，不需要 D。M3508、M2006 和 GM6020 的控制器输出单位现在都为 A，但参数、限幅和机构模型仍必须各自独立。

第二条是 GM6020 舵向串级：连续角目标进入位置 P，输出有限速度目标；速度 PI 直接输出 A 并调用 `motor::setCurrent()`，raw 换算只在 profile/core 内部。位置外环先不加 I，靠内环消除稳态速度误差。软限位外允许向安全方向返回，禁止继续撞向限位；任一必要反馈无效时调用 Bus 停机，而不是只清某台缓存。

第三条是大 Yaw 和小 Yaw 各自的单轴闭环。此处只证明两轴分别稳定，完全不做协调。若也是 GM6020，可以复用同一种纯 `JointController` 数据结构，但每个关节必须有独立参数、状态、零点、方向和限位。

### 落到代码

```text
M3508：
requested velocity
  → acceleration limiter
  → speed PI + kS/kV/kA
  → safe current A clamp
  → motor::setCurrent
  → dji::Bus::flush once

GM6020 / Yaw：
continuous position target
  → soft-limit target
  → position P/PD
  → limited velocity target
  → velocity PI + current feedforward
  → safe current A
  → motor::setCurrent
  → 同一 dji::Bus::flush once
```

建议纯关节接口只计算、不碰设备：

```cpp
enum class JointMode : std::uint8_t {
    Disabled,
    Current,
    Velocity,
    Position,
};

struct JointFeedback {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float current_a = 0.0f;
    std::uint32_t stamp_ms = 0;
    bool position_valid = false;
    bool velocity_valid = false;
    bool current_valid = false;
};

struct JointCommand {
    JointMode mode = JointMode::Disabled;
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float current_ff_a = 0.0f;
};

struct JointOutput {
    float current_a = 0.0f;
    float position_error_rad = 0.0f;
    float velocity_target_rad_s = 0.0f;
    float velocity_error_rad_s = 0.0f;
    bool saturated = false;
    bool limited = false;
};
```

模式切换要统一：进入 Disabled 立即输出零并 reset 两环；`Disabled → Position` 默认捕获当前角，不能跳向旧目标；反馈过期等同失能；位置环输出先限速；最终电流总限幅；正常停机、失联和驱动 Offline 使用同一 reset 路径。

DM-J4310 若使用 MIT 内部阻抗控制，不要同时给很大 MIT Kp/Kd、外部位置 PID 大力矩输出和电机内部位置模式。三种选择只能明确选一种：直接 MIT `p/v/kp/kd/t_ff`；外部控制器输出 torque 且 MIT Kp/Kd 为零；或使用电机内部位置速度模式而上层只给平滑目标。

本步自检：M3508 速度斜坡、稳态与停机可解释；GM6020 能追几个小角度且不撞限位；大、小 Yaw 分别捕获当前角后平稳使能；所有环能看见 P/I/D/FF/饱和；断反馈在几个控制周期内归零。

---

## 第 10 步：舵轮先只写数学，完全不连接 Zephyr device

全项目统一坐标为 `+x` 前、`+y` 左、`+z` 上，从上向下看逆时针为正 Yaw；位置 m、线速度 m/s、角度 rad、角速度 rad/s。四个模块的位置放在 `ModuleGeometry[4]`，公式循环计算，安装方向和零点只在配置中出现，绝不在公式里写四套临时负号。

每个模块的接地点速度由平移与刚体旋转相加。零速时 `atan2(0,0)` 没有方向，所以默认保持上一舵角；X-lock 是显式模式。最短转向不是简单把角包回单圈，而是在 `theta` 和 `theta+π` 两个机械等价解中，分别展开到当前连续角附近，选择转角较短的一支，再独立输出 `drive_reversed` 与 `alignment_scale`。四轮若有一轮超速，必须用同一个比例缩放全部轮速，否则合成运动方向会改变。

### 落到代码

```cpp
struct ModuleGeometry {
    float x_m = 0.0f;
    float y_m = 0.0f;
    float wheel_radius_m = 0.0f;
    float drive_gear_ratio = 1.0f;
    float steer_zero_offset_rad = 0.0f;
    std::int8_t drive_sign = 1;
    std::int8_t steer_sign = 1;
};

struct ModuleTarget {
    float steer_continuous_rad = 0.0f;
    float wheel_speed_m_s = 0.0f;
    float drive_velocity_rad_s = 0.0f;
    float alignment_scale = 0.0f;
    bool drive_reversed = false;
    bool angle_held = false;
};
```

单模块基础公式：

```text
v_ix = vx - wz * y_i
v_iy = vy + wz * x_i
speed = hypot(v_ix, v_iy)
theta = atan2(v_iy, v_ix)
```

最短转向按固定顺序写：

```text
若 speed < hold_threshold：
    target = last_target
    wheel_speed = 0
    angle_held = true
否则：
    A = theta，drive sign +1
    B = theta + pi，drive sign -1
    把 A/B 分别展开到最接近 current_continuous_angle 的圈
    选择绝对误差较小的候选
    drive_reversed = 选择 B
    alignment_scale = clamp(cos(chosen_error), 0, 1)
    wheel_speed = signed_speed * alignment_scale

四模块算完后：
    peak = max(abs(wheel_speed[0..3]))
    若 peak > max：四轮共同乘 max / peak

驱动电机目标：
    wheel_omega = wheel_speed / wheel_radius
    motor_omega = wheel_omega * drive_gear_ratio * drive_sign
```

正运动学不要复制长公式。每个模块给两条方程：

```text
s_i*cos(alpha_i) = vx - wz*y_i
s_i*sin(alpha_i) = vy + wz*x_i
```

四模块共八条方程组成 `b=A*[vx,vy,wz]^T`，用最小二乘 `(AᵀA)⁻¹Aᵀb`；矩阵 A 只由几何决定，可以初始化时预计算。

测试仍沿用第 2 步的纯 ztest 形状：纯前进四轮角 0；纯左移四轮角 `π/2`；纯逆时针旋转得到各模块切向向量；当前 0°、目标 100°时选择反驱和 80°转向；连续角 `4π+0.1` 不跳回 0.1；零速保持；一轮超速四轮同比缩放；逆解送正解还原原命令；NaN、零轮径和非法 sign 返回错误并输出零速度。

本步自检：测试不 include Zephyr CAN/device；所有单位为 SI；最短转向输出连续角；反转和对齐缩放可分别观察；正逆解共享同一模块顺序和几何。

---

## 第 11 步：先接一只舵轮，再接四只和底盘状态机

一只模块只连接一台 M3508 和一台 GM6020。先让舵向以很小电流验证方向，再按第 9 步闭合速度内环和位置外环；舵角误差很大时驱动轮保持零，逐渐对齐后才按 `alignment_scale` 放开极小速度。最短转向选择反驱时，M3508 目标必须反号，但 GM6020 仍只走较短角度。

单模块通过后，逐个标定四个零点和安装符号，再依次测试 `+vx`、`+vy`、`+wz`、组合命令。不要四模块同时通电后再靠“哪个方向不对就改负号”调车。同一物理 CAN 每拍只调用一次 Bus flush，由 Bus 对每个 command ID 各发一帧；任一关键舵向失联，第一版整车停，不急着实现三轮降级。

### 落到代码

```text
单模块一拍：
读取 drive + steer 新鲜反馈
  → 运行第 10 步 inverse 得到 module target
  → steer position P 产生 steer velocity target
  → steer velocity PI 产生 GM6020 current A
  → drive target 乘 alignment_scale
  → drive speed PI 产生 M3508 current A
  → 两路各自按机构上限限幅
  → 两次 motor::setCurrent，只缓存
  → drive/steer 所属 Bus 各 flush 一次；同一 CAN 时按 Bus 去重
```

底盘状态机建议保持保守：

```text
Boot
  → 所有 device ready
WaitFeedback
  → 八台电机反馈 fresh
WaitManualArm
  → 收到本次启动后的新人工 arm token
Arming
  → 依次 arm 全部 Bus；后一个失败就 stop 已成功的 sibling Bus
Aligning
  → 四舵对齐，驱动电流保持 0
Ready
  → 收到新的运动 enable token
Active
  ├─ command stale → Stopping → Ready
  ├─ 任一关键反馈失效 → Fault
  └─ send 失败/严重错误 → Fault

Fault
  → 保留最先失败 Bus 的原 FlushReport 和 Fault
  → 对其余 Armed/Safe sibling Bus 调 stop，全部分组发全零
  → Fault Bus 可重试零帧但保持 Fault，不在这里 recover
  → reset 所有 PID
  → 任一零帧失败则物理断电
  → 根因排除后才允许人工 recover；成功只回 WaitManualArm
```

四模块一拍的唯一顺序：

```text
复制最新命令快照
  → 检查 mode/sequence/timestamp/finite
  → 检查八电机 freshness
  → 坐标变换到 chassis frame
  → vx/vy/wz 限幅和斜坡
  → swerve_inverse
  → 四个 module_step
  → 写四 M3508 + 四 GM6020 的 A 缓存
  → 每条物理 CAN 各 flush 一次
  → 后发 Bus 失败时，立即给先成功的 sibling Bus stop
  → Bus 内部按实际 profile 发送 0x200/0x1FE 等分组
  → 正运动学并发布状态快照
```

总线负载必须实测。八台电机约 1kHz 反馈接近一条 1Mbps Classic CAN 的极限，推荐四 M3508 与四 GM6020 分两条 CAN，板间通信用第三条 CAN/CAN-FD 或独立 UART；不要未经测量把板间帧塞入已经接近满载的电机总线。跨 Bus 提交不具备原子性：后发 Bus 失败时，先发 Bus 可能已经短暂执行目标帧，所以 sibling stop、零帧证据和物理断电升级必须作为专门的 fake-CAN 故障测试。

本步自检：单模块未对齐不驱动；四个方向只来自配置；纯三轴运动符合第 10 步向量；每条 Bus 每拍只 flush 一次；任一关键反馈 stale 后整车所有 sibling Bus 都进入停机流程；共同轮速缩放不改变方向；多 Bus 部分 arm/flush 失败不会自动 recover，且任一零帧无证据都会升级为物理断电。

---

## 第 12 步：把一个空 application 拆成两个固件，再写纯板间协议

底盘板拥有四个舵轮、大 Yaw 本地闭环和底盘安全状态；云台板拥有小 Yaw、上层 IMU、视觉/遥控目标和唯一 Yaw 协调器。跨板只传目标、测量摘要、有效位和故障，不把高速电机闭环搬到另一块板，也不把一个 `application/main.c` 发展成能猜自己身份的万能程序。

第一版分别构建 `application/chassis` 和 `application/gimbal`，先不引入 sysbuild。两块板各自有 `CMakeLists.txt`、`prj.conf`、overlay 和 `main.cpp`。其中 Zephyr 配置接线参考第 3 步。

板间协议先是纯字节 codec，不绑定 UART/CAN。不要 `memcpy` packed struct 或裸 float ABI。第一版更适合固定点：线速度用 mm/s 的 `int16_t`，角速度/单圈角用 mrad 的 `int16_t`，连续角用 mrad 的 `int32_t`。每个帧有 version、type、payload length、sequence、source time、flags 和 CRC；接收者还用本地到达时间计算 freshness，不能假设两块板时钟同步。

### 落到代码

```text
application/
├── chassis/
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── boards/dm_mc02.overlay
│   └── src/main.cpp
└── gimbal/
    ├── CMakeLists.txt
    ├── prj.conf
    ├── boards/dm_mc02.overlay
    └── src/main.cpp
```

```text
底盘板接收：vx/vy/wz、坐标系、大 Yaw 目标、mode、enable token
底盘板发送：大 Yaw 状态、底盘 yaw/gyro、正解速度、online/fault bits、accepted sequence

云台板接收：大 Yaw 状态、底盘 yaw/gyro、底盘 online/fault
云台板发送：底盘命令、大 Yaw 目标、mode、enable token、sequence
```

逻辑消息和 wire bytes 分开：

```cpp
struct LinkHeader {
    std::uint8_t protocol_version = 1;
    std::uint8_t message_type = 0;
    std::uint16_t sequence = 0;
    std::uint32_t source_time_ms = 0;
    std::uint32_t flags = 0;
};

int encodeChassisCommand(const ChassisCommandMessage &message,
                         std::uint8_t *buffer,
                         std::size_t capacity,
                         std::size_t &written);

int decodeChassisCommand(const std::uint8_t *buffer,
                         std::size_t length,
                         ChassisCommandMessage &message);
```

UART framing 第一版可固定为：

```text
magic[2]
protocol_version u8
message_type u8
payload_length_le u16
sequence_le u16
source_timestamp_ms_le u32
flags_le u32
payload[N]
crc16_le u16
```

CRC 参数不能只写“CRC16”，必须在协议注释中固定 polynomial、init、refin/refout、xorout 和结果字节序，并给黄金向量。parser 行为如下：

```text
找 magic
  → 头不足：保留，等下一块
  → version/length 非法：只丢一个字节，重新找 magic
  → 整帧不足：保留
  → CRC 错：只丢一个字节，重新同步
  → 合法：逐字段 decode，投递完整消息，继续处理粘包
```

`enable_token` 防止重启后旧命令恢复运动：上层每次 enable 生成新 token；底盘只接受当前会话；任一板重启都必须重新握手。纯测试沿用第 2 步的 ztest 形状，覆盖一次一帧、一次一字节、帧头跨块、两帧粘包、噪声、坏 CRC 后接好帧、超长 payload、sequence 重复/倒退/回绕、NaN/Inf 拒绝以及没有新字节时 freshness 自己变 stale。

超时先用可配置起点做受控断链试验，而不是永久写死：命令 age 超过约 30ms 开始斜坡归零，超过约 100ms 清电流并 reset；大 Yaw 命令同理；任一高速电机反馈超过约 3～5 个控制周期立即禁止对应模块；板间状态超过约 100ms 退出协同。板间命令、板间状态、电机反馈和 DJI command TTL 是四个独立 freshness 域，各自按正常周期与最坏抖动实测；电机门限引用第 3 步 Kconfig，不再拿不存在的“第 2 步 100ms heartbeat”比较。

本步自检：两个 application 能分别构建；codec 不 include UART/CAN；没有 packed struct；重启使旧 token 失效；parser 能从坏帧恢复；命令 stale 后目标斜坡归零并最终清电流。

---

## 第 13 步：第一次完整实现 Zephyr 异步 UART 双缓冲

异步 UART 与第 1 步 CAN callback 的原则相同：回调只搬字节与维护 buffer 所有权，worker 才运行 parser。`UART_RX_RDY` 必须使用 `buf + offset`，因为同一个 DMA buffer 可以产生多次事件；`UART_RX_BUF_REQUEST` 必须用 `uart_rx_buf_rsp()` 提供第二块当前未被驱动占用的 buffer；只有收到 `UART_RX_BUF_RELEASED` 后才能复用那块内存。TX buffer 也必须一直存活到 `UART_TX_DONE` 或 `UART_TX_ABORTED`，不能像当前 VOFA 的 static buffer 一样在异步发送尚未完成时被下一帧覆盖。

下面给出单链路最小完整骨架。它采用一个 callback 生产、一个 worker 消费的 ring buffer；Zephyr ring buffer 本身不提供多生产者锁，因此不要再让第二个 callback 写同一个 ring。裁判常规链路和图传链路各实例化一份完整 context。

### 落到代码：完整配置与 RX 骨架

先开启异步 UART，并用 alias 让应用选择实际 UART；引脚、DMA 和波特率仍由对应 board DTS/overlay 决定：

```ini
# prj.conf
CONFIG_CPP=y
CONFIG_SERIAL=y
CONFIG_UART_ASYNC_API=y
```

```dts
/* boards/dm_mc02.overlay：示例节点必须替换成第 0 步的真实分配。 */
/ {
    aliases {
        boardlink = &usart1;
    };
};

&usart1 {
    status = "okay";
    current-speed = <115200>;
};
```

```cpp
#include <cstddef>
#include <cstdint>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

namespace {

constexpr std::size_t RX_DMA_BYTES = 256;
constexpr std::size_t RX_RING_BYTES = 2048;

struct AsyncUartRx {
    const struct device *uart = nullptr;
    alignas(4) std::uint8_t dma[2][RX_DMA_BYTES]{};
    atomic_t in_driver_mask{};
    struct ring_buf ring{};
    std::uint8_t ring_storage[RX_RING_BYTES]{};
    struct k_work restart_work{};
    atomic_t dropped_bytes{};
    atomic_t rx_stops{};
    atomic_t parser_reset_requested{};
};

AsyncUartRx link;
K_SEM_DEFINE(rx_data_ready, 0, 1);

int bufferIndex(AsyncUartRx &ctx, const std::uint8_t *ptr)
{
    if (ptr == ctx.dma[0]) return 0;
    if (ptr == ctx.dma[1]) return 1;
    return -1;
}

int claimFreeBuffer(AsyncUartRx &ctx)
{
    for (int index = 0; index < 2; ++index) {
        if (!atomic_test_and_set_bit(&ctx.in_driver_mask, index)) {
            return index;
        }
    }
    return -1;
}

void restartWork(struct k_work *work)
{
    auto *ctx = CONTAINER_OF(work, AsyncUartRx, restart_work);
    const int index = claimFreeBuffer(*ctx);
    if (index < 0) return;

    const int ret = uart_rx_enable(ctx->uart,
                                   ctx->dma[index],
                                   RX_DMA_BYTES,
                                   2000);
    if (ret == 0) {
        return;
    }
    atomic_clear_bit(&ctx->in_driver_mask, index);
}

void uartCallback(const struct device *dev,
                  struct uart_event *evt,
                  void *user_data)
{
    auto *ctx = static_cast<AsyncUartRx *>(user_data);
    if (ctx == nullptr || evt == nullptr) return;

    switch (evt->type) {
    case UART_RX_RDY: {
        const std::uint8_t *src =
            evt->data.rx.buf + evt->data.rx.offset;
        const std::uint32_t len =
            static_cast<std::uint32_t>(evt->data.rx.len);
        const std::uint32_t written = ring_buf_put(&ctx->ring, src, len);
        if (written != len) {
            atomic_add(&ctx->dropped_bytes,
                       static_cast<atomic_val_t>(len - written));
            atomic_set(&ctx->parser_reset_requested, 1);
        }
        k_sem_give(&rx_data_ready);
        break;
    }

    case UART_RX_BUF_REQUEST: {
        const int index = claimFreeBuffer(*ctx);
        if (index >= 0) {
            const int ret = uart_rx_buf_rsp(dev,
                                            ctx->dma[index],
                                            RX_DMA_BYTES);
            if (ret < 0) {
                atomic_clear_bit(&ctx->in_driver_mask, index);
            }
        }
        break;
    }

    case UART_RX_BUF_RELEASED: {
        const int index = bufferIndex(*ctx, evt->data.rx_buf.buf);
        if (index >= 0) {
            atomic_clear_bit(&ctx->in_driver_mask, index);
        }
        break;
    }

    case UART_RX_STOPPED:
        atomic_inc(&ctx->rx_stops);
        atomic_set(&ctx->parser_reset_requested, 1);
        break;

    case UART_RX_DISABLED:
        k_work_submit(&ctx->restart_work);
        break;

    default:
        break;
    }
}

void parserWorker(void *, void *, void *)
{
    std::uint8_t chunk[128];
    for (;;) {
        k_sem_take(&rx_data_ready, K_FOREVER);

        if (atomic_cas(&link.parser_reset_requested, 1, 0)) {
            /* frame_parser_reset(&parser); */
        }

        for (;;) {
            const std::uint32_t count =
                ring_buf_get(&link.ring, chunk, sizeof(chunk));
            if (count == 0) break;
            /* frame_parser_push(&parser, chunk, count, onFrame, ...); */
        }
    }
}

K_THREAD_DEFINE(parser_worker_id,
                3072,
                parserWorker,
                nullptr, nullptr, nullptr,
                5,
                0,
                0);

} // namespace

int asyncUartStart(const struct device *uart)
{
    if (uart == nullptr || !device_is_ready(uart)) return -ENODEV;

    link.uart = uart;
    atomic_clear(&link.in_driver_mask);
    atomic_clear(&link.dropped_bytes);
    atomic_clear(&link.rx_stops);
    atomic_clear(&link.parser_reset_requested);
    ring_buf_init(&link.ring, RX_RING_BYTES, link.ring_storage);
    k_work_init(&link.restart_work, restartWork);

    int ret = uart_callback_set(uart, uartCallback, &link);
    if (ret < 0) return ret;

    const int index = claimFreeBuffer(link);
    if (index < 0) return -EBUSY;
    ret = uart_rx_enable(uart, link.dma[index], RX_DMA_BYTES, 2000);
    if (ret < 0) {
        atomic_clear_bit(&link.in_driver_mask, index);
        return ret;
    }
    return ret;
}

int main()
{
    const struct device *uart =
        DEVICE_DT_GET(DT_ALIAS(boardlink));
    return asyncUartStart(uart);
}
```

上例用原子位图让启动线程、UART callback 和 restart work 共同管理两块 DMA buffer；claim 成功后若 API 调用失败，必须立即清回占用位。worker 可以在系统启动时创建，因为 `rx_data_ready` 是静态初始化的 semaphore；在 `asyncUartStart()` 完成并真正收到 `UART_RX_RDY` 以前，它只会阻塞等待，不会读取尚未初始化的 ring。ring overflow 后必须请求 parser reset，因为中间丢字节会让现有半帧失去意义。

TX 不能只保存一个 static buffer，建议队列中的每个元素自己拥有字节：

```cpp
struct TxFrame {
    std::uint16_t length = 0;
    std::uint8_t bytes[320]{};
};

K_MSGQ_DEFINE(tx_queue, sizeof(TxFrame), 8, 4);

/* tx worker 从队列取一个 TxFrame，放入 current_tx；
 * uart_tx(current_tx.bytes, current_tx.length, ...);
 * 只有 UART_TX_DONE/ABORTED 后才允许覆盖 current_tx。
 */
```

板间 UART、裁判常规 UART 和图传 UART 都复用本步，但每条链路拥有独立 context、双 buffer、ring、parser、sequence、统计和 TX queue。回调不做业务 decode，不拿 store mutex，不跑 Yaw 协调，不打印逐帧日志。

本步自检：正确使用 RX offset；双 buffer 只在 RELEASED 后复用；半帧能跨事件；粘包能连续解析；overflow 有统计并重同步；RX stopped 能恢复；异步 TX 期间内存不被改写；两条链路绝不共享 parser 状态。

---

## 第 14 步：大小 Yaw 先各自稳定，再由唯一协调器做低频卸载

在近似同轴的第一版假设下，定义底盘世界角 `ψc`、大 Yaw 相对底盘角 `θb`、小 Yaw 相对大 Yaw 角 `θs`，最终上层世界角满足 `ψg = ψc + sb·θb + ss·θs + offset`。上层 IMU若直接测得 `ψg`，世界角闭环优先使用 IMU；编码器关系用于方向核对、中心偏移、限位和通信降级。世界误差通常使用最短角，关节限位和电缆管理则使用连续相对角，两种角不能共用一个含糊变量。

小 Yaw 负责快速世界角稳定与瞄准，大 Yaw 负责慢速把小 Yaw 拉回舒适区。协调器只在云台板运行，因为那里拥有最终目标、上层 IMU和小 Yaw 状态；底盘板只接收大 Yaw 目标并做本地闭环。如果两块板各自算一套“自动回中”，两个轴会互相追赶。

第一版先只做中心卸载，不加加速度前馈：小 Yaw 偏离中心超过 deadband 后，大 Yaw 以受限低速朝减小偏差的方向移动；大 Yaw 造成的世界角扰动由小 Yaw 快环消除。这个符号必须用极低速实测，偏差变大立即清零。等中心卸载稳定后，再把最终相对角速度需求低通分配给大 Yaw，其余留给小 Yaw。

### 落到代码

```cpp
enum class YawCoordinationMode : std::uint8_t {
    Disabled = 0,
    Independent,
    FollowAndRecenter,
};

struct YawJointState {
    float relative_angle_rad = 0.0f;
    float continuous_angle_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    std::uint64_t timestamp_ms = 0;
    bool valid = false;
};

struct YawCoordinatorConfig {
    float small_center_rad = 0.0f;
    float small_center_deadband_rad = 0.0f;
    float small_soft_limit_rad = 0.0f;
    float small_hard_limit_rad = 0.0f;
    float big_rate_max_rad_s = 0.0f;
    float big_accel_max_rad_s2 = 0.0f;
    float center_kp = 0.0f;
    float split_filter_tau_s = 0.0f;
};

struct YawCoordinatorOutput {
    float big_yaw_rate_ref_rad_s = 0.0f;
    float small_yaw_world_target_rad = 0.0f;
    float small_yaw_rate_ff_rad_s = 0.0f;
    std::uint32_t status_bits = 0;
};
```

```text
center_error = theta_small_relative - theta_small_center
center_term  = center_kp * deadzone(center_error)

若还没有可靠世界角速度目标：
    big_rate_ref = rate_and_accel_limit(center_term)
    small_yaw 继续独立追世界角

若已有 omega_g_ref 和 omega_chassis：
    required_relative_rate = omega_g_ref - omega_chassis
    big_rate_ff = lowpass(required_relative_rate)
    big_rate_ref = clamp(big_rate_ff + center_term)
    small_rate_ff = required_relative_rate - big_rate_ref
```

安全行为必须写进协调器契约：输入 stale 或 dt 非法返回 `-EAGAIN` 并令大 Yaw rate 为 0；小 Yaw 到 soft limit 时提高回中优先级并限制目标；到 hard limit 时禁止向外并置 fault；大 Yaw 到限位时也只允许向安全方向退出；模式切换捕获当前目标并 reset 相关 PID。

前馈在最后添加。每个轴自己的关节控制器计算 `ka·a_ref + kv·v_ref + ks·sign`，小 Yaw 参数偏快速，大 Yaw 参数偏保守；不能把同一份完整前馈同时加给两轴。若两轴并非同轴或以后加入 pitch/roll，再新增小型四元数 frame math 并用已知 90°旋转验证乘法顺序，不搬一个大一统 Gimbal 类。

本步自检：Independent 模式两轴分别稳定；协调器只有云台板一份；静止居中时大 Yaw rate 为 0；小 Yaw 正偏时大 Yaw 运动能减小偏差；底盘慢转时最终世界角保持；链路断开后底盘、大 Yaw 目标和协调状态都按第 12 步超时降级；软硬限位不靠 PID 饱和硬顶。

---

## 第 15 步：裁判系统先做 RX、decoder 和 store，赛季变化只换描述表

裁判系统不是一个大号串口驱动。传输层只收发字节，FrameParser 只认识通用帧，Decoder 按当前赛季把 payload 逐字段变成语义数据，Store 保存 value/valid/stamp/generation，功率、发射和 UI 才是消费者。四层分开后，赛季长度变化只需要替换 CommandSpec、decoder 和黄金向量，不必重写 UART 与业务控制。

旧文档在 2026-08 的审计基线使用《RoboMaster 2026 高校系列赛通信协议 V1.3.0（20260327）》。线上复核同时能看到后续 V1.4.0 比赛规则和官方赛事引擎异常公告，因此本文不把 V1.3.0 宣布为未来一直有效：开始本步和比赛前各重新下载一次官方协议，记录 URL、发布日期、版本和 SHA256，再生成项目的赛季描述表。通信协议回答线上字段，比赛规则回答允许策略，两者不能互相替代。

参考 `sp_middleware` 的类型头混有 2025/2026 长度，不能作为线上真值；其仓库当前页面仍未显示许可证文件，所以只能吸收架构与测试需求，不复制函数体、CRC 表、packed 类型和注释。

### 落到代码：模块边界

```text
常规 UART 115200                 图传 UART 921600
       │                                │
       └─ 各自 AsyncUartRx（第 13 步） ─┘
                        │
                 FrameParser × 2
           CRC8 header / length / CRC16
                        │
                     FrameView
                        │
       CommandSpec(current season, link, length)
              ┌─────────┴─────────┐
       ordinary cmd_id       0x0301 二级分发
              └─────────┬─────────┘
                        │
                  RefereeStore
           value/valid/stamp/generation
              ┌─────────┼─────────┐
          power policy  shooter   UI/app
```

两条链路是两个完整实例，不能共用半帧、sequence 或发送队列：

```cpp
enum class RefereeLinkKind : std::uint8_t {
    Regular,
    Video,
};

struct RefereeLinkConfig {
    RefereeLinkKind kind;
    const struct device *uart;
    std::uint32_t baud_rate;
    std::uint16_t max_payload_len;
    std::uint16_t rx_ring_capacity;
};

struct RefereeLinkStats {
    std::uint32_t rx_bytes;
    std::uint32_t valid_frames;
    std::uint32_t header_crc_errors;
    std::uint32_t frame_crc_errors;
    std::uint32_t length_errors;
    std::uint32_t dropped_bytes;
    std::uint32_t sequence_gaps;
    std::uint32_t unknown_cmd_ids;
    std::uint32_t rx_overflows;
    std::uint32_t tx_frames;
    std::uint32_t tx_busy;
    std::uint32_t tx_errors;
};
```

### 落到代码：parser、描述表与 decoder

官方通用串口帧形状是：

```text
SOF 0xA5       1 byte
data_length    2 bytes little-endian
sequence       1 byte
header CRC8    1 byte
cmd_id         2 bytes little-endian
data           N bytes
frame CRC16    2 bytes little-endian
```

parser 状态机与第 12 步相同，但 CRC8/CRC16 参数和长度来自当赛季官方附录。失败时只丢当前 SOF，再继续搜下一处 `0xA5`；sequence 只用于统计丢帧、重复和乱序，`255→0` 是正常回绕，不能当认证。

```cpp
enum class MessageLifecycle : std::uint8_t {
    PeriodicState,
    LatchedState,
    Event,
    VariablePayload,
};

struct CommandSpec {
    std::uint16_t cmd_id;
    RefereeLinkKind link;
    std::uint16_t min_payload_len;
    std::uint16_t max_payload_len;
    MessageLifecycle lifecycle;
    std::uint32_t expected_period_ms;
    std::uint32_t stale_after_ms;
    int (*decode)(const std::uint8_t *payload,
                  std::uint16_t payload_len,
                  std::uint32_t stamp_ms,
                  RefereeStore *store);
};
```

Decoder 必须用 `read_u16_le/read_u32_le/read_f32_le` 逐字段读取，不把未对齐 payload 强转为结构或 float。float 先将 little-endian 位模式放入 `uint32_t`，再 `memcpy` 到 float，并拒绝 NaN/Inf。位域用移位和掩码，例如伤害字节的低 4 bit 与高 4 bit分别读取。所有字段先写局部临时对象，完整验证成功后一次性提交 store；失败不能覆盖上一份有效数据。

旧审计以 V1.3.0 为例时，`0x0201` 是 13 字节，以下代码只能放在明确命名的 `protocol_2026_v1_3` profile，协议升级后由新 profile 替换：

```text
payload[0]       robot_id
payload[1]       robot_level
payload[2..3]    current_hp
payload[4..5]    maximum_hp
payload[6..7]    shooter cooling value
payload[8..9]    shooter heat limit
payload[10..11]  chassis power limit W
payload[12]      gimbal/chassis/shooter output bits
```

`0x0301` 必须二级分发：先验证外层至少包含 `data_cmd_id/sender_id/receiver_id`，再按 `data_cmd_id` 的子描述表检查 payload。未知子命令只计数并跳过，不能破坏后续帧。

### 落到代码：Store 的生命周期与线程语义

```cpp
template <typename T>
struct RefereeSample {
    T value{};
    std::uint32_t stamp_ms = 0;
    std::uint32_t generation = 0;
    std::uint8_t source_sequence = 0;
    bool valid = false;
};
```

周期状态保存上一值但按 age 变 stale；比赛结果等锁存状态持续到 session reset；伤害/发射等事件至少用 generation 区分多次相同值，更稳妥的是小队列。未知不能用 999 伪装，store 使用 `valid=false/fresh=false`，UI 决定显示 `---`。

第一版使用清楚的 mutex 快照：parser worker 解码到局部对象，拿 `k_mutex` 一次性更新 store；消费者拿同一 mutex 复制所需字段后立刻释放，在锁外做控制。UART callback 不拿这个 mutex，因为它只搬字节。

发送方向也保持四层。Encoder 把一个已验证的语义 payload 编成自持有内存的帧；权限层检查 link、sender、receiver、robot ID 和长度；Scheduler 管每条链路只能有一个进行中的 TX、单 cmd 与共享频率、优先级和过期时间；最后才交给第 13 步 UART。sequence 在请求真正被接受发送时递增，不在入队失败时消耗。

```cpp
struct EncodedFrame {
    RefereeLinkKind link;
    std::uint16_t cmd_id = 0;
    std::uint16_t length = 0;
    std::uint8_t bytes[320]{};
};

enum class TxPriority : std::uint8_t {
    Safety,
    Control,
    Interaction,
    UiStatic,
    UiDynamic,
    Debug,
};

struct TxRequest {
    EncodedFrame frame;
    TxPriority priority;
    std::uint32_t earliest_send_ms = 0;
    std::uint32_t expiry_ms = 0;
};
```

过期帧直接丢弃，不能在堵塞恢复后补发一串旧控制目标；UI 和多机通信必须共享 `0x0301` 的当赛季总频率/带宽预算，具体额度放进赛季 profile，不写死在通用 scheduler。

测试沿用第 2 步的纯 ztest 形状，先做 byte codec 与官方 CRC 黄金帧，再喂 parser 的拆包、粘包、噪声、坏 CRC 和假长度，最后对每个 decoder 验证正确长度、少一字节、多一字节和旧赛季长度。第一版优先 `0x0201/0x0202/0x0001`，不要一次录完整本协议。

本步自检：官方版本与散列已记录；两链路独立；回调只搬字节；parser 可重同步；decoder 逐字段且 float 有限；周期/锁存/事件生命周期分开；消费者获得一致 snapshot；拔串口后 link 与 topic freshness 会在没有新字节时自行失效。

---

## 第 16 步：裁判数据只是功率约束输入，不是高速功率控制器

不要按 `robot_level` 在本地再查一张隐藏功率表。旧审计所用 2026 V1.3.0 profile 已经在 `0x0201` 下发 `chassis_power_limit`，它才是当前裁判事实；level 用于显示与一致性诊断。数据过期时也不能把默认数字伪装成有效，而要明确切到赛前填写的低功率 fallback，并平滑下降。

允许功率不等于实际已经限制住 M3508。真正策略还要结合缓冲能量、超级电容、电池/母线测量、轮速和四轮命令，计算总预算后共同缩放四轮。裁判 parser 不直接写电流，10Hz 数据也不能参与 500Hz 电流环反馈。

### 落到代码

```cpp
struct RefereePowerInput {
    std::uint16_t chassis_power_limit_w = 0;
    std::uint16_t buffer_energy_j = 0;
    std::uint8_t remaining_energy_raw = 0;
    bool chassis_output_enabled = false;
    bool power_limit_fresh = false;
    bool buffer_energy_fresh = false;
    bool remaining_energy_fresh = false;
};

struct ChassisPowerOutput {
    float common_scale = 0.0f;
    float total_current_budget_a = 0.0f;
    bool using_fallback = true;
    bool hard_disabled = true;
};
```

```text
RefereePowerInput snapshot
  + super-cap feedback
  + bus voltage/current
  + requested_four_wheel_current_a[4]
  → ChassisPowerPolicy
  → common_scale in [0,1]
  → 四轮 requested current A 同比缩放
  → 每台型号硬限幅
  → motor::setCurrent(A)
  → 每条 dji::Bus flush once
```

降级应连续：从未有效时用明确测试/安全上限；短暂少一两帧可以保持上一值但置 warning；持续过期平滑降到 fallback；裁判输出位显示 chassis 口关闭则立即零输出并 reset PID。具体 fallback 由赛项、硬件和第 0 步实测决定，本文不猜数值。

本步自检：parser 不 include 电机头；level 不覆盖新鲜 power limit；四轮按共同系数缩放；stale 可观察；突然降额不会产生不受控尖峰；chassis output off 会走统一停机路径。

---

## 第 17 步：UI 和发射机构是后期业务，分别建立自己的状态机

UI 值得保留“线、圆、文字等语义元素”和官方 1/2/5/7 图形批量规则，但语义对象不能直接持有 packed wire bit-field。稳定 ID 用源码中显式三字节常量，场景层描述最终画面，Diff 层只生成 Add/Modify/Delete，Batcher 再按官方规则组合，Encoder 逐位写 payload，最后与多机通信共享第 15 步 TX scheduler 的带宽。

异步 TX 每帧独占内存直到完成，见第 13 步。链路上线且 robot ID 新鲜后先 DeleteAll 一次，再分批 Add；运行时只发 dirty 变化；重连或 robot ID 改变回到重建状态。不要每 33ms 重发全部静态图形，也不要让 UI 抢占控制消息。

### 落到代码：UI

```text
UiScene   最终想显示什么
  → UiDiff   与已确认发送状态的差异
  → UiBatcher 1/2/5/7 或 String
  → UiEncoder 逐字段/逐位生成 0x0301 payload
  → TxScheduler 权限、频率、过期、优先级
  → Async UART TX ownership（第 13 步）
```

```text
Offline
  → 等常规链路和新鲜 robot_id
NeedReset
  → DeleteAll 一次
Adding
  → 分批 Add 静态元素
Running
  → 只发送 dirty Modify/Delete
Resync
  → 回到 NeedReset
```

发射机构也不能塞进裁判 service。弹丸初速度由本地摩擦轮转速闭环决定，射频由拨弹节拍决定；裁判系统提供规则约束、当前热量、允许发弹量、供电位和每发后的测速观测。`0x0207.initial_speed` 是上一发的稀疏测量，不是摩擦轮 PID 的高速反馈。

建议拆成 `ShooterRulePolicy`、`MuzzleVelocityMap`、左右独立 `FlywheelController`、`ShooterHeatPolicy/FireInterlock`、`FeederController` 和 `ShotObserver`。先确认赛项、兵种、弹丸与当场规则；不装弹调两摩擦轮速度；建立稳定到速判定；在挡弹设施、护目和独立急停人员到位后才实弹标定 rpm→m/s 单调表；第一版拨弹只做单发角度步进，最后才做连发与堵转恢复。

### 落到代码：发射业务

```text
当前规则状态
  → ShooterRulePolicy
  → 带安全余量的目标弹速 m/s
  → MuzzleVelocityMap
  → left/right rpm target + ramp
  → 两个独立 speed PI
  → 若使用 DJI 电机则输出 current A

operator request
  + heat limit/current heat/cooling
  + projectile allowance
  + shooter output enabled
  + both flywheels stable-ready
  + feeder feedback/fault
  → FireInterlock
  → 接受一次单发动作
  → LocalFireLedger 立即预约热量和弹量
```

```cpp
enum class FireBlockReason : std::uint8_t {
    None,
    OperatorNotRequesting,
    RefereeDataStale,
    ShooterPowerOff,
    NoProjectileAllowance,
    HeatBudgetInsufficient,
    FlywheelNotReady,
    FlywheelFeedbackLost,
    FeederFault,
    MuzzleSpeedFault,
};

struct FireDecision {
    bool allow_next_shot = false;
    float max_frequency_hz = 0.0f;
    FireBlockReason reason = FireBlockReason::OperatorNotRequesting;
};
```

裁判低频状态到来前，本地 ledger 要记录已经批准但尚未反映的弹量与热量，所有无符号减法先比较再减，避免 0 变成 65535。`ShotObserver` 对有效测速做统计和限幅后的慢偏置，不能因这一发低 1m/s 就在下一控制周期大幅加 rpm。实测超本地阈值要锁存故障并停止继续拨弹。

本步自检：UI ID 稳定；重连能重建；TX buffer 不被复用；UI 遵守共享预算；两个摩擦轮 PID 状态独立；关键裁判数据 stale、供电关闭、任一轮掉线或未持续到速时绝不拨弹；单发 ledger 不会重复消费同一 generation；实弹过程具备完整物理防护。

---

## 第 18 步：只在基础闭环稳定后做增强，并用总清单收口

第三方 `TongjiSuperPower/sp_middleware` 对本项目最有价值的是测试需求和行为边界：舵轮零速保持、连续角最短转向、纯关节串级、测量微分、Yaw 轨迹前馈和流式 parser。它的 HAL CAN/UART/Timer、整个 Gimbal 大类、模糊 PID、packed 裁判类型和 static TX buffer 不进入 SkyWalker。没有明确许可证授权前，不复制源码、CRC 表、注释或结构定义，也不作为 submodule 编译。

基础通过后，增强顺序建议是 `kS/kV/kA` → 裁判 TX scheduler → UI scene/diff → 超级电容融合 → slip telemetry → 必要时三维 frame math。打滑检测先只遥测；若底盘速度本身来自轮速里程计，它不是独立真值，不能凭一份未经实车验证的阈值接管驱动力。当前已有 IMU/滤波层，不因第三方仓库有 Mahony 就直接替换。HAL 外设代码一律由 Zephyr API承担。

### 每天开工的最小流程

```text
在本文找到当前步骤
  → 只选一个未完成自检项
  → 先写/更新纯测试
  → 构建通过后才接硬件
  → 硬件先 RX-only，再周期全零，再极小命令
  → 保存构建日志、抓包、参数和预期/实际
  → 本步自检全过才进入下一步
```

DJI 破坏性重构建议在独立分支按“冻结三型号纯 profile/codec → 新 core/Bus 测试 → 一次性切换所有调用方并删除旧 API → 启用 M2006 binding/验证 → 启用 GM6020 current binding/验证”推进。切换提交必须同时改公开头、CMake、binding、sample 和全部调用者，允许编译器暴露漏改，但禁止加兼容 wrapper。DJI 重构完成前不要夹带 PID、舵轮、板间协议或裁判业务修改。

### 建议由你执行的构建与测试命令

本文没有执行这些命令，因为它们会生成 workspace artifact。目录创建后按实际 Zephyr revision 调整：

```bash
# 统一 DJI sample（完成第 3 步后才存在）
west build -b dm_mc02/stm32h723xx \
  samples/motor/dji_unified \
  -d build/dji_unified -p always \
  -- -DBOARD_ROOT=$PWD

# 两个固件
west build -b dm_mc02/stm32h723xx \
  application/chassis -d build/chassis -p always \
  -- -DBOARD_ROOT=$PWD

west build -b dm_mc02/stm32h723xx \
  application/gimbal -d build/gimbal -p always \
  -- -DBOARD_ROOT=$PWD

# 纯测试；当前仓库还没有 tests/，须先按第 2/3 步创建
west twister -T tests/motor -p native_sim -v \
  -O build/twister/motor
# 若宿主机缺少 32 位 multilib，再尝试当前 Zephyr 支持的 64 位 native target
west twister -T tests/motor -p native_sim/native/64 -v \
  -O build/twister/motor-64

west twister -T tests/pid -p native_sim -v
west twister -T tests/control_math -p native_sim -v
west twister -T tests/swerve_kinematics -p native_sim -v
west twister -T tests/board_link -p native_sim -v
west twister -T tests/yaw_coordinator -p native_sim -v
west twister -T tests/referee -p native_sim -v
```

### 硬件上电总顺序

```text
1. C620 动力电源关闭，先烧录默认零锁定固件
2. 驱动轮架空、舵向/Yaw/发射机构卸载或可靠限位
3. 全断电检查 CAN/PWM 不同时连接、唯一 ID 和终端；两端 120Ω 时测约 60Ω
4. 先实测物理急停/动力切断有效，安排独立观察者
5. 只收反馈，不发非零命令；无 ACK 节点时不要在电调未上电前周期发送
6. 核对 motor-id、profile 推导出的 feedback/command/slot、UART 波特率与引脚
7. 每条 dji::Bus arm 前发送全部分组全零并抓包
8. 在 current-limit=0 或动力级隔离下验证 timeout、TTL、bus-off、stop/recover 和零帧失败升级
9. 每次启动收到新的人工 arm token 后，才允许单电机极小限时命令
10. 单速度环
11. 单关节串级与软限位
12. 单舵轮对齐保护
13. 四舵轮纯 +vx，再 +vy，再 +wz
14. 双板命令在 Disabled 下观察
15. 大、小 Yaw Independent
16. FollowAndRecenter
17. 裁判 RX-only
18. 功率策略先只记录缩放，不影响电机
19. UI
20. 发射机构无弹闭环
21. 具备挡弹与监督后才实弹单发
```

出现反馈时间戳停止、电流持续饱和、舵向误差越控越大、机构逼近硬限位、CAN error 快速上升、sequence 停止/倒退、Yaw 回中反而扩大偏差或实测弹速超阈值，立即清零并物理断电，不继续靠调参试错。

### 按现象查问题

| 现象                            | 先查什么                                                        | 不要先做什么           |
| ------------------------------- | --------------------------------------------------------------- | ---------------------- |
| device not ready                | Kconfig、CMake、binding、overlay status、phandle，见第 3 步     | 改 CAN 字节协议        |
| device ready 但 timestamp 为 0  | feedback ID、CAN 接线、电调 ID、filter                          | 改 PID                 |
| 命令 0A 仍有输出                | 检查`FlushReport.zero_sent`、抓所有分组零帧、查唯一 Bus owner | 只清软件缓存后停止发送 |
| Bus attach 报`-EADDRINUSE`    | feedback ID 冲突或同 command ID/slot 重复，核对 profile 表      | 绕过检查直接发帧       |
| Bus Fault 且`zero_sent=false` | CAN 发送/供电/接线异常，立即物理断电                            | 自动重新 arm           |
| recover 后旧非零复活            | lifecycle epoch/command epoch 实现错误，保持断电                | 增大 command TTL       |
| 8191→0 位置跳变                | 第 3 步统一 core 的回绕累计                                     | 在 PID 中补一圈        |
| M2006 温度总是 0                | C610 该字段为空，应取消 valid                                   | 伪造 0℃有效值         |
| GM6020 比例/ID 错               | 固件/Assistant/电流环证据及`0x1FE/0x2FE` profile              | 尝试版本不明的电压 raw |
| DM 全部比例错                   | 调试助手 P/V/T_MAX 与 Master ID                                 | 先改移位顺序           |
| 第二种电机接入就乱转            | 是否绕过 Bus 冲突检查、是否存在第二发送 owner，立即断电         | 靠换 command 数值排查  |
| setpoint 阶跃有 D 尖峰          | 第 7 步测量微分、滤波与 reset                                   | 盲目减所有增益         |
| 舵轮停车突然回零                | 第 10 步零速保持                                                | 用`atan2(0,0)`       |
| 四轮限速后运动方向改变          | 是否分别 clamp，改为共同缩放                                    | 单独放宽最大轮速       |
| UART 有字节但 CRC 全错          | baud、offset、CRC 参数、DMA buffer 所有权                       | 改业务 decoder         |
| UART 粘包偶尔失败               | 第 12/13 步半帧保留与逐字节重同步                               | 假设一次 callback 一帧 |
| UI 偶发花屏                     | async TX buffer、稳定 ID、共享频率                              | 每周期 DeleteAll       |
| 裁判数值错位                    | 当前赛季长度/offset、packed struct、端序                        | 用旧结构强转           |
| 断线仍继续运动                  | feedback timeout、command TTL、Bus Fault、零帧发送证据          | 只看驱动 heartbeat     |

### 最终进度板

- [ ] 第 0 步：硬件参数、ID 冲突和安全表完整。
- [ ] 第 1 步：唯一 A/SI 公共 API、profile 边界、Bus owner 和删除清单定稿。
- [ ] 第 2 步：三个型号 route/A↔raw/codec 纯测试及非法向量通过。
- [ ] 第 3 步：统一 core/Bus、generation/TTL/全零 Fault 测试通过；旧 API 零命中；M3508 安全回归。
- [ ] 第 4 步：启用 M2006 binding；温度 invalid、36:1、冲突和全零验证通过。
- [ ] 第 5 步：启用 GM6020 current binding；固件/电流环证据、ID7 上限、冲突和限位通过；无电压 API。
- [ ] 第 6 步：DM-J4310 MIT；不用时明确暂缓。
- [ ] 第 7 步：PID、角度、滤波、轨迹纯测试。
- [ ] 第 8 步：控制线程只输出 A；真实 dt、新反馈门控、命令 TTL、Bus 停机和物理断电升级完整。
- [ ] 第 9 步：M3508、GM6020、大/小 Yaw 单轴闭环。
- [ ] 第 10 步：舵轮正逆解、最短转向、共同缩放纯测试。
- [ ] 第 11 步：单模块、四模块和底盘状态机。
- [ ] 第 12 步：双固件、纯板间 codec/parser、enable token。
- [ ] 第 13 步：UART 双缓冲、worker、TX ownership。
- [ ] 第 14 步：大小 Yaw Independent 与低频卸载。
- [ ] 第 15 步：当前赛季裁判 RX、decoder、store。
- [ ] 第 16 步：保守降级与底盘功率共同缩放。
- [ ] 第 17 步：UI scene/diff；需要时完成安全发射业务。
- [ ] 第 18 步：增强功能有实测收益，整车总验收完成。

### 仍未验证、会阻止对应步骤通过的假设

- 当前半迁移的 `dji_protocol.hpp` 尚未证明可编译；新 core/Bus 与统一 sample 尚未实现。
- 仓库当前没有 `tests/`；本文列出的 Twister 命令只有创建测试目录后才能运行。
- M3508 标准减速箱是否确为 `3591/187`，还是当前实物已换过齿轮组。
- 每台 DJI 电机的机构安全电流、command TTL 和反馈 timeout 实测值。
- DJI 反馈 `current_raw` 是否能按命令满量程直接换算为安培；未以 Assistant/台架交叉验证前，不置 `FeedbackCurrent` valid。
- 当前板级 watchdog 或电源门能否在 owner 线程完全卡死时可靠切断动力；command TTL 本身不覆盖这种故障。
- DM-MC02 的 CAN 收发器/终端与 XT30 实际供电拓扑；C620/C610 是否使用外部 24V。
- GM6020 实物固件、Assistant 版本和电流环开关状态。
- 第一台整车是否实际使用 DM-J4310。
- 四舵轮几何、轮径、零点、方向和真实允许电流。
- 大、小 Yaw 型号、传动比、同轴程度、IMU 层级和限位。
- 双板最终使用 UART、Classic CAN 还是 CAN-FD，以及完整总线负载。
- 参赛赛项、兵种、弹丸和比赛前最终官方协议/规则版本。
- 裁判常规/图传 UART 在 DM-MC02 上的实际节点与引脚。
- 是否已经从 `sp_middleware` 维护者取得明确源码复用授权。

### 固定资料入口

电机协议优先使用仓库 `motor_docs/` 中的 C610、C620、M2006、M3508、GM6020 和 DM-J4310 固定版手册，并保留 `DOWNLOAD_LINKS.txt` 的来源记录。GM6020 电流模式以 2023 v1.4 手册为准；旧版/新版电压量程冲突是本轮明确排除电压 API 的原因，不应把任一数值写成跨固件常量。Zephyr 写法以当前工作区固定提交的头文件、官方 Device Model、CAN Controller 和 Async UART 文档为准。裁判系统每次实施前从 RoboMaster 官方资料中心重新下载当赛季通信协议与规则，不以本文示例偏移替代赛前核对。

第三方参考仓库固定审计入口是 `https://github.com/TongjiSuperPower/sp_middleware`。在许可证状态改变前，只把它当设计参考；若以后得到授权，保存许可证与书面记录、固定 commit、加入第三方清单，并先在隔离分支验证 C++14、Zephyr 平台边界和测试，再决定是否继续保留独立实现。
