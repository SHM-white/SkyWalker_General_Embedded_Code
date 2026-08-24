# SkyWalker C++ 电机驱动：0.1B 小模型执行手册 v2

> 目标：自己实现 SkyWalker 电机驱动。
> 风格：C++14 + Zephyr Device Model。
> 禁止：照搬 Breeze、异常、RTTI、new/delete、iostream、复杂 STL、大量 virtual。
> 协议字段只依据官方电机文档。

# 1. 开发顺序

```text
CAN smoke
→ M3508 raw
→ motor.hpp
→ M3508 正式 driver
→ DJI group TX
→ heartbeat
→ DM
→ LK
```

前一步没有编译并实际验证，不进入下一步。

# 2. CAN smoke

文件：

```text
samples/motor/can_smoke/
├── CMakeLists.txt
├── prj.conf
└── src/main.cpp
```

`main.cpp` 必须实现：

```cpp
static void rxCallback(
    const struct device *dev,
    struct can_frame *frame,
    void *user_data);

int main();
```

`main()` 顺序：

```text
DEVICE_DT_GET(can1)
→ device_is_ready()
→ can_start()
→ can_add_rx_filter()
→ 等待接收
```

验收：`CAN ready`，能收到至少一帧。

# 3. M3508 raw demo

文件：

```text
samples/motor/dji_m3508_raw/
├── CMakeLists.txt
├── prj.conf
└── src/
    ├── main.cpp
    ├── dji_m3508_protocol.hpp
    └── dji_m3508_protocol.cpp
```

## 3.1 `dji_m3508_protocol.hpp`

必须定义：

```cpp
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

## 3.2 `dji_m3508_protocol.cpp`

必须实现：

```cpp
static std::uint16_t readBe16(
    const std::uint8_t *data);

bool decodeM3508Feedback(...);

void buildGroupCurrentFrame(...);
```

职责分别是：

```text
readBe16                 两个大端字节 → uint16
decodeM3508Feedback      CAN frame → encoder/rpm/current/temp
buildGroupCurrentFrame   4 个 int16 command → 8 字节 DJI 控制帧
```

这里不允许访问 `device`、线程、DTS、heartbeat。

## 3.3 `main.cpp`

必须定义：

```cpp
struct MotorRuntime {
    skywalker::motor::dji::M3508Feedback feedback{};
    std::int64_t last_rx_ms = 0;
};

static MotorRuntime motor;
```

必须实现：

```cpp
static void rxCallback(...);
int main();
```

发送循环：

```cpp
struct can_frame frame{};

std::int16_t current[4] = {
    300, 0, 0, 0
};

while (true) {
    skywalker::motor::dji::buildGroupCurrentFrame(
        frame, 0x200, current);

    can_send(
        can,
        &frame,
        K_MSEC(1),
        nullptr,
        nullptr);

    k_sleep(K_MSEC(20));
}
```

验收：反馈合理，第一台 M3508 小电流稳定转动。

# 4. 公共 API：`motor.hpp`

文件：

```text
include/skywalker/motor/motor.hpp
```

必须定义：

```cpp
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

struct Api {
    int (*enable)(const struct device *dev);
    int (*disable)(const struct device *dev);
    int (*set_torque)(const struct device *dev, float torque_nm);
    int (*read_feedback)(const struct device *dev, Feedback *out);
    State (*get_state)(const struct device *dev);
};

}
```

还必须提供 inline wrapper：

```cpp
int enable(const struct device *dev);
int disable(const struct device *dev);
int setTorque(const struct device *dev, float torque_nm);
int readFeedback(const struct device *dev, Feedback &out);
State getState(const struct device *dev);
```

wrapper 只做：

```text
dev 检查
→ dev->api 检查
→ 函数指针检查
→ 调真实 driver
```

# 5. M3508 正式 driver

建立：

```text
drivers/motor/dji/
├── CMakeLists.txt
├── dji_protocol.cpp
├── dji_m3508.cpp
└── dji_tx_group.cpp

include/skywalker/motor/
├── motor.hpp
├── dji_protocol.hpp
└── dji_tx_group.hpp
```

raw demo 已验证的协议代码移动到 `dji_protocol.*`。

# 6. `dji_m3508.cpp`

这是正式 driver 的核心。

## 6.1 私有结构体

两个结构体都放在 `dji_m3508.cpp`。

```cpp
struct M3508Config {
    const struct device *can;
    std::uint16_t feedback_id;
    std::uint16_t command_id;
    std::uint8_t command_slot;
    float gear_ratio;
};

struct M3508Data {
    skywalker::motor::Feedback feedback{};
    std::int16_t command_raw = 0;
    std::uint64_t last_rx_ms = 0;
    struct k_spinlock lock{};
    int rx_filter_id = -1;
};
```

规则：

```text
Config = 永远不变
Data   = 运行状态
```

## 6.2 必须实现的函数

```cpp
static void m3508RxCallback(
    const struct device *can_dev,
    struct can_frame *frame,
    void *user_data);

static int m3508Init(
    const struct device *dev);

static int m3508Enable(
    const struct device *dev);

static int m3508Disable(
    const struct device *dev);

static int m3508SetTorque(
    const struct device *dev,
    float torque_nm);

static int m3508ReadFeedback(
    const struct device *dev,
    skywalker::motor::Feedback *out);

static skywalker::motor::State m3508GetState(
    const struct device *dev);

static const skywalker::motor::Api m3508_api;
```

最后还必须有：

```cpp
DEVICE_DT_INST_DEFINE(...)
DT_INST_FOREACH_STATUS_OKAY(...)
```

## 6.3 `m3508RxCallback()`

严格顺序：

```text
1. 检查 frame / user_data
2. user_data → const device *
3. dev->data → M3508Data *
4. dev->config → M3508Config *
5. decodeM3508Feedback()
6. rpm → rad/s
7. encoder → position
8. temperature → float
9. timestamp = k_uptime_get()
10. spin_lock
11. 更新 feedback 和 last_rx_ms
12. spin_unlock
```

禁止：

```text
sleep
PID
can_send
每帧 LOG_INF
```

## 6.4 `m3508Init()`

必须：

```text
1. 取 cfg/data
2. device_is_ready(cfg->can)
3. can_start(cfg->can)
4. 创建 filter
5. filter.id = cfg->feedback_id
6. filter.mask = CAN_STD_ID_MASK
7. can_add_rx_filter(...)
8. 保存 rx_filter_id
9. command_raw = 0
10. last_rx_ms = 0
11. 清 feedback
12. return 0
```

`can_start()` 的 `-EALREADY` 可以接受。

## 6.5 `m3508Enable()`

C620 没有 DM 那种独立 enable 帧。

第一版：

```cpp
return 0;
```

以后如果增加软件 enable flag，再扩展。

## 6.6 `m3508Disable()`

必须线程安全地：

```text
command_raw = 0
```

这里只更新本地命令，不直接发 CAN。

## 6.7 `m3508SetTorque()`

这里不要瞎算。

如果当前官方资料只能可靠得到：

```text
raw command ↔ current
```

而不能可靠得到输出轴：

```text
torque Nm ↔ current
```

那么：

```cpp
m3508SetTorque(...)
```

先返回：

```cpp
-ENOSYS
```

另外实现一个 driver 内部函数：

```cpp
static int m3508SetCurrentRaw(
    const struct device *dev,
    std::int16_t current_raw);
```

等有明确 `Kt`、减速比和效率模型后再实现 Nm API。

## 6.8 `m3508ReadFeedback()`

必须：

```text
out != nullptr
→ spin_lock
→ *out = data->feedback
→ spin_unlock
→ return 0
```

不要返回内部指针。

## 6.9 `m3508GetState()`

规则：

```text
last_rx_ms == 0
→ Offline

now - last_rx_ms > CONFIG_SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS
→ Offline

否则
→ Ready
```

# 7. M3508 DTS binding

文件：

```text
dts/bindings/motor/dji,m3508.yaml
```

至少提供：

```yaml
description: DJI M3508 motor with C620

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

overlay 例子：

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

# 8. `DEVICE_DT_INST_DEFINE`

目标是每个 DTS motor 节点生成：

```text
1 个 M3508Data
1 个 M3508Config
1 个 Zephyr device
```

概念实现：

```cpp
#define DT_DRV_COMPAT dji_m3508

#define M3508_DEFINE(inst)                         \
    static M3508Data data_##inst;                 \
                                                   \
    static const M3508Config config_##inst = {    \
        DEVICE_DT_GET(                             \
            DT_INST_PHANDLE(inst, can_bus)),       \
        DT_INST_PROP(inst, feedback_id),           \
        DT_INST_PROP(inst, command_id),            \
        DT_INST_PROP(inst, command_slot),          \
        static_cast<float>(                        \
            DT_INST_PROP(inst, gear_ratio)),       \
    };                                             \
                                                   \
    DEVICE_DT_INST_DEFINE(                         \
        inst,                                      \
        m3508Init,                                 \
        nullptr,                                   \
        &data_##inst,                              \
        &config_##inst,                            \
        POST_KERNEL,                               \
        CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY,      \
        &m3508_api);

DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)
```

宏细节以实际 Zephyr 4.4 编译结果调整。

# 9. DJI group TX

文件：

```text
include/skywalker/motor/dji_tx_group.hpp
drivers/motor/dji/dji_tx_group.cpp
```

## 9.1 `dji_tx_group.hpp`

必须定义：

```cpp
namespace skywalker::motor::dji {

class TxGroup {
public:
    int init(
        const struct device *can,
        std::uint16_t command_id);

    int setSlot(
        std::uint8_t slot,
        std::int16_t command);

    int send();

    void clear();

private:
    const struct device *can_ = nullptr;
    std::uint16_t command_id_ = 0;

    std::int16_t command_[4] = {
        0, 0, 0, 0
    };

    struct k_spinlock lock_{};
};

}
```

## 9.2 `TxGroup::init()`

```text
can 非空
→ device_is_ready
→ 保存 can
→ 保存 command_id
→ 清零 4 个 slot
```

## 9.3 `TxGroup::setSlot()`

```text
slot < 4
→ lock
→ command_[slot] = command
→ unlock
```

否则 `-EINVAL`。

## 9.4 `TxGroup::send()`

严格：

```text
1. 建 local command[4]
2. lock
3. copy command_
4. unlock
5. buildGroupCurrentFrame()
6. can_send()
7. 返回结果
```

禁止拿着 spinlock 调 `can_send()`。

## 9.5 `TxGroup::clear()`

锁内把四个 slot 清零。

第一版由 sample/control loop 每 10~20 ms 调一次：

```cpp
group.send();
```

不要先写万能 TX manager。

# 10. heartbeat

不新增线程。

Kconfig：

```Kconfig
config SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS
    int "Motor heartbeat timeout"
    default 100
    range 10 5000
```

heartbeat 只靠：

```text
RX 更新 last_rx_ms
getState() 比较当前时间
```

# 11. DM

文件：

```text
include/skywalker/motor/dm_protocol.hpp
include/skywalker/motor/dm_motor.hpp

drivers/motor/dm/
├── CMakeLists.txt
├── dm_protocol.cpp
└── dm_motor.cpp
```

# 12. `dm_protocol.hpp`

必须定义：

```cpp
namespace skywalker::motor::dm {

struct Limits {
    float pos_min;
    float pos_max;
    float vel_min;
    float vel_max;
    float torque_min;
    float torque_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
};

struct MitCommand {
    float position = 0.0f;
    float velocity = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;
};

struct RawFeedback {
    std::uint8_t state = 0;
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    std::uint8_t mos_temperature = 0;
    std::uint8_t motor_temperature = 0;
};

bool buildMitFrame(
    struct can_frame &frame,
    std::uint16_t can_id,
    const MitCommand &cmd,
    const Limits &limits);

bool buildEnableFrame(
    struct can_frame &frame,
    std::uint16_t can_id);

bool buildDisableFrame(
    struct can_frame &frame,
    std::uint16_t can_id);

bool buildSaveZeroFrame(
    struct can_frame &frame,
    std::uint16_t can_id);

bool decodeFeedback(
    const struct can_frame &frame,
    const Limits &limits,
    RawFeedback &out);

}
```

# 13. `dm_protocol.cpp`

必须实现：

```cpp
static float clampFloat(...);

static std::uint32_t floatToUint(
    float value,
    float min,
    float max,
    unsigned bits);

static float uintToFloat(
    std::uint32_t value,
    float min,
    float max,
    unsigned bits);
```

以及：

```text
MIT 16/12-bit 打包
enable 帧
disable 帧
save-zero 帧
feedback 解包
```

这里只做协议，不访问 Zephyr device。

# 14. `dm_motor.hpp`

只放 DM 特有公共 API：

```cpp
namespace skywalker::motor::dm {

int setMit(
    const struct device *dev,
    const MitCommand &cmd);

int saveZero(
    const struct device *dev);

}
```

# 15. `dm_motor.cpp`

私有结构：

```cpp
struct DmConfig {
    const struct device *can;
    std::uint16_t tx_id;
    std::uint16_t rx_id;
    skywalker::motor::dm::Limits limits;
};

struct DmData {
    skywalker::motor::Feedback feedback{};
    std::uint64_t last_rx_ms = 0;
    struct k_spinlock lock{};
    int rx_filter_id = -1;
};
```

必须实现：

```cpp
static void dmRxCallback(...);

static int dmInit(
    const struct device *dev);

static int dmEnable(
    const struct device *dev);

static int dmDisable(
    const struct device *dev);

static int dmSetTorque(
    const struct device *dev,
    float torque_nm);

static int dmReadFeedback(
    const struct device *dev,
    skywalker::motor::Feedback *out);

static skywalker::motor::State dmGetState(
    const struct device *dev);

int dmSetMit(
    const struct device *dev,
    const skywalker::motor::dm::MitCommand &cmd);

int dmSaveZero(
    const struct device *dev);
```

调用关系：

```text
dmEnable
→ buildEnableFrame
→ can_send

dmDisable
→ buildDisableFrame
→ can_send

dmSetMit
→ buildMitFrame
→ can_send

dmSetTorque
→ 构造 kp=0 kd=0 pos=0 vel=0 的 MitCommand
→ dmSetMit

dmRxCallback
→ decodeFeedback
→ 公共 Feedback
→ 保存 timestamp
```

# 16. LK

没有官方协议文档前不要正式实现。

未来文件：

```text
include/skywalker/motor/lk_protocol.hpp
include/skywalker/motor/lk_motor.hpp

drivers/motor/lk/
├── lk_protocol.cpp
└── lk_motor.cpp
```

第一版只做：

```text
enable
disable
torque
speed
feedback
```

`lk_protocol.hpp` 至少需要：

```cpp
bool buildEnableFrame(...);
bool buildDisableFrame(...);
bool buildTorqueFrame(...);
bool buildSpeedFrame(...);
bool decodeFeedback(...);
```

`lk_motor.cpp` 至少需要：

```cpp
static void lkRxCallback(...);
static int lkInit(...);
static int lkEnable(...);
static int lkDisable(...);
static int lkSetTorque(...);
static int lkReadFeedback(...);
static skywalker::motor::State lkGetState(...);
```

vendor API：

```cpp
int setSpeed(
    const struct device *dev,
    float rad_s);
```

# 17. Kconfig

`drivers/motor/Kconfig`：

```Kconfig
menuconfig SKYWALKER_MOTOR
    bool "SkyWalker motor subsystem"
    depends on CAN

if SKYWALKER_MOTOR

config SKYWALKER_MOTOR_DJI
    bool "DJI motors"
    default y

config SKYWALKER_MOTOR_DM
    bool "Damiao motors"

config SKYWALKER_MOTOR_LK
    bool "Lingkong motors"

config SKYWALKER_MOTOR_INIT_PRIORITY
    int "Motor init priority"
    default 90
    range 0 99

config SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS
    int "Motor heartbeat timeout"
    default 100
    range 10 5000

endif
```

在 `drivers/Kconfig`：

```Kconfig
rsource "./motor/Kconfig"
```

# 18. CMake

`drivers/CMakeLists.txt`：

```cmake
add_subdirectory_ifdef(
    CONFIG_SKYWALKER_MOTOR
    motor
)
```

`drivers/motor/CMakeLists.txt`：

```cmake
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

`drivers/motor/dji/CMakeLists.txt`：

```cmake
zephyr_library()

zephyr_library_sources(
    dji_protocol.cpp
    dji_m3508.cpp
    dji_tx_group.cpp
)
```

DM/LK 同理。

# 19. 文件职责检查

`protocol.cpp`：

```text
只编码/解码
不保存 runtime state
不访问 device
```

`driver.cpp`：

```text
Config/Data
CAN 注册
调用 protocol
Device API
```

public header：

```text
只放应用层要用的 API
不暴露 Config/Data
```

# 20. 当前执行顺序

如果 raw M3508 已经跑通：

第一步只写：

```text
include/skywalker/motor/motor.hpp
```

完成：

```text
State
FeedbackValid
Feedback
Api
enable()
disable()
setTorque()
readFeedback()
getState()
```

然后写：

```text
drivers/motor/dji/dji_m3508.cpp
```

第一轮只实现：

```text
M3508Config
M3508Data
m3508RxCallback
m3508Init
m3508ReadFeedback
m3508GetState
```

这部分能 `west build` 后，再实现：

```text
m3508Enable
m3508Disable
m3508SetTorque / setCurrentRaw
TxGroup
```

# 21. 给本地小模型的强制规则

```text
1. 一次只实现当前指定文件/函数。
2. 使用 C++14。
3. 禁止 new/delete、异常、RTTI、复杂 STL。
4. 不复制 Breeze 源码。
5. 协议字段只从官方文档提取。
6. protocol.cpp 只做协议。
7. driver.cpp 只做 Device/CAN/state。
8. Config/Data 保持 driver 私有。
9. CAN RX callback 不 sleep、不 PID、不高频日志。
10. 不提前写 manager/factory/template framework。
11. 每个新增函数必须有明确调用者。
12. 每一阶段先 west build 成功再继续。
13. 协议参数不确定时禁止猜测。
```
