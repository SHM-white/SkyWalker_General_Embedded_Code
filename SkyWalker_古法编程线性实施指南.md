# SkyWalker 古法编程线性实施指南

> 适用仓库：当前 `skywalker_code` 工作区
> Zephyr 基线：`west.yml` 固定提交 `6085aadeb337f27ee7411fa2ecbe8cf49164e360`，本地版本 `4.4.99`
> 目标平台：`dm_mc02/stm32h723xx`
> 语言边界：C11 + C++14
> 文档角色：这是唯一施工主线；它合并并取代原先分散的电机、PID、舵轮、双板、裁判系统和 `sp_middleware` 审计文档。

这份指南把源码当作教材，不替你修改业务代码。你每次只做当前步骤，完成自检并保存证据后才进入下一步。本文特意把解释放在代码前面：先把一段话读完，先在脑中建立数据流和安全边界，再看紧随其后的完整示例、伪代码或流程图。第一次出现 Zephyr 特有写法时会展开完整形状；后面遇到相同写法只标“见第 N 步”，避免一份文档里出现几套略有差异的模板。

## 怎么使用这份文档

每一步都按同一种节奏阅读。正文先说明本步解决什么问题、哪些文件负责什么、错误时应该停在哪里；“落到代码”才给示例；“本步自检”是出口。示例中的参数不是实车默认值，尤其是电流、力矩、限位、PID 和通信超时，必须用第 0 步的实测表替换。

后文采用以下回指规则：

```text
“Zephyr 设备接线见第 1 步”
    = 复用 Kconfig → CMake → binding → overlay → DEVICE_DT_INST_DEFINE → sample 的完整链

“纯协议测试见第 3 步”
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

## 当前起点和最终数据流

当前仓库已经有 M3508 的 Zephyr device、CAN RX、raw 电流缓存、一个 M3508 专用 `TxGroup` 和 sample；PID 只有最小位置式算法；`application/` 仍为空；M2006、GM6020、DM-J4310、舵轮、双板链路和裁判系统都还没有业务实现。现有 M3508 不是“已经完成”，而是第 1～3 步的可回归基线。

整个项目只按下面一条线施工：

```text
第 0 步  固定硬件参数和安全边界
   ↓
第 1 步  看懂一次完整 Zephyr 驱动接线
   ↓
第 2 步  修正并回归现有 M3508
   ↓
第 3 步  抽出 DJI 公共层并建立纯协议测试
   ↓
第 4 步  M2006/C610
   ↓
第 5 步  GM6020 电流模式
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

第一次电机上电必须架空或拆负载，先证明“持续发送全 0”真的能停，再允许极小非零命令。DJI 的 `disable()` 只清某台电机的软件缓存，不代表共享分组帧已经送到电调；真正停机必须由分组所有者清空四槽并发送。任何舵向、Yaw 和发射机构测试都要有伸手可及的物理断电路径。

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

## 第 1 步：只用 M3508 看懂一次完整的 Zephyr 驱动接线

Zephyr 驱动不是“写一个 `.cpp` 然后 include”。一项功能从配置到运行要经过 Kconfig、CMake、devicetree binding、board/application overlay、设备实例宏和应用取设备六个环节。把这条链看懂一次，后面的 M2006、GM6020 和 DM-J4310 就只改变协议与型号语义，不再重复学习框架魔法。

当前仓库已经具备这条链，所以本步以“对照阅读”为主，不要求你覆盖文件。模块入口 `zephyr/module.yml` 让 Zephyr 找到仓库根 `Kconfig` 与根 `CMakeLists.txt`；根 Kconfig 再 `rsource` 到 drivers，CMake 则按 `CONFIG_...` 符号决定是否编译子目录。注意 Kconfig 声明时写 `SKYWALKER_DRIVER_MOTOR`，只有在 C/C++ 和 `prj.conf` 中才带 `CONFIG_` 前缀。

### 落到代码：完整的配置与构建接线

根模块入口已经存在，应保持这种形状：

```yaml
# zephyr/module.yml
name: "skywalker"
build:
  kconfig: Kconfig
  cmake: .
  settings:
    board_root: .
    dts_root: .
```

```kconfig
# Kconfig
rsource "drivers/Kconfig"
rsource "lib/Kconfig"
```

```kconfig
# drivers/Kconfig 中与 motor 有关的完整片段
menu "Skywalker Device Drivers"

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

```kconfig
# drivers/motor/Kconfig
config SKYWALKER_MOTOR_DJI
    bool "DJI CAN motors"
    default y

config SKYWALKER_MOTOR_INIT_PRIORITY
    int "Motor driver init priority"
    default 90
    range 0 99

config SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS
    int "Motor feedback timeout in milliseconds"
    default 100
    range 10 5000
```

```cmake
# 根 CMakeLists.txt 中的模块入口
zephyr_include_directories(include)
add_subdirectory(drivers)
add_subdirectory(lib)
```

```cmake
# drivers/CMakeLists.txt
add_subdirectory_ifdef(CONFIG_SKYWALKER_DRIVER_MOTOR motor)
```

```cmake
# drivers/motor/CMakeLists.txt
add_subdirectory_ifdef(CONFIG_SKYWALKER_MOTOR_DJI dji)
```

```cmake
# drivers/motor/dji/CMakeLists.txt
zephyr_library()
zephyr_library_sources(
    dji_protocol.cpp
    dji_m3508.cpp
    dji_tx_group.cpp
)
```

应用的 `prj.conf` 打开最终参与上述条件编译的符号：

```ini
# samples/motor/dji_m3508_driver/prj.conf
CONFIG_CPP=y
CONFIG_CAN=y
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
CONFIG_SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS=100
```

```cmake
# samples/motor/dji_m3508_driver/CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dji_m3508_driver)
target_sources(app PRIVATE src/main.cpp)
```

### 落到代码：完整的 binding、overlay 与实例映射

Binding 描述“一个合法的设备树节点必须有哪些属性”，overlay 描述“这块板上实际有几台、每台是什么参数”。属性名在 DTS/YAML 里使用连字符，进入 C 宏时写成下划线。`compatible = "dji,m3508"` 会对应 `#define DT_DRV_COMPAT dji_m3508`。

```yaml
# dts/bindings/motor/dji,m3508.yaml
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

```dts
/* samples/motor/dji_m3508_driver/app.overlay */
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

驱动 `.cpp` 中最容易看不懂的是实例宏。它不是运行时工厂，而是在编译期为每个 `status = "okay"` 的节点生成一份只读 Config、一份可写 Data 和一个 `struct device`。下面是完整注册形状；RX、单位换算和 API 函数仍以当前 `drivers/motor/dji/dji_m3508.cpp` 为准。

```cpp
#define DT_DRV_COMPAT dji_m3508

struct M3508Config {
    const struct device *can;
    std::uint16_t feedback_id;
    std::uint16_t command_id;
    std::uint8_t command_slot;
    float gear_ratio;
};

struct M3508Data {
    skywalker::motor::Feedback feedback{};
    skywalker::motor::dji::M3508Feedback raw_feedback{};
    std::int16_t command_raw = 0;
    std::uint64_t last_rx_ms = 0;
    struct k_spinlock lock{};
    int rx_filter_id = -1;
};

static int m3508Init(const struct device *dev);
static const skywalker::motor::Api m3508_api;

#define M3508_DEFINE(inst)                                                    \
    static M3508Data m3508_data_##inst;                                       \
    static const M3508Config m3508_config_##inst = {                          \
        DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                        \
        static_cast<std::uint16_t>(DT_INST_PROP(inst, feedback_id)),          \
        static_cast<std::uint16_t>(DT_INST_PROP(inst, command_id)),           \
        static_cast<std::uint8_t>(DT_INST_PROP(inst, command_slot)),          \
        static_cast<float>(DT_INST_PROP(inst, gear_ratio)),                   \
    };                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                               \
                          m3508Init,                                          \
                          nullptr,                                            \
                          &m3508_data_##inst,                                 \
                          &m3508_config_##inst,                               \
                          POST_KERNEL,                                        \
                          CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY,               \
                          &m3508_api);

DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)
```

`m3508Init()` 的完整 Zephyr CAN 接收骨架如下。`can_add_rx_filter()` 的 callback 运行在中断上下文，因此只做 DLC/字节解码、时间戳和一次短自旋锁提交；不能 sleep、不能做 PID、不能打印高频日志，也不能调用可能等待邮箱的 `can_send()`。

```cpp
static void m3508RxCallback(const struct device *can_dev,
                            struct can_frame *frame,
                            void *user_data)
{
    ARG_UNUSED(can_dev);
    if (frame == nullptr || user_data == nullptr) {
        return;
    }

    const auto *dev = static_cast<const struct device *>(user_data);
    auto *data = static_cast<M3508Data *>(dev->data);
    const auto *cfg = static_cast<const M3508Config *>(dev->config);

    skywalker::motor::dji::M3508Feedback raw{};
    if (!skywalker::motor::dji::decodeM3508Feedback(*frame, raw)) {
        return;
    }

    skywalker::motor::Feedback next{};
    if (cfg->gear_ratio > 0.0f) {
        next.velocity_rad_s = static_cast<float>(raw.rpm)
                            * (6.283185307179586f / 60.0f)
                            / cfg->gear_ratio;
        next.valid |= skywalker::motor::FeedbackVelocity;
    }
    next.temperature_c = static_cast<float>(raw.temperature);
    next.valid |= skywalker::motor::FeedbackTemperature;
    next.timestamp_ms = static_cast<std::uint64_t>(k_uptime_get());

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->raw_feedback = raw;
    data->feedback = next;
    data->last_rx_ms = next.timestamp_ms;
    k_spin_unlock(&data->lock, key);
}

static int m3508Init(const struct device *dev)
{
    if (dev == nullptr) {
        return -EINVAL;
    }

    auto *data = static_cast<M3508Data *>(dev->data);
    const auto *cfg = static_cast<const M3508Config *>(dev->config);
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

    ret = can_add_rx_filter(cfg->can,
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
```

应用通过 alias 获取编译期设备句柄，再用 `device_is_ready()` 判断驱动初始化是否成功。以后 M2006、GM6020 和 DM-J4310 都复用这条链，不再重复解释 `DT_DRV_COMPAT` 和 `DEVICE_DT_INST_DEFINE`。

```cpp
#define MOTOR0_NODE DT_ALIAS(motor0)

int main()
{
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    if (!device_is_ready(motor)) {
        return -ENODEV;
    }

    for (;;) {
        skywalker::motor::Feedback feedback{};
        const int ret = skywalker::motor::readFeedback(motor, feedback);
        if (ret == 0) {
            /* 低频记录，不在 CAN callback 中打印。 */
        }
        k_sleep(K_MSEC(100));
    }
}
```

本步自检：你应当能从 overlay 的 `motor0` 一路指出它如何变成 `M3508Config`、`M3508Data` 和 `struct device`，也能解释为什么 callback 只更新反馈快照。此时不要改协议和控制算法。

---

## 第 2 步：修正公共 API，并把现有 M3508 回归成可信基线

新增型号前先修一个很小却会扩散的错误：当前 `motor.hpp` 的 `disable()`、`readFeedback()` 和 `getState()` 检查错了函数指针。若不先修，某个驱动缺少对应方法时可能调用空指针，或者因为 `enable` 恰好存在而掩盖错误。公共 API 还应显式包含 `<errno.h>`，不要依赖 Zephyr 头文件间接带入错误码。

M3508 的首次回归只允许 raw 0。当前 sample 中写了 `setCurrentRaw(motor, 300)`，你必须先改成 0，抓到全零 `0x200`，验证反馈与 Offline timeout，再由人工确认允许极小非零方向测试。`clear()` 只改缓存；本步至少要继续调用 `send()`，第 3 步再正式增加 `clearAndSend()`。

### 落到代码：完整公共 API 形状

文件 `include/drivers/motor/motor.hpp` 应整理成下面的接口。结构布局保持简单；物理单位只有确实可换算时才标 valid。

```cpp
#pragma once

#include <cstdint>
#include <errno.h>
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

struct Api {
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

### 落到代码：安全的 M3508 回归顺序

sample 的控制部分先保持下面的流程。这里刻意不提供可直接误上电的非零值；极小方向测试由你在全零证据完成后临时填写，并立即恢复为 0。

```cpp
int ret = tx_group.init(can, 0x200);
if (ret < 0) return ret;

ret = tx_group.bindMotor(motor);
if (ret < 0) return ret;

ret = skywalker::motor::dji::m3508::setCurrentRaw(motor, 0);
if (ret < 0) return ret;

for (;;) {
    ret = tx_group.send();
    if (ret < 0) {
        tx_group.clear();
        /* 仍要设法发送零帧；第 3 步会统一为 clearAndSend。 */
    }

    skywalker::motor::dji::M3508Feedback raw{};
    skywalker::motor::dji::m3508::readRawFeedback(motor, raw);
    const auto state = skywalker::motor::getState(motor);

    /* 20 ms 发送；打印另行降频。 */
    k_sleep(K_MSEC(20));
}
```

建议由你运行，本文没有运行会生成 `build/` 的命令：

```bash
west build \
  -b dm_mc02/stm32h723xx \
  samples/motor/dji_m3508_driver \
  -d build/dji_m3508_driver \
  -p always \
  -- -DBOARD_ROOT=$PWD

west flash -d build/dji_m3508_driver
```

本步自检：构建成功；`device_is_ready` 为真；raw 0 的八字节命令全零；encoder/rpm/current/temperature 字节合理；拔掉 CAN 后约在配置心跳时间进入 Offline；两台 M3508 的 slot 0/1 不互相覆盖；清零后总线上仍持续出现零命令。任何一项失败都不要进入第 3 步。

---

## 第 3 步：把 DJI 公共层收口，并第一次建立纯协议 ztest

M2006 和 GM6020 的 8 字节反馈外形与 M3508 相近，但温度语义、命令量程、ID 与物理单位不同。正确复用点是“按大端拆 16 位”和“把四个 `int16_t` 放进四个槽”，不是复制整份 M3508 驱动。因此先把 `M3508Feedback` 收口成协议中立的 `DjiFeedbackRaw`，把 `buildGroupCurrentFrame()` 收口成 `buildGroupCommandFrame()`，保留一小段兼容包装让现有 M3508 继续工作。

当前 `TxGroup` 直接调用 `m3508::*`，下一台电机无法绑定。分组发送器应只认识一个 `GroupCommandSource`：其中保存电机、CAN、command ID、slot 和读写 raw 的函数指针。它不需要用 RTTI 判断型号。重复槽位必须报 `-EADDRINUSE`，CAN 不同必须报错，读取任一槽失败时不能发送半新半旧的帧。

### 落到代码：公共 codec 和分组契约

```cpp
struct DjiFeedbackRaw {
    std::uint16_t encoder = 0;
    std::int16_t rpm = 0;
    std::int16_t effort_raw = 0;
    std::uint8_t temperature = 0;
};

bool decodeFeedback(const struct can_frame &frame,
                    DjiFeedbackRaw &out);

void buildGroupCommandFrame(struct can_frame &frame,
                            std::uint16_t command_id,
                            const std::int16_t command[4]);

/* 迁移期保留旧字段名，不能简单 using：旧调用方使用 current_raw，
 * 新公共结构使用 effort_raw。包装函数显式逐字段转换。
 */
struct M3508Feedback {
    std::uint16_t encoder = 0;
    std::int16_t rpm = 0;
    std::int16_t current_raw = 0;
    std::uint8_t temperature = 0;
};

inline bool decodeM3508Feedback(const struct can_frame &frame,
                                M3508Feedback &out)
{
    DjiFeedbackRaw common{};
    if (!decodeFeedback(frame, common)) return false;
    out.encoder = common.encoder;
    out.rpm = common.rpm;
    out.current_raw = common.effort_raw;
    out.temperature = common.temperature;
    return true;
}
```

四槽打包先把有符号值转换成同位宽无符号位模式，再移位；不要依赖负数右移是否算术扩展：

```cpp
for (int i = 0; i < 4; ++i) {
    const std::uint16_t bits =
        static_cast<std::uint16_t>(command[i]);
    frame.data[i * 2] =
        static_cast<std::uint8_t>((bits >> 8) & 0xFFu);
    frame.data[i * 2 + 1] =
        static_cast<std::uint8_t>(bits & 0xFFu);
}
```

```cpp
using ReadRawCommand =
    int (*)(const struct device *dev, std::int16_t &out);

using WriteRawCommand =
    int (*)(const struct device *dev, std::int16_t value);

struct GroupCommandSource {
    const struct device *motor = nullptr;
    const struct device *can = nullptr;
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
    ReadRawCommand read_raw = nullptr;
    WriteRawCommand write_raw = nullptr;
};

class TxGroup {
public:
    int init(const struct device *can, std::uint16_t command_id);
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

```text
TxGroup::send
  先令 local_command = [0, 0, 0, 0]
  对四槽逐一调用 read_raw
  任一失败：返回错误，不发送
  全部成功：buildGroupCommandFrame
  在线程中 can_send

TxGroup::clearAndSend
  对所有已绑定 source 调 write_raw(..., 0)，记录第一个缓存错误
  不再通过 read_raw 拼帧，而是直接构造 local_zero = [0,0,0,0]
  buildGroupCommandFrame(command_id, local_zero) 并尝试 can_send
  若缓存清零失败：group 保持 Fault，禁止下一次普通 send
  返回值同时能区分缓存清零失败与 CAN 零帧发送失败
```

### 落到代码：第一次完整的 native_sim + ztest 示例

协议纯函数不依赖真实 CAN 控制器，应该先在 `native_sim` 上跑。以后 PID、舵轮、板间 codec 和裁判 parser 都复用这个目录形状，只替换被测源文件和断言；不要每章重新发明测试入口。

```text
tests/motor/dji_protocol/
├── CMakeLists.txt
├── prj.conf
├── testcase.yaml
└── src/main.cpp
```

```cmake
# tests/motor/dji_protocol/CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(skywalker_dji_protocol_test)

target_sources(app PRIVATE
    src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../../drivers/motor/dji/dji_protocol.cpp
)
```

```ini
# tests/motor/dji_protocol/prj.conf
CONFIG_CPP=y
CONFIG_ZTEST=y
```

```yaml
# tests/motor/dji_protocol/testcase.yaml
tests:
  skywalker.motor.dji_protocol:
    platform_allow:
      - native_sim
    integration_platforms:
      - native_sim
    tags:
      - motor
      - protocol
```

```cpp
// tests/motor/dji_protocol/src/main.cpp
#include <cstdint>
#include <zephyr/drivers/can.h>
#include <zephyr/ztest.h>

#include <drivers/motor/dji_protocol.hpp>

using namespace skywalker::motor::dji;

ZTEST(dji_protocol, test_decode_feedback_big_endian)
{
    struct can_frame frame{};
    frame.id = 0x201;
    frame.dlc = 8;
    const std::uint8_t bytes[8] = {
        0x12, 0x34, 0xFF, 0x9C, 0x00, 0x7B, 0x37, 0x00
    };
    for (int i = 0; i < 8; ++i) frame.data[i] = bytes[i];

    DjiFeedbackRaw out{};
    zassert_true(decodeFeedback(frame, out), "decode failed");
    zassert_equal(out.encoder, 0x1234, "encoder");
    zassert_equal(out.rpm, -100, "rpm sign extension");
    zassert_equal(out.effort_raw, 123, "effort");
    zassert_equal(out.temperature, 55, "temperature byte");
}

ZTEST(dji_protocol, test_encode_four_slots)
{
    const std::int16_t command[4] = {
        static_cast<std::int16_t>(0x1234),
        static_cast<std::int16_t>(-1),
        static_cast<std::int16_t>(-16384),
        static_cast<std::int16_t>(0),
    };
    const std::uint8_t expected[8] = {
        0x12, 0x34, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00
    };

    struct can_frame frame{};
    buildGroupCommandFrame(frame, 0x200, command);
    zassert_equal(frame.id, 0x200, "command id");
    zassert_equal(frame.dlc, 8, "dlc");
    for (int i = 0; i < 8; ++i) {
        zassert_equal(frame.data[i], expected[i], "byte %d", i);
    }
}

ZTEST(dji_protocol, test_reject_wrong_dlc)
{
    struct can_frame frame{};
    frame.dlc = 7;
    DjiFeedbackRaw out{};
    zassert_false(decodeFeedback(frame, out), "DLC 7 accepted");
}

ZTEST_SUITE(dji_protocol, nullptr, nullptr, nullptr, nullptr, nullptr);
```

由你运行：

```bash
west twister -T tests/motor/dji_protocol -p native_sim -v
```

本步自检：codec 不引用具体电机 namespace；TxGroup 不引用 `m3508::*`；空槽发送 0；重复槽报错；`clearAndSend()` 在总线上产生全零；固定向量通过；第 2 步的 M3508 行为没有改变。

---

## 第 4 步：用 M2006 验证 DJI 公共层真的可复用

M2006/C610 与 M3508 最接近，适合做第一个新增型号。它的命令 raw 硬范围是 `±10000`，默认减速比按本地手册记录为 36；反馈第 6、7 字节在 C610 协议中是空字段，不能因为公共 raw 结构里有 `temperature` 就把 `FeedbackTemperature` 标成有效。协议结构可以复用，型号语义必须留在 `dji_m2006.cpp`。

位置需要跨 8191→0 做多圈累计。第一次收到 encoder 只建立基准，不假装相对位置突然跳到某个绝对角；后续差值超过半圈 4096 才修正回绕。这个算法以后 GM6020 舵向连续角也能参考，但是否允许多圈要受真实线缆与机械限位约束。

Zephyr binding、overlay、`DT_DRV_COMPAT` 和 device 实例写法全部见第 1 步；测试目录见第 3 步。本步只写型号差异。

### 落到代码

```cpp
struct M2006Data {
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

feedback.position_rad =
    static_cast<float>(data->total_encoder_ticks) *
    (6.283185307179586f / 8192.0f) /
    cfg->gear_ratio;

feedback.velocity_rad_s =
    static_cast<float>(raw.rpm) *
    (6.283185307179586f / 60.0f) /
    cfg->gear_ratio;

feedback.valid = skywalker::motor::FeedbackPosition |
                 skywalker::motor::FeedbackVelocity;
/* 不设置 FeedbackTemperature。 */
```

```cpp
int setCurrentRaw(const struct device *dev, std::int16_t value)
{
    if (dev == nullptr) return -EINVAL;
    if (value < -10000 || value > 10000) return -ERANGE;
    /* 加锁提交 command_raw，形状见现有 M3508。 */
    return 0;
}
```

初始化时由 feedback ID 推导并交叉检查 command ID/slot：

```text
ESC ID = feedback_id - 0x200

ID 1..4：command_id 必须为 0x200，slot = ID - 1
ID 5..8：command_id 必须为 0x1FF，slot = ID - 5
否则初始化失败，不允许带错配置运行
```

本步自检：只接收时能看到正确 ID；手转输出轴跨零不跳一圈；温度 valid 未设置；周期全零无运动；全零证据后才用极小 raw 检查方向；同组多台不覆盖；拔线后控制层清零。raw 电流不是 rpm，空载持续小电流也可能越转越快。

---

## 第 5 步：实现 GM6020 时只保留一种默认语义——电流模式

旧文档中曾有一处完成清单误写成电压模式，本合并稿统一裁决：项目默认 GM6020 电流模式，ID 1～4 发 `0x1FE`，ID 5～7 发 `0x2FE`，raw `±16384` 约对应 `±3A`。电压模式若以后确有硬件需求，必须以单独 `setVoltageRaw()`、`0x1FF/0x2FF` 和 `±30000` 路径显式实现，不能让 `setRaw()` 在两种单位间含糊切换。

GM6020 反馈 ID 是 `0x204 + motor_id`，直驱角度/速度不除减速比。协议 raw 结构仍复用第 3 步，Zephyr device 接线仍见第 1 步，TxGroup 通过 `makeGroupCommandSource()` 接入。控制器内部建议用 A，发送适配层才换 raw；M3508 和 GM6020 虽然都出现 `16384`，对应的安培比例并不相同。

### 落到代码

```text
motor ID    feedback ID    current command ID    slot
1           0x205          0x1FE                 0
2           0x206          0x1FE                 1
3           0x207          0x1FE                 2
4           0x208          0x1FE                 3
5           0x209          0x2FE                 0
6           0x20A          0x2FE                 1
7           0x20B          0x2FE                 2

0x2FE 的第 4 槽永远保持 0
```

```cpp
int setCurrentAmps(const struct device *dev,
                   float current_a,
                   float mechanism_safe_current_a)
{
    if (dev == nullptr || !std::isfinite(current_a)) return -EINVAL;
    if (!(mechanism_safe_current_a > 0.0f) ||
        mechanism_safe_current_a > 3.0f) return -ERANGE;
    if (current_a < -mechanism_safe_current_a ||
        current_a > mechanism_safe_current_a) return -ERANGE;

    const float raw_f = current_a * 16384.0f / 3.0f;
    const auto raw = static_cast<std::int16_t>(std::lround(raw_f));
    return setCurrentRaw(dev, raw);
}
```

```dts
gm6020_1: motor-1 {
    compatible = "dji,gm6020";
    status = "okay";
    can-bus = <&can1>;
    feedback-id = <0x205>;
    command-id = <0x1FE>;
    command-slot = <0>;
    control-mode = "current";
};
```

本步自检：初始化严格校验 ID/组/slot；反馈位置、速度、温度 valid 正确；命令只走 `0x1FE/0x2FE`；全零停机抓包通过；很小安全电流方向通过；软/硬限位尚未验证前不闭环追大角度。

---

## 第 6 步：DM-J4310 另起协议族，第一版只做 MIT

DM-J4310 不是四槽 DJI 协议，不能通过改几个 ID 塞进 `TxGroup`。它每台电机一帧，反馈 filter 匹配调试助手配置的 Master ID，命令发往 Motor ID。第一版只做完整 MIT 帧、反馈、显式 enable/disable，以及当前固件确认支持后才开放的设零；不要同时实现 MIT、位置速度、速度和参数读写大全。

写代码前先用达妙调试工具记录 P_MAX、V_MAX、T_MAX。它们决定协议量化比例，不等于硬件安全上限。本地 V1.1 手册记录硬件峰值 7N·m，但机构允许值通常更小，所以协议 Limits 和 SafetyLimits 必须分开。产品手册说明输出轴单圈绝对位置；不要再做 DJI 8192 累计，也不要擅自再除以 10。

构建开关、子目录和 device 实例复用第 1 步；纯映射测试复用第 3 步。

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

`a_ref` 必须来自轨迹生成器或经过斜坡后的速度目标，不能直接差分摇杆阶跃。输出域若是 M3508 raw，则 `kS/kV/kA` 的单位都是 raw 系列；若是 GM6020 A，则系数是 A 系列；若是 DM-J4310 N·m，则是力矩系列，参数绝不跨型号复制。

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

纯测试沿用第 3 步的 ztest 目录，至少覆盖：`P+FF=4.5`、输出饱和仍记录 unsaturated、正饱和时积分不继续增大、setpoint 阶跃时 D 接近 0、deadband 内保留 FF、reset 后 D 为 0，以及 `dt=0/负数/NaN/过大` 返回错误且不改状态。角度测试必须跨 `±π`、首次初始化、NaN 和 Inf。

在线调参时，设备树生成的 `dev->config` 是只读开机默认值，不能强转后写。启动时把默认配置复制到 RAM；UART callback 只把 `key/value` 放进 `k_msgq`；控制线程在周期边界校验 finite、范围和上下限后应用，大幅修改 Ki/Kd 时 reset。当前 `lib/vofa/vofa.c` 的 TX 使用一个 static buffer 交给异步 `uart_tx()`，高频连续发送可能覆盖仍由 DMA 使用的内容；在拿它记录 500Hz PID 前，先按第 13 步增加 TX busy/独立 frame queue 或专用遥测线程。控制线程本身不做无节制打印。

本步自检：PID 算法不 include CAN/device；所有错误路径不更新状态；reset 能捕获当前 measurement；D 使用测量微分；有 anti-windup；每项输出可遥测；目标经过限速/限加速度；一个环一份状态。

---

## 第 8 步：第一次完整写出 Zephyr 定时控制线程

CAN RX callback 是中断上下文，只提交反馈；`k_timer` 的 expiry callback 也不应做浮点控制或 CAN 发送；真正的闭环放在专用线程。最小可靠模式是 timer 每周期只 `k_sem_give()`，控制线程 `k_sem_take()` 后读取快照、计算真实 dt、运行 PID、写完同组所有 raw，再调用一次 `TxGroup::send()`。

真实 dt 应来自单调时间，而不是永远写死 0.002。设备“在线”的 100ms 心跳不能当高速闭环的可控超时；控制层通常只允许 3～5 个反馈周期。反馈时间戳没变化时，不更新积分和 D。CAN send 失败、dt 异常、反馈 stale 或状态不是 Ready，都走同一个 `zero → clearAndSend → pid_reset` 路径。

### 落到代码：完整最小控制线程示例

下面示例展示 Zephyr 原语的完整形状，具体 PID 和安全转换调用替换成第 7 步实现。它不是建议直接让电机转；`velocity_ref` 初始必须为 0，输出安全限幅由第 0 步填写。

```cpp
#include <cmath>
#include <cstdint>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/motor.hpp>
#include <drivers/motor/dji_m3508.hpp>
#include <drivers/motor/dji_tx_group.hpp>
#include <drivers/pid/pid.h>

LOG_MODULE_REGISTER(m3508_speed_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)
#define CAN_NODE DT_NODELABEL(can1)

static constexpr std::int64_t CONTROL_PERIOD_US = 2000;
static constexpr std::uint64_t CONTROL_FEEDBACK_TIMEOUT_MS = 10;
static constexpr float SAFE_RAW_LIMIT = 0.0f; /* 实测前保持 0。 */

K_SEM_DEFINE(control_tick, 0, 1);
static skywalker::motor::dji::TxGroup tx_group;
static pid_data speed_pid{};
static pid_config speed_cfg{};

static void controlTimerExpiry(struct k_timer *)
{
    k_sem_give(&control_tick);
}

K_TIMER_DEFINE(control_timer, controlTimerExpiry, nullptr);

static void resetAndZero(const struct device *motor,
                         float current_measurement)
{
    skywalker::motor::dji::m3508::setCurrentRaw(motor, 0);
    const int ret = tx_group.clearAndSend();
    if (ret < 0) {
        /* 限频统计；物理急停仍必须独立存在。 */
    }
    pid_reset(&speed_pid, current_measurement);
}

static std::int16_t safeFloatToRaw(float value)
{
    if (!std::isfinite(value)) return 0;
    float limited = value;
    if (limited > SAFE_RAW_LIMIT) limited = SAFE_RAW_LIMIT;
    if (limited < -SAFE_RAW_LIMIT) limited = -SAFE_RAW_LIMIT;
    return static_cast<std::int16_t>(std::lround(limited));
}

static void controlThread(void *, void *, void *)
{
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    const struct device *can = DEVICE_DT_GET(CAN_NODE);

    if (!device_is_ready(motor) || !device_is_ready(can)) {
        LOG_ERR("motor or CAN not ready");
        return;
    }

    int ret = tx_group.init(can, 0x200);
    if (ret < 0) return;

    skywalker::motor::dji::GroupCommandSource source{};
    ret = skywalker::motor::dji::m3508::makeGroupCommandSource(motor, source);
    if (ret < 0) return;
    ret = tx_group.bind(source);
    if (ret < 0) return;

    std::int64_t previous_us = k_ticks_to_us_floor64(k_uptime_ticks());
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
            (feedback.valid & skywalker::motor::FeedbackVelocity) != 0;
        const bool fresh =
            feedback.timestamp_ms != 0 &&
            (now_ms - feedback.timestamp_ms) <=
                CONTROL_FEEDBACK_TIMEOUT_MS;

        if (ret < 0 ||
            skywalker::motor::getState(motor) !=
                skywalker::motor::State::Ready ||
            !velocity_valid || !fresh ||
            !(dt > 0.0f) || dt > 0.01f || !std::isfinite(dt)) {
            resetAndZero(motor, feedback.velocity_rad_s);
            last_feedback_stamp = feedback.timestamp_ms;
            continue;
        }

        if (feedback.timestamp_ms == last_feedback_stamp) {
            /* 无新测量：不积 I、不更新 D，只维持安全发送策略。 */
            tx_group.send();
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
            resetAndZero(motor, feedback.velocity_rad_s);
            continue;
        }

        const std::int16_t raw = safeFloatToRaw(result.saturated);
        ret = skywalker::motor::dji::m3508::setCurrentRaw(motor, raw);
        if (ret < 0 || tx_group.send() < 0) {
            resetAndZero(motor, feedback.velocity_rad_s);
            continue;
        }

        last_feedback_stamp = feedback.timestamp_ms;
    }
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

同一 `TxGroup` 的四台电机必须按以下顺序，不能算完一台就 send 一次：

```text
读取四台反馈快照
  → 验证四台 freshness
  → 计算四套控制器
  → 写四个 raw 缓存
  → 调用一次 TxGroup::send
```

本步自检：timer callback 只有 `k_sem_give`；PID 和 `can_send` 只在线程；使用实际 dt；新反馈才更新状态；控制 timeout 短于 driver heartbeat；所有故障走统一零输出；控制线程没有高频 `printk`。

---

## 第 9 步：先完成三条单轴闭环，不写万能关节类

第一条是 M3508 速度 PI：目标速度先经第 7 步斜坡，PI 输出 raw current，再由第 8 步线程统一发送。先 `Ki=Kd=FF=0` 调很小 Kp，方向正确后才加少量 Ki；多数速度环最终是 `PI + kS/kV/kA`，不需要 D。

第二条是 GM6020 舵向串级：连续角目标进入位置 P，输出有限速度目标；速度 PI 输出 A；最后按第 5 步换 raw。位置外环先不加 I，靠内环消除稳态速度误差。软限位外允许向安全方向返回，禁止继续撞向限位；任一必要反馈无效时立即零输出。

第三条是大 Yaw 和小 Yaw 各自的单轴闭环。此处只证明两轴分别稳定，完全不做协调。若也是 GM6020，可以复用同一种纯 `JointController` 数据结构，但每个关节必须有独立参数、状态、零点、方向和限位。

### 落到代码

```text
M3508：
requested velocity
  → acceleration limiter
  → speed PI + kS/kV/kA
  → safe raw clamp
  → setCurrentRaw
  → TxGroup once

GM6020 / Yaw：
continuous position target
  → soft-limit target
  → position P/PD
  → limited velocity target
  → velocity PI + current feedforward
  → safe current A
  → A-to-raw adapter
  → TxGroup once
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

测试仍沿用第 3 步：纯前进四轮角 0；纯左移四轮角 `π/2`；纯逆时针旋转得到各模块切向向量；当前 0°、目标 100°时选择反驱和 80°转向；连续角 `4π+0.1` 不跳回 0.1；零速保持；一轮超速四轮同比缩放；逆解送正解还原原命令；NaN、零轮径和非法 sign 返回错误并输出零速度。

本步自检：测试不 include Zephyr CAN/device；所有单位为 SI；最短转向输出连续角；反转和对齐缩放可分别观察；正逆解共享同一模块顺序和几何。

---

## 第 11 步：先接一只舵轮，再接四只和底盘状态机

一只模块只连接一台 M3508 和一台 GM6020。先让舵向以很小电流验证方向，再按第 9 步闭合速度内环和位置外环；舵角误差很大时驱动轮保持零，逐渐对齐后才按 `alignment_scale` 放开极小速度。最短转向选择反驱时，M3508 目标必须反号，但 GM6020 仍只走较短角度。

单模块通过后，逐个标定四个零点和安装符号，再依次测试 `+vx`、`+vy`、`+wz`、组合命令。不要四模块同时通电后再靠“哪个方向不对就改负号”调车。同一 CAN 分组每拍只发送一次；任一关键舵向失联，第一版整车停，不急着实现三轮降级。

### 落到代码

```text
单模块一拍：
读取 drive + steer 新鲜反馈
  → 运行第 10 步 inverse 得到 module target
  → steer position P 产生 steer velocity target
  → steer velocity PI 产生 GM6020 current A
  → drive target 乘 alignment_scale
  → drive speed PI 产生 M3508 current raw
  → 两路安全限幅
  → 写缓存并统一 send
```

底盘状态机建议保持保守：

```text
Boot
  → 所有 device ready
WaitFeedback
  → 八台电机反馈 fresh
Aligning
  → 四舵对齐，驱动电流保持 0
Ready
  → 收到新的 enable token
Active
  ├─ command stale → Stopping → Ready
  ├─ 任一关键反馈失效 → Fault
  └─ send 失败/严重错误 → Fault

Fault
  → clearAndSend 所有分组
  → reset 所有 PID
  → 只允许人工定义的恢复握手
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
  → 写四 M3508 + 四 GM6020 raw
  → 0x200 组 send 一次，0x1FE 组 send 一次
  → 正运动学并发布状态快照
```

总线负载必须实测。八台电机约 1kHz 反馈接近一条 1Mbps Classic CAN 的极限，推荐四 M3508 与四 GM6020 分两条 CAN，板间通信用第三条 CAN/CAN-FD 或独立 UART；不要未经测量把板间帧塞入已经接近满载的电机总线。

本步自检：单模块未对齐不驱动；四个方向只来自配置；纯三轴运动符合第 10 步向量；每组每拍一次发送；任一关键反馈 stale 后整组清零；共同轮速缩放不改变方向；状态机能从拔线进入 Fault。

---

## 第 12 步：把一个空 application 拆成两个固件，再写纯板间协议

底盘板拥有四个舵轮、大 Yaw 本地闭环和底盘安全状态；云台板拥有小 Yaw、上层 IMU、视觉/遥控目标和唯一 Yaw 协调器。跨板只传目标、测量摘要、有效位和故障，不把高速电机闭环搬到另一块板，也不把一个 `application/main.c` 发展成能猜自己身份的万能程序。

第一版分别构建 `application/chassis` 和 `application/gimbal`，先不引入 sysbuild。两块板各自有 `CMakeLists.txt`、`prj.conf`、overlay 和 `main.cpp`。其中 Zephyr 配置接线仍见第 1 步。

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

`enable_token` 防止重启后旧命令恢复运动：上层每次 enable 生成新 token；底盘只接受当前会话；任一板重启都必须重新握手。纯测试沿用第 3 步，覆盖一次一帧、一次一字节、帧头跨块、两帧粘包、噪声、坏 CRC 后接好帧、超长 payload、sequence 重复/倒退/回绕、NaN/Inf 拒绝以及没有新字节时 freshness 自己变 stale。

超时先用可配置起点做拔线试验，而不是永久写死：命令 age 超过约 30ms 开始斜坡归零，超过约 100ms 清电流并 reset；大 Yaw 命令同理；任一高速电机反馈超过约 3～5 个控制周期立即禁止对应模块；板间状态超过约 100ms 退出协同。实际阈值必须大于正常周期与最坏抖动，又明显短于第 2 步的 100ms device heartbeat。

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

测试沿用第 3 步，先做 byte codec 与官方 CRC 黄金帧，再喂 parser 的拆包、粘包、噪声、坏 CRC 和假长度，最后对每个 decoder 验证正确长度、少一字节、多一字节和旧赛季长度。第一版优先 `0x0201/0x0202/0x0001`，不要一次录完整本协议。

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
    float total_current_budget = 0.0f;
    bool using_fallback = true;
    bool hard_disabled = true;
};
```

```text
RefereePowerInput snapshot
  + super-cap feedback
  + bus voltage/current
  + requested four-wheel effort
  → ChassisPowerPolicy
  → common_scale in [0,1]
  → 四轮 effort 同比缩放
  → 每台型号硬限幅
  → TxGroup once
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
  → 电机 current raw

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

建议一次提交只做一个可回滚目标：修 API dispatch、泛化 DJI codec、泛化 TxGroup、M2006 RX-only、GM6020 zero-output、PID reset/dt validation、swerve inverse tests、board-link parser tests。不要在一个提交里同时改电机协议、PID、设备树和应用状态机。

### 建议由你执行的构建与测试命令

本文没有执行这些命令，因为它们会生成 workspace artifact。目录创建后按实际 Zephyr revision 调整：

```bash
# 现有 M3508
west build -b dm_mc02/stm32h723xx \
  samples/motor/dji_m3508_driver \
  -d build/dji_m3508_driver -p always \
  -- -DBOARD_ROOT=$PWD

# 两个固件
west build -b dm_mc02/stm32h723xx \
  application/chassis -d build/chassis -p always \
  -- -DBOARD_ROOT=$PWD

west build -b dm_mc02/stm32h723xx \
  application/gimbal -d build/gimbal -p always \
  -- -DBOARD_ROOT=$PWD

# 纯测试
west twister -T tests/motor -p native_sim -v
west twister -T tests/pid -p native_sim -v
west twister -T tests/control_math -p native_sim -v
west twister -T tests/swerve_kinematics -p native_sim -v
west twister -T tests/board_link -p native_sim -v
west twister -T tests/yaw_coordinator -p native_sim -v
west twister -T tests/referee -p native_sim -v
```

### 硬件上电总顺序

```text
1. 驱动轮架空、舵向/Yaw/发射机构卸载或可靠限位
2. 物理断电触手可及，安排独立观察者
3. 只收反馈，不发非零命令
4. 核对每个 CAN ID、slot、UART 波特率与引脚
5. 周期发送全零并抓包
6. 单电机极小命令逐一确认方向
7. 单速度环
8. 单关节串级与软限位
9. 单舵轮对齐保护
10. 四舵轮纯 +vx，再 +vy，再 +wz
11. 双板命令在 Disabled 下观察
12. 大、小 Yaw Independent
13. FollowAndRecenter
14. 裁判 RX-only
15. 功率策略先只记录缩放，不影响电机
16. UI
17. 发射机构无弹闭环
18. 具备挡弹与监督后才实弹单发
```

出现反馈时间戳停止、电流持续饱和、舵向误差越控越大、机构逼近硬限位、CAN error 快速上升、sequence 停止/倒退、Yaw 回中反而扩大偏差或实测弹速超阈值，立即清零并物理断电，不继续靠调参试错。

### 按现象查问题

| 现象 | 先查什么 | 不要先做什么 |
|---|---|---|
| device not ready | Kconfig、CMake、binding、overlay status、phandle，见第 1 步 | 改 CAN 字节协议 |
| device ready 但 timestamp 为 0 | feedback ID、CAN 接线、电调 ID、filter | 改 PID |
| raw 0 仍有输出 | 抓真实分组帧、确认持续发送、slot 与 command ID | 只清软件缓存后停止发送 |
| 8191→0 位置跳变 | 第 4 步回绕累计 | 在 PID 中补一圈 |
| M2006 温度总是 0 | C610 该字段为空，应取消 valid | 伪造 0℃有效值 |
| GM6020 比例/ID 错 | 是否误用电压模式 `0x1FF/±30000` | 继续增大 raw |
| DM 全部比例错 | 调试助手 P/V/T_MAX 与 Master ID | 先改移位顺序 |
| 第二种电机接入就乱转 | 发送/反馈 ID 冲突表，立即断电 | 靠换 command 数值排查 |
| setpoint 阶跃有 D 尖峰 | 第 7 步测量微分、滤波与 reset | 盲目减所有增益 |
| 舵轮停车突然回零 | 第 10 步零速保持 | 用 `atan2(0,0)` |
| 四轮限速后运动方向改变 | 是否分别 clamp，改为共同缩放 | 单独放宽最大轮速 |
| UART 有字节但 CRC 全错 | baud、offset、CRC 参数、DMA buffer 所有权 | 改业务 decoder |
| UART 粘包偶尔失败 | 第 12/13 步半帧保留与逐字节重同步 | 假设一次 callback 一帧 |
| UI 偶发花屏 | async TX buffer、稳定 ID、共享频率 | 每周期 DeleteAll |
| 裁判数值错位 | 当前赛季长度/offset、packed struct、端序 | 用旧结构强转 |
| 断线仍继续运动 | freshness、enable token、统一零输出状态 | 只看驱动 100ms heartbeat |

### 最终进度板

- [ ] 第 0 步：硬件参数、ID 冲突和安全表完整。
- [ ] 第 1 步：能解释 Zephyr 驱动六段接线。
- [ ] 第 2 步：M3508 RX、全零、极小方向、Offline、双槽回归。
- [ ] 第 3 步：DJI codec/TxGroup 通用化和纯测试。
- [ ] 第 4 步：M2006 RX、累计位置、全零、小命令。
- [ ] 第 5 步：GM6020 电流模式、ID、A/raw、全零和限位。
- [ ] 第 6 步：DM-J4310 MIT；不用时明确暂缓。
- [ ] 第 7 步：PID、角度、滤波、轨迹纯测试。
- [ ] 第 8 步：控制线程、真实 dt、新反馈门控、统一停机。
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

- 现有 M3508 是否已在当前源码、当前硬件和当前工具链完整回归。
- 第一台整车是否实际使用 DM-J4310。
- 四舵轮几何、轮径、零点、方向和真实允许电流。
- 大、小 Yaw 型号、传动比、同轴程度、IMU 层级和限位。
- 双板最终使用 UART、Classic CAN 还是 CAN-FD，以及完整总线负载。
- 参赛赛项、兵种、弹丸和比赛前最终官方协议/规则版本。
- 裁判常规/图传 UART 在 DM-MC02 上的实际节点与引脚。
- 是否已经从 `sp_middleware` 维护者取得明确源码复用授权。

### 固定资料入口

电机协议优先使用仓库 `motor_docs/` 中的 C610、C620、M2006、M3508、GM6020 和 DM-J4310 固定版手册，并保留 `DOWNLOAD_LINKS.txt` 的来源记录。Zephyr 写法以当前工作区固定提交的头文件、官方 Device Model、CAN Controller 和 Async UART 文档为准。裁判系统每次实施前从 RoboMaster 官方资料中心重新下载当赛季通信协议与规则，不以本文示例偏移替代赛前核对。

第三方参考仓库固定审计入口是 `https://github.com/TongjiSuperPower/sp_middleware`。在许可证状态改变前，只把它当设计参考；若以后得到授权，保存许可证与书面记录、固定 commit、加入第三方清单，并先在隔离分支验证 C++14、Zephyr 平台边界和测试，再决定是否继续保留独立实现。
