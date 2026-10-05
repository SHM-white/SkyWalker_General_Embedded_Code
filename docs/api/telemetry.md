# VOFA、日志与一致快照：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

从同一帧快照解释来源、目标、许可、反馈和错误；JustFloat 有界排队，主机离线不阻塞控制。

**接入状态：已有代码。** 单舵轮 16 通道，M2006 速度环 USB CDC 12 通道与反馈/CAN 前后快照诊断；详细通道见各样例 README。

## 职责与关联

VOFA 有界 JustFloat/调参行；Latest 与 SnapshotCache 交换值副本，应用观察线程不参与目标授权。

输入 / 依赖：直接对接底层设备或算法。

消费者：由应用或样例直接装配。

## 接口契约

### 1. int vofa_init(Vofa *vofa, const struct device *uart)

```cpp
int vofa_init(Vofa *vofa, const struct device *uart)
```

一次性绑定支持 IRQ/FIFO API 的独占 USART 或 USB CDC ACM；不等待主机连接。

| 参数 | 含义与边界 |
| --- | --- |
| `vofa` | 零初始化、静态或等同生命周期的 Vofa 实例 |
| `uart` | 就绪设备，不能与 console/shell/AsyncUart 分享回调 |

**返回 / 输出：** 0：初始化成功；负 errno：失败，失败后不能 send。

**线程 / 时序：** 线程中调用一次；初始化后不能复制、移动或重新初始化；USB 栈由应用/Zephyr 启动。

**错误 / 边界：** -EINVAL：参数非法；-ENODEV：设备未就绪；或 UART 回调注册错误。

### 2. int vofa_send(Vofa *vofa, const float *data, uint8_t num)

```cpp
int vofa_send(Vofa *vofa, const float *data, uint8_t num)
```

非阻塞复制完整 JustFloat 帧进入深度 4 的队列；成功后可立即复用 data，多生产者可共享实例。

| 参数 | 含义与边界 |
| --- | --- |
| `vofa` | 成功初始化的实例 |
| `data` | num 个 float 的数组 |
| `num` | 1～16 个通道，VOFA_MAX_FLOATS=16 |

**返回 / 输出：** 0：完整帧已入队，不表示主机接收；负 errno：整帧拒绝。

**线程 / 时序：** 可在线程或普通 ISR 中调用；无等待 USB/DTR。

**错误 / 边界：** -ENOBUFS：队列满，丢本次完整帧；-EINVAL：参数非法；-ENODEV/驱动错误：设备发送不可用。

### 3. int vofa_set_handler(Vofa *vofa, uint8_t *rx_buf, size_t rx_buf_size, vofa_cmd_handler on_cmd)

```cpp
int vofa_set_handler(Vofa *vofa, uint8_t *rx_buf, size_t rx_buf_size, vofa_cmd_handler on_cmd)
```

设置一次接收行缓冲与 key=value handler，自动启用 IRQ 接收，支持 LF/CRLF。

| 参数 | 含义与边界 |
| --- | --- |
| `rx_buf / rx_buf_size` | 持续存活的接收缓冲，至少 2 字节 |
| `on_cmd` | void (*)(const char *key, float val)，key 只在回调期间有效 |

**返回 / 输出：** 0：handler 已设置；负 errno：失败。

**线程 / 时序：** 线程配置一次；handler 在 UART 驱动回调运行，可能 ISR/工作队列，不能阻塞。要执行控制变更时先入队给应用线程。

**错误 / 边界：** -EINVAL/-ENODEV/-EALREADY；超长或含 NUL 的行丢弃至下一换行，不并发配置或热替换 handler。

### 4. int Latest<T>::put(const T &value); int Latest<T>::get(T &value)

```cpp
int Latest<T>::put(const T &value); int Latest<T>::get(T &value)
```

用 K_NO_WAIT mutex 拷贝完整最新值，没有 I/O 和回调；get 非消费且不保证新的序号。

| 参数 | 含义与边界 |
| --- | --- |
| `value` | put 的输入或 get 的读者输出；跨线程传值，不保存内部引用 |

**返回 / 输出：** 0：完整复制；-EAGAIN：锁忙。初始内部值为 T{}。

**线程 / 时序：** 线程之间交换适当大小的快照；mutex 不用于 ISR。

**错误 / 边界：** Latest 本身没有 have_value/时间有效性标志；读者必须检查值里的 stamp.valid/sequence，-EAGAIN 时保留旧值仍要过期。

### 5. int bench::Telemetry::start(const device *vofa_uart); void bench::Telemetry::emit(const skywalker::robotics::CommandSnapshot &frame)

```cpp
int bench::Telemetry::start(const device *vofa_uart); void bench::Telemetry::emit(const skywalker::robotics::CommandSnapshot &frame)
```

命令 sample 私有封装：日志约每 100 ms 输出同帧的模式/来源/年龄/原因，VOFA 可选每 20 ms 输出最终命令与拆分原因位。

| 参数 | 含义与边界 |
| --- | --- |
| `vofa_uart` | 观测专属设备 |
| `frame` | manager.snapshot 得到的同一次发布，主通道使用 decision.command |

**返回 / 输出：** start 返回 VOFA 初始化结果（未编译 VOFA 时为 0）；emit 不返回，记录拒绝帧计数。

**线程 / 时序：** sample 观测线程调用；不是 lib 的通用机器人 telemetry 服务。

**错误 / 边界：** VOFA 背压不会阻塞 manager；telemetry 不应反复重发旧运动命令或在控制线程打印高频长日志。

## 调用示例

### 将同一次发布的最终命令送到 VOFA

```cpp
#include <lib/vofa/vofa.h>
#include <robotics/command/command_manager.hpp>
using namespace skywalker;

int telemetryLoop(const device *telemetry_uart,
                  const robotics::CommandManager &manager) {
    static Vofa vofa{};
    const int init = vofa_init(&vofa, telemetry_uart);
    if (init < 0) return init;
    robotics::CommandSnapshot frame{};
    for (;;) {
        if (manager.snapshot(frame) == 0) {
            const auto &c = frame.decision.command;
            const float values[] = {c.chassis.vx_m_s, c.chassis.vy_m_s,
                                    c.chassis.wz_rad_s, c.gimbal.yaw_rate_rad_s,
                                    float(frame.decision.error)};
            const int ret = vofa_send(&vofa, values, 5);
            // ret==-ENOBUFS 时记录丢观测帧，继续下一周期。
            (void)ret;
        }
        k_sleep(K_MSEC(20));
    }
}
```

本函数仅调用一次，telemetry_uart 与遥控/裁判/视觉/板间设备独立。通道单位和顺序在主机侧对应配置。

## 调用顺序

1. 选独占观测 UART/CDC，静态构造 Vofa 并 vofa_init 一次。
2. 采集一份完整 CommandSnapshot/执行快照；只在观测线程组装通道和低频日志。
3. vofa_send 复制整帧，有背压则丢本次观测，下一周期用新值。
4. 接收调参行时回调只复制请求，应用线程决定何时应用参数，保留控制所有权。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `CONFIG_SKYWALKER_LIB_VOFA` | 依赖 SERIAL 与 SERIAL_SUPPORT_INTERRUPT，选择 UART_INTERRUPT_DRIVEN。 |
| `VOFA_MAX_FLOATS / VOFA_TX_QUEUE_DEPTH` | 16 个 float 通道、4 帧有界队列；不是无限数据缓存。 |
| `CONFIG_COMMAND_MANAGER_VOFA` | 三源 sample 的可选观测开关；串口选择和通道顺序见该 sample 配置。 |
| `日志与观测周期` | 控制/仲裁线程持续运行，观测采用更低频率；当前命令台架 log=100 ms、VOFA=20 ms。 |

- 浮点通道不要直接存完整 32 位 bitmask/大序号；需要精确原因位可像台架一样拆为低/高 16 位。
- data 入队成功不等于主机已显示；串口断开也不能卡住控制循环。
- Latest.get 的 0 不证明已经 publish；值类型要包含有效标志。
- 对比仲裁观测和最终输出时用同一个 CommandSnapshot，避免分别读导致跨帧误判。

## 正文与源码

- [调试指南](../guides/debugging.md)
- [命令观测通道](../../samples/robotics/command_manager/README.md)
- [视觉 VOFA 示例](../../samples/communication/vision/README.md)
- [README.md](../../samples/motor/m2006_speed_control/README.md)
- [README.md](../../samples/robotics/swerve/README.md)

- [vofa.h](../../include/lib/vofa/vofa.h)
- [latest.hpp](../../include/latest.hpp)
- [snapshot_cache.hpp](../../include/robotics/execution/snapshot_cache.hpp)
- [telemetry.cpp](../../samples/robotics/command_manager/src/telemetry.cpp)
- [main.cpp](../../samples/motor/m2006_speed_control/src/main.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
