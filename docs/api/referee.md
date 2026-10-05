# 裁判许可与功率预算：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

校验版本化裁判帧，分别记录机构输出许可、功率限额和缓冲能量的有效期。

**接入状态：已有代码。** 当前实现 profile 为 Rm2026V1_3，识别 0x0201/0x0202；sentry_gimbal 的板级配置仍是 Unspecified，需要明确配置后才有有效许可。

## 职责与关联

RefereeParser 负责 CRC 与已支持命令，RefereeService 计算在线状态，RefereeReceiver 用 poll 推进 UART。裁判许可约束运动命令，不能与遥控或视觉竞争控制来源。

输入 / 依赖：[异步 UART 与 DMA](uart.md)

消费者：[来源适配与手动映射](command-sources.md)、[摩擦轮与拨盘发射执行](shooter.md)

## 接口契约

### 1. RefereeReceiver(const device *device, AsyncUart::DmaBuffers &dma, RefereeVersion version, std::uint32_t timeout_ms = 500)

```cpp
RefereeReceiver(const device *device, AsyncUart::DmaBuffers &dma, RefereeVersion version, std::uint32_t timeout_ms = 500)
```

绑定裁判 UART、DMA 与明确版本。构造不做 I/O，不按 payload 长度猜版本。

| 参数 | 含义与边界 |
| --- | --- |
| `device` | 裁判串口设备，独占 |
| `dma` | 静态 __nocache 缓冲 |
| `version` | 当前可用 profile：RefereeVersion::Rm2026V1_3；Unspecified 不能获得有效许可 |
| `timeout_ms` | 接收状态离线阈值，默认 500 ms |

**返回 / 输出：** 轮询式接收器；没有独立 start()。

**线程 / 时序：** 对象和缓冲持续存活至重启；一个线程拥有 poll/error。

**错误 / 边界：** 实际驱动与解析错误由 poll 后 error() 报告。

### 2. robotics::RefereeState RefereeReceiver::poll(std::uint64_t now_ms); int RefereeReceiver::error() const

```cpp
robotics::RefereeState RefereeReceiver::poll(std::uint64_t now_ms); int RefereeReceiver::error() const
```

懒初始化、持续恢复 UART、最多读取 8 个块、丢弃断流半帧，并返回按当前时刻判断的裁判值副本。

| 参数 | 含义与边界 |
| --- | --- |
| `now_ms` | 当前本机单调时间 ms |

**返回 / 输出：** poll 返回 RefereeState，可能 online=false 或 stamp 无效；error 返回最近 UART/解析错误。

**线程 / 时序：** 唯一接收线程；若由 RefereePermissionSource 驱动，则不要再在应用中直接 poll 同一对象。

**错误 / 边界：** 初始化失败约 100 ms 后重试；-EOVERFLOW 丢弃半帧；error=0 仅表示 I/O 没有当前错误，不能替代许可有效性检查。

### 3. int RefereeParser::reset(); int RefereeParser::consume(const std::uint8_t *bytes, std::size_t size, std::uint64_t timestamp_ms)

```cpp
int RefereeParser::reset(); int RefereeParser::consume(const std::uint8_t *bytes, std::size_t size, std::uint64_t timestamp_ms)
```

纯解析入口：reset 检查版本并清状态；consume 校验帧头、CRC8、长度与 CRC16，接受支持命令并记录统计。

| 参数 | 含义与边界 |
| --- | --- |
| `bytes / size` | 字节流；零长度用于推进半帧超时 |
| `timestamp_ms` | 数据接收的原始 ms 时间 |

**返回 / 输出：** 0 或负 errno；state()/stats() 提供 owner 线程内引用。

**线程 / 时序：** 单线程解析；不得把内部引用跨线程保存。

**错误 / 边界：** 未指定或不支持的版本会返回配置错误；坏 CRC/长度/未知命令由 Stats 记录并跳过，不把坏帧伪装成新的许可。

### 4. int RefereeService::processBytes(const std::uint8_t *bytes, std::size_t size, std::uint64_t now_ms); int RefereeService::snapshot(std::uint64_t now_ms, robotics::RefereeState &out) const

```cpp
int RefereeService::processBytes(const std::uint8_t *bytes, std::size_t size, std::uint64_t now_ms); int RefereeService::snapshot(std::uint64_t now_ms, robotics::RefereeState &out) const
```

自定义输入线程的服务封装；snapshot 根据整体 stamp 判在线，但机器人许可与功率字段各有自己的 stamp。

| 参数 | 含义与边界 |
| --- | --- |
| `bytes / size` | 带原始接收时间的裁判字节 |
| `now_ms` | processBytes 的接收时间，或 snapshot 的当前时间 |
| `out` | 裁判输出副本 |

**返回 / 输出：** snapshot：0 在线；-EAGAIN 尚无合法消息；-ESTALE 过期但仍写出 out。

**线程 / 时序：** 唯一 owner 调用；其他线程读另行发布的副本。

**错误 / 边界：** 调用 discardPartial() 丢弃断流半帧；整体 online 不能证明每个许可或功率字段新鲜。

## 调用示例

### 使用明确版本持续轮询裁判

```cpp
#include <communication/referee/referee_receiver.hpp>
#include "board_config.hpp"
using namespace skywalker;
static communication::AsyncUart::DmaBuffers dma __nocache;
static communication::RefereeReceiver referee(
    board_config::referee_uart, dma, communication::RefereeVersion::Rm2026V1_3);

void refereeTask() {
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const auto state = referee.poll(now);
        const auto &permit = state.robot.gimbal_output;
        const bool gimbal_allowed = permit.valid && permit.enabled &&
            robotics::isFresh(permit.stamp, now, 300);
        // 在短锁内发布 state 副本；正式应用交给命令仲裁。
        (void)gimbal_allowed;
        k_sleep(K_MSEC(1));
    }
}
```

board_config::referee_uart 来自对应样例/应用。正式 CommandManager 会通过 RefereePermissionSource 调用 poll，因此不另启第二个 owner。

## 调用顺序

1. 按实际协议选择明确版本、UART 与 DMA。
2. 唯一线程不断 poll；断线时也继续轮询以推进恢复。
3. 许可进入命令仲裁，power.limit_stamp 与 power.stamp 分别进入底盘预算判断。
4. 每个机构读取自己的许可有效性；禁止用任何新裁判帧刷新所有字段的年龄。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `CONFIG_SKYWALKER_REFEREE / CONFIG_SKYWALKER_UART_TRANSPORT` | 启用纯 parser 与 UART 接收能力；纯 parser 不要求 Receiver。 |
| `RefereeVersion` | 必须显式绑定实际兼容版本；sentry_gimbal 当前 Unspecified 是待配置项。 |
| `timeout_ms 与 permission_timeout_ms` | 接收器默认离线 500 ms；仲裁器默认按机构许可 300 ms 判断，两者职责不同。 |

- 当前 parser 只实现列出的 profile 与两个命令；仓库存在新版协议 PDF 不等于 parser 自动兼容该版本。
- 0x0201 更新功率限额，不刷新实际功率/缓冲能量 stamp。
- 当前 profile 保留功率字段不会被当成实测 chassis_power_w；功率模型需要应用侧标定。
- 裁判缺失、过期或禁止时，require_referee_for_motion=true 的命令链撤销相应运动许可。

## 正文与源码

- [裁判样例](../../samples/communication/referee/README.md)
- [命令与许可](../modules/robotics/command-service.md)
- [通信模块](../modules/communication/communication.md)

- [轮询接收器](../../include/communication/referee/referee_receiver.hpp)
- [流式解析器](../../include/communication/referee/referee_parser.hpp)
- [支持的版本与线格式](../../include/communication/referee/referee_protocol.hpp)
- [许可与功率消息](../../include/robotics/messages/referee.hpp)
- [解析实现](../../lib/communication/referee.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
