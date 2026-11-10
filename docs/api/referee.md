# 裁判许可与功率预算：接口与调用参考

协议适配基线：`dev@233cb534`（2026-11-10 检查）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

校验版本化裁判帧，分别记录机构输出许可、功率限额和缓冲能量的有效期。

**接入状态：已有代码。** 当前提供 `Rm2027V1_0` 和 `Rm2026V1_3` 两个显式 profile，只解析 `0x0201`/`0x0202`；两版都需要匹配实际裁判固件。`sentry_gimbal` 正式应用尚未启用裁判许可接入，已接入裁判的测试样例默认选择 2027。

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
| `version` | `RefereeVersion::Rm2027V1_0`（2027 V1.0.0）或 `Rm2026V1_3`（旧协议）；Unspecified 不发布许可 |
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

纯解析入口：reset 清除状态与统计；consume 校验帧头、CRC8、长度与 CRC16，按构造时指定的版本解析受支持命令并记录统计。

| 参数 | 含义与边界 |
| --- | --- |
| `bytes / size` | 字节流；零长度用于推进半帧超时 |
| `timestamp_ms` | 数据接收的原始 ms 时间 |

**返回 / 输出：** 0 或负 errno；state()/stats() 提供 owner 线程内引用。

**线程 / 时序：** 单线程解析；不得把内部引用跨线程保存。

**错误 / 边界：** `Unspecified` 不解码业务帧（只记 `unknown_commands`）；坏 CRC、长度不符或未知命令由 Stats 记录并跳过，不把坏帧伪装成新的许可。

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

## 2027 V1.0.0 与 2026 V1.3.0 的已适配差异

按《RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0（2026-10-29）》第 3、5、13～14 页：

| 命令 | RM2026 V1.3.0 | RM2027 V1.0.0 | 适配策略 |
| --- | --- | --- | --- |
| 帧封装 | A5 + 5 字节头 / 2 字节命令 / CRC16 | 不变 | 复用流式扫描与 CRC8/CRC16 |
| 0x0201 | 13 字节，偏移 12 为输出许可 | 17 字节，偏移 12～15 是 float 初速度上限，偏移 16 是输出许可 | 按显式 profile 检查精确长度及各字段偏移 |
| 0x0202 | 14 字节，偏移 8～9 为缓冲能量 | 不变 | 保留 0～7 字节不产生“实测功率” |

特别注意：2027 `0x0201` 的偏移 12 是浮点数**第一个字节**，不能当输出许可掩码读取，否则可能误允许底盘、云台或发射机构。本模块在两个方向均拒绝不匹配的报文长度。

`robot.bullet_speed_limit_stamp.valid` 仅在收到 2027 `0x0201` 后才为真；2026 不提供该字段。接收任意其他命令也不会刷新已有许可的时间戳。协议版本升级后应首先通过

```sh
west build -b native_sim/native/64 tests/protocols -d build/test-protocols
```

运行回归测试，再在 UART 样例上检查输出许可和超时失效。

## 调用示例

### 使用明确版本持续轮询裁判

```cpp
#include <communication/referee/referee_receiver.hpp>
#include "board_config.hpp"
using namespace skywalker;
static communication::AsyncUart::DmaBuffers dma __nocache;
static communication::RefereeReceiver referee(
    board_config::referee_uart, dma, communication::RefereeVersion::Rm2027V1_0);

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
| `RefereeVersion` | 必须显式绑定实际兼容版本；示例默认 RM2027，真实主应用应先确认电气接线与裁判固件版本。 |
| `timeout_ms 与 permission_timeout_ms` | 接收器默认离线 500 ms；仲裁器默认按机构许可 300 ms 判断，两者职责不同。 |

- 当前 parser 仅覆盖两个命令（机器人状态、缓冲能量），不代表已实现 2027 全量协议，如比赛状态、哨兵决策、小地图、UI 交互和自定义终端 MQTT。
- `0x0201` 更新功率限额与三个输出许可，不刷新实际功率/缓冲能量 stamp；2027 版本额外更新 `robot.bullet_speed_limit_m_s` 和专属 `bullet_speed_limit_stamp`。
- 两个 profile 的 `0x0202` 前 8 字节为保留字段，不会被当成实测 `chassis_power_w`；功率模型需要应用侧标定。
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
