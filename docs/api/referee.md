# 裁判许可与功率预算：接口与调用参考

协议适配基线：`dev@233cb534`（2026-11-10 检查）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

校验版本化裁判帧，分别记录机构输出许可、功率限额和缓冲能量的有效期。

**接入状态：已有代码。** 通过 Kconfig `choice` 在构建时选择 RM2027 V1.0.0（默认）或 RM2026 V1.3.0，仅编译链接选中版本的解码器。现仅处理 `0x0201`/`0x0202`，配置必须与实际裁判固件相符。`sentry_gimbal` 正式应用尚未启用裁判许可接入，已接入裁判的测试样例默认选择 2027。

## 职责与关联

RefereeParser 只负责共享的流式帧组装、CRC 与编译期固定的解码调用，`Referee2026Parser` / `Referee2027Parser` 各自负责本版本的命令号、长度和字段布局；各版之间不共享命令解码分支。RefereeService 计算在线状态，RefereeReceiver 用 poll 推进 UART。裁判许可约束运动命令，不能与遥控或视觉竞争控制来源。

输入 / 依赖：[异步 UART 与 DMA](uart.md)

消费者：[来源适配与手动映射](command-sources.md)、[摩擦轮与拨盘发射执行](shooter.md)

## 接口契约

### 1. RefereeReceiver(const device *device, AsyncUart::DmaBuffers &dma, std::uint32_t timeout_ms = 500)

```cpp
RefereeReceiver(const device *device, AsyncUart::DmaBuffers &dma, std::uint32_t timeout_ms = 500)
```

绑定裁判 UART 和 DMA；协议版本在构建时通过 Kconfig 确定，构造过程不做 I/O。

| 参数 | 含义与边界 |
| --- | --- |
| `device` | 裁判串口设备，独占 |
| `dma` | 静态 __nocache 缓冲 |
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

纯解析入口：reset 清除状态与统计；consume 只做帧头、CRC8、帧总长与 CRC16 校验，成功后直接调用编译期选中的解析类。两个版本的命令长度检查与字段解码分别位于 `referee_2026.cpp` 和 `referee_2027.cpp`。只有成功解码才发布新状态、更新时间戳和序号。

| 参数 | 含义与边界 |
| --- | --- |
| `bytes / size` | 字节流；零长度用于推进半帧超时 |
| `timestamp_ms` | 数据接收的原始 ms 时间 |

**返回 / 输出：** 0 或负 errno；state()/stats() 提供 owner 线程内引用。

**线程 / 时序：** 单线程解析；不得把内部引用跨线程保存。

**错误 / 边界：** 版本在编译期固定，不再存在 `Unspecified`。坏 CRC、长度不符或未知命令由 Stats 记录并跳过，不把坏帧伪装成新的许可。

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

## 编译期版本选择

通过 `lib/Kconfig` 的 `SKYWALKER_REFEREE_PROTOCOL` 互斥选项选择一个版本：

```conf
# 默认
CONFIG_SKYWALKER_REFEREE_RM2027=y
# 2026 固件单独构建时，改为
# CONFIG_SKYWALKER_REFEREE_RM2026=y
```

配置通过 `lib/communication/CMakeLists.txt` 控制源文件：只编译 `referee_2027.cpp` 或 `referee_2026.cpp`。公共 `RefereeParser` 的解码调用在预处理阶段选择，不保存版本枚举、不在每帧执行 `switch`，`RefereeReceiver` / `RefereeService` 构造函数也无需版本参数。

实机串口样例可通过 `samples/communication/referee/rm2026.conf` 切换旧协议；测试目录为两个版本各提供一套 Twister 测试构建。**不同年份的协议不能在同一固件中运行时切换；改变版本须重编译。**

## 2027 V1.0.0 与 2026 V1.3.0 的已适配差异

按《RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0（2026-10-29）》第 3、5、13～14 页：

| 命令 | RM2026 V1.3.0 | RM2027 V1.0.0 | 适配策略 |
| --- | --- | --- | --- |
| 帧封装 | A5 + 5 字节头 / 2 字节命令 / CRC16 | 不变 | 公共 `RefereeParser` 复用流式扫描与 CRC8/CRC16；解析类由 Kconfig 固定 |
| 0x0201 | 13 字节，偏移 12 为输出许可 | 17 字节，偏移 12～15 是 float 初速度上限，偏移 16 是输出许可 | `Referee2026Parser` / `Referee2027Parser` 各自检查长度并解码 |
| 0x0202 | 14 字节，偏移 8～9 为缓冲能量 | 不变 | 保留 0～7 字节不产生“实测功率” |

特别注意：2027 `0x0201` 的偏移 12 是浮点数**第一个字节**，不能当输出许可掩码读取，否则可能误允许底盘、云台或发射机构。每种编译配置都拒绝不匹配的报文长度。

`robot.bullet_speed_limit_stamp.valid` 仅在收到 2027 `0x0201` 后才为真；2026 不提供该字段。接收任意其他命令也不会刷新已有许可的时间戳。协议版本升级后应首先通过

```sh
west twister -T tests/protocols -p native_sim/native/64
```

运行回归测试，再在 UART 样例上检查输出许可和超时失效。

## 调用示例

### 使用编译期协议配置持续轮询裁判

```cpp
#include <communication/referee/referee_receiver.hpp>
#include "board_config.hpp"
using namespace skywalker;
static communication::AsyncUart::DmaBuffers dma __nocache;
static communication::RefereeReceiver referee(
    board_config::referee_uart, dma);

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

1. 在 Kconfig 中按实际裁判固件选择协议，再配置 UART 与 DMA；修改版本后必须重新构建和刷写。
2. 唯一线程不断 poll；断线时也继续轮询以推进恢复。
3. 许可进入命令仲裁，power.limit_stamp 与 power.stamp 分别进入底盘预算判断。
4. 每个机构读取自己的许可有效性；禁止用任何新裁判帧刷新所有字段的年龄。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `CONFIG_SKYWALKER_REFEREE / CONFIG_SKYWALKER_UART_TRANSPORT` | 启用纯 parser 与 UART 接收能力；纯 parser 不要求 Receiver。 |
| `CONFIG_SKYWALKER_REFEREE_RM2027 / CONFIG_SKYWALKER_REFEREE_RM2026` | Kconfig 互斥选择，默认 2027；CMake 仅链接选中版本，不能在运行时切换。 |
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
- [通用 CRC 与解码结果类型](../../include/communication/referee/referee_protocol.hpp)
- [Kconfig 版本选择](../../lib/Kconfig)
- [许可与功率消息](../../include/robotics/messages/referee.hpp)
- [公共帧组装与分派](../../lib/communication/referee.cpp)
- [2026 解析接口](../../include/communication/referee/referee_2026_parser.hpp) / [实现](../../lib/communication/referee_2026.cpp)
- [2027 解析接口](../../include/communication/referee/referee_2027_parser.hpp) / [实现](../../lib/communication/referee_2027.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
