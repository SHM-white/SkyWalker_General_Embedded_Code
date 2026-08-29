# SkyWalker 裁判系统架构古法实施框架

> 目标：从 `TongjiSuperPower/sp_middleware` 的裁判系统部分吸收合理的模块边界，在当前 Zephyr 项目中由你亲手实现一套可跨赛季维护、能处理串口拆包粘包、能判断数据新鲜度、能安全异步发送并支持 UI 的裁判系统框架。
>
> 本文只给架构、接口、伪代码、实施顺序和验收方法，没有修改任何业务源码。

## 0. 一句话结论

有值得借鉴的架构，但不能照搬 `PM02` 大类和那份 `referee_protocol.hpp`。

真正值得借鉴的是：

1. 用 `cmd_id` 将通用帧分发给不同消息处理器。
2. 对 `0x0301` 机器人交互数据再按 `data_cmd_id` 做二级分发。
3. 每类易过期数据保存 `valid + last_update_ms`，消费者不能只看数值。
4. UI 分为“线、圆、矩形、文字等语义元素”和“协议打包器”。
5. UI 按协议允许的 1/2/5/7 个图形批量组帧。

SkyWalker 应当把它扩展成下面这套结构：

```text
常规链路 UART（115200）              图传链路 UART（921600）
          │                                      │
          ├─ 各自的 RX 双缓冲 / ring buffer ─────┤
          │                                      │
          └────────── RefereeLinkContext × 2 ────┘
                               │
                    通用流式 FrameParser
                  帧头 CRC8 / 长度 / 整帧 CRC16
                               │
                         FrameView
                               │
                  CommandDispatcher 静态描述表
                    ┌──────────┴──────────┐
                    │                     │
             普通 cmd_id 解码      0x0301 二级分发
                    │                     │
                    └──────────┬──────────┘
                               │
                  RefereeStore 类型化数据仓库
                  value/valid/stamp/generation
                               │
             ┌─────────────────┼─────────────────┐
             │                 │                 │
        底盘功率策略       发射机构策略       UI/应用状态

发送方向：

应用/UI/多机通信
      → 语义消息编码
      → 发送权限校验
      → 频率/带宽调度
      → 拥有独立存储的 TxFrame 队列
      → 对应物理链路 UART
```

这套架构最重要的原则是：

> 线上字节格式、赛季业务数据、应用策略和 UART 传输必须是四层，不能重新塞回一个 `PM02` 类。

## 1. 本次核查依据与版本风险

### 1.1 参考仓库

- 仓库：`https://github.com/TongjiSuperPower/sp_middleware`
- 核查提交：`608b09e712fb53dd2cf86d5c8e622fc7e1006bb3`
- 重点目录：
  - `referee/referee_protocol/`
  - `referee/pm02/`
  - `referee/ui/`
  - `referee/vt02/`
  - `referee/vt03/`
  - `tools/crc/`

参考仓库当前没有许可证文件。本文只吸收架构思想和协议事实，不复制其函数体、结构定义或 CRC 查表。

### 1.2 当前官方依据

截至本次核查日期，找到的 2026 官方通信协议是：

- 《RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0（20260327）》
- 官方 PDF：`https://bbs-web-static.robomaster.com/ef4f084944e34393aa70378a4a405c681774586313231/RoboMaster%202026%20%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%E9%AB%98%E6%A0%A1%E7%B3%BB%E5%88%97%E8%B5%9B%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%20V1.3.0%EF%BC%8820260327%EF%BC%89.pdf`

发射机构限速和性能体系还需要同时对照比赛规则。本文此次核查使用：

- 《RoboMaster 2026 机甲大师超级对抗赛比赛规则手册 V1.3.0（20260209）》
- 官方 PDF：`https://bbs-web-static.robomaster.com/c85923a9d793448288b5071f88a289721770635192667/RoboMaster%202026%20%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%E8%B6%85%E7%BA%A7%E5%AF%B9%E6%8A%97%E8%B5%9B%E6%AF%94%E8%B5%9B%E8%A7%84%E5%88%99%E6%89%8B%E5%86%8CV1.3.0%EF%BC%8820260209%EF%BC%89.pdf`

通信协议回答“线上有哪些字段”，比赛规则回答“什么情况下允许多少”。两份文档必须一起看，不能从旧版 C 结构体反推当前规则。

官方协议明确区分：

- 电源管理模块 User 串口的常规链路：115200 baud。
- 图传发送端到机器人的图传链路：921600 baud。
- 雷达无线链路。
- 通用串口帧：5 字节帧头、2 字节 `cmd_id`、n 字节 data、2 字节 CRC16。
- `0x0301` 等上行消息有明确频率和带宽限制。

### 1.3 为什么参考类型头不能直接使用

参考仓库 README 写的是 2025 V1.8.0，但头文件又零散加入了 2026 字段，已经形成混合版本。

下面是几个直接可见的冲突：

| 消息 | 参考仓库 `static_assert` | 2026 V1.3.0 官方数据段长度 |
|---|---:|---:|
| `0x0003` 机器人血量 | 20 | 16 |
| `0x0201` 机器人性能体系 | 17 | 13 |
| `0x0203` 机器人位置 | 12 | 16 |
| `0x0208` 允许发弹量 | 8 | 6 |
| `0x020D` 哨兵自主决策信息 | 14 | 6 |
| `0x0310` 机器人到自定义客户端 | 300 | 300 |

这说明不能通过“某些长度刚好正确”推断整个头文件可靠。

正确做法：

1. 官方协议是唯一主依据。
2. 每次赛季更新都先比较命令描述表。
3. 协议版本改变时替换 decoder/spec 层，不修改 UART parser 和上层业务接口。
4. 比赛前再次到官方资料中心确认是否出现 V1.3.1/V1.4.0 或增补说明。

## 2. 先理解五个术语

### Transport，传输层

只负责从 Zephyr UART 收发字节，不知道 `0x0201` 是什么。

### Frame parser，帧解析器

从连续字节流中找出一个完整的 `0xA5...CRC16` 帧。它知道通用帧格式，但不知道 payload 的业务含义。

### Decoder，消息解码器

根据 `cmd_id` 和赛季规范，把 payload 字节逐字段转成 `RobotStatus` 等本地语义数据。

### Store，数据仓库

保存最新数据、是否有效、何时更新、更新了多少次，为多线程消费者提供一致快照。

### Policy，策略层

根据裁判数据做底盘功率限制、发射许可或 UI 内容。策略不能反过来污染协议解析器。

## 3. 推荐文件布局

下面是你后续亲手创建的建议目录，不是本文已经创建的文件：

```text
include/referee/
├── referee_types.hpp
├── referee_store.hpp
├── referee_service.hpp
├── protocol/
│   ├── byte_codec.hpp
│   ├── crc.hpp
│   ├── frame.hpp
│   ├── frame_parser.hpp
│   ├── command_spec.hpp
│   ├── decoder.hpp
│   └── protocol_2026_v1_3.hpp
├── transport/
│   ├── referee_link.hpp
│   └── referee_uart.hpp
├── tx/
│   ├── frame_encoder.hpp
│   └── tx_scheduler.hpp
├── interaction/
│   ├── interaction.hpp
│   └── interaction_dispatcher.hpp
└── ui/
    ├── ui_types.hpp
    ├── ui_scene.hpp
    ├── ui_diff.hpp
    └── ui_encoder.hpp

lib/referee/
├── CMakeLists.txt
├── Kconfig
├── protocol/
│   ├── byte_codec.cpp
│   ├── crc.cpp
│   ├── frame_parser.cpp
│   ├── command_spec_2026_v1_3.cpp
│   └── decoder_2026_v1_3.cpp
├── transport/
│   └── referee_uart.cpp
├── tx/
│   ├── frame_encoder.cpp
│   └── tx_scheduler.cpp
├── interaction/
│   └── interaction_dispatcher.cpp
├── ui/
│   ├── ui_scene.cpp
│   ├── ui_diff.cpp
│   └── ui_encoder.cpp
├── referee_store.cpp
└── referee_service.cpp

tests/referee/
├── crc/
├── frame_parser/
├── decoder_2026_v1_3/
├── referee_store/
├── tx_scheduler/
└── ui_encoder/
```

第一版不要把它注册成一个“裁判系统 Zephyr sensor driver”。它更像一个协议 service：包含两个 UART 链路、多种消息和发送队列，不符合单一硬件传感器的简单模型。

## 4. 两条链路必须是两个实例

参考仓库用 `PM02`、`VT03` 两个大类区分设备，这个“物理入口分开”的思路是对的，但公共解析逻辑被复制了。

你的实现建议：

```cpp
enum class RefereeLinkKind : uint8_t {
    Regular,
    Video,
};

struct RefereeLinkConfig {
    RefereeLinkKind kind;
    const struct device *uart;
    uint32_t baud_rate;
    uint16_t max_payload_len;
    uint16_t rx_ring_capacity;
};

struct RefereeLinkStats {
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t header_crc_errors;
    uint32_t frame_crc_errors;
    uint32_t length_errors;
    uint32_t dropped_bytes;
    uint32_t sequence_gaps;
    uint32_t unknown_cmd_ids;
    uint32_t rx_overflows;
    uint32_t tx_frames;
    uint32_t tx_busy;
    uint32_t tx_errors;
};
```

每条链路各自拥有：

- UART device。
- 两块异步 RX buffer。
- 一个 ring buffer。
- 一个 `FrameParser`。
- 自己的接收 sequence 统计。
- 自己的发送 sequence。
- 自己的发送队列和 UART busy 状态。
- 自己的统计信息。

不要共用 parser 状态。常规链路的半帧和图传链路的半帧绝不能进入同一个缓存。

## 5. UART 回调只搬字节，不解业务帧

当前项目 [lib/vofa/vofa.c](lib/vofa/vofa.c) 已经展示了 Zephyr `uart_callback_set()`、`UART_RX_RDY` 和 `uart_rx_enable()` 的基本用法，可以参考 API 调用形状。

但裁判系统不要直接复用 VOFA 的回调逻辑，因为：

- 一帧可能跨越多个 `UART_RX_RDY` 事件。
- 一个事件可能含多个帧。
- Zephyr 的 `evt->data.rx.offset` 也必须正确处理。
- 需要处理 `UART_RX_BUF_REQUEST` 并提供第二块 RX buffer。
- 需要在 `UART_RX_STOPPED` 中记录原因并安全重启。
- 解析和更新几十种消息不应阻塞 UART 回调。

建议回调职责：

```text
UART_RX_RDY
  → 读取 buf + offset 指向的新增字节
  → 写入该链路 ring buffer
  → give semaphore / submit work
  → 返回

UART_RX_BUF_REQUEST
  → uart_rx_buf_rsp() 交付另一块空闲 buffer

UART_RX_BUF_RELEASED
  → 标记对应 buffer 可复用

UART_RX_DISABLED
  → 在安全上下文重新 uart_rx_enable()

UART_RX_STOPPED
  → 记录错误原因、清 parser 半帧、安排重启

UART_TX_DONE / UART_TX_ABORTED
  → 释放当前 TxFrame 所有权、唤醒发送调度器
```

线程/中断规则：

- UART callback 不调用业务 decoder。
- UART callback 不拿会睡眠的 `k_mutex`。
- UART callback 不调用 UI 更新和功率策略。
- ring buffer 满时宁可计数并丢弃，再让 parser 重新同步；不能覆盖未处理字节而不记录。
- `RefereeService` worker 是 parser 和 store 的唯一写者。

## 6. 通用流式帧解析器

### 6.1 接口草案

```cpp
struct FrameView {
    RefereeLinkKind link;
    uint8_t sequence;
    uint16_t cmd_id;
    const uint8_t *payload;
    uint16_t payload_len;
};

enum class ParserEvent : uint8_t {
    NeedMoreData,
    FrameReady,
    DroppedNoise,
    HeaderCrcError,
    FrameCrcError,
    LengthError,
};

struct FrameParserConfig {
    uint8_t sof;
    uint16_t max_payload_len;
};

struct FrameParser;

using FrameHandler = void (*)(const FrameView &frame, void *user);

int frame_parser_push(
    FrameParser *parser,
    RefereeLinkKind link,
    const uint8_t *bytes,
    size_t len,
    FrameHandler handler,
    void *user);

void frame_parser_reset(FrameParser *parser);
```

`FrameView.payload` 只在 callback 返回前有效。decoder 必须在回调内完成逐字段复制，不能把裸指针保存到仓库。

### 6.2 状态机

```text
SeekSof
  ├─ 当前字节不是 0xA5：丢一个字节，继续找
  └─ 找到 0xA5：进入 ReadHeader

ReadHeader
  ├─ 不足 5 字节：等待更多数据
  ├─ CRC8 错：只丢当前 SOF，回到 SeekSof
  ├─ payload_len 超上限：只丢当前 SOF，回到 SeekSof
  └─ 合法：计算 total_len，进入 ReadFrame

ReadFrame
  ├─ 字节不足：等待更多数据
  ├─ CRC16 错：只丢当前 SOF，回到 SeekSof
  └─ 合法：产生 FrameView，消费整帧，回到 SeekSof
```

参考 `PM02::update()` 只会在 buffer 第一个字节正好是 SOF 时继续；遇到前导噪声、半帧或坏 CRC 会直接 return。它能递归解析粘在一起的完整帧，但不是真正的流解析器：

- 半个尾帧不会被保留。
- 下一次 callback 的数据不能和上一次半帧拼接。
- 前面多一个噪声字节会丢掉后面所有正确帧。
- CRC 错帧之后的正确帧不会自动重新同步。

你的 parser 必须覆盖这些情况，建议用迭代循环，不使用递归。

### 6.3 sequence 怎么用

- sequence 只用于统计丢帧、重复和乱序。
- sequence 回绕 `255 → 0` 是正常行为。
- 某一帧 sequence 跳号不能让 parser 拒绝后面的有效帧。
- 常规链路和图传链路各自统计，不能混用。
- 应用不得把 sequence 当安全认证；它只有 8 位，不防伪造。

## 7. 赛季命令描述表：架构中最关键的隔离层

不要把所有长度散落在几十个 switch case 中。为当前赛季建立一张静态表：

```cpp
enum class MessageLifecycle : uint8_t {
    PeriodicState,
    LatchedState,
    Event,
    VariablePayload,
};

struct CommandSpec {
    uint16_t cmd_id;
    RefereeLinkKind link;
    uint16_t min_payload_len;
    uint16_t max_payload_len;
    MessageLifecycle lifecycle;
    uint32_t expected_period_ms;
    uint32_t stale_after_ms;
    int (*decode)(
        const uint8_t *payload,
        uint16_t payload_len,
        uint32_t stamp_ms,
        RefereeStore *store);
};

const CommandSpec *find_command_spec(
    RefereeLinkKind link,
    uint16_t cmd_id);
```

为什么同时需要 `min` 和 `max`：

- 固定长度消息令二者相等。
- 自定义或交互 payload 允许变长。
- 未支持的新协议长度必须明确拒绝，不能 memcpy 到旧结构。

为什么 `link` 也是匹配条件：

- 同样的通用帧格式可能出现在不同物理链路。
- 每个命令官方规定了允许的发送方、接收方和链路。
- 收到“不该出现在这条链路”的 cmd 应计数并拒绝。

赛季升级时主要修改：

```text
protocol_2026_v1_3.hpp
command_spec_2026_v1_3.cpp
decoder_2026_v1_3.cpp
对应黄金向量测试
```

UART transport、FrameParser、RefereeStore 和应用策略不应因为一个字段长度变化而重写。

## 8. Decoder 必须逐字段读取，不能 memcpy packed struct

### 8.1 字节工具

```cpp
uint16_t read_u16_le(const uint8_t *p);
uint32_t read_u32_le(const uint8_t *p);
uint64_t read_u64_le(const uint8_t *p);
float read_f32_le(const uint8_t *p);

void write_u16_le(uint8_t *p, uint16_t value);
void write_u32_le(uint8_t *p, uint32_t value);
int write_f32_le(uint8_t *p, float value);
```

`read_f32_le` 要通过 `uint32_t` 位模式和 `memcpy` 得到 float，不能解引用未对齐的 `float *`。解出后检查 `std::isfinite()`。

### 8.2 位域工具

不要用编译器 C/C++ bit-field 直接映射线上字节。正确做法：

```text
raw = payload[0]
low_4_bits  = raw & 0x0F
high_4_bits = (raw >> 4) & 0x0F
```

例如 `0x0206` 伤害数据：

```text
armor_or_module_id = payload[0] & 0x0F
reason             = (payload[0] >> 4) & 0x0F
```

这比 packed bit-field 更啰嗦一点，但字节序、编译器行为和测试结果都明确。

### 8.3 decoder 返回值和更新规则

```cpp
enum RefereeDecodeError {
    DecodeOk = 0,
    DecodeNull = -1,
    DecodeWrongLength = -2,
    DecodeInvalidEnum = -3,
    DecodeInvalidFloat = -4,
    DecodeInvalidId = -5,
    DecodeReservedBits = -6,
};
```

行为要求：

- 先完整验证临时局部变量。
- 全部合法后一次性提交到 store。
- 解码失败不得覆盖上一份有效数据。
- 失败必须增加对应 cmd 的错误计数。
- 未知 enum 值可以保留 raw 并标 `recognized=false`，不能随便映射成某个正常状态。
- 保留位异常是否拒绝由 spec 决定，但必须能统计。

参考仓库的 `copy_fixed()` 有“长度必须精确相等”这一点值得保留；它后面的整结构 `memcpy` 则不应保留。

## 9. RefereeStore：每类数据都要知道“什么时候来的”

### 9.1 通用 Sample

```cpp
template <typename T>
struct RefereeSample {
    T value{};
    uint32_t stamp_ms = 0;
    uint32_t generation = 0;
    uint8_t source_sequence = 0;
    bool valid = false;
};
```

`generation` 每成功更新一次加一。消费者可判断“这是上一次读过的旧事件，还是新事件”。

### 9.2 三类生命周期不能混

#### 周期状态

例如按 10Hz 或 1Hz 周期发送的状态。需要 `is_fresh(now)`：

```text
valid && unsigned(now_ms - stamp_ms) <= stale_after_ms
```

超时后不一定要抹掉 value，但 `fresh=false`，消费者不能再把它当实时控制依据。

#### 锁存状态

例如比赛结果。成功收到后保持，直到下一次比赛 session/reset，不因几秒没重发就归零。

#### 事件

例如伤害和发射事件。不能只存“最后一次值”让应用轮询，因为快速连续两次事件可能被覆盖。

第一版至少使用 `generation`；更稳妥的是给事件建小型队列：

```cpp
struct RefereeEvent {
    uint16_t cmd_id;
    uint32_t stamp_ms;
    uint32_t generation;
    // 已解码的事件 union/variant
};
```

### 9.3 不要用“999”伪装未知

参考 `PM02` 会把部分雷达未知数据替换为 999/9999。这便于临时显示，但容易让业务误把 999 当真实值。

正确表示：

```text
value = 上一次合法值或零初始化值
valid = false
fresh = false
```

如果 UI 想显示 `---` 或 `999`，那是 UI 层的表现策略，不是 store 中的协议事实。

### 9.4 一致快照与线程语义

参考仓库把所有消息公开为可变成员，UART 回调更新时，任务可能读到一半新、一半旧的数据。

第一版使用最容易写对的方案：

```cpp
int referee_store_read_snapshot(
    RefereeStore *store,
    RefereeSnapshot *out);
```

行为：

1. worker 解码出局部消息。
2. worker 获取 `k_mutex`。
3. 更新 store 中对应 sample。
4. 释放 mutex。
5. 消费者获取同一个 mutex，将所需字段复制到自己的 snapshot。
6. 消费者释放 mutex，在锁外做控制计算。

UART callback 不使用这个 mutex，因为 callback 不解码。

后续证明 mutex 成为瓶颈后，再考虑双缓冲或 sequence lock。第一版不要自己发明无锁并发。

## 10. `0x0301` 二级分发值得完整保留

参考仓库先按外层 `cmd_id=0x0301`，再解析：

```text
data_cmd_id
sender_id
receiver_id
user_data
```

这是正确架构，应当独立成为 `InteractionDispatcher`：

```cpp
struct InteractionView {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    const uint8_t *payload;
    uint16_t payload_len;
};

struct InteractionSpec {
    uint16_t data_cmd_id;
    uint16_t min_payload_len;
    uint16_t max_payload_len;
    int (*decode)(const InteractionView &, uint32_t stamp_ms, RefereeStore *);
};
```

二级分发前依次验证：

1. 外层 data 至少有 6 字节交互头。
2. `sender_id` 是当前赛季合法 ID。
3. `receiver_id` 符合该子命令允许范围。
4. 如果消息声称发给本机，本机 robot ID 必须已经从一份新鲜的 `0x0201` 获得。
5. `data_cmd_id` 是官方开放或团队自定义允许区间。
6. payload 长度匹配该子命令。
7. 未知子命令只计数并跳过，不破坏后续帧。

对于团队自定义的 `0x0200~0x02FF`，建议 payload 内再放自己的小版本号和 sequence，不要只依赖外层 8 位 sequence。

## 11. 发送架构：编码、权限、调度、UART 四层

### 11.1 编码器不拥有静态发送 buffer

```cpp
constexpr size_t REFEREE_MAX_FRAME_BYTES = 320;

struct EncodedFrame {
    RefereeLinkKind link;
    uint16_t cmd_id;
    uint16_t length;
    uint8_t bytes[REFEREE_MAX_FRAME_BYTES];
};

int referee_encode_frame(
    RefereeLinkKind link,
    uint8_t sequence,
    uint16_t cmd_id,
    const uint8_t *payload,
    uint16_t payload_len,
    EncodedFrame *out);
```

`EncodedFrame` 自己拥有字节，直到 UART 发完。

参考 `UI_Manager` 和 `VT03::send_custom_client_data()` 都会复用内部/static buffer；异步 DMA 尚未完成时再次 pack/send，会改写正在发送的内存。这一做法不能照搬。

### 11.2 发送权限校验

在编码前检查：

- cmd 是否允许从机器人发出。
- 是否允许走指定链路。
- sender ID 是否等于本机新鲜 robot ID。
- receiver ID 是否在当前赛项允许范围。
- payload 长度是否符合官方限制。
- 浮点内容是否有限。

若 robot ID 尚未有效，不发送 UI 和多机通信；不能像参考 UI 那样把未知 ID 默认映射成裁判服务器 ID。

### 11.3 调度器接口

```cpp
enum class TxPriority : uint8_t {
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
    uint32_t earliest_send_ms;
    uint32_t expiry_ms;
};

int referee_tx_enqueue(
    RefereeTxScheduler *scheduler,
    const TxRequest &request);

void referee_tx_process(
    RefereeTxScheduler *scheduler,
    uint32_t now_ms);
```

调度器负责：

- 每条物理链路同一时刻只有一个 UART TX。
- 每个 cmd 的频率上限。
- `0x0301` 下所有子命令共享的总频率上限，而不是每种 UI 各 30Hz。
- 官方对机器人接收字节数的限制。
- 过期帧丢弃，不在堵塞恢复后补发一串旧控制目标。
- 高优先级控制消息优先于动态 UI。
- `UART_TX_DONE` 后才释放/复用那块 `EncodedFrame`。
- sequence 在真正接受发送时递增，不在入队失败时消耗。

当前官方 V1.3.0 说明 `0x0301` 上行总频率最大为 30Hz；哨兵/雷达和其他兵种的接收字节上限也不同。具体额度写在赛季 profile 中，不写死在通用 scheduler。

## 12. UI：参考仓库最值得借，但要再加“场景和差分”两层

### 12.1 值得保留的设计

- Line/Rectangle/Circle/Ellipse/Arc/Float/Integer/String 作为语义类型。
- 元素设置 layer、color、width、位置等属性。
- 打包器根据协议选择 1/2/5/7 图形批量。
- 字符串单独使用字符图形消息。

### 12.2 参考 UI 的问题

- 元素直接内含 packed 协议 bit-field，语义对象和 wire layout 未分开。
- 全局 static 自增 ID，不可预测、不可持久化，也不适合并发。
- setter 没有检查坐标、颜色、线宽和角度字段范围。
- float 乘 1000 后直接塞入 int32，未检查 NaN、Inf 和溢出。
- `reinterpret_cast<uint16_t *>` 写 CRC 尾可能未对齐，也依赖本机字节序。
- 单一内部帧 buffer 不适合异步 UART。
- demo 每 33ms 重发七个元素，没有场景差分，也没有和其他 `0x0301` 业务共享预算。
- 没有处理裁判系统重连、客户端刷新或 robot ID 改变后的 UI 重建。

### 12.3 推荐四层 UI

```text
UiScene          我希望屏幕最终长什么样
    ↓
UiDiff           和上次确认发送状态相比，哪些要 Add/Modify/Delete
    ↓
UiBatcher        按 1/2/5/7 和 String 规则组合
    ↓
UiEncoder        逐位编码为 0x0301 payload
    ↓
TxScheduler      和多机通信共享频率/带宽
```

### 12.4 语义接口草案

```cpp
struct UiElementId {
    uint8_t bytes[3];
};

enum class UiElementType : uint8_t {
    Line,
    Rectangle,
    Circle,
    Ellipse,
    Arc,
    FloatNumber,
    IntegerNumber,
    Text,
};

struct UiElement {
    UiElementId id;
    UiElementType type;
    uint8_t layer;
    uint8_t color;
    uint16_t width;
    // 使用一个本地语义 union 保存对应几何，不直接保存 wire bit-field
    UiGeometry geometry;
    bool visible;
};

int ui_scene_upsert(UiScene *scene, const UiElement &element);
int ui_scene_remove(UiScene *scene, UiElementId id);
void ui_scene_mark_all_dirty(UiScene *scene);
```

ID 使用源码中明确写出的稳定常量，例如：

```text
底盘模式文本：{'C','M','D'}
小 yaw 限位：{'Y','L','M'}
超级电容条：{'C','A','P'}
```

不要依赖对象构造顺序产生 ID。固件重启后，同一元素仍应使用同一 ID。

### 12.5 UI 生命周期

```text
Offline
  → 等待常规链路和新鲜 robot_id

NeedReset
  → 发送一次 DeleteAll
  → 等待发送完成和最小间隔

Adding
  → 按优先级分批 Add 静态元素

Running
  → 只发送 dirty 的 Modify/Delete

Resync
  → 链路重连、robot_id 改变或显式请求时回到 NeedReset
```

不要每周期 DeleteAll，也不要每 33ms 重发完全没变化的静态图形。

### 12.6 UI 自检

- 同一元素 Add 后修改，只发 Modify，不重新随机生成 ID。
- 7 个元素正确选七图形子命令。
- 3 个元素可按五图形帧补空操作，或拆成 2+1；策略固定并测试。
- 字符串最长长度截断后明确返回 `Truncated`，不能悄悄成功。
- 坐标超出字段范围返回错误或显式 clamp。
- NaN/Inf 数值拒绝编码。
- 链路重连后能自动 DeleteAll + Add 重建。
- UI 流量被控制消息抢占时，只延迟更新，不阻塞控制消息。

## 13. 裁判数据不能直接等于底盘功率控制器

### 13.1 “不同等级允许不同功率”的逻辑到底在哪里

结论：不在当前 SkyWalker，也不在 `sp_middleware` 的学生端代码中查表计算。

当前数据流是：

```text
比赛规则 / 赛事引擎 / 裁判系统性能体系
  → 根据当前赛季、兵种、等级和性能配置确定允许值
  → 电源管理模块通过常规链路发送 0x0201（10Hz）
  → 机器人同时收到：
       robot_level
       chassis_power_limit
       power_management_*_output
  → 学生代码直接使用新鲜的 chassis_power_limit
```

2026 V1.3.0 的 `0x0201` 数据段长度为 13 字节，其中：

```text
payload[0]       robot_id
payload[1]       robot_level
payload[10..11]  chassis_power_limit，little-endian，单位 W
payload[12]      电源管理模块各输出口状态位
```

因此不要写：

```text
if level == 1: power_limit = ...
if level == 2: power_limit = ...
if level == 3: power_limit = ...
```

原因是“等级”未必是决定功率的唯一条件，而且赛季规则、兵种和性能体系可能变化。官方已经下发最终的 `chassis_power_limit`，本地再按等级查表可能和裁判系统真实限制不一致。

在 `sp_middleware` 中能追到的路径只有：

```text
referee/referee_protocol/referee_protocol.hpp
  → 声明 robot_level 和 chassis_power_limit 字段

referee/pm02/pm02.cpp
  → 收到 0x0201 后 copy_fixed 到 robot_status

motor/super_cap/readme.md
  → 将 robot_status.chassis_power_limit 和 buffer_energy 交给 SuperCap::write()

motor/super_cap/super_cap.cpp
  → 只把参数编码进 CAN 数据，不计算“等级对应多少瓦”
```

整个参考仓库没有找到 `switch(robot_level)`、等级功率表或按等级修改四轮输出的控制器。

当前 SkyWalker 仓库也还没有裁判 decoder 和底盘功率控制器。

### 13.2 本项目应该怎么实现这条链

Decoder 只负责提取事实：

```cpp
struct RobotPerformanceStatus {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t chassis_power_limit_w;
    bool chassis_output_enabled;
};
```

伪代码：

```text
要求 payload_len == 当前赛季 0x0201 的精确长度

temporary.robot_id = payload[0]
temporary.robot_level = payload[1]
temporary.chassis_power_limit_w = read_u16_le(payload + 10)
temporary.chassis_output_enabled = (payload[12] & (1 << 1)) != 0

全部字段验证完成
  → 一次性提交 RefereeStore
  → stamp_ms/generation/valid 同时更新
```

业务层读取时：

```text
若 0x0201 fresh：
    allowed_power_w = chassis_power_limit_w
否则：
    allowed_power_w 平滑退到项目配置的保守安全值
```

`robot_level` 建议用于：

- UI 显示。
- 日志和赛场问题诊断。
- 与其他性能字段做一致性检查。
- 观察升级事件。

不要用它覆盖一份新鲜的 `chassis_power_limit_w`。

离线调试没有裁判系统时，可以准备一个显式的 `test_power_limit_w`，但必须满足：

- 只在测试 build/config 中启用。
- UI/日志明确显示当前是 fallback，不是假装来自裁判系统。
- 取保守值。
- 一旦收到新鲜 `0x0201`，立即切换到官方下发上限。
- 不用一张隐藏的“等级表”在正式比赛路径中自动猜值。

### 13.3 “允许功率”和“真正把功率限制住”是两件事

`chassis_power_limit_w` 只是约束输入。要让四个驱动轮实际不超限，还需要单独实现：

```text
允许功率 chassis_power_limit_w
  + 缓冲能量 buffer_energy_j
  + 超级电容状态
  + 电池/母线测量
  + 四轮当前命令和速度
  → ChassisPowerPolicy
  → 总电流/功率预算
  → 四轮共同缩放系数
  → 各驱动电机最终电流命令
```

参考仓库没有完成这一层。`SuperCap::write()` 把允许功率告诉电容控制板，并不等于已经限制住 M3508 的总功率。

第一版接口可以按本文下面的 `RefereePowerInput` 设计；等舵轮基本运动稳定后，再在《古法编程总路线图》的阶段 10B 接入底盘。

裁判模块只提供经过验证、带新鲜度的事实：

```cpp
struct RefereePowerInput {
    uint16_t chassis_power_limit_w;
    uint16_t buffer_energy_j;
    uint8_t remaining_energy_raw;
    bool power_limit_fresh;
    bool buffer_energy_fresh;
    bool remaining_energy_fresh;
};
```

实际底盘功率策略单独实现：

```text
RefereeSnapshot
   + 超级电容反馈
   + 电池/母线测量
   + 四轮命令需求
   → ChassisPowerPolicy
   → 四轮共同缩放系数/总电流预算
```

原因：

- 2026 `0x0201` 提供底盘功率上限。
- 2026 `0x0202` 的部分旧功率字段已成为保留位，不能沿用旧赛季含义。
- 裁判数据存在约百毫秒延迟和丢包，不适合直接作为高速电流环反馈。
- 数据过期时需要保守降级策略，而不是 parser 自动写一个看似正常的默认功率。

建议降级：

- 裁判数据从未有效：使用赛前配置的低功率安全上限。
- 10Hz 状态短暂丢一两帧：保持上一合法上限但标 stale warning。
- 持续超时：平滑下降到安全上限，不突然把底盘命令砍成零造成失稳。
- 电源管理输出位显示 chassis 口关闭：立刻停止底盘电机命令并 reset PID。

具体安全上限需要你按参赛机器人、硬件和当赛季规则填写，本文不能替你猜。

### 13.4 发弹速度到底在哪里控制

先给最直接的结论：

1. 当前 SkyWalker 仓库还没有发射机构业务代码，所以现在没有任何地方在控制发弹速度。
2. `sp_middleware` 也没有摩擦轮或拨弹电机控制器。它只解析裁判反馈，不能整块借来完成发射控制。
3. 弹丸**初速度**由本地摩擦轮转速闭环控制；弹丸**射频**由拨弹轮/推弹机构的节拍控制；裁判系统只给约束和事后测量。
4. 2026 协议的 `0x0201` **不下发弹丸初速度上限**。参考仓库中 `RobotStatus::bullet_speed_limit` 是旧赛季残留，正是它让参考结构体长成 17 字节，而 2026 V1.3.0 官方长度只有 13 字节。

#### 13.4.1 先分清四种“速度”

| 名称 | 单位 | 谁产生 | 应该由谁使用 |
|---|---:|---|---|
| 允许弹丸初速度上限 | m/s | 当赛季比赛规则、兵种和状态 | `ShooterRulePolicy` |
| 实测弹丸初速度 | m/s | 测速模块，经 `0x0207` 回传 | 记录、超限保护、慢速标定修正 |
| 摩擦轮电机转速 | rpm 或 rad/s | 电机编码器 | `FlywheelController` 的高速闭环 |
| 弹丸射频 | Hz，发/秒 | 拨弹机构节拍，`0x0207` 也会反馈测量值 | `FeederController` 和热量策略 |

最容易犯的错误是把 `0x0207.initial_speed` 直接喂给摩擦轮 PID。它是“弹丸已经射出去之后”的事件反馈，不是每 1ms 都有的新测量，不能代替电机编码器速度反馈。

正确数据流是：

```text
当赛季/兵种/比赛状态
  → ShooterRulePolicy
  → 允许目标弹速上限（m/s）
  → 留出经过实测确定的安全余量
  → MuzzleVelocityMap：目标弹速 → 摩擦轮目标 rpm
  → 左右摩擦轮各自的速度 PID
  → 电机电流命令
  → 摩擦轮实际 rpm（高速闭环反馈）

裁判 0x0207 实测初速度（每次发弹后才来）
  → ShotObserver
  → 记录均值/方差/超限
  → 仅缓慢、小幅修正 MuzzleVelocityMap 的偏置
```

发射节拍走另一条链：

```text
操作手单发/连发命令
  + 0x0201 热量上限和冷却值
  + 0x0202 当前热量
  + 0x0208 剩余允许发弹量
  + shooter 供电口状态
  + 摩擦轮是否稳定到速
  → ShooterHeatPolicy / FireInterlock
  → 允许单发或允许的最大射频
  → FeederController
  → 拨弹轮角度步进或速度目标
```

#### 13.4.2 当前官方协议到底提供什么

2026 V1.3.0 的 `0x0201` 长度为 13 字节：

```text
payload[0]       robot_id
payload[1]       robot_level
payload[2..3]    current_hp
payload[4..5]    maximum_hp
payload[6..7]    shooter_barrel_cooling_value，每秒冷却值
payload[8..9]    shooter_barrel_heat_limit，热量上限
payload[10..11]  chassis_power_limit，W
payload[12]      gimbal/chassis/shooter 供电口状态位
```

这里没有 `bullet_speed_limit`。

2026 V1.3.0 的 `0x0207` 长度为 7 字节，弹丸发射后发送：

```text
payload[0]       bullet_type
payload[1]       shooter_number
payload[2]       launching_frequency，Hz
payload[3..6]    initial_speed，IEEE-754 float，m/s，little-endian
```

它描述“刚才实际打成什么样”，不发控制命令。解码 float 时先用 `read_u32_le()` 取出位模式，再 `memcpy` 到 `float`，并拒绝 NaN/Inf；不要把未对齐的 `payload + 3` 强转成 `float *`。

`0x0202` 提供当前射击热量，`0x0208` 提供允许发弹量。它们控制“还能不能继续打”，仍不负责调摩擦轮 rpm。

#### 13.4.3 等级是否决定弹丸初速度

按本文核对的 2026 超级对抗赛 V1.3.0：等级会改变发射机构的**热量上限和冷却速率**，但规则表没有让步兵的初速度上限随等级变化。

规则手册中的例子是：

- 英雄 42mm：通常为 12m/s；选择远程优先并进入部署模式后为 16.5m/s。
- 步兵、空中、哨兵的 17mm：规则表列为 25m/s。
- 步兵选择“爆发优先/冷却优先”等发射机构性能体系时，变化的是热量上限和冷却速率，不是 `0x0201` 中某个弹速字段。

这些数值只适用于上面明确标注的赛事和版本。你需要先确认本队参加的是超级对抗赛还是其他赛事、机器人兵种、弹丸口径以及官方是否发布更新，才能填写项目配置。

因此本项目不要写：

```text
speed_limit = table[robot_level]
```

而要写成明确、可审计的赛季配置和状态转换：

```cpp
enum class RobotClass : uint8_t {
    Hero,
    Infantry,
    Aerial,
    Sentry,
};

enum class ProjectileKind : uint8_t {
    Mm17,
    Mm42,
};

struct ShooterRuleConfig {
    RobotClass robot_class;
    ProjectileKind projectile;
    float normal_muzzle_limit_mps;
    float special_mode_muzzle_limit_mps;
};

struct ShooterRuleState {
    bool special_mode_allowed;
    bool special_mode_active;
    bool state_fresh;
};
```

`ShooterRulePolicy` 返回当前允许上限，但它不能偷偷把赛事规则硬塞进协议 decoder：

```cpp
enum class ShooterRuleResult : uint8_t {
    Ok,
    InvalidConfig,
    StateStale,
    ModeNotAllowed,
};

ShooterRuleResult shooter_rule_limit(
    const ShooterRuleConfig &config,
    const ShooterRuleState &state,
    float *limit_mps);
```

边界行为：

- 配置为 NaN、Inf、零或负数：`InvalidConfig`，禁止发弹。
- 特殊模式请求存在，但当前兵种/性能体系不允许：`ModeNotAllowed`，不能擅自使用更高上限。
- 特殊模式状态过期：`StateStale`，退回普通上限或禁止发射；比赛版本确定后固定一种策略。
- `robot_level` 只影响裁判实际下发的热量/冷却等性能字段，不用来覆盖弹速配置。

#### 13.4.4 发射控制器应该拆成哪些接口

不要造一个同时解析串口、算热量、调两个 PID、驱动拨盘的大 `Shooter` 类。第一版拆成六个小模块：

```text
ShooterRulePolicy
  只回答当前允许的最大初速度

MuzzleVelocityMap
  把期望 m/s 映射成左右摩擦轮 rpm

FlywheelController
  用电机编码器速度做左右轮高速闭环，输出电流

ShooterHeatPolicy / FireInterlock
  根据热量、弹量、供电、到速状态决定能否拨一发

FeederController
  执行单发角度步进、连发节拍、堵转识别和有限次数退弹

ShotObserver
  消费 0x0207，统计实际初速度，做限幅后的慢修正
```

这些是机器人业务控制，不是裁判协议，也不是单个电机驱动。建议你亲手新增到下面的位置：

```text
include/control/shooter/
├── shooter_types.hpp
├── shooter_rule_policy.hpp
├── muzzle_velocity_map.hpp
├── flywheel_controller.hpp
├── shooter_heat_policy.hpp
├── feeder_controller.hpp
└── shot_observer.hpp

lib/control/shooter/
├── CMakeLists.txt
├── Kconfig
├── shooter_rule_policy.cpp
├── muzzle_velocity_map.cpp
├── flywheel_controller.cpp
├── shooter_heat_policy.cpp
├── feeder_controller.cpp
└── shot_observer.cpp

tests/control/shooter/
├── rule_policy/
├── muzzle_velocity_map/
├── heat_policy/
├── fire_interlock/
└── shot_observer/
```

文件级接入顺序：

1. 在现有 `lib/CMakeLists.txt` 加入 `add_subdirectory_ifdef(CONFIG_SKYWALKER_CONTROL_SHOOTER control/shooter)`；根 `CMakeLists.txt` 已经进入 `lib`，不要再重复进入同一目录。
2. 在现有 `lib/Kconfig` 增加总开关 `SKYWALKER_CONTROL_SHOOTER`，由 `lib/control/shooter/CMakeLists.txt` 收集六个 `.cpp`。
3. 纯规则、映射和热量计算不依赖设备树；摩擦轮/拨弹控制器只依赖抽象 `Motor` API，不包含具体 M3508/M2006 型号头。
4. 新建 `application/src/shooter_task.cpp` 负责实例化、周期调度和传递快照，不写协议偏移、不重复 PID 算法。当前入口是 C 文件；若保留 `main.c`，就给 task 提供一个小型 `extern "C"` 启动函数，不要在 C 文件里直接包含 `.hpp`。
5. 在 `application/CMakeLists.txt` 中显式加入 `shooter_task.cpp`，并在 `application/prj.conf` 打开总开关；先保持拨弹功能默认禁用。

如果你不想现在建立顶层 `control/`，就先用上面的 `lib/control/shooter` 方案。不要把这些文件塞到 `drivers/motor/`：电机驱动只负责“给定电流、读取反馈”，不知道什么叫弹丸、热量或比赛许可。

推荐先定义纯数据接口：

```cpp
struct FlywheelFeedback {
    float left_rpm;
    float right_rpm;
    bool left_fresh;
    bool right_fresh;
};

struct FlywheelTarget {
    float left_rpm;
    float right_rpm;
};

struct FlywheelOutput {
    int16_t left_current_raw;
    int16_t right_current_raw;
    bool ready;
    bool fault;
};

struct FireConstraintSnapshot {
    uint16_t heat_limit;
    uint16_t current_heat;
    uint16_t cooling_per_second;
    uint16_t projectile_allowance;
    bool shooter_power_enabled;
    bool performance_fresh;
    bool heat_fresh;
    bool allowance_fresh;
};

enum class FireBlockReason : uint8_t {
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
    bool allow_next_shot;
    float max_frequency_hz;
    FireBlockReason reason;
};
```

建议调用关系：

```text
1 kHz 或电机控制周期：
    FlywheelController.step(target_rpm, motor_feedback, dt)
    FeederController.step(feeder_target, feeder_feedback, dt)

100 Hz 左右策略周期：
    读取一次一致的 RefereeSnapshot
    ShooterRulePolicy 计算允许弹速
    ShooterHeatPolicy 计算 allow_next_shot/max_frequency_hz
    FireInterlock 合并摩擦轮到速、供电和故障状态

收到 0x0207 事件时：
    ShotObserver.on_shot(measurement)
```

各层的线程边界：

- UART 回调只把字节放进 ring buffer，绝不调 PID。
- parser/decoder 更新 `RefereeStore`，绝不直接写电机电流。
- 策略线程读取一致快照，产出允许/禁止和目标值。
- 电机控制线程只使用已经发布的目标，不能阻塞等待裁判帧。
- `0x0207` 事件队列满时计数并报警，不能在中断里打印或做浮点标定。

#### 13.4.5 目标初速度怎样变成摩擦轮 rpm

只能先用理论关系得到起点，最终必须实弹标定：

```text
摩擦轮表面速度 ≈ pi × 有效直径 × rpm / 60
弹丸初速度 ≠ 摩擦轮表面速度
```

实际还受压缩量、轮材、磨损、温度、电池电压和弹丸一致性影响。所以建议实现单调分段表，而不是在代码里散落一个神秘乘数：

```cpp
struct MuzzleMapPoint {
    float target_mps;
    float left_rpm;
    float right_rpm;
};

enum class MuzzleMapResult : uint8_t {
    Ok,
    InvalidTable,
    BelowRange,
    AboveRange,
};

MuzzleMapResult muzzle_map_lookup(
    const MuzzleMapPoint *points,
    size_t count,
    float target_mps,
    FlywheelTarget *target);
```

要求：

- `target_mps` 超过规则策略给出的上限时先 clamp 到**带安全余量的目标**，并记录一次限幅。
- 表中 `target_mps` 必须严格递增；重复点或 NaN/Inf 直接判表无效。
- 左右轮方向由安装定义，表中明确保存符号，不能在多个调用点反复取负。
- 目标 rpm 需要斜坡上升，防止上电电流冲击。
- `ready` 必须要求左右轮误差都进入窗口并持续一段时间，不能刚穿过目标就拨弹。
- 安全余量不能拍脑袋写死，应根据多组测速样本的均值、方差和最坏工况确定。

摩擦轮速度 PID 的测量值是电机反馈 rpm。输出先经过 PID 自身限幅，再经过电机驱动允许电流限幅；禁用时输出 0 并 reset 积分项。不要让两个摩擦轮共用同一个 PID 状态对象。

`0x0207.initial_speed` 只适合做慢修正，例如：

```text
同一目标档位累计若干个有效样本
  → 排除 NaN/Inf、错误弹种、错误发射机构和明显离群值
  → 计算低通后的平均误差
  → 每次只允许很小的 rpm_bias 变化
  → bias 有上下限，掉电是否保存由标定策略决定
```

不能“这一发低了 1m/s，下一控制周期立刻大幅加 rpm”，否则稀疏、延迟且有噪声的弹速反馈会让摩擦轮目标跳动，反而更容易超限。

#### 13.4.6 射频由拨弹机构控制，不由摩擦轮 PID 控制

若目标射频为 `f_hz`，理想发弹间隔是：

```text
period_ms = 1000 / f_hz
```

但不能每到时间就无条件拨一发。每发之前都重新检查：

```text
操作手仍在请求
AND 允许发弹量 > 0
AND 当前热量 + 单发热量 + reserve <= 热量上限
AND 裁判关键数据 fresh
AND shooter 供电口开启
AND 左右摩擦轮持续到速
AND 拨弹电机反馈 fresh
AND 没有堵转/超速故障
```

按本文所用 2026 规则，裁判每检测到一发 17mm 弹丸热量增加 10，每发 42mm 增加 100；热量以 10Hz 结算冷却。你可以用这些值预测“下一发会不会越线”，但仍应以当赛季规则为配置依据。

持续射击的理论长期射频上限可用 `cooling_per_second / heat_per_shot` 做第一层估算；短时间爆发还必须受当前剩余热量预算约束。不要因为理论平均值安全，就忽略这一发打完后的瞬时热量。

`0x0202` 和 `0x0208` 都是低频裁判状态。在两帧之间，控制器必须自己保留已经批准但裁判系统尚未反映的发弹消耗：

```cpp
struct LocalFireLedger {
    uint16_t referee_allowance;
    uint16_t reserved_shots;
    float predicted_heat;
    uint32_t allowance_generation;
    uint32_t heat_generation;
};
```

每批准一发，而不是每收到一次按键时：

```text
available = max(0, referee_allowance - reserved_shots)

若 available == 0：禁止
若 predicted_heat + heat_per_shot + reserve > heat_limit：禁止

真正接受一次新的拨弹动作：
    reserved_shots += 1
    predicted_heat += heat_per_shot
```

收到更新一代的 `0x0202/0x0208` 后再和裁判事实对账。对账必须满足：

- 同一 `generation` 被策略循环读取多次，不能重复清空预约。
- 新帧里的允许发弹量没有下降时，不能武断认为上一发没有发生并立即恢复令牌；测速/计数反馈可能有延迟。
- 可以结合拨弹到位事件和 `0x0207` 到达情况清除“进行中”状态，但超时必须进入故障，不能静默补发。
- `predicted_heat` 至少取“本地预测”和“新鲜裁判热量”中更保守的一侧，再按明确的时间基准扣除冷却；时间倒退或超长 `dt` 时不自行冷却。
- 所有减法先比较再减，避免 `uint16_t` 下溢把 0 发弹量变成 65535。

拨弹控制建议先做“单发角度步进”：一次请求只把拨弹电机目标角增加一个弹位角，确认到位后才接受下一发。连发只是在上层按允许周期重复提交单发请求，不要一开始就让拨弹轮无限速度旋转。

#### 13.4.7 参考仓库能借什么、不能借什么

可以参考：

- `referee/pm02/pm02.cpp` 中 `0x0201`、`0x0202`、`0x0207`、`0x0208` 的分发位置，用来理解消息属于哪类状态。
- `io/vision/vision.*` 中把 `bullet_speed` 作为状态传给视觉的思路；它是观测数据，不是闭环控制器。

不能照搬：

- `referee/referee_protocol/referee_protocol.hpp` 的 `RobotStatus`。其中多出的 `float bullet_speed_limit` 不属于 2026 V1.3.0 的 13 字节 `0x0201`。
- `copy_fixed()` 对 packed struct 的整块复制方式。当前项目应逐字段解码并验证长度、端序和 float 有限性。
- 把 `pm02.shoot.initial_speed` 当目标值；它是上一发的测量值。
- 期待 `sp_middleware` 提供摩擦轮 PID、拨弹状态机或热量策略；此次全仓搜索没有找到这些实现。

#### 13.4.8 推荐你亲手实现的顺序

1. 先确认赛事、兵种、弹丸口径和当前官方版本，填写 `ShooterRuleConfig`。
2. 完成 `0x0201` 解码：热量冷却、热量上限、shooter 供电位必须有新鲜度。
3. 完成 `0x0202` 解码：只取当前赛季定义的 17mm/42mm 热量字段。
4. 完成 `0x0208` 解码：允许发弹量为 0 或过期时禁止拨弹。
5. 完成 `0x0207` 解码和事件队列，先只记录，不做自动修正。
6. 不装弹，单独闭环左右摩擦轮 rpm；验证斜坡、方向、掉线归零和双 PID 状态独立。
7. 不驱动拨弹轮，验证摩擦轮 `ready` 必须持续稳定才成立。
8. 在安全设施和有经验队员监督下，用多个 rpm 档位做实弹测速，建立 `MuzzleVelocityMap`。
9. 只做单发角度步进，并逐发核对 `0x0207`、当前热量和允许发弹量。
10. 最后才加连发调度、堵转恢复和慢速弹速偏置修正。

每一步的验收点：

- 参考仓库的 17 字节 `0x0201` 测试帧必须被当前 13 字节 spec 拒绝。
- 手工构造的 7 字节 `0x0207` 能正确得到 Hz 和 float m/s；把 float 改成 NaN 后 decoder 拒绝提交。
- 裁判数据过期、shooter 口关闭、任一摩擦轮反馈掉线时，拨弹命令必定被拦截。
- 两个摩擦轮未同时稳定到速时，按下发射键不会拨弹。
- 允许发弹量为 1 时只允许完成一发；不要在本地先减成负数。
- 预测下一发会越过热量预算时禁止发射，并输出明确的 `FireBlockReason`。
- 实测初速度超过本地安全阈值时锁存 `MuzzleSpeedFault`，停止继续拨弹，不能靠下一发再试。
- 禁用、反馈掉线或控制周期 `dt` 非法时，两侧电流输出为 0，PID 积分清零。

实弹测试具有机械和弹丸风险。必须使用可靠挡弹设施、清空射界、固定机器人、佩戴护目镜、安排独立急停人员；第一次闭环和方向检查不得装弹，第一次拨弹不得用连发。

## 14. CRC 模块怎么处理

参考仓库把 CRC8/CRC16 放在独立 `tools/crc`，这个拆分是对的。但当前仓库没有许可证，不能复制其查表；官方协议附录本身提供了 CRC 示例和参数，可以依据官方材料独立实现。

建议接口：

```cpp
uint8_t referee_crc8(
    const uint8_t *data,
    size_t len,
    uint8_t init);

uint16_t referee_crc16(
    const uint8_t *data,
    size_t len,
    uint16_t init);

bool referee_verify_header_crc8(
    const uint8_t *header,
    size_t len);

bool referee_verify_frame_crc16(
    const uint8_t *frame,
    size_t len);
```

边界行为：

- `data==nullptr && len>0` 返回错误路径，不解引用。
- header 验证长度必须精确为 5。
- frame 验证长度必须至少为 9。
- 不允许 `len-1` 或 `len-2` 在无符号类型中下溢。
- 尾部 CRC 使用明确 little-endian 写入，不用未对齐指针强转。

测试向量应从官方附录或你实际抓取并确认合法的帧生成，测试至少包含修改任意一个 bit 后校验失败。

## 15. 错误处理和可观测性

至少暴露下面的状态：

```cpp
struct RefereeServiceStatus {
    bool regular_link_alive;
    bool video_link_alive;
    bool robot_id_valid;
    uint16_t last_regular_cmd_id;
    uint16_t last_video_cmd_id;
    uint32_t last_regular_frame_ms;
    uint32_t last_video_frame_ms;
    RefereeLinkStats regular_stats;
    RefereeLinkStats video_stats;
};
```

调试输出建议：

- 每秒打印一次统计差值，不逐帧 printk。
- VOFA 观察 `valid_frames/s`、CRC error/s、ring overflow、sequence gaps。
- 为每个关键 topic 显示 age，而不是只显示 value。
- 未知 cmd_id 计数；调试版可以限频打印 ID 和长度。
- release 版仍保留计数，但不打印 payload，避免串口负担和信息泄露。

## 16. 推荐实施顺序

### 阶段 0：建立 2026 V1.3.0 协议核对表

你先手工建立一张表，至少包含：

```text
cmd_id
link
payload length/range
发送方向
接收对象
官方发送周期或频率上限
生命周期：周期状态/锁存状态/事件/变长消息
本项目 decoder 是否已实现
```

第一版只做当前机器人真正需要的 cmd，不要一口气录入整本协议。

建议优先级：

1. `0x0201` 机器人性能体系。
2. `0x0202` 缓冲能量和射击热量。
3. `0x0001` 比赛状态。
4. `0x0204` 增益/底盘能量。
5. `0x0208` 允许发弹量。
6. 你兵种需要的事件和交互消息。
7. UI 发送。
8. 图传/自定义客户端。

### 阶段 1：byte codec 和 CRC

只实现纯函数，不接 UART。

自检：

- little-endian 最小/最大/中间值。
- 未对齐地址读取。
- 官方 CRC 黄金帧。
- 空指针和过短长度。

### 阶段 2：FrameParser

用内存数组逐块喂数据，不启动 Zephyr。

必须通过：

- 一帧一次输入。
- 一次一个字节。
- 帧头跨输入边界。
- 两帧粘包。
- 噪声 + 正确帧。
- 坏 CRC + 正确帧。
- 假 SOF + 非法长度 + 正确帧。
- payload 内含 `0xA5`。
- 最大合法帧。
- 超最大 payload。

### 阶段 3：CommandSpec 和前三个 decoder

先做 `0x0201/0x0202/0x0001`。

每个 decoder 至少两类测试：

- 官方长度和已知字节得到正确语义值。
- 旧赛季长度/多一个字节/少一个字节全部拒绝，store 不变。

### 阶段 4：RefereeStore

测试周期状态、锁存状态和事件 generation。

自检：

- 数据收到前 `valid=false`。
- 周期数据按设定 age 变 stale。
- 锁存数据不会无故归零。
- 两次相同事件仍让 generation 增加两次。
- snapshot 复制出的字段来自同一次锁保护。

### 阶段 5：只接常规链路 RX

先不发 UI、不接电机策略，只接电源管理模块 User 串口。

硬件上电顺序：

1. 机器人动力输出保持禁用。
2. 只给主控和裁判模块供电。
3. 确认 UART 电平、GND、TX/RX 交叉和 115200 配置。
4. 启动 RX，观察每秒有效帧和 CRC 错误。
5. 对照官方频率检查 `0x0201/0x0202` 的接收周期。
6. 拔掉串口，确认 link age 和 topic fresh 正确失效。
7. 插回，确认 parser 不重启系统也能恢复。

### 阶段 6：发送编码器和调度器

先用本地 fake UART 或编码结果测试，不立即发比赛链路。

检查：

- frame header length、cmd_id、CRC 正确。
- 异步 TX 期间 frame 内存保持不变。
- UART busy 时入队或明确返回 `-EBUSY`，不能覆盖。
- 过期帧不会补发。
- 每 cmd 和共享频率限制有效。

### 阶段 7：UI 最小闭环

只做一个固定文本或一条线：

1. 等待新鲜 robot ID。
2. DeleteAll 一次。
3. Add 一次。
4. Modify 一次。
5. 停止发送，确认 UI 保持。
6. 模拟重连，确认自动重建。

再实现 batch 和完整 scene diff。

### 阶段 8：接底盘功率和发射策略

只给策略传 snapshot，不让策略读 parser 内部成员。

先用日志验证限制量，再允许它影响电机。对过期、突然降额和比赛阶段切换做斜坡/状态机测试。

### 阶段 9：图传链路和 `0x0301` 多机通信

最后接第二个 UART 实例。复用相同 parser/encoder 代码，但使用独立 context、buffer、sequence、队列和统计。

## 17. 建议由你执行的测试命令

本文没有运行这些命令。等你创建对应 `tests/referee` 后，可按实际 Zephyr 工作区调整：

```bash
west twister -T tests/referee -v
```

只跑解析器目录：

```bash
west twister -T tests/referee/frame_parser -v
```

如果使用 native_sim：

```bash
west build -b native_sim tests/referee/frame_parser -p always
west build -t run
```

如果当前 Zephyr revision 的板名或测试入口不同，以该 revision 文档和 `west boards` 输出为准，不要直接复制命令报错后改源码迁就命令。

## 18. 常见故障排查

### 完全收不到帧

- 检查接的是电源管理模块 User 串口还是图传串口。
- 检查 baud：常规链路 115200，图传链路 921600。
- 检查 TX/RX 是否交叉、是否共地、电平是否匹配。
- 检查 Zephyr pinctrl 和 overlay。
- 先看 `rx_bytes`，再看 `valid_frames`；两者都为零是物理层问题。

### 有字节但 CRC 全错

- baud/串口格式错误。
- `UART_RX_RDY` 忽略了 offset，重复解析了 buffer 开头。
- CRC 参数、初值或尾部字节序错误。
- RX buffer 在解析前被 DMA 复用。

### 偶尔能解析，粘包时失败

- 仍在假设一次 callback 等于一帧。
- 没有保存半帧。
- CRC 失败后丢掉了整块 buffer，而不是逐字节重同步。

### 数值看起来错位

- 使用了 2025/2026 混合结构长度。
- memcpy packed struct 或编译器 bit-field 布局不同。
- float 字节序/未对齐读取错误。
- cmd table 长度正确，但 decoder offset 没随赛季更新。

### UI 偶尔花屏或 CRC 错

- 异步 UART 正在发送时复用了同一个 buffer。
- 元素 ID 不稳定。
- UI 和多机通信超过共享 `0x0301` 频率。
- Add/Modify 操作类型不符合当前 scene 状态。
- robot ID/receiver ID 不正确或已过期。

## 19. 哪些参考代码可以看，哪些不能照搬

| 参考文件/行为 | 可以借鉴 | 不可照搬 |
|---|---|---|
| `referee/pm02/pm02.cpp` | 外层 cmd switch、`0x0301` 二级 switch、长度先检查、部分数据 freshness | HAL UART、递归块解析、整结构 memcpy、public 数据仓库、混合协议版本 |
| `referee/referee_protocol/referee_protocol.hpp` | cmd 命名和需要覆盖的消息清单 | 作为 2026 真值、packed bit-field、跨赛季固定类型 |
| `referee/ui/element.*` | UI 语义类型、稳定 setter 形状 | 对象直接持有 wire struct、全局自增 ID、无范围检查 |
| `referee/ui/manager.*` | 1/2/5/7 批量、String 独立帧 | 单 buffer 异步发送、未对齐 CRC 写入、没有调度/场景差分 |
| `referee/vt03/vt03.*` | 常规帧和自定义数据属于不同业务入口、alive 概念 | HAL、packed 遥控 bit-field、开始解析前就刷新 alive、static TX buffer |
| `tools/crc/*` | CRC 独立模块 | 在无许可证情况下复制查表和函数体、过短 len 下溢 |
| 本地 `lib/vofa/vofa.c` | Zephyr async UART API 的基本形状 | 把逐帧/逐行解析放在 callback、共享 static TX buffer |

## 20. 最终勾选清单

### 协议依据

- [ ] 已固定本赛季、赛项和官方协议版本。
- [ ] 已保存 cmd/link/length/frequency 描述表。
- [ ] 没有使用 `sp_middleware` 混合版本头作为真值。
- [ ] 比赛前重新检查官方增补。

### RX

- [ ] 常规/图传链路拥有独立 parser 和统计。
- [ ] UART callback 只搬运字节。
- [ ] 正确使用 RX offset 和双 buffer。
- [ ] parser 支持拆包、粘包、噪声和坏 CRC 重同步。
- [ ] 非法长度不会导致越界或长时间等待假帧。

### Decoder/Store

- [ ] 每个固定消息严格检查当前赛季长度。
- [ ] 多字节字段逐字段按 little-endian 解码。
- [ ] 不用 packed bit-field 映射线上数据。
- [ ] float 检查 NaN/Inf。
- [ ] 周期状态、锁存状态和事件分开处理。
- [ ] 每类关键数据有 valid/stamp/generation。
- [ ] 消费者读取一致 snapshot。

### TX/UI

- [ ] 每个异步 TX frame 独立拥有内存直到完成。
- [ ] sender/receiver/link 权限经过校验。
- [ ] scheduler 遵守单 cmd 和共享带宽限制。
- [ ] 过期发送请求会丢弃。
- [ ] UI ID 稳定且显式。
- [ ] UI 使用 scene/diff，不重复刷静态元素。
- [ ] 链路重连能重建 UI。

### 安全

- [ ] 裁判数据过期不会被默认数值伪装成有效。
- [ ] 功率策略与 parser 分离。
- [ ] 裁判链路断开时有明确保守功率策略。
- [ ] 上电 RX 验证阶段保持电机动力禁用。
- [ ] CRC/长度/ID 错误都有计数可查。

## 21. 尚未验证、需要你决定的内容

- 你参加 RMUC、RMUL 还是其他赛项，机器人具体兵种是什么。
- 当前硬件实际使用哪一代电源管理模块和图传发送端。
- DM-MC02 上分配给常规链路、图传链路的具体 UART 节点和引脚。
- 第一版是否需要 UI、多机通信、自定义控制器或自定义客户端。
- 裁判数据应该放在底盘板还是云台板，以及另一块板需要哪些摘要。
- 你们比赛前最终采用的官方通信协议版本。
- 是否已经取得 `sp_middleware` 源码复用授权。

如果裁判串口只接在底盘板，建议底盘板保存完整 `RefereeStore`，跨板只发送云台真正需要的摘要，例如比赛阶段、发射热量/弹量、机器人 ID 和相关有效位；不要把所有原始裁判帧再透传一遍。
