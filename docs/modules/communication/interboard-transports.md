# 板间 UART、RS485 与 CAN

三种方式均已实现。`InterBoardEndpoint` 保留 `submit / setReferee / setStatus / snapshot / poll`；构造时绑定 `InterBoardTransport`，此后不更换后端。现有 V1 业务帧、CRC、序号、boot_id、恢复 generation 和命令有效期继续使用。

## 初始化选择

```cpp
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
using namespace skywalker;

static communication::AsyncUart::DmaBuffers dma __nocache;
static const communication::ConfiguredInterBoardTransport::Config config = [] {
    communication::ConfiguredInterBoardTransport::Config c{};
    c.kind = communication::InterBoardTransportKind::Rs485; // Uart / Rs485 / Can
    c.uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart));
    c.rs485.uart = DEVICE_DT_GET(DT_ALIAS(interboard_rs485));
    c.rs485.role = communication::Rs485InterBoardTransport::Role::Coordinator;
    c.can.can = DEVICE_DT_GET(DT_ALIAS(interboard_can));
    c.can.tx_id = 0x600;
    c.can.rx_id = 0x601;
    return c;
}();
static communication::ConfiguredInterBoardTransport transport(config, &dma);
static communication::InterBoardEndpoint link(
    transport, {robotics::BoardRole::GimbalController, 100, 100, 60});
```

底盘端使用 `ChassisController`、RS485 `Responder`，CAN 的 TX/RX ID 对调。两板选择相同传输方式。业务线程提交值、复制快照；唯一通信线程持续调用 `link.poll(k_uptime_get())`，建议间隔 1 ms，包括故障期间。

执行线程在每次真实生产时填 `RunStatus.stamp={now_ms, sequence, true}` 再调用 setStatus。Config 第五个字段 `status_timeout_ms` 默认 100 ms；发送心跳和底盘反馈前检查该年龄。状态无效、过期或来自未来时间时，V1 就绪和使能信息撤销并报告 FeedbackStale，心跳依然正常发送。setStatus 与 poll 都不重盖执行生产时间；停止执行线程时，通信线程不能继续宣布旧 Ready/Active。默认无状态生产者的 endpoint 会报告未就绪。

`ConfiguredInterBoardTransport` 通过 `std::variant` 在对象内部构造一个后端，不分配堆内存，不启动未选择的设备。也可以直接构造 `UartInterBoardTransport`、`Rs485InterBoardTransport` 或 `CanInterBoardTransport` 并传给 Endpoint。纯 CAN 构造不需要传 DMA 指针。对象不能在运行中移动、复制或销毁。

在 `applications/sentry_gimbal`、`applications/sentry_chassis` 中，只需修改各自 `src/board_config.hpp` 的 `interboard_transport.kind`；现有 overlay 已提供三种设备别名。默认仍是 UART。

## 大 Yaw V2 扩展

V1 心跳、底盘控制、约束和反馈保留原字段、ID、帧版本及枚举含义。新增轴通过单独的 V2 消息接通 codec、parser、link 和 endpoint；不能从 V1 的 `ready` 或轮控 `resume_generation` 推导大 Yaw 授权。

V2 使用同一帧头、长度、小端编码与 CRC16，帧版本字节为 2。`FrameMeta.version` 默认 1；Parser 可同时接收两种版本，并要求消息 ID 对应的帧版本正确。序号缓存扩为七项，原四个 V1 索引保持不变。

| V2 ID | 消息 | payload 长度 | 业务内容 |
| --- | --- | --- | --- |
| `0x0401` | `CapabilitiesV2` / `InterBoardCapabilities` | 16 字节 | 契约版本、能力位、发送板 boot_id；大 Yaw 能力为 bit0 |
| `0x0402` | `BigYawRequestV2` / `BigYawRequest` | 40 字节 | 禁用/回中模式、目标关节角速度、原始输入序号与年龄、最终命令年龄、目标板 boot_id、独立大 Yaw 恢复代次、输出许可及其年龄、命令生产序号 |
| `0x0403` | `BigYawFeedbackV2` / `BigYawFeedback` | 28 字节 | 执行状态、ready/armed/valid、实际关节角速度、原因位、独立恢复代次、最后接受命令序号、执行反馈生产序号与年龄 |

`BigYawMode::Disabled` 为 0，`FollowCenter` 为 1，速度单位为 rad/s。契约版本当前为 `kBigYawContractVersion=2`。这是业务契约版本；RS485 与 CAN 外层分片封装仍使用原来的版本 1。

新能力默认关闭，应用须显式启用：

```cpp
static communication::InterBoardEndpoint endpoint(transport, [] {
    communication::InterBoardEndpoint::Config c{robotics::BoardRole::GimbalController};
    c.enable_big_yaw = true;
    c.big_yaw_contract_version = robotics::kBigYawContractVersion;
    return c;
}());
```

云台执行线程通过 `submitBigYaw(request)` 交付回中请求，底盘执行线程通过 `setBigYawFeedback(feedback)` 交付真实生产的反馈。通信线程继续独占 `poll()`；其他线程读取 `Snapshot.big_yaw_request / big_yaw_feedback / capabilities / big_yaw_compatible` 的副本。`enable_big_yaw=false` 的既有应用继续只发送 V1 业务消息。

兼容性要求对端心跳与能力声明均新鲜、能力 boot_id 对应当前心跳、双方契约版本为 2 且支持大 Yaw。旧固件没有能力声明、声明过期或版本失配时，`big_yaw_compatible=false`，新增轴请求不获得授权；V1 业务语义继续独立处理。`version_mismatch.conf` 可让板间无输出样例中的一端声明版本 1，观察此边界。

云台端还要收到有效且未过期的大 Yaw 执行反馈，才能绑定目标板 boot_id 和该轴的恢复代次。首次建立上下文、轴恢复代次变化或有效反馈中断后重新建立上下文时，以当前原始源序号设定边界；回中请求须来自边界之后的新源。底盘执行器再次校验目标 boot_id 和自己的独立恢复代次。轮控心跳 generation 的变化不替代此过程。

### 生产年龄与状态停止

消息 `stamp` 在本地生产时记录生产时间；解码后记录本地原始接收时间。不同板的启动时间不必同步，接收端用线上的生产年龄加接收后的本地经过时间判断有效期。不能仅凭新的帧序号认定输入或反馈刚生产。

`submitBigYaw` 的调用者在请求生产时填入 `source_age_ms` 与 `command_age_ms`，分别表示原始输入、仲裁命令已经存在多久，并保存原始 `source_sequence`。Endpoint 发送时只累计从请求生产到发送的经过时间，背压或重发不归零。许可保留自己的原始 `permission.stamp`，发送前计算其年龄。接收快照会撤销超过命令/源有效期的请求，执行器仍监督原始源、权限和恢复边界。

`setBigYawFeedback` 的调用者保留执行生产 `stamp`。发送时计算 `production_age_ms`，保留独立恢复代次；状态停止生产后，即便心跳和通信线程持续运行，超过 `status_timeout_ms` 也会撤销 `valid/ready/armed` 并报告 `FeedbackStale`。对端同样以反馈生产年龄检查状态，不能继续使用旧 Ready/Active。

协议无输出演练见[板间样例](../../../samples/communication/interboard/README.md)，带真实独立速度执行器的双角色入口见[双 Yaw 回中样例](../../../samples/robotics/dual_yaw_centering/README.md)。这些入口尚需实际双板、重启、断流和停更验收。

## 实际硬件路径

| 方式 | MC02 外设与接口 | 配置 |
| --- | --- | --- |
| UART | USART1，PA9 TX / PA10 RX | 460800，8N1 |
| RS485 | USART2，PD5 TX / PD6 RX / PD4 DE，经板载收发器到 A/B | 460800，8N1，设备树 `de-enable` |
| CAN | FDCAN3，经板载收发器到 CAN3 H/L | 经典 CAN，1 Mbps |

RS485 与普通串口共用 AsyncUart。CAN 使用 Zephyr CAN API，收发器前端连接 FDCAN 控制器，不能用 UART API 代替。

USART2 DMA 使用 DMAMUX 通道 0(TX request 44)、5(RX request 43)，避开板级 SPI2 的 1/2、USART1 的 3/4、UART5 的 6。启用其他 DMA 外设时仍需为各通道分配独占资源。串口与 console、遥控、IMU、VOFA 不可复用同一设备。

## 公共传输契约

| 方法 | 行为 |
| --- | --- |
| `service(now_ms)` | 首次初始化，后续推进 I/O 和恢复；0 表示本地后端可服务，`-EAGAIN` 表示恢复等待，其他负值为错误。没有数据也可以返回 0 |
| `read(out)` | 成功复制最多 64 字节及接收时间；无数据 `-EAGAIN`；丢包/重组中断 `-EOVERFLOW`，Endpoint 会丢弃当前 V1/V2 半帧 |
| `send(bytes,size,timeout_ms)` | 复制整个 1～240 字节批次；默认总生命周期 60 ms，允许 1～1000 ms；忙返回 `-EAGAIN`；非法参数 `-EINVAL`，过大 `-EMSGSIZE`，未就绪 `-EACCES` |
| `txBusy()` | 后端仍持有待发批次或硬件传输，Endpoint 暂不生成下一批次 |
| `kind()` | 仅用于类型识别和诊断 |

`send()` 返回 0 表示后端接受批次，不表示对端收到或执行。Endpoint 只在接受后推进发送周期；背压时以后用最新业务值重新编码，不积压无限运动命令队列。发送周期是最早发送机会，不保证不同总线下都达到 UART 的实际更新率。

回调只复制原始数据和完成状态；解码、组包和恢复在通信线程中进行。UART 接收使用原始接收块时间；RS485 和 CAN 完整批次使用第一部分到达的时间，组包/排队不能刷新时间戳。溢出时先报告断流，再交付新代次数据。

`Snapshot.error` 提供最近服务结果，恢复等待保留此前错误；服务成功可以清除它。`online` 仍由心跳新鲜度确定，并非物理总线状态。应用继续检查各消息的 stamp 与 boot/generation。`Snapshot.transport / parser_stats / rejected_frames` 提供所选后端和 V1/V2 接收诊断；Parser CRC 计数不包含下层封装被丢弃的帧。

选择未编译进固件的后端返回 `-ENOTSUP`；三个后端本身已有实现。RS485 设备不支持硬件 DE 或不是 8N1 时也会返回 `-ENOTSUP`。

## UART

直接发送业务字节流；V1 帧与原 UART 板间固件的线格式兼容，启用大 Yaw 时同一流还包含独立 V2 帧。DMA 缓冲仍由 AsyncUart 独占；TX abort 通过新增的 `AsyncUart::txError()` 报告，busy 只由驱动完成/中止回调释放。

## RS485：单协调端轮询

一个协调端 Coordinator 周期发请求，Responder 只能在收到校验通过的请求后应答。没有业务数据时也发送空请求/空应答，让心跳和反馈始终有反向发送机会。发送器方向由 STM32 硬件 DE 控制，软件安排双方的访问时机。

传输封装为以下小端格式；payload 是零个或多个完整 V1/V2 业务帧，总长不超过 240 字节：

| 字节 | 字段 |
| --- | --- |
| 0～1 | `D3 91` |
| 2 | 版本 1 |
| 3 | 请求 1 / 应答 2 |
| 4～7 | 32 位请求 token；应答原样回显 |
| 8～9 | payload 长度，0～240 |
| 10～11 | 协调端授予的应答窗口，毫秒；应答回显 |
| 12～ | payload |
| 最后 2 字节 | 前面全部字节的 CRC16-CCITT，小端；同 V1 CRC 算法 |

默认应答窗口 20 ms，方向切换保护 1 ms，两次请求之间至少额外间隔 5 ms。协调端保留整个应答窗口，即使应答提前完成也不立即抢占。应答方根据原始接收时间扣除排队延迟和预计线上发送耗时，错过窗口就保持静默；不向下个请求期间延迟补发。

协调端启动后先静默约 1002 ms，覆盖上一次启动可能授出的最大 1000 ms 应答窗口。端点无需额外启动命令，之后自动交换心跳。默认 460800 下完整轮询往返约几十毫秒，实际速率受批次长度和线程调度影响。

两侧固件必须理解此封装。现有主动上报的 DM IMU RS485 协议不使用该封装，不能把这个板间后端直接替换到 IMU Source 中。第一版支持两节点硬件 DE 两线链路，不支持 GPIO DE、多从机寻址或 RS485 Modbus。

## CAN：完整批次分片

使用经典 CAN 数据帧，支持 11 位标准或 29 位扩展 ID；FD 控制器可运行经典模式，本实现不发送 CAN FD 帧。默认云台发送 `0x600`、接收 `0x601`；底盘相反。ID 需由实际系统独占分配。

完整批次先封装成 `版本1(1字节) + 长度LE16 + 业务字节批次 + CRC16LE`。CRC 覆盖版本、长度与批次。随后每个 CAN 帧最多承载 6 字节封装数据：

| CAN 数据字节 | 字段 |
| --- | --- |
| 0 | bit7 首片，bit6 末片，bit0～5 为从 0 开始的分片序号 |
| 1 | 8 位批次 token，每次接受新批次递增 |
| 2～7 | 最多 6 字节封装数据；最后一片用实际 DLC，不补到 8 字节 |

发送方一次只提交一个 CAN 帧，完成回调后才能提交下一片，避免控制器对同 ID 排队重排。接收使用 64 项原始帧队列，按 ID、帧格式、token、顺序、总长度、首尾标记、CRC 校验；只有完整批次通过才交给支持 V1/V2 的 Parser。重组默认超时 60 ms。错误、乱序、溢出和缺片中止当前重组，不交付部分控制指令。

轮询间隔影响发送吞吐：1 ms poll 下最大 240 字节批次需要 41 个分片，约几十毫秒。总线仲裁和线程延迟也计入 60 ms 发送期限；不要提高 poll 间隔后仍假设最大批次一定能按时发送。

后端独占整个 CAN 控制器，初始化要求控制器未启动；bus-off、发送超时或发送失败后停止控制器、淘汰旧批次、等待中止回调，再按默认 100 ms 重试恢复。正常 CAN 发送使用 `K_NO_WAIT` 和完成回调，不等 ACK；底层控制器 start/stop 仍可能有驱动自身的有限等待，不能把恢复操作当作硬实时无阻塞过程。

原板间通信样例默认采用 CAN3 与电机 CAN1 分离。大 Yaw 已占用底盘 CAN3 的组合使用 UART 板间通信；更换传输方式前须重新分配物理控制器。**同一控制器不能同时交给 motor::CanBus 和本后端**；启动状态检测不是跨模块所有权注册表。若需要共线，需要先统一控制器生命周期、路由、ID 和带宽分配。CAN 链路层 ACK 也不代表远端业务接受批次。

## 构建配置与上机

```conf
CONFIG_CPP=y
CONFIG_STD_CPP20=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_SKYWALKER_LIB_COMMUNICATION=y
CONFIG_SKYWALKER_INTERBOARD=y
CONFIG_ENTROPY_GENERATOR=y
CONFIG_SKYWALKER_INTERBOARD_ENDPOINT=y
# UART / RS485:
CONFIG_SERIAL=y
CONFIG_SKYWALKER_UART_TRANSPORT=y
CONFIG_SKYWALKER_INTERBOARD_RS485=y
# CAN:
CONFIG_CAN=y
CONFIG_SKYWALKER_INTERBOARD_CAN=y
```

只使用一种后端时可以关闭其余实现。UART/RS485 的 DMA 和熵源设备树也要可用。完整双板配置命令见[板间样例](../../../samples/communication/interboard/README.md)。

初次上机先用不连接电机的样例；断电接好所选总线后给两板上电，确认双方角色、速率、接口及端接。UART TX/RX 交叉；RS485 A/B 按板卡接口标记对应连接；CAN H/L 对应连接，保持必要的信号参考与正确终端电阻。MC02 CAN 收发器需要手册规定的供电。三类接口的电气信号不能直接混接。

本文描述源码实现与目标行为；没有进行实际双板收发、断线重连、示波器时序或电机联动验证。
