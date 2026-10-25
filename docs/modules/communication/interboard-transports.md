# 板间 UART、RS485 与 CAN

更新日期：2026-10-25。库中的三种后端均已实现；当前两个最终应用固定使用 UART。`InterBoardEndpoint` 保留 `submit / setReferee / setStatus / snapshot / poll`；构造时绑定 `InterBoardTransport`，此后不更换后端。当前统一 v4 业务帧保留 CRC、通信/生产者序号、boot_id 和原始输入有效期；恢复授权 generation 已删除。

业务帧的完整偏移、枚举、物理量、停止与重启合同见[板间协议 v4](../../protocols/interboard-v4.md)。本页主要说明业务帧之外的传输封装和设备所有权。

## 初始化选择

```cpp
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
using namespace skywalker;

static communication::AsyncUart::DmaBuffers dma __nocache;
static const communication::ConfiguredInterBoardTransport::Config config{
    .kind = communication::InterBoardTransportKind::Uart,
    .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart)),
};
static communication::ConfiguredInterBoardTransport transport(config, &dma);
static communication::InterBoardEndpoint link(
    transport, {robotics::BoardRole::GimbalController, 100, 100, 60});
```

底盘端使用 `ChassisController`。业务线程提交值、复制快照；`poll()` 由唯一所有者持续调用，包括故障期间。它可以由独立通信线程调用，也可以像当前应用一样放在本板主循环中：最终云台周期 5 ms，最终底盘周期 2 ms。需要评估可选 CAN 分片吞吐时，可采用约 1 ms 的专用通信周期，不能把建议周期当成当前所有应用的实际时序。

库级的其他选择：RS485 需要 `kind=Rs485` 并填写 `rs485.uart` 与 Coordinator/Responder 角色；CAN 需要 `kind=Can` 并填写 `can.can`、tx_id/rx_id，另一端交换 TX/RX ID。两板必须选择相同传输封装。当前实车共享 overlay 只承诺已固定的资源映射，不提供一组可随意切换的三后端别名。

执行线程在每次真实生产时填 `RunStatus.stamp={now_ms, sequence, true}` 再调用 setStatus。Config 第五个字段 `status_timeout_ms` 默认 100 ms；发送心跳和底盘反馈前检查该年龄。状态无效、过期或来自未来时间时，就绪和使能诊断撤销并报告 FeedbackStale，心跳依然正常发送。setStatus 与 poll 都不重盖执行生产时间；停止执行线程时，通信线程不能继续宣布旧 Ready/Active。默认无状态生产者的 endpoint 会报告未就绪。

`ConfiguredInterBoardTransport` 通过 `std::variant` 在对象内部构造一个后端，不分配堆内存，不启动未选择的设备。也可以直接构造 `UartInterBoardTransport`、`Rs485InterBoardTransport` 或 `CanInterBoardTransport` 并传给 Endpoint。纯 CAN 构造不需要传 DMA 指针。对象不能在运行中移动、复制或销毁。

[云台应用](../../../applications/sentry_gimbal/README.md)与[底盘应用](../../../applications/sentry_chassis/README.md)各自在自己的 `src/main.cpp` 中显式构造 UART transport 和 Endpoint；不调用公共 `vehicle_bench.hpp::linkConfig()`。其 [installed 共享物理配置](../../../config/vehicle/installed/README.md)固定云台 `interboard-uart=&usart10`、底盘 `interboard-uart=&usart1`，均为 460800 8N1。历史 `vehicle_integration` 公共运行体和[通用板间样例](../../../samples/communication/interboard/README.md)仍可描述多后端构造，但它们的宏、额外 conf 和别名布局不是最终应用的现场入口。

## 统一 v4 业务协议

所有消息统一帧版本 4，不支持旧版本探测、解码或占位 generation。Heartbeat/ChassisControl/BigYawRequest/BigYawFeedback 的载荷分别为 20/32/36/24 字节；其余消息载荷不变。帧头、消息 ID、小端和 CRC16 沿用。

删除 resume_generation。两板使用同批 v4 固件，Endpoint 按实际 boot/link 建立输入上下文；电机恢复不建立新授权边界。库级 BigYaw 发送不依赖反馈 ready/armed；valid 表示实际速度测量是否有效。独立双 Yaw 上车入口对非零速度另加真实反馈新鲜度条件，缺少测量时最多发已授权零速 FollowCenter，这属于阶段策略。

发送保留原生产年龄，重发同一 producer sequence 不能刷新控制命令缓存年龄。frame sequence 仅是通信发送次数，不是目标生产次数。状态停止生产后，不再宣告旧 ready/armed，但反馈状态不能反向停止新的目标发送。

`BigYawRequest` 在传输可用时按约10 ms周期发送。对端离线、操作授权停止/急停/过期、请求过期或重连后尚无新生产请求时，发送端改发 `Disabled + 0`，仍保留原生产序号与累计年龄。接收端允许更新的 frame sequence 将同一 producer sequence 从运行降为停止，不更新缓存时间戳，也不允许相同 producer sequence 恢复运行。链路故障时发送不保证送达，接收端超时和急停保护继续生效。

完整字节偏移、年龄与停止语义以[协议 v4 契约](../../protocols/interboard-v4.md)为准；旧[实施指南](../../dev/电机持续指令与独立自动恢复重构实施指南.md)保留迁移背景。实车先运行[独立板间观察入口](../../../samples/bringup/README.md)，通用多后端实验见[板间样例](../../../samples/communication/interboard/README.md)。

## 实际硬件路径

当前应用与上车入口使用如下代码基线，DAPLink 实际接线仍需现场确认：

| 板卡与职责 | 外设与引脚 | 配置与所有者 |
| --- | --- | --- |
| 云台板间 | USART10，PE3 TX / PE2 RX | 460800 8N1，Endpoint transport |
| 底盘板间 | USART1，PA9 TX / PA10 RX | 460800 8N1，Endpoint transport |
| 两板诊断 | UART7，PE8 TX / PE7 RX | 115200 8N1，统一 Diagnostics，TX 接 DAPLink RX 并共地 |
| 云台头部 IMU | CAN3 | 1 Mbps，DmImuCanSource 独占；不是板间 CAN |
| 底盘电机 | CAN1 / CAN2 / CAN3 | 舵向 / 轮驱 / 大 Yaw，各由一个 motor::CanBus 拥有 |

云台 USART10 使用 DMAMUX 3(TX request 119)、4(RX request 118)，共享 overlay 让 USART1 释放这两路 DMA；底盘 USART1 保留自己的 3/4 通道。SPI2 的 DMA1/2 和遥控 UART5 的 DMA6 不能被挪给其他启动对象。UART7 已给诊断独占，不能同时启动旧视觉接收器、console、VOFA 或 AsyncUart 接收者。Log/VOFA 均通过共享 `diagnostic_output` 代码变量选择，见[上车诊断说明](../../../samples/bringup/README.md)。

库的 RS485/CAN 示例另有设备布局：[通用板间样例 overlay](../../../samples/communication/interboard/app.overlay) 将 RS485 映射到 USART2（PD5 TX / PD6 RX / PD4 DE，经板载收发器到 A/B），460800 8N1；其 DMAMUX 为 0(TX request 44)、5(RX request 43)。CAN 示例使用 FDCAN3、经典 CAN 1 Mbps。这些候选资源必须按所选工程核对，不能覆盖实车 CAN3 IMU/电机所有者或复制到其他 overlay 后直接启用。

RS485 与普通 UART 使用 AsyncUart，CAN 使用 Zephyr CAN API；电气形式和驱动类型必须匹配。串口与 console、遥控、IMU、诊断不可重复拥有同一设备。

## 公共传输契约

| 方法 | 行为 |
| --- | --- |
| `service(now_ms)` | 首次初始化，后续推进 I/O 和恢复；0 表示本地后端可服务，`-EAGAIN` 表示恢复等待，其他负值为错误。没有数据也可以返回 0 |
| `read(out)` | 成功复制最多 64 字节及接收时间；无数据 `-EAGAIN`；丢包/重组中断 `-EOVERFLOW`，Endpoint 会丢弃当前 v4 半帧 |
| `send(bytes,size,timeout_ms)` | 复制整个 1～240 字节批次；默认总生命周期 60 ms，允许 1～1000 ms；忙返回 `-EAGAIN`；非法参数 `-EINVAL`，过大 `-EMSGSIZE`，未就绪 `-EACCES` |
| `txBusy()` | 后端仍持有待发批次或硬件传输，Endpoint 暂不生成下一批次 |
| `kind()` | 仅用于类型识别和诊断 |

`send()` 返回 0 表示后端接受批次，不表示对端收到或执行。Endpoint 只在接受后推进发送周期；背压时以后用最新业务值重新编码，不积压无限运动命令队列。发送周期是最早发送机会，不保证不同总线下都达到 UART 的实际更新率。

回调只复制原始数据和完成状态；解码、组包和恢复在通信线程中进行。UART 接收使用原始接收块时间；RS485 和 CAN 完整批次使用第一部分到达的时间，组包/排队不能刷新时间戳。溢出时先报告断流，再交付新代次数据。

`Snapshot.error` 提供最近服务结果，恢复等待保留此前错误；服务成功可以清除它。`online` 仍由心跳新鲜度确定，并非物理总线状态。应用继续检查各消息的 stamp、boot 和原始输入年龄。`Snapshot.transport / parser_stats / rejected_frames` 提供所选后端和 v4 接收诊断；Parser CRC 计数不包含下层封装被丢弃的帧。

选择未编译进固件的后端返回 `-ENOTSUP`；三个后端本身已有实现。RS485 设备不支持硬件 DE 或不是 8N1 时也会返回 `-ENOTSUP`。

## UART

直接发送业务字节流；所有业务帧均使用 v4，旧固件明确拒绝。DMA 缓冲仍由 AsyncUart 独占；TX abort 通过新增的 `AsyncUart::txError()` 报告，busy 只由驱动完成/中止回调释放。

## RS485：单协调端轮询

一个协调端 Coordinator 周期发请求，Responder 只能在收到校验通过的请求后应答。没有业务数据时也发送空请求/空应答，让心跳和反馈始终有反向发送机会。发送器方向由 STM32 硬件 DE 控制，软件安排双方的访问时机。

传输封装为以下小端格式；payload 是零个或多个完整 v4 业务帧，总长不超过 240 字节：

| 字节 | 字段 |
| --- | --- |
| 0～1 | `D3 91` |
| 2 | 版本 1 |
| 3 | 请求 1 / 应答 2 |
| 4～7 | 32 位请求 token；应答原样回显 |
| 8～9 | payload 长度，0～240 |
| 10～11 | 协调端授予的应答窗口，毫秒；应答回显 |
| 12～ | payload |
| 最后 2 字节 | 前面全部字节的 CRC16-CCITT，小端；沿用现有 CRC 算法 |

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

发送方一次只提交一个 CAN 帧，完成回调后才能提交下一片，避免控制器对同 ID 排队重排。接收使用 64 项原始帧队列，按 ID、帧格式、token、顺序、总长度、首尾标记、CRC 校验；只有完整批次通过才交给支持 v4 的 Parser。重组默认超时 60 ms。错误、乱序、溢出和缺片中止当前重组，不交付部分控制指令。

轮询间隔影响发送吞吐：1 ms poll 下最大 240 字节批次需要 41 个分片，约几十毫秒。总线仲裁和线程延迟也计入 60 ms 发送期限；不要提高 poll 间隔后仍假设最大批次一定能按时发送。

后端独占整个 CAN 控制器，初始化要求控制器未启动；bus-off、发送超时或发送失败后停止控制器、淘汰旧批次、等待中止回调，再按默认 100 ms 重试恢复。正常 CAN 发送使用 `K_NO_WAIT` 和完成回调，不等 ACK；底层控制器 start/stop 仍可能有驱动自身的有限等待，不能把恢复操作当作硬实时无阻塞过程。

通用板间通信样例提供 CAN3 与电机 CAN1 分离的实验布局。当前底盘应用的 CAN1 舵向、CAN2 轮驱和 CAN3 大 Yaw 已占满三路控制器，云台 CAN3 则由头部 IMU 独占；两应用因此固定使用各自共享 overlay 中的板间 UART。更换传输方式需要先重新分配物理控制器和相应独立入口，不能只切一个库配置字段。**同一控制器不能同时交给 motor::CanBus 和本后端**；启动状态检测不是跨模块所有权注册表。若需要共线，需要先统一控制器生命周期、路由、ID 和带宽分配。CAN 链路层 ACK 也不代表远端业务接受批次。

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

上述 Kconfig 是库级可选后端的能力清单，不要求当前 UART 应用全部开启。只使用一种后端时可以关闭其余实现；UART/RS485 的 DMA 和熵源设备树也要可用。实车操作者通过[独立工程目录](../../../samples/bringup/README.md)选择阶段，每项使用固定 build 目录，不叠加旧角色/场景 conf。通用三后端实验的配置命令仍见[板间样例](../../../samples/communication/interboard/README.md)。

初次上机先用不连接电机的样例；断电接好所选总线后给两板上电，确认双方角色、速率、接口及端接。UART TX/RX 交叉；RS485 A/B 按板卡接口标记对应连接；CAN H/L 对应连接，保持必要的信号参考与正确终端电阻。MC02 CAN 收发器需要手册规定的供电。三类接口的电气信号不能直接混接。

本文描述源码实现与目标行为；没有进行实际双板收发、断线重连、示波器时序或电机联动验证。
