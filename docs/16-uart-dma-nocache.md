# STM32 UART DMA 的 nocache 报错：原因、修复与排查

本文针对本工作区 Zephyr 4.4.99 的 `uart_stm32` 驱动，以及 SkyWalker 的 `AsyncUart`。不同 Zephyr 版本可能采用不同缓存策略，排查时以实际参与构建的驱动为准。

## 1. 日志到底在说什么

```text
<inf> dr16_bench: online=0 rc=-11 seq=0 rx_age_ms=0 ...
<err> uart_stm32: Rx buffer should be placed in a nocache memory region
```

第二行是明确的内存配置错误：驱动拒绝使用交给它的 RX 缓冲区，此次 DMA 接收尚未启动。它不是串口线上收到坏帧后产生的错误，也不是遥控器未配对的直接证据。

本地驱动 `zephyr/drivers/serial/uart_stm32.c` 的三个入口都会检查缓冲区：

| 入口 | 用途 | 检查失败 |
|---|---|---|
| `uart_rx_enable()` | 开始接收，提交第一个 RX 缓冲区 | 打印 RX 报错并返回 `-EFAULT` |
| `uart_rx_buf_rsp()` | 在回调中提交下一个 RX 缓冲区 | 打印 RX 报错并返回 `-EFAULT` |
| `uart_tx()` | 提交异步发送缓冲区 | 打印 TX 报错并返回 `-EFAULT` |

检查由 `zephyr/soc/st/stm32/common/stm32_cache.c` 中的 `stm32_buf_in_nocache()` 完成，判断的是整个地址范围，而非只看指针是否对齐。关闭 `CONFIG_DCACHE` 时，本地头文件提供的检查实现直接返回 true。

DR16 日志中的 `rc` 是遥控服务 `snapshot()` 的返回值，`-11` 表示 `-EAGAIN`，即还没有可用快照；它不是 UART 初始化返回值。应同时看开机的 `DR16 physical UART init=...`。没有有效帧时，`online=0`、`seq=0`、`rx_age_ms=0` 并不表示刚刚收到了一帧。

## 2. 为什么 DMA 和 CPU 会看见不同的数据

CPU 开启数据缓存 D-cache 后，读写可能先落在缓存里；DMA 在独立的数据通路上访问内存，并不自动替 CPU 同步缓存。

```text
普通缓存内存：CPU <-> D-cache <-> RAM <-> DMA <-> UART
不可缓存内存：CPU <-----------> RAM <-> DMA <-> UART
```

RX 时，DMA 已把新字节写入 RAM，但 CPU 可能仍读取缓存中的旧字节。TX 时，CPU 写入的新字节可能还停留在缓存中，DMA 却从 RAM 取到旧内容。当前 STM32 UART 驱动采用缓冲区地址检查，提前拒绝不满足要求的存储。

Zephyr 提供 `CONFIG_NOCACHE_MEMORY` 和 `__nocache`，用于创建不可缓存区域并把指定变量放入其中。官方说明见 [Caching Basics](https://docs.zephyrproject.org/latest/hardware/cache/guide.html)。

### 本项目原来的具体问题

`AsyncUart` 原先把下列数组作为普通类成员：

```cpp
alignas(32) std::uint8_t rx_[2][128]{};
alignas(32) std::uint8_t tx_[256]{};
```

示例中的 `static AsyncUart uart(...)` 具有足够长的生命周期，但成员仍位于普通对象所在的内存区域。

- `static` 保证存活时间，不保证不可缓存。
- `alignas(32)` 保证地址对齐，不改变内存属性。
- `CONFIG_NOCACHE_MEMORY=y` 提供专用区域，不会自动迁移所有数组。
- DTS 的 `dmas` 选择硬件传输通路，不决定 C++ 对象放在哪里。
- 编译成功只能说明源码和链接规则成立；旧实现中的地址约束在运行时才被驱动检查。

达妙 H723 构建开启了 D-cache，所以触发检查。C 板 F407 没有同样的数据缓存条件，因此同一段代码可能在 C 板运行时不报这个错。这并不证明原来的缓冲区设计适合两种芯片。

### 为什么会反复出现，或者过一会儿才出现

`AsyncUart::init()` 注册回调后启动 RX；首次 RX 失败后，`service()` 会继续重试，后续重试间隔为 100 ms。固定的错误地址不会因重试自行变好，所以日志持续重复。

其他工程还可能只迁移了第一个 RX 缓冲区：启动正常，直到 `UART_RX_BUF_REQUEST` 换到普通内存里的第二个缓冲区才报错。若只修 RX，之后发送时仍可能遇到对应的 TX 报错。线程栈上的局部数组、普通堆分配、迁移板型或更换驱动版本也可能暴露原先未显现的问题；这些是排查方向，不能仅凭当前日志认定过去每次报错都是同一原因。

## 3. 本次代码修复

### 分离 DMA 数据与控制对象

`include/communication/async_uart.hpp` 新增 `AsyncUart::DmaBuffers`：

- 两个 128 字节 RX 缓冲区，供异步接收交替使用。
- 一个 256 字节 TX 缓冲区。
- 各数组保持 32 字节对齐，适用于当前两块板。
- 不包含消息队列、原子状态或 C++ 初始化逻辑。

`AsyncUart` 持有调用者提供的 `DmaBuffers&`，不再把 DMA 数组嵌在普通对象中。消息队列和状态仍放在普通内存。构造接口变更为：

```cpp
#include <communication/async_uart.hpp>

using skywalker::communication::AsyncUart;

// 静态存储，整个 DMA 数据对象进入 nocache 区域。
// 不添加 {} 或非零初始化器；该区域可能是 NOLOAD。
static AsyncUart::DmaBuffers dma_buffers __nocache;
static AsyncUart uart(DEVICE_DT_GET(DT_ALIAS(remote_uart)), dma_buffers);
```

`__nocache` 属性必须加在实际变量定义处，不能仅靠类型名或给非静态类成员加 section 属性。头文件已包含 Zephyr 的 section 属性定义。

每个 UART 实例必须使用自己独立的 `DmaBuffers`。云台应用的遥控器、裁判系统和板间串口各自分配一份，不能三路共享一个 DMA 数组。实例和缓冲区都必须存活至最后一次回调完成；当前应用使用静态对象，覆盖整个固件生命周期。同一个 UART 设备也不能同时交给多个 `AsyncUart` 管理。

### 首次 RX、续接 RX、TX 都改用专用存储

`lib/communication/async_uart.cpp` 同时更新：

1. `startRx()` 把 `dma_.rx[0]` 交给 `uart_rx_enable()`。
2. `UART_RX_BUF_REQUEST` 把空闲的 `dma_.rx[i]` 交给 `uart_rx_buf_rsp()`。
3. `UART_RX_BUF_RELEASED` 按实际地址释放占用状态。
4. `send()` 先复制有效发送字节到 `dma_.tx`，再提交 `uart_tx()`。

这些数组不依赖启动时清零：RX 只读取驱动报告有效的字节，TX 在发送前复制全部有效字节。NOLOAD 区域复位后的旧内容不会被当成有效包。

回调仍只复制接收数据并维护状态，业务线程仍调用 `service()`、`read()`、`send()`。构造函数不执行 I/O；初始化和接收重试语义保持原样。增加不可复制约束，防止复制控制对象后意外共用缓冲区及回调状态。

编译期断言要求：启用 D-cache 时必须同时启用 `CONFIG_NOCACHE_MEMORY`。达妙板已满足；C 板未开启 D-cache 时，`__nocache` 可以退化为空属性。该断言只能防止漏开配置，不能替代变量上的 `__nocache`。

已更新全部 8 个实例，覆盖 DR16、裁判系统、板间通信、command_safety、sentry_gimbal 和 sentry_chassis，以及通信文档的用法示例。外部代码若仍用单参数构造函数，需按上述方式迁移。

## 4. 不建议采用的替代办法

| 做法 | 局限 |
|---|---|
| 只加 `alignas(32)` | 对齐与缓存属性不同，驱动仍会拒绝 |
| 只打开 `CONFIG_NOCACHE_MEMORY` | 普通变量不会自动进入新段 |
| 手动 flush/invalidate 后继续传普通数组 | 当前驱动仍检查地址，缓存维护不能绕过这个 API 约束 |
| 把整个带状态和初始化逻辑的对象放进 NOLOAD 区 | 初始化与复位语义更复杂，并占用更多不可缓存内存 |
| 所有串口共享一个全局 RX/TX 数组 | 多路并发会覆盖数据，破坏驱动的缓冲区所有权 |
| 关闭全局 D-cache | 当前地址检查可能消失，但影响整体性能，并掩盖缓冲区归属问题 |
| 随便放到 DTCM/CCM | 不可缓存不等于所选 DMA 控制器可访问，必须核对芯片内存总线连接 |

本次保留 D-cache，也不修改 Zephyr 驱动或新增板型 overlay。

## 5. 重新构建与烧录

在仓库根目录、已激活工作区 Python 环境后执行：

```sh
west build -p always -b dm_mc02/stm32h723xx samples/communication/dr16 -d build/dr16_dm
west flash -d build/dr16_dm
```

C 板使用独立目录：

```sh
west build -p always -b rm_typec/stm32f407xx samples/communication/dr16 -d build/dr16_c
west flash -d build/dr16_c
```

`-p always` 会清理对应构建目录，确保没有残留的旧 overlay 或旧配置。烧录目录必须与刚构建的目录相同。

首次验证使用独立 DR16 示例，仅连接开发板和接收机即可，不需要电机。RX 接线为达妙 UART5/PD2、C 板 USART3/PC11；接收机供电、电平、共地及 DBUS 反相通路仍须符合实际硬件。

预期：初始化返回 0，不再打印 nocache 报错；遥控器已配对且信号正确时 `online=1`，`seq` 随有效帧增长。初始化成功但遥控器离线时，仍可能输出 `rc=-11`，不要把它误判为本问题未修复。

## 6. 若仍报错，按这个顺序查

1. **确认烧录产物**：是否烧了当前目录的 ELF/HEX，而非旧构建目录？查看开机初始化日志。
2. **区分 RX 与 TX**：首次 RX 就失败，查 `rx[0]`；运行一段时间后失败，查换缓冲区；发送时失败，查 `tx`。
3. **查看最终配置**：H723 的 `build/.../zephyr/.config` 应包含 `CONFIG_DCACHE=y`、`CONFIG_NOCACHE_MEMORY=y`，并启用 DMA 和 UART 异步 API。
4. **查看最终链接结果**：在 `zephyr.map` 查找 `.nocache` 和 `_nocache_ram_start` / `_nocache_ram_end`。实际 DMA 数组必须完整落在区间内，仅有段存在还不够。
5. **查看实际提交指针**：用调试器检查传给上述三个 UART API 的地址和长度，防止局部数组、临时对象或旧成员仍被提交。
6. **确认物理可达性**：如果修改过 linker、chosen SRAM 或 MPU，确认这个内存区域既不可缓存又能被该 DMA 访问。
7. **内存错误消失后再排串口**：检查最终 `zephyr.dts` 中 `remote-uart`、RX DMA、100000/8E1，以及接线与反相。不要通过调整遥控协议去掩盖明确的内存错误。

可用命令：

```sh
rg -n 'CONFIG_(DCACHE|NOCACHE_MEMORY|DMA|UART_ASYNC_API)=' build/dr16_dm/zephyr/.config
rg -n 'nocache|dma_buffers' build/dr16_dm/zephyr/zephyr.map
```

STM32 驱动地址检查失败的 `-EFAULT` 与缺少 RX DMA 时的 `-ENODEV` 是不同问题；对应排查内存和设备/DMA 配置。`uart_rx_enable()` 返回 `-EBUSY` 则表示接收已开启，也不应通过改内存段解决。

## 7. 验证范围与源码依据

本次执行结果见下方验证记录。编译与链接地址检查不能证明真实遥控帧已收到；未连接开发板执行烧录、运行或断线重连测试。

主要源码依据（相对 west 工作区）：

- `skywalker_code/include/communication/async_uart.hpp`
- `skywalker_code/lib/communication/async_uart.cpp`
- `zephyr/drivers/serial/uart_stm32.c`
- `zephyr/soc/st/stm32/common/stm32_cache.c`、`stm32_cache.h`
- `zephyr/include/zephyr/linker/section_tags.h`

这些本地文件决定当前工程的实际行为；在线最新文档仅辅助理解缓存机制，不能替代当前驱动源码。

### 本次验证记录

以下 8 组固件均编译、链接成功：

| 构建目标 | MC02 / STM32H723 | C 板 / STM32F407 |
|---|---|---|
| `samples/communication/dr16` | 通过 | 通过 |
| `samples/robotics/command_safety` | 通过 | 通过 |
| `samples/communication/referee` | 通过 | 未执行 |
| `samples/communication/interboard` | 通过 | 未执行 |
| `applications/sentry_gimbal` | 通过 | 未执行 |
| `applications/sentry_chassis` | 通过 | 未执行 |

本次 MC02 DR16 的 `zephyr.map` 显示：DMA 存储对象占 `0x200`（512）字节，起始地址 `0x24000000`；不可缓存区域为 `[0x24000000, 0x24000400)`，整个对象位于其中。RX 两块数组及 TX 数组均包含在这一对象内。地址会随应用和链接布局变化，不应在代码中硬编码该地址。

固件运行入口需要真实开发板；本环境未执行上板运行，因此最终仍需按第 5 节确认初始化日志和有效遥控帧。
