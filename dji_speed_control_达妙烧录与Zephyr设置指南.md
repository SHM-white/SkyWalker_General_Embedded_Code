# `dji_speed_control` 烧录与 Zephyr 设置指南

## 1. 本次检查和烧录结果

目标是达妙 MC02 开发板，板上 MCU 为 STM32H723VG，工程样例为：

```text
samples/motor/dji_speed_control
```

本次使用当前工作树中的实际文件构建，没有替换或修改仓库源码。工作树在操作前已经存在未提交修改，因此本次镜像包含这些现有修改，尤其包括：

- `samples/motor/dji_speed_control/prj.conf`
- `samples/motor/dji_speed_control/app.overlay`
- `samples/motor/dji_speed_control/src/main.cpp`
- `boards/damiao/dm_mc02/dm_mc02.dts`
- VOFA 头文件和实现

已完成的检查：

| 项目       | 结果                                             |
| ---------- | ------------------------------------------------ |
| Zephyr     | 4.4.99，revision`6085aade…`                   |
| west       | v1.5.0，位于工作区`.venv/bin/west`             |
| 编译器     | Zephyr SDK 1.0.1                                 |
| 构建目标   | `dm_mc02`                                      |
| 镜像构建   | 成功，无编译错误                                 |
| Flash 占用 | 112652 B，约 10.74%                              |
| 调试器     | ST-LINK V2J37S7                                  |
| MCU 探测   | STM32H723，目标电压约 4.73 V                     |
| 烧录       | OpenOCD 写入成功，Verify 成功                    |
| 烧录后状态 | CPU 保持`reset halt`，没有自动运行电机控制程序 |

镜像临时构建目录是：

```text
/tmp/skywalker-dji-speed.XX6Fro
```

这个临时目录不属于仓库；仓库内没有因为本次构建产生新的 build 文件。

## 2. 当前程序的数据流

```text
GM6020
  │ CAN1，标准帧，1 Mbit/s
  ▼
STM32H723 FDCAN1
  │ Zephyr CAN API
  ▼
DJI motor driver
  │ 反馈：角度、转速、电流、温度
  ▼
5 ms 速度环
  │ 斜坡参考 → 复合前馈 PID → 电流限幅
  ▼
DJI command frame
  │
  ├── CAN1 → GM6020
  └── USART1 → VOFA+ JustFloat，同时也是当前 Zephyr console
```

当前应用的关键行为不是“上电后等待按键”：

1. 等待电机反馈变为新鲜状态。
2. 自动 `arm()` 并发送零电流帧。
3. 进入 5 ms 控制周期。
4. 当前代码生成最大约 `±20 rad/s` 的正弦速度请求。
5. 当前软件电流限幅是 `±3 A`，overlay 中设备边界也是 `3000 mA`。
6. 运行约 300 s 后发送零电流并停止。

因此，在电机供电、CAN 反馈和机械负载都连接时，不能把“烧录完成”理解成“可以直接运行”。第一次运行必须让电机悬空，并准备断电或急停。

## 3. 必须先解决的 Flash 布局问题

当前板级 DTS 声明的 Flash 分区是：

```text
0x000000 - 0x040000  bootloader  256 KiB
0x040000 - 0x0C0000  image-0     512 KiB
0x0C0000 - 0x100000  storage     256 KiB
```

但本次 `dji_speed_control` 最终配置显示：

```text
CONFIG_FLASH_LOAD_OFFSET=0
CONFIG_ROM_START_OFFSET=0x0
# CONFIG_BOOTLOADER_MCUBOOT is not set
```

ELF 的第一个 Flash 段从 `0x08000000` 开始，OpenOCD 也确实把镜像写到了 `0x08000000`。这是一种“裸 Zephyr 直写”模式，不是 MCUboot 的 `image-0` 应用模式。

这里必须二选一，不能混用。

### 方案 A：裸 Zephyr 直写

适合板上没有需要保留的 MCUboot，或者你明确决定让 Zephyr 从 Flash 起始地址启动。

- 保持 `CONFIG_FLASH_LOAD_OFFSET=0`。
- 保持 `CONFIG_ROM_START_OFFSET=0x0`。
- 使用 `west flash --runner openocd` 直接写 `0x08000000`。
- 不要把 DTS 中的 `boot_partition` 当成真实可用的 MCUboot 保留区。
- 最好由用户后续手工整理 DTS 分区定义，避免分区声明和实际启动方式不一致。

本次烧录就是这个方案。因此，如果这块板原来确实有 MCUboot，本次直写可能已经覆盖了 `0x08000000–0x08040000` 的原 bootloader；仓库中没有发现原 bootloader 备份，不能假设它仍然存在。

### 方案 B：MCUboot 加载应用

适合需要 OTA、回滚或保留 bootloader 的正式部署。

应用 overlay 至少要把代码分区选出来：

```dts
/ {
    chosen {
        zephyr,code-partition = &app_partition;
    };
};
```

应用配置需要按项目的 MCUboot/sysbuild 方案启用 bootloader，例如：

```conf
CONFIG_BOOTLOADER_MCUBOOT=y
CONFIG_USE_DT_CODE_PARTITION=y
```

然后重新构建并检查最终配置，必须看到应用链接位置与 `image-0` 一致，典型结果应接近：

```text
CONFIG_FLASH_LOAD_OFFSET=0x40000
```

并且 ELF 的 Flash 段应从 `0x08040000` 开始。若仍从 `0x08000000` 开始，不能烧录为 MCUboot 应用。

MCUboot 模式还需要单独准备并烧录匹配的 bootloader、签名配置和应用镜像；当前仓库没有发现已经配置好的 `dm_mc02` sysbuild/签名流程，所以不要只添加一行 `CONFIG_BOOTLOADER_MCUBOOT=y` 就直接烧录。先恢复或重新构建 bootloader，再验证应用的实际地址。

## 4. `prj.conf` 中需要的 Zephyr 设置

当前样例已经能编译通过，下面是按功能整理的设置。`y` 表示打开，数值必须依据实际硬件和控制目标调整。

### C++ 和控制算法

```conf
CONFIG_CPP=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_SKYWALKER_LIB_CONTROL=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
```

C++20 主程序和 DJI C++ 驱动依赖前两项；控制库和电机驱动依赖后两项。关闭其中任何一项都可能导致链接缺少 C++ 运行库或驱动没有进入构建。

### CAN

```conf
CONFIG_CAN=y
```

`CONFIG_CAN=y` 只负责启用 Zephyr CAN 子系统。实际使用哪一路和波特率由设备树决定；当前 overlay 指向 `&can1`，板级 DTS 将 CAN1 配置为 1 Mbit/s。

必须同时满足：

- 电机实际 CAN ID 与 `motor-id` 相同。
- 电机和 MC02 使用相同的 1 Mbit/s 仲裁速率。
- CANH、CANL 没有接反。
- 总线两端有正确的 120 Ω 终端，不能在每个节点都随意并联终端。
- 电机电源、CAN 收发器电源和 MC02 共地。

### 日志和串口

```conf
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_SERIAL=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
```

板级 defconfig 已经打开了 UART、DMA 和 UART 日志后端，样例中的配置用于确保应用日志可用。

### VOFA+ 异步发送

当前工作树中的 `main.cpp` 调用了 VOFA+ 发送接口，因此必须保留：

```conf
CONFIG_SKYWALKER_LIB_VOFA=y
CONFIG_UART_ASYNC_API=y
```

当前每个 5 ms 周期发送 10 个 `float`，即 200 Hz。115200 baud 的串口带宽不足以稳定承载“每周期 10 个 float 加帧尾”再叠加大量文本日志。建议先把 telemetry 降到 20–50 Hz，或者提高专用 VOFA 串口的波特率；这属于应用代码/设备树调整，不是单独打开 Kconfig 就能解决的问题。

还有一个更重要的串口冲突：

- 当前板级 `chosen.zephyr,console` 是 `usart1`。
- 当前 `main.cpp` 也把 `usart1` 传给 VOFA。
- 因此文本日志和二进制 JustFloat 数据会混在同一物理串口中。

推荐保留 USART1 做 console，把 VOFA 改用已启用的 USART10：

```cpp
const struct device *vofa_uart = DEVICE_DT_GET(DT_NODELABEL(usart10));
```

如果你决定暂时不用 VOFA，则应同时从应用代码移除 VOFA 调用，再移除 `CONFIG_SKYWALKER_LIB_VOFA` 和 `CONFIG_UART_ASYNC_API`；只删配置、不删 C 代码会导致链接失败。

### 控制周期、反馈和命令超时

当前配置为：

```conf
CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS=20
CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS=20
```

主循环目标周期是 5 ms。20 ms 意味着连续约四个周期没有有效反馈，驱动就会进入失效路径并发送零命令。这个安全策略可以保留；如果系统日志或 VOFA 发送造成周期抖动，不要盲目增大超时，应先降低日志/遥测频率并确认 CAN 中断正常。

### 建议补充的栈和调试设置

当前镜像使用了默认 `CONFIG_MAIN_STACK_SIZE=1024`，编译没有失败，但 C++、日志格式化和 VOFA 同时启用时栈余量需要实际测量。建议用户在手工调整时先尝试：

```conf
CONFIG_MAIN_STACK_SIZE=8192
```

当前已有：

```conf
CONFIG_DEBUG_OPTIMIZATIONS=y
```

台架调试可以保留；需要更稳定的控制周期时，应在确认日志和调试功能不再影响时序后再选择优化级别，并通过最终 `.config` 和 ELF 重新确认。

## 5. `app.overlay` 中必须核对的设置

当前应用 overlay 的核心内容是：

```dts
/ {
    aliases {
        motor0 = &gm6020_7;
    };

    gm6020_7: motor-7 {
        compatible = "dji,gm6020-current";
        status = "okay";
        can-bus = <&can1>;
        motor-id = <4>;
        current-limit-ma = <3000>;
        gear-ratio-num = <1>;
        gear-ratio-den = <1>;
        current-loop-confirmed;
    };
};
```

需要按真实电机逐项修改：

- `motor0`：必须存在，因为主程序通过 `DT_ALIAS(motor0)` 获取电机。
- `compatible`：只有确实是 GM6020 电流环协议时才使用这个 binding。
- `can-bus`：当前是 `&can1`，必须和实际接线一致。
- `motor-id`：必须填电机实际 ID，不是随意使用节点名中的数字。
- `current-limit-ma`：是驱动设备边界；不能高于硬件和机械允许值。
- `gear-ratio-num/den`：直接驱动填 1/1；有减速机构时按实际输出轴关系填写。
- `current-loop-confirmed`：只有电机固件和配置确实已启用当前驱动所需的电流环时才保留。

第一次带电测试时，建议把 overlay 的 `current-limit-ma` 和 `main.cpp` 的软件电流上限一起降到很小的实验值，二者必须一致。当前镜像的 `3000 mA` 不是“低风险首次测试值”。

## 6. XT30 电源使能的现状

板级 DTS 中两个 XT30 regulator 默认都是：

```dts
regulator-boot-off;
```

这表示系统启动时不会自动打开它们。当前 `dji_speed_control` 没有调用 Zephyr regulator API，因此如果电机电源来自 MC02 的 XT30，程序本身不会自动给 XT30 上电。不要为了让电机“有反馈”而直接删除 `regulator-boot-off`；应先确认电源路径、急停和负载状态，再决定是否由应用显式控制 `power1` 或 `power2`。

## 7. 推荐的用户侧构建检查

以下命令是给用户后续在工作区执行的，不要在未确认 Flash 模式前直接运行烧录命令。

```bash
cd /home/shm-white/skywalker_ws/skywalker_code

../.venv/bin/west build \
  -d build/dji_speed_control \
  -b dm_mc02 \
  samples/motor/dji_speed_control
```

检查最终 Kconfig 和设备树：

```bash
rg '^CONFIG_(CPP|REQUIRES_FULL_LIBCPP|CAN|LOG|SERIAL|CONSOLE|UART_CONSOLE|UART_ASYNC_API|SKYWALKER|MAIN_STACK_SIZE|FLASH_LOAD_OFFSET|ROM_START_OFFSET|BOOTLOADER_MCUBOOT|USE_DT_CODE_PARTITION)=' \
  build/dji_speed_control/zephyr/.config

rg -n 'motor0|gm6020_7|can1|bitrate|motor-id|current-limit-ma|zephyr,code-partition' \
  build/dji_speed_control/zephyr/zephyr.dts

readelf -l build/dji_speed_control/zephyr/zephyr.elf | sed -n '1,80p'
```

直写模式应确认第一个 Flash 段为 `0x08000000`；MCUboot 模式应确认它位于 `0x08040000` 附近。地址不符合预期时停止，不要执行 `west flash`。

## 8. 烧录命令和安全上电顺序

### 裸 Zephyr 直写模式

确认已经选择方案 A 后，使用：

```bash
../.venv/bin/west flash \
  -d build/dji_speed_control \
  --runner openocd
```

也可以使用仓库已有的 `board.cmake` 默认 runner，但当前环境没有 pyOCD，所以必须显式使用 OpenOCD。

### 建议的上电顺序

1. 断开电机功率电源，或拔掉电机相应 XT30；机械输出轴悬空。
2. 连接 ST-LINK 的 SWD、GND 和目标板供电。
3. 先构建并检查最终 Flash 地址。
4. 烧录后先保持复位或 halt，确认镜像没有写错分区。
5. 连接 USART1 console，115200 baud、8N1；若 VOFA 改到 USART10，另接对应串口。
6. 确认急停、断电手段和 CAN 终端电阻。
7. 先用很小电流限幅运行，观察日志中的 motor ID、反馈新鲜度和 CAN 错误。
8. 只有反馈稳定且方向确认后，才逐步提高速度目标和电流限幅。

当前程序没有控制台 arm 口令，而是收到新鲜反馈后自动 arm。因此最后两步不能省略。

## 9. 常见故障定位

### 日志显示 `no fresh feedback`

按顺序检查：

1. GM6020 是否真的上电。
2. XT30 是否仍处于 boot-off。
3. CANH/CANL 是否接反。
4. CAN1 的 PD0/PD1 接线是否与板卡原理图一致。
5. 电机 ID 是否与 overlay 的 `motor-id = <4>` 一致。
6. 电机端和 Zephyr 的 CAN 速率是否都是 1 Mbit/s。
7. 总线上是否有 120 Ω 终端。
8. 是否误把 CAN2 或 CAN3 接到了 CAN1 配置。

### 能看到日志，但 VOFA 曲线乱码

这是 USART1 同时承载 console 文本和 JustFloat 二进制数据的预期风险。把 VOFA 改到 USART10，或者暂时关闭文本日志/降低遥测频率；不要把乱码当成 CAN 故障。

### 一烧录就无法启动原来的 MCUboot

检查 ELF 是否仍从 `0x08000000` 开始。当前工程的镜像确实按裸 Zephyr 模式写过这个地址；如果产品必须保留 MCUboot，需要先恢复匹配的 bootloader，再按第 3 节方案 B 重新配置并构建应用。

### 电机方向反了或出现抖动

先断电，不要通过增大 `kd` 或电流限幅硬压问题。确认：

- CAN 反馈速度正负号。
- `kRequestedVelocityRadS` 的符号。
- 电机机械方向和减速比。
- PID 的单位是 A、rad/s 和 s，而不是 rpm、mA 混用。
- 应用软件限幅与 overlay `current-limit-ma` 一致。

## 10. 最终检查清单

- [ ] 已决定采用裸 Zephyr 直写还是 MCUboot。
- [ ] 最终 ELF 的 Flash 地址与所选启动方式一致。
- [ ] 若采用 MCUboot，bootloader 已恢复/构建，且应用从 `image-0` 链接。
- [ ] `CONFIG_CPP`、完整 libc++、CAN、DJI 驱动和控制库已启用。
- [ ] 使用 VOFA 时启用 `CONFIG_SKYWALKER_LIB_VOFA` 和 `CONFIG_UART_ASYNC_API`。
- [ ] console 与 VOFA 没有共用同一个物理 UART，或已接受数据混流。
- [ ] `can-bus`、CAN 速率、motor ID、当前环状态和减速比已实测核对。
- [ ] 电机功率已断开，机械负载悬空，急停可用。
- [ ] 首次测试已把软件和设备电流限幅降到安全值。
- [ ] 烧录后先检查日志和反馈，再释放复位运行。
- [ ] 已确认本次操作没有再修改仓库业务源码；需要改动的地方由用户手工完成并重新构建。
