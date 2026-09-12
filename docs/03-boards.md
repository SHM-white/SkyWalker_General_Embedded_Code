# 03 板级支持

本框架支持两块板卡：达妙 **MC02**（STM32H723）与大疆 RoboMaster
**Type-C C 板**（STM32F407）。板卡目录位于 `boards/`，因为 module 声明了
`board_root: .`，构建时可直接用 `-b dm_mc02` / `-b rm_typec`。

---

## 1. 板卡速览

| 板卡 | board name | SoC | 时钟 | CAN | 惯性传感器 | 上位机串口 |
|---|---|---|---|---|---|---|
| 达妙 MC02 | `dm_mc02` / `dm_mc02/stm32h723xx` | STM32H723VG（Cortex-M7） | 480 MHz | FDCAN1/2/3 → `can1/can2/can3` | BMI088（SPI2） | `telemetry-uart = &usart1` |
| 大疆 C 板 | `rm_typec` | STM32F407IG（Cortex-M4） | 168 MHz | CAN1/CAN2 → `can1/can2` | BMI088（SPI1） | `telemetry-uart = &usart6` |

`board.yml` 登记的 SoC qualifier：`dm_mc02` → `stm32h723xx`，
`rm_typec` → `stm32f407xx`。写全 `dm_mc02/stm32h723xx` 最稳妥，省写
`dm_mc02` 也会自动解析。

---

## 2. 达妙 MC02（`boards/damiao/dm_mc02/`）

### 主要外设

| 外设 | 节点 | 引脚/说明 |
|---|---|---|
| 控制台 / shell | `usart10` | `zephyr,console`、`zephyr,shell-uart` |
| 遥测串口 | `usart1` | alias `telemetry-uart`（VOFA 用） |
| RS485 | `usart2`、`usart3` | 带 DE 引脚 `usart3_de_pb14` |
| 其他串口 | `uart5` | — |
| FDCAN | `fdcan1/2/3` | alias `can1/can2/can3`，均 1 Mbps 组网常用 |
| IMU | `bmi08x_accel` + `bmi08x_gyro` | SPI2，8 MHz，`spi2_sck_pb13/miso_pc2/mosi_pc1` |
| RGB | `rgb_led`（WS2812） | SPI6，TI 帧格式，alias `led-strip` |
| 用户按键 | `user_button` | PA15，alias `sw0`，`zephyr,button` |
| 电源使能 | `power1`/`power2` | XT30_1 = PC14，XT30_2 = PC13；均 `regulator-boot-off` |
| 加热 PWM | `timers3` 通道 4 | 由 `imu_test` overlay 打开，PB1 |

### Flash 分区

`dm_mc02.dts` 中 `flash0` 是 **1 MB 三分区**（MCUboot 预留）：

| 分区 | 偏移 | 大小 | 标签 |
|---|---|---|---|
| bootloader | `0x00000000` | 256 KB | `bootloader`（只读） |
| app | `0x00040000` | 512 KB | `image-0` |
| storage | `0x000c0000` | 256 KB | `storage`（`zephyr,storage-partition`） |

> `board.yml` / 文档里若出现 512 KB 的容量描述，仅作 Zephyr 元数据参考，
> **以设备树分区为准**。

### 默认配置要点（`dm_mc02_defconfig`）

- MPU、硬件栈保护（`CONFIG_ARM_MPU`、`CONFIG_HW_STACK_PROTECTION`）
- `CONFIG_DMA / GPIO / COUNTER / PWM / SERIAL / SPI / I2C / CAN / REGULATOR`
- `CONFIG_SENSOR` + `CONFIG_SENSOR_ASYNC_API` + `CONFIG_BMI08X`
- `CONFIG_LED_STRIP`（WS2812）
- `CONFIG_NOCACHE_MEMORY`（DMA 缓存一致性）
- 线程运行统计（板载状态灯显示 CPU 负载）

### 烧录

默认 runner 为 **openocd**（`runners.yaml: flash-runner: openocd`），
另有 pyocd / stm32cubeprogrammer / jlink：

```bash
west flash                         # 默认 openocd
west flash -r pyocd                # target=stm32h723vgtx
west flash -r stm32cubeprogrammer  # swd, sw reset
west flash -r jlink                # device=STM32H723VG
```

`boards/damiao/dm_mc02/support/` 提供 `openocd.cfg`、`openocd_stlink.cfg`、
`openocd_stlink_hla.cfg` 三套探针配置，用
`-DSKYWALKER_OPENOCD_PROBE=cmsis-dap|stlink|stlink-hla` 选择。

---

## 3. 大疆 C 板（`boards/rm_typec/`）

### 主要外设

| 外设 | 节点 | 说明 |
|---|---|---|
| 控制台 | `usart1` | alias `usart2`、`zephyr,console` |
| 遥测串口 | `usart6` | alias `usart1`、`telemetry-uart`（VOFA 用） |
| 其他串口 | `usart3` | — |
| CAN | `can1`、`can2` | `can1_rx_pd0/tx_pd1`、`can2_rx_pb5/tx_pb6`；`zephyr,canbus = &can2` |
| IMU | `bmi08x_accel` + `bmi08x_gyro` | SPI1，8 MHz |
| USB | `cdc_acm_uart0` | USB CDC-ACM |
| 音频 | `cs43l22` | I2C1 + I2S3 |
| 蜂鸣器 | `buzzer` | PWM4 CH3（PD14） |
| LED | `red/green/blue_led` | GPIO LED，alias `led0/1/2` |
| 按键 | gpio-keys | — |
| 温度 | `die_temp` | — |

> 注意 C 板上 **UART 别名与物理接口是错位的**：`usart1` alias 指向 `usart6`，
> `usart2` alias 指向 `usart1`。写样例时用 alias（如 `telemetry-uart`），
> 不要用物理编号猜。

### 默认配置要点（`rm_typec_defconfig`）

- MPU、硬件栈保护
- `CONFIG_USE_SEGGER_RTT`
- DMA/GPIO/COUNTER/PWM/SERIAL/SPI/I2C/CAN
- BMI088（`CONFIG_BMI08X`）+ sensor 异步 API
- USB device stack next + CDC-ACM

### 烧录

无板载调试器，`board.cmake` 接入 OpenOCD：

```bash
west flash -r openocd
west build -p -b rm_typec -DSKYWALKER_OPENOCD_PROBE=stlink samples/motor/dji_position_control
```

`support/openocd.cfg` 默认 CMSIS-DAP；`openocd_stlink.cfg` 用于较新
ST-Link 固件（`interface/stlink-dap.cfg` + `dapdirect_swd`），
`openocd_stlink_hla.cfg` 用于旧固件（`hla_swd`）。

> WSL2：ST-Link 必须 `usbipd attach --wsl` 透传，且透传不持久；
> 普通用户可能需 udev 规则或 `sudo chmod 666 /dev/bus/usb/*/*`。

---

## 4. 别名使用建议

设备树 alias 是应用与板卡解耦的关键，样例统一通过这些别名取设备：

| alias | dm_mc02 | rm_typec | 用途 |
|---|---|---|---|
| `telemetry-uart` | `usart1` | `usart6` | VOFA 上位机 |
| `led-strip` | `rgb_led` | —（无） | 状态灯 |
| `sw0` | `user_button` | gpio-keys | 用户按键 |
| `motor0` | 由样例 overlay 定义 | 同左 | 电机节点 |
| `can1/can2/can3` | FDCAN1/2/3 | CAN1/CAN2 | 电机总线 |

> **板级 `dts` 里没有 `motor0`**：电机 alias 由每个样例的 `app.overlay`
> 提供，因此不存在“板子自带电机配置”。

---

## 5. 构建示例

```bash
# MC02：IMU + VOFA（overlay 在 samples/imu_test/boards/dm_mc02.overlay）
west build -p -b dm_mc02/stm32h723xx -d build/imu_test samples/imu_test

# C 板：DJI 位置环
west build -p -b rm_typec -d build/dji_pos samples/motor/dji_position_control

# MC02：达妙 MIT 位置环
west build -p -b dm_mc02/stm32h723xx -d build/dm_pos samples/motor/dm_mit_position_control
```

---

## 6. 相关文档

- [01 快速开始](01-getting-started.md)
- [04 DJI 电机驱动](04-drivers-motor-dji.md)
- [05 达妙 DM 电机驱动](05-drivers-motor-dm.md)
- [06 IMU 与 EKF](06-drivers-imu.md)
