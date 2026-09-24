# 03 板级支持

## 1. 板卡速览

| board | SoC | RAM / Flash metadata | 主要外设 | 默认烧录 |
|---|---|---:|---|---|
| `dm_mc02/stm32h723xx` | STM32H723 | 192 KB / 512 KB | FDCAN1/2/3、BMI088 SPI2、WS2812 SPI6、USART/RS485、UART5、USART10 | OpenOCD |
| `rm_typec` | STM32F407 | 128 KB / 1024 KB | CAN1/2、BMI088、USB CDC-ACM、USART1/3/6、音频、蜂鸣器 | OpenOCD + 外部探针 |

MCU 频率和引脚以各自 DTS 为准。表中的 RAM/Flash 是 `board.yml` 元数据；实际链接布局还应查看生成的 linker map 和 build 输出。

## 2. 达妙 MC02

源码位置：`boards/damiao/dm_mc02/`。

已在 DTS 中打开或定义的资源包括：

- STM32H723，CPU 时钟 480 MHz。
- `can1`、`can2`、`can3`，默认 1 Mbps。
- BMI088 加速度计和陀螺仪挂在 SPI2。
- SPI6 上的 WS2812。
- USART1（默认 telemetry/通用 UART）、USART2/3 RS485、UART5 遥控输入、USART10 console/shell。
- XT30_1 / XT30_2 固定电源 regulator，默认关闭，由应用显式控制。
- 用户按键、RGB LED、rng、backup SRAM 等。

MC02 的 `board.cmake` 默认选择 OpenOCD，可用：

```bash
west build -p -b dm_mc02/stm32h723xx \
  -d build/app \
  -DSKYWALKER_OPENOCD_PROBE=cmsis-dap \
  samples/hello
west flash -d build/app
```

允许的 probe 值是 `cmsis-dap`、`stlink`、`stlink-hla`。也注册了 pyOCD、STM32CubeProgrammer、ST-Link GDB server 和 J-Link runner，具体 runner 参数看 `boards/damiao/dm_mc02/board.cmake`。

## 3. RoboMaster Type-C C 板

源码位置：`boards/rm_typec/`。

- STM32F407，CPU 时钟 168 MHz。
- CAN1 和 CAN2 默认 1 Mbps。
- BMI088、USB FS CDC-ACM、USART1/3/6；USART6 是 DTS 中的 console/telemetry alias，USART3 默认 100000 baud，可用于遥控/串行设备。
- 板载 LED、按键、PWM 蜂鸣器、音频 I2S/I2C 外设。
- 无板载调试器，需要外接 CMSIS-DAP 或 ST-Link。

示例：

```bash
west build -p -b rm_typec \
  -d build/dji_position \
  samples/motor/dji_position_control
west flash -d build/dji_position
```

## 4. Overlay 与 alias

电机样例在 C++ 中配置型号、ID 和限幅，overlay 只需启用物理外设或声明 UART 等 alias。例如：

```dts
/ {
    aliases {
        telemetry-uart = &usart1;
    };
};
&can1 { status = "okay"; };
```

应用通过 `DEVICE_DT_GET(DT_NODELABEL(can1))` 获取物理 CAN，电机对象由 `motor::dji::gm6020(...)` 等工厂构造。`applications/sentry_*` 的 `board_config.hpp` 保留 `connections_configured=false` 安全默认值，实机接线和参数确认后再启用。

## 5. 设备树配置排查

1. 物理 CAN/UART 节点必须为 `okay`，引脚和波特率与接线一致。
2. 使用的 alias 或 node label 必须在 board DTS / overlay 中存在。
3. 修改 `app.overlay` 后使用 pristine build。
4. 生成后检查 `build/<name>/zephyr/zephyr.dts`，确认物理控制器、状态和引脚配置。
5. 电机 ID、模式、限幅和 Group 关系在应用 C++ 配置中核对。

不要把板卡 DTS 里的 alias 名字直接当成所有 sample 的契约；每个 sample/application 可能在 overlay 里覆盖或新增 alias。
