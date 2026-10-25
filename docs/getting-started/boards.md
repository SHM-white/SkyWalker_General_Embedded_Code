# 03 板级支持

板卡层只描述物理外设、引脚和 alias；电机型号、ID、限幅、Group 与业务安全参数在 C++ 配置。完整顺序是“overlay 启用设备 → Kconfig 纳入模块 → 应用取得 `device` → `CanBus`/通信驱动初始化”，见 [架构](architecture.md)和[模块联动](../applications/module-integration.md)。

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
- BMI088、USB FS CDC-ACM、USART1/3/6；当前 DTS 的 console/shell 使用 USART1，`telemetry-uart` alias 使用 USART6，USART3 默认 100000 baud，可用于遥控/串行设备。alias `usart1` 也指向 USART6；`DT_ALIAS(usart1)` 与 `DT_NODELABEL(usart1)` 不是同一个设备。
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

应用通过 `DEVICE_DT_GET(DT_NODELABEL(can1))` 获取物理 CAN，电机对象由 `motor::dji::gm6020(...)` 等工厂构造。两个 `applications/sentry_*` 与新的上车工程共享 `include/robotics/vehicle/` profile 和标定，不维护应用私有 `board_config.hpp`。当前中央连接/IMU/发射确认值为 true、功率模型为 false；这些值仍需现场核对，不能当作验收结果。

### 当前实车共享 overlay

上面的 console/telemetry 映射是板级默认值。`samples/bringup/` 和两个最终应用固定包含 [config/vehicle/installed](../../config/vehicle/installed/README.md) 的两板 overlay，实际使用：

| 用途 | 云台板 | 底盘板 |
| --- | --- | --- |
| 单口诊断 | UART7，PE8 TX / PE7 RX，115200 8N1，无 DMA | 同左 |
| 板间 UART | USART10，PE3 TX / PE2 RX，460800，DMA 通道 3/4 | USART1，460800，DMA 通道 3/4 |
| CAN1 | 小 Yaw | 四舵向 |
| CAN2 | Pitch | 四轮驱 |
| CAN3 | DM 头部 IMU 独占 | 大 Yaw |

云台 USART1 释放 DMA 3/4 给 USART10；板载 SPI2 的 DMA1 通道 1/2 保留。新配置关闭 console/shell/banner，删除 `telemetry-uart` 并使用 `diagnostic-uart`，不提供 `vision-uart`。UART7 是 DAPLink 诊断候选，实际接线尚需确认，不能同时交给视觉或第二个发送者。各阶段只启动需要的对象，设备树中的其他设备声明不会自动使能电机。

固定模式选择见 [诊断配置](../../include/robotics/vehicle/diagnostics_config.hpp)，工程导航见 [独立上车入口](../../samples/bringup/README.md)。

## 5. 设备树配置排查

1. 物理 CAN/UART 节点必须为 `okay`，引脚和波特率与接线一致。
2. 使用的 alias 或 node label 必须在 board DTS / overlay 中存在。
3. 修改 `app.overlay` 后使用 pristine build。
4. 生成后检查 `build/<name>/zephyr/zephyr.dts`，确认物理控制器、状态和引脚配置。
5. 电机 ID、模式、限幅和 Group 关系在应用 C++ 配置中核对。

不要把板卡 DTS 里的 alias 名字直接当成所有 sample 的契约；每个 sample/application 可能在 overlay 里覆盖或新增 alias。
