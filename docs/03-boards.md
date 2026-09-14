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

样例通常把真实节点放在 `/ { ... };` 下，并使用 alias 给应用稳定名字：

```dts
/ {
    aliases {
        motor0 = &my_motor;
    };

    my_motor: motor {
        compatible = "dji,gm6020-current";
        status = "okay";
        can-bus = <&can1>;
        motor-id = <1>;
        current-limit-ma = <500>;
        gear-ratio-num = <1>;
        gear-ratio-den = <1>;
        encoder-zero-ticks = <0>;
        current-loop-confirmed;
    };
};
```

应用通过 `DT_ALIAS(motor0)` 获取节点；如果节点 `status` 为 `disabled`，`DT_NODE_HAS_STATUS` 会让对应指针变成空指针。`applications/sentry_*` 正是利用这个机制保留“模板可编译、真实连接未启用”的安全默认值。

## 5. 设备树配置排查

1. `compatible` 必须能在 `dts/bindings/` 找到。
2. 所有 binding 的 required 属性必须填写。
3. `can-bus` 指向 board DTS 中状态为 `okay` 的 CAN 节点。
4. 修改 `app.overlay` 后使用 pristine build。
5. 生成后检查 `build/<name>/zephyr/zephyr.dts`，确认节点、ID、状态和 phandle 都是预期值。

不要把板卡 DTS 里的 alias 名字直接当成所有 sample 的契约；每个 sample/application 可能在 overlay 里覆盖或新增 alias。
