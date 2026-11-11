# 状态灯台架自检

板载 WS2812 的单灯自检入口，不启动电机、CAN、遥控或板间链路。用于确认调色板、颜色通道和满亮度效果。

## 行为

上电后串口打印 `status_led: start=0`，然后循环：

1. `HardwareTest` 灯语：红→绿→蓝→白→灭，每段 300 ms。
2. 红、绿、蓝、白单通道满亮，各保持 700 ms。
3. 白色阶梯 `16/32/64/128/255`，各保持 400 ms。
4. 熄灭 1 s，回到第 1 步。

`start` 打印负 errno 时灯不会亮，先查 `led-strip` 别名、SPI6 设备树和 `CONFIG_SKYWALKER_STATUS_LED`。没有 LED 的 C 板会走空实现，打印提示后空转。

## 构建与烧录

```sh
west build -p always -b dm_mc02/stm32h723xx samples/status_led -d build/status_led
west flash -d build/status_led
```

## 注意

- 满白电流约 60 mA（每通道约 20 mA），单灯无散热风险；若灯珠实际接 3.3 V，颜色和亮度会打折。
- LED 只用于显示，不参与任何运动授权。
- 串口是 MC02 的 console（USART10），115200 8N1。
