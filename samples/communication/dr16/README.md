# DR16 实机输入

这是独立的上机微型项目，只依赖正式模块库。

## 接线

一块 MC02、DR16 接收机。默认 UART5：PD2 RX、PC12 TX，100000 baud、8E1；必须确认板卡 SBUS/DBUS 通路的电平与反相。接收机 TX 接主控 RX，并共地。

控制台沿用 MC02 板定义的 USART10，115200 baud。

## 构建与刷写

```sh
west build -b dm_mc02 samples/communication/dr16 -d build/bench_dr16
west flash -d build/bench_dr16
```

## 操作与预期现象

移动左右摇杆、滚轮和拨杆，观察每 100 ms 输出；鼠标键盘需要遥控器的对应数据入口。断开接收机后约 100 ms online 变为 0，重连恢复计数；异常帧不会刷新旧数据时刻。

## 自行配置

`app.overlay` 改 UART/DMA/格式，`src/board_config.hpp` 改解码中心、范围、死区和超时。

目前已做固件编译，未在此环境刷写或连接真实外设验证。
