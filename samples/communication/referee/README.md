# 裁判 UART 实机输入

这是独立的上机微型项目，只依赖正式模块库。

## 接线

一块 MC02 与裁判 User 串口。默认 USART1 PA10 RX / PA9 TX，115200、8N1；接裁判 TX 与共地。

控制台沿用 MC02 板定义的 USART10，115200 baud。

## 构建与刷写

```sh
west build -b dm_mc02 samples/communication/referee -d build/bench_referee
west flash -d build/bench_referee
```

## 操作与预期现象

观察 frames、crc、robot、outputs、permission_fresh、limit_W、buffer_J。拔线后 online 和 permission_fresh 分别按各自超时失效。若帧计数增加但状态不更新，核对协议 profile、命令号和长度；未知命令只计数。

## 自行配置

`app.overlay` 改串口，`src/board_config.hpp` 选择协议版本。目前支持 Rm2026V1_3；Unspecified 只统计，不发布许可。该版本保留字节不输出假功率。

目前已做固件编译，未在此环境刷写或连接真实外设验证。
