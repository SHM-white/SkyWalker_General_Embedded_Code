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

观察 frames、crc、robot、outputs、permission_fresh、limit_W、buffer_J。2027 的发射初速度上限可通过 `parser.state().robot.bullet_speed_limit_m_s` 读取，需检查 `bullet_speed_limit_stamp.valid`；2026 该字段无效。拔线后 online 和 permission_fresh 分别按各自超时失效。若帧计数增加但状态不更新，核对协议 profile、命令号和长度；未知命令只计数。

## 自行配置

`app.overlay` 改串口，`src/board_config.hpp` 明确选择协议版本。默认 `Rm2027V1_0`，仍可切回 `Rm2026V1_3`；`Unspecified` 只统计，不发布许可。2027 的 `0x0201` 必须是 17 字节，2026 是 13 字节；不允许跨版本按长度猜测。两版 `0x0202` 保留区都不会被当成底盘实测功率。

目前已做固件编译，未在此环境刷写或连接真实外设验证。
