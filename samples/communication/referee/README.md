# 裁判 UART 实机输入

这是独立的上机微型项目，只依赖正式模块库。

## 接线

一块 MC02 与裁判 User 串口。默认 USART1 PA10 RX / PA9 TX，115200、8N1；接裁判 TX 与共地。

控制台沿用 MC02 板定义的 USART10，115200 baud。

## 构建与刷写

```sh
west build -b dm_mc02 samples/communication/referee -d build/bench_referee
west flash -d build/bench_referee

# 如果裁判设备仍使用 2026 协议，请单独编译：
west build -b dm_mc02/stm32h723xx samples/communication/referee -d build/bench_referee_2026 -- -DEXTRA_CONF_FILE=rm2026.conf
```

## 操作与预期现象

观察 frames、crc、robot、outputs、permission_fresh、limit_W、buffer_J。2027 的发射初速度上限可通过 `parser.state().robot.bullet_speed_limit_m_s` 读取，需检查 `bullet_speed_limit_stamp.valid`；2026 该字段无效。拔线后 online 和 permission_fresh 分别按各自超时失效。若帧计数增加但状态不更新，核对协议 profile、命令号和长度；未知命令只计数。

## 自行配置

`app.overlay` 用来调整串口，协议版本由 Zephyr Kconfig `choice` 在编译期选择。默认 `CONFIG_SKYWALKER_REFEREE_RM2027=y`；如需旧设备，使用 `rm2026.conf` 构建独立固件。CMake 只链接选中年份的解析类，运行时不再传版本参数。2027 的 `0x0201` 长 17 字节，2026 长 13 字节；两版 `0x0202` 保留区均不作为实测底盘功率。

目前已做固件编译，未在此环境刷写或连接真实外设验证。
