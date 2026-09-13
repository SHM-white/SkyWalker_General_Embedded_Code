# 真实遥控命令与安全联调

这是独立的上机微型项目，只依赖正式模块库。

## 接线

MC02 + DR16，默认 UART5 接线同本项目 app.overlay。没有电机，也不会发送 CAN 输出。

控制台沿用 MC02 板定义的 USART10，115200 baud。

## 构建与刷写

```sh
west build -b dm_mc02 samples/robotics/command_safety -d build/bench_command_safety
west flash -d build/bench_command_safety
```

## 操作与预期现象

左拨杆 Middle 进入 Manual，Down 禁用，Up 的 Auto 当前输出禁用。移动摇杆观察 chassis 与 yaw_rate；断开遥控器观察有效期和安全动作。在控制台输入 ! 锁存急停，r 明确复位。这个项目显式使用无裁判台架模式。

## 自行配置

本目录的 overlay、board_config.hpp 与 main.cpp 中 mapper/manager/safety 配置；不读取正式应用配置。

目前已做固件编译，未在此环境刷写或连接真实外设验证。
