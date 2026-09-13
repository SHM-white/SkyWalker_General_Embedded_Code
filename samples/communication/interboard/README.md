# 两板 UART 联调

这是独立的上机微型项目，只依赖正式模块库。

## 接线

两块 MC02。默认 USART1：A PA9 -> B PA10，B PA9 -> A PA10，GND 相连，460800、8N1。此项目不连接电机。

控制台沿用 MC02 板定义的 USART10，115200 baud。

## 构建与刷写

```sh
west build -b dm_mc02 samples/communication/interboard -d build/bench_interboard
west flash -d build/bench_interboard
```

第二块板独立构建：

```sh
west build -b dm_mc02 samples/communication/interboard -d build/bench_interboard_chassis -- -DEXTRA_CONF_FILE=chassis.conf
west flash -d build/bench_interboard_chassis
```

## 操作与预期现象

一块使用默认云台角色，另一块使用 chassis.conf。正常时 peer_online=1，底盘看到命令序号持续增长。在云台控制台输入 p，仅命令生产暂停，心跳继续；底盘 cmd_fresh 应在 100 ms 后变为 0，再按 p 恢复。底盘按 g 改变 generation，观察云台随后回显。重启一块板，观察 boot ID 变化和自动同步。

## 自行配置

`app.overlay` 改 UART，`src/board_config.hpp` 改台架目标；chassis.conf 仅设置本独立项目的角色。

目前已做固件编译，未在此环境刷写或连接真实外设验证。
