# 单电机断电恢复

这是独立的上机微型项目，只依赖正式模块库。

## 接线

一块 MC02 + 一台电机，默认 CAN1 的 GM6020 电流模式 ID1；DM MIT 使用本目录 dm.overlay。电机单独供电、CAN 正确终端并共地；MC02 逻辑供电需保持。

控制台沿用 MC02 板定义的 USART10，115200 baud。

## 构建与刷写

```sh
west build -b dm_mc02 samples/motor/recovery -d build/bench_recovery
west flash -d build/bench_recovery
```

DM MIT 替代配置（不是叠加默认 GM6020 overlay）：

```sh
west build -b dm_mc02 samples/motor/recovery -d build/bench_recovery_dm -- -DDTC_OVERLAY_FILE=dm.overlay
west flash -d build/bench_recovery_dm
```

## 操作与预期现象

启动先显示 Waiting/Ready，不输出运动目标。按 e 以 2 rad/s 低限幅运行；仅切断电机供电，观察 uptime 持续、状态退回等待、恢复计数增长。恢复供电后 generation 增加并从零速度参考斜坡恢复。空格普通暂停，e 恢复；! 急停后必须 r、e。日志每 250 ms 输出。

## 自行配置

board_config.hpp 配速度、限幅、PID 与恢复阈值；overlay 配协议参数、CAN、ID、反馈 Master ID。DM 的 PMAX/VMAX/TMAX 必须与上位机一致，切换电机型号后核对能力和单位。

目前已做固件编译，未在此环境刷写或连接真实外设验证。
