# 一套真实舵轮模块

这是独立的上机微型项目，只依赖正式模块库。

## 接线

一块 MC02、一台 GM6020 舵向和一台 M3508 驱动，共享 CAN1，两者默认 ID1（不同反馈 ID/命令组）。仅连接并悬空一套舵轮，不把此固件接到完整八电机底盘。

控制台沿用 MC02 板定义的 USART10，115200 baud。

## 构建与刷写

```sh
west build -b dm_mc02 samples/robotics/swerve -d build/bench_swerve
west flash -d build/bench_swerve
```

## 操作与预期现象

e 使能；w/s 前后、a/d 左右、q 旋转分量。运动学计算四个安装点，这个 sample 只把 FL 的结果交给真实单模块。观察最短舵角、翻转后的速度、驱动速度与两个电流。空格禁用，! 急停，r 解除锁存后再 e。关键反馈离线时两个电机一起暂停，恢复后重新播种。

## 自行配置

board_config.hpp 配轮径、两电机方向、速度和 PID；overlay 配 ID、减速比、电流上限、舵向零点。main.cpp 的四个坐标是该微型项目独有的运动学配置。

目前已做固件编译，未在此环境刷写或连接真实外设验证。
