# 小 Yaw 实机调试

这是独立的上机微型项目，只依赖正式模块库。

## 接线

一块 MC02 + GM6020，默认 CAN1 ID1、电流模式、1:1、编码器零点 0。必须按机构填写零点与限位；电机独立供电，机械支撑。

控制台沿用 MC02 板定义的 USART10，115200 baud。

## 构建与刷写

```sh
west build -b dm_mc02 samples/robotics/yaw_gimbal -d build/bench_yaw_gimbal
west flash -d build/bench_yaw_gimbal
```

## 操作与预期现象

e 使能并锁定当前位置；a/d 为正/负 0.3 rad/s，h 固定保持进入时角度，0/1 请求 0/0.5 rad。目标按最大 0.5 rad/s 变化。空格 Disable，! 急停，r 复位后再 e。可以只切电机电源验证新的位置基准和自动恢复。

## 自行配置

board_config.hpp 配拓扑、限位、速度与 PID；overlay 配 CAN、ID 和装配零点。Continuous 要求 AbsoluteNearest；Limited 要求与限位一致的校准 DriverContinuous，不能直接以环形最短路径跨机械限位。

目前已做固件编译，未在此环境刷写或连接真实外设验证。
