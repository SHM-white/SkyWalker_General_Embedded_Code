# 单轴 Yaw 实机调试

本样例展示一台独立 Motor，不创建 Group。GM6020 电流模式默认接 CAN1、ID1，电流限幅 0.5 A，减速比 1:1，编码器零点 0。型号、CAN、ID、零点、控制参考与 PID 全在 `src/board_config.hpp`；`app.overlay` 不再声明电机节点。CAN1 和 USART10 来自 MC02 板级 DTS。

## 调用与操作

应用先 `CanBus.attach/start`，再配置 PositionMotor。按 `e` 后待 Motor.ready 才显式请求 enable；只有 Motor.active 时 YawGimbal 才计算位置目标、PositionMotor 暂存电流，周期末由 CanBus.commit 提交。失联或控制错误后不会自动恢复运动，需再次按 `e`。

`a/d` 给正/负 0.3 rad/s；`h` 保持进入时角度；`0/1` 请求 0/0.5 rad。空格撤销输出，`!` 锁存急停，`r` 释放急停并请求清除可清故障。控制台为 USART10，115200 baud。Continuous 拓扑使用 AbsoluteNearest；若改 Limited，需使用已校准的 DriverContinuous 坐标和真实机械限位。上电前支撑机构并核对 GM6020 电流模式、零点与方向。

## 构建

```sh
west build -b dm_mc02 samples/robotics/yaw_gimbal -d ../build/bench_yaw_gimbal
```

已在 MC02 编译链接通过；未刷写或实机验证。日志显示使能代次、电机状态、目标角、反馈、输出与故障。
