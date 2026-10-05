# 公共模块地图

公共头文件按 drivers、control、communication、robotics 和 core 的职责组织。应用先通过 Kconfig/设备树启用所需实现，再构造长期存活的模块对象；多数对象只处理数据，不自动创建线程或完成整个机器人的恢复逻辑。

接口签名、参数、时序与完整调用片段见[独立 Markdown API 参考](../api/README.md)，可直接阅读，无需启动浏览器服务。电机从[公共模型](drivers/motor.md)开始，再选型号、控制环和[迁移指引](../guides/motor-migration.md)。更新时按[维护清单](../maintenance.md)逐项同步。

## 架构边界

~~~text
Zephyr device / CAN / SPI / UART / PWM
                 │
                 ▼
drivers: 设备采集、协议驱动、Motor / CanBus / Group
                 │
                 ▼
lib/control: 纯数值控制、线性 Kalman / QuaternionEkf、VelocityMotor / PositionMotor
lib/communication: UART transport、解析器、接收器、消息快照
                 │
                 ▼
lib/robotics: CommandManager / CommandArbiter、惯性与回中适配、公开机构执行器、GimbalAxis、SwerveChassis
                 │
                 ▼
applications / samples: 线程所有权、真实板级绑定、本地执行与恢复
~~~

| 模块 | 主题页 | 入口示例 |
|---|---|---|
| CAN 电机 | [DJI](drivers/motor-dji.md)、[达妙](drivers/motor-dm.md) | [混合拓扑与 Group](../../samples/motor/mixed_topology/) |
| IMU 与姿态 | [IMU](drivers/imu.md)、[Kalman/矩阵](drivers/kalman-matrix.md) | [双 IMU](../../samples/imu/dual_imu/README.md) |
| 控制环 | [纯 C 算法](control/algorithms.md)、[电机控制器](control/motor-control.md) | [DJI 速度](../../samples/motor/dji_speed_control/src/main.cpp)、[DJI 位置](../../samples/motor/dji_position_control/src/main.cpp) |
| 通信 | [串口、遥控、裁判、板间](communication/communication.md)、[视觉](communication/vision.md) | [命令管理样例](../../samples/robotics/command_manager/README.md) |
| 机器人 | [命令、云台、舵轮](robotics/robotics.md)、[惯性/回中/发射执行器](robotics/executors.md)、[恢复语义](robotics/command-recovery.md) | [双轴云台](../../samples/robotics/gimbal_control/README.md)、[舵轮](../../samples/robotics/swerve/README.md) |

[封装模块调用示例](call-examples.md) 按“构造与初始化 → 读取快照 → 每周期调用 → 错误处理”展示公开对象用法。样例源码链接用于核对完整配置、线程与硬件细节。
