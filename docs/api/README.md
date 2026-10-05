# 独立 Markdown 接口参考

接口签名、参数、返回值、时序、边界、示例与生命周期在这里维护。无需启动服务即可直接阅读；在线浏览器的模块详情读取这些原始文件，JS 只保存关系图与导航摘要。

源码基线：`main@99a97c9`（2026-10-05）。覆盖常用公开入口，不替代头文件的完整声明。

电机先阅读[公共契约](../modules/drivers/motor.md) → [迁移指引](../guides/motor-migration.md) → [周期调用](../guides/motor-workflow.md)。

| 分类 | 模块 | 接口参考 |
| --- | --- | --- |
| 机器人与仲裁 | 双主控应用与执行器 | [接口与示例](application.md) |
| 驱动与感知 | DJI 电机与共享 CAN 总线 | [接口与示例](motor-dji.md) |
| 驱动与感知 | 达妙电机：MIT / 速度 / 位置速度 | [接口与示例](motor-dm.md) |
| 驱动与感知 | IMU：独立采集、姿态与加热 | [接口与示例](imu.md) |
| 控制算法 | 姿态 EKF 与通用 Kalman / Matrix | [接口与示例](kalman.md) |
| 控制算法 | PID、前馈、斜坡与角度工具 | [接口与示例](pid.md) |
| 控制算法 | VelocityMotor / PositionMotor 硬件闭环 | [接口与示例](motor-control.md) |
| 工程与板级 | 板级：MC02 / RoboMaster Type-C | [接口与示例](boards.md) |
| 通信与输入 | 异步 UART 与 DMA | [接口与示例](uart.md) |
| 通信与输入 | DR16 遥控输入 | [接口与示例](remote.md) |
| 通信与输入 | 裁判许可与功率预算 | [接口与示例](referee.md) |
| 通信与输入 | 双主控板间通信 | [接口与示例](interboard.md) |
| 通信与输入 | 视觉链路与 AB 协议 | [接口与示例](vision.md) |
| 机器人与仲裁 | 命令仲裁与后台服务 | [接口与示例](command.md) |
| 机器人与仲裁 | 来源适配与手动映射 | [接口与示例](command-sources.md) |
| 机器人与仲裁 | 云台单轴与本地执行 | [接口与示例](gimbal.md) |
| 机器人与仲裁 | 舵轮底盘与功率缩放 | [接口与示例](chassis.md) |
| 调试与观测 | VOFA、日志与一致快照 | [接口与示例](telemetry.md) |
| 机器人与仲裁 | 头部惯性云台适配 | [接口与示例](inertial.md) |
| 机器人与仲裁 | 大 Yaw 回中与独立速度环 | [接口与示例](big-yaw.md) |
| 机器人与仲裁 | 摩擦轮与拨盘发射执行 | [接口与示例](shooter.md) |

维护流程见[文档同步清单](../maintenance.md)。新增文档由扫描器自动发现，接口内容仍需根据源码变化更新。
