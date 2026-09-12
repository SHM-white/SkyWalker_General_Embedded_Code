# SkyWalker 项目文档

本目录是「巡天御风通用电控框架」的文档中心。`docs/` 根目录下的 `*.pdf`
是电机与开发板的原始手册（datasheet），下方编号 Markdown 是按主题拆分的框架文档。

> 快速上手请直接看 [01 快速开始](01-getting-started.md)；只想跑一个样例看
> [10 样例索引](10-samples.md)。

---

## 阅读路线

| 目标 | 建议顺序 |
|---|---|
| 第一次跑起来 | 01 快速开始 → 10 样例索引 → 12 故障排查 |
| 理解整体设计 | 02 架构与构建 → 03 板级支持 → 04/05 电机驱动 |
| 调电机 | 04 DJI 驱动 / 05 达妙驱动 → 09 控制封装 → 11 调试 |
| 做姿态解算 | 06 IMU 与 EKF → 07 Kalman 与矩阵 → 11 调试 |
| 写控制算法 | 08 纯 C 控制算法 → 09 控制封装 |

---

## 文档清单

| 文档 | 主题 |
|---|---|
| [01 快速开始](01-getting-started.md) | 环境准备、west 构建、烧录、最小工程 |
| [02 架构与构建](02-architecture.md) | 分层结构、仓库布局、module/Kconfig/CMake 机制、单位约定 |
| [03 板级支持](03-boards.md) | `dm_mc02`（STM32H723）与 `rm_typec`（STM32F407）外设、别名与 runner |
| [04 DJI 电机驱动](04-drivers-motor-dji.md) | M3508/M2006/GM6020、设备树 binding、CAN 协议、Bus 生命周期 |
| [05 达妙 DM 电机驱动](05-drivers-motor-dm.md) | DM-J4310-2EC、MIT/位置速度/速度三种模式、特殊命令、Bus |
| [06 IMU 与 EKF](06-drivers-imu.md) | `skywalker,imu` 驱动、四元数 EKF、恒温加热控制 |
| [07 Kalman 与矩阵库](07-kalman-matrix.md) | `skywalker,kalman_filter` 设备与 CMSIS-DSP 矩阵封装 |
| [08 纯 C 控制算法](08-control-algorithms.md) | PID、前馈、复合控制、斜坡限幅、角度工具、速度/位置内核 |
| [09 统一速度/位置封装](09-motor-wrapper.md) | `MotorBackend`/`MotorRuntime`/`VelocityMotor`/`PositionMotor` |
| [10 样例索引](10-samples.md) | 每个样例的用途、构建命令与运行前提 |
| [11 调试与上位机](11-debugging.md) | VOFA+ JustFloat、命令行调参、日志与断点注意事项 |
| [12 故障排查](12-troubleshooting.md) | 编译/链接/运行/硬件常见问题速查 |

---

## 与源码的对应关系

| 文档 | 主要源码位置 |
|---|---|
| 02 / 08 / 09 | `CMakeLists.txt`、`Kconfig`、`lib/`、`include/control/` |
| 03 | `boards/` |
| 04 / 05 | `drivers/motor/`、`include/drivers/motor/`、`dts/bindings/motor/` |
| 06 | `drivers/imu/`、`include/drivers/imu/`、`dts/bindings/imu/` |
| 07 | `drivers/kalman_filter/`、`lib/matrix/`、`include/lib/matrix/` |
| 10 | `samples/` |

---

## 手册资料（本目录 PDF）

- 大疆：`M3508_User_Guide_V1.0.pdf`、`C620_User_Guide_V1.01.pdf`、
  `M3508_Mix_Control.pdf`、`M2006_P36_User_Guide.pdf`、`C610_User_Guide.pdf`、
  `GM6020_User_Guide.pdf`、`RoboMaster GM6020直流无刷电机使用说明20231013.pdf`、
  `RoboMaster  开发板 C 型用户手册.pdf`
- 达妙：`DM-J4310-2EC_V1.1_User_Manual.pdf`、
  `调试助手使用说明书（达妙驱动控制协议）V1.4.pdf`、
  `达妙科技 DM-MC-Board02电机开发板使用说明书.pdf`
- 下载直链与在线手册：`DOWNLOAD_LINKS.txt`

---

## 文档约定

- **单位**：角度 `rad`，角速度 `rad/s`，电流 `A`，力矩 `N·m`，时间 `s`（
  Kconfig/设备树中的毫秒、毫弧度字段会显式带单位后缀）。
- **反馈坐标系**：`Feedback::position_rad` 默认是**输出轴连续角度**，
  首帧为 0；`absolute_position_rad` 才是固定零点单圈值。
- **线程语义**：驱动与封装 API 默认在**单一线程**中调用，不可在 ISR 中使用；
  CAN 回调运行在系统工作队列/中断上下文，只做解码与缓存。
- **源码只读**：本仓库默认处于「古法编程模式」，文档只描述现状，
  不代替你修改业务代码。
