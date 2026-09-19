# SkyWalker 文档中心

本文档以当前源码为准，覆盖从 Zephyr 工程入口到双主控机器人应用骨架的完整链路。文档中的“支持”分为三种含义：

- **库/驱动已实现**：对应源码和公共头文件已经存在，并由 Kconfig/CMake 接入。
- **样例可验证**：`samples/` 中有独立 `CMakeLists.txt`、`prj.conf` 和入口代码。
- **应用骨架**：`applications/sentry_*` 已搭好线程和消息流，但需要用户按真实接线、ID、零点和权限策略配置。

## 推荐阅读路线

| 目标 | 阅读顺序 |
|---|---|
| 第一次编译/烧录 | [01 快速开始](01-getting-started.md) → [10 样例索引](10-samples.md) → [12 故障排查](12-troubleshooting.md) |
| 理解工程与模块 | [02 架构与构建](02-architecture.md) → [03 板级支持](03-boards.md) |
| 调试 DJI / 达妙 | [04 DJI 驱动](04-drivers-motor-dji.md) 或 [05 达妙驱动](05-drivers-motor-dm.md) → [09 统一电机封装](09-motor-wrapper.md) → [11 调试](11-debugging.md) |
| 做 IMU 姿态 | [06 IMU 与 EKF](06-drivers-imu.md) → [07 Kalman 与矩阵](07-kalman-matrix.md) |
| 做通信与双主控 | [13 通信协议](13-communication.md) → [14 机器人算法](14-robotics.md) → [15 应用骨架](15-applications.md) |

## 主题文档

| 文档 | 内容 |
|---|---|
| [01-getting-started.md](01-getting-started.md) | 工作区、依赖、构建、烧录、最小工程 |
| [02-architecture.md](02-architecture.md) | 分层、源码布局、west module、Kconfig/CMake、线程边界 |
| [03-boards.md](03-boards.md) | `dm_mc02` 与 `rm_typec` 的 SoC、外设、设备树和 runner |
| [04-drivers-motor-dji.md](04-drivers-motor-dji.md) | M3508、M2006、GM6020 和 DJI Bus |
| [05-drivers-motor-dm.md](05-drivers-motor-dm.md) | DM-J4310-2EC、三种模式、反馈路由和恢复 |
| [06-drivers-imu.md](06-drivers-imu.md) | BMI088、IMU API、EKF 和恒温控制 |
| [07-kalman-matrix.md](07-kalman-matrix.md) | Kalman 设备、CMSIS-DSP 矩阵和 Flash 存储 |
| [08-control-algorithms.md](08-control-algorithms.md) | PID、前馈、斜坡、角度、速度/位置内核 |
| [09-motor-wrapper.md](09-motor-wrapper.md) | `MotorBackend`、恢复生命周期、速度/位置封装 |
| [10-samples.md](10-samples.md) | 所有当前 sample、用途、硬件前提和构建入口 |
| [11-debugging.md](11-debugging.md) | VOFA+、日志、异步 UART 和实时调试 |
| [12-troubleshooting.md](12-troubleshooting.md) | 构建、设备树、CAN、电机、通信、应用故障排查 |
| [13-communication.md](13-communication.md) | DR16、裁判系统、板间协议、`AsyncUart` |
| [14-robotics.md](14-robotics.md) | 命令、安全、舵轮、Yaw、功率限幅 |
| [15-applications.md](15-applications.md) | `sentry_chassis` / `sentry_gimbal` 双主控应用骨架 |
| [16-uart-dma-nocache.md](16-uart-dma-nocache.md) | UART DMA 缓冲区、D-cache 报错原因、修复方式与实机排查 |

## 源码对照

| 主题 | 主要位置 |
|---|---|
| 工程门控 | `west.yml`、`zephyr/module.yml`、根 `CMakeLists.txt`、`Kconfig` |
| 板卡 | `boards/` |
| 设备树 binding | `dts/bindings/` |
| 电机驱动 | `drivers/motor/`、`include/drivers/motor/` |
| IMU / Kalman | `drivers/imu/`、`drivers/kalman_filter/` |
| 控制库 | `lib/control/`、`include/control/` |
| 通信库 | `lib/communication/`、`include/communication/` |
| 机器人库 | `lib/robotics/`、`include/robotics/` |
| 样例 | `samples/` |
| 应用骨架 | `applications/sentry_chassis/`、`applications/sentry_gimbal/` |

## 原始手册

`docs/` 下的 PDF 只作为硬件和协议原始资料，不由本仓库源码自动同步。主要包括：

- DJI：M3508、C620、M2006、C610、GM6020、RoboMaster Type-C C 板。
- 达妙：DM-J4310-2EC V1.1、DM-MC-Board02、调试助手协议。

文件清单和外部下载链接见 [DOWNLOAD_LINKS.txt](DOWNLOAD_LINKS.txt)。

## 文档维护约定

- 路径、Kconfig 名称、设备树属性和样例名称必须以仓库当前文件为准。
- 规划中的功能使用“计划/模板/待配置”标识，不写成已实现能力。
- 电机、通信和安全文档都要说明数据新鲜度、线程归属、失败动作和硬件前提。
- 单位默认使用 `rad`、`rad/s`、A、N·m、s；设备树中带 `millirad`、`millinewton-meter`、`ms` 的属性保持原名。
