# SkyWalker 文档中心

文档按当前源码边界组织：工程与板级入口、可复用模块、跨模块工作流、整机应用。旧的 docs/01…19 文件名保留为迁移提示；正文以本目录链接指向的新页面为准。设计记录和待实施方案仍放在 dev/，不能当作当前 API 契约。

## 按任务开始

| 目标 | 阅读路线 |
|---|---|
| 首次构建和烧录 | [快速开始](getting-started/quickstart.md) → [板级支持](getting-started/boards.md) → [样例索引](getting-started/samples.md) |
| 理解源码分层 | [架构与构建](getting-started/architecture.md) → [模块目录](modules/README.md) |
| 控制 DJI / 达妙电机 | [DJI](modules/drivers/motor-dji.md) 或 [达妙](modules/drivers/motor-dm.md) → [统一电机控制器](modules/control/motor-control.md) → [电机工作流](guides/motor-workflow.md) |
| 读取 IMU 或姿态 | [IMU](modules/drivers/imu.md) → [Kalman 与矩阵](modules/drivers/kalman-matrix.md) → [双 IMU 样例](../samples/imu/dual_imu/README.md) |
| 接收遥控、裁判或板间消息 | [通信模块](modules/communication/communication.md) → [调用示例](modules/call-examples.md) |
| 从命令走到云台或底盘 | [命令来源服务](modules/robotics/command-service.md) → [机器人模块](modules/robotics/robotics.md) → [模块联动](applications/module-integration.md) |
| 配置双主控应用 | [双主控应用](applications/dual-controller.md) → [模块联动](applications/module-integration.md) |
| 查现场问题 | [调试与观测](guides/debugging.md) → [故障排查](guides/troubleshooting.md)；UART DMA 专项见 [说明](guides/uart-dma.md) |

## 按源码层查找

### 工程与板级

- [快速开始](getting-started/quickstart.md)：west workspace、构建、烧录与第一次上电。
- [架构与构建](getting-started/architecture.md)：module、Kconfig/CMake、线程与消息边界。
- [板级支持](getting-started/boards.md)：MC02、RoboMaster Type-C C 板与设备树。
- [样例索引](getting-started/samples.md)：独立构建项目及硬件前提。

### 可复用模块

| 源码区域 | 文档 | 核心调用对象 |
|---|---|---|
| drivers/motor | [DJI](modules/drivers/motor-dji.md)、[达妙](modules/drivers/motor-dm.md) | Motor、Group、CanBus |
| drivers/imu | [IMU](modules/drivers/imu.md) | ImuSource、ImuReceiver |
| lib/control、lib/matrix | [控制算法](modules/control/algorithms.md)、[电机控制器](modules/control/motor-control.md)、[Kalman 与矩阵](modules/drivers/kalman-matrix.md) | C PID、KalmanFilter、QuaternionEkf、VelocityMotor、PositionMotor |
| lib/communication | [UART、遥控、裁判、板间](modules/communication/communication.md)、[视觉](modules/communication/vision.md) | AsyncUart、RemoteReceiver、RefereeReceiver、InterBoardEndpoint、VisionReceiver |
| lib/robotics | [命令来源服务](modules/robotics/command-service.md)、[云台与舵轮](modules/robotics/robotics.md)、[恢复](modules/robotics/command-recovery.md) | CommandManager、CommandArbiter、GimbalAxis、SwerveChassis |
| 所有封装对象 | [调用示例手册](modules/call-examples.md) | 初始化、快照、周期更新与错误路径 |

### 工作流与应用

- [电机端到端链路](guides/motor-workflow.md)：反馈、控制目标、总线提交、停机与恢复。
- [模块联动](applications/module-integration.md)：遥控到云台、双主控到舵轮底盘。
- [双主控应用](applications/dual-controller.md)：应用线程、默认阻断和硬件配置。
- [调试](guides/debugging.md)、[故障排查](guides/troubleshooting.md)、[UART DMA 缓冲](guides/uart-dma.md)。

## 资料、交互图和开发记录

- datasheets/ 保存电机、传感器、裁判系统和开发板 PDF；下载索引见 [DOWNLOAD_LINKS.txt](datasheets/DOWNLOAD_LINKS.txt)。
- [架构与接口浏览器](architecture-browser/README.md) 提供当前双主控架构、最终上车蓝图、可点击模块关系、接口契约、调用示例和端到端链路；支持在页面内阅读本目录的 Markdown 正文及本地源码。运行 `bash docs/architecture-browser/run.sh`，打开脚本打印的地址。
- [开发专题索引](dev/README.md) 收录设计方案、分析与待实施记录；以模块文档和源码判断当前状态。
- samples/ 是独立验证入口；applications/sentry_* 是需按真实接线和安全策略配置的应用骨架。
