# 架构与构建

## 1. 源码分层

~~~text
applications/、samples/
  线程、板级设备绑定、命令来源注册、执行恢复
        │
lib/robotics
  CommandManager / CommandArbiter / GimbalAxis / SwerveChassis
lib/communication
  AsyncUart、接收器、协议解析和带时间戳的消息
lib/control
  纯 C 控制内核、线性 Kalman、QuaternionEkf、VelocityMotor、PositionMotor
        │
drivers/
  CAN 电机、独立 IMU sources
        │
Zephyr device API / CAN / UART / SPI / PWM / CMSIS-DSP / STM32 HAL
~~~

应用组织跨模块工作。驱动处理设备协议，通信层把字节流转换为消息快照，CommandArbiter 根据输入时间与权限形成候选命令，应用执行器再按本地反馈和参考条件管理运动。CommandManager 只负责来源生命周期、后台采样、调用仲裁器和快照发布，不直接操作执行器。

## 2. 仓库布局

| 路径 | 责任 |
|---|---|
| boards/ | MC02、RoboMaster Type-C C 板的板级定义 |
| dts/bindings/ | Zephyr 设备 binding |
| drivers/motor/ | DJI / 达妙 CAN 电机协议与统一总线 |
| drivers/imu/ | BMI088、DM-IMU RS485、ImuState、ImuReceiver 与可选 EKF/温控 |
| include/ | 对应 drivers/lib 的公共 C / C++ 头文件 |
| lib/control/ | 数值控制算法、线性 Kalman、姿态 EKF 和速度/位置电机封装 |
| lib/communication/ | UART 传输、DR16、裁判、板间、视觉协议与 receiver |
| lib/robotics/ | 命令来源服务、仲裁、云台轴、舵轮和功率 limiter |
| samples/ | 独立 west build 的台架与算法入口 |
| applications/ | sentry_gimbal / sentry_chassis 应用骨架 |
| tests/ | 电机回归及命令、视觉/IMU 软件检查 |

## 3. Zephyr module 集成

west.yml 锁定 Zephyr 和白名单依赖。zephyr/module.yml 将仓库声明为 module，并把本地 boards 和 dts/bindings 加入 Zephyr 搜索路径。根 CMakeLists.txt 将 drivers/ 与 lib/ 加入应用构建；根 Kconfig 递归包含 drivers/Kconfig 与 lib/Kconfig。

应用应选择 Kconfig 能力并在 overlay 启用物理设备，再通过 C++ 构造对象。不要把公共模块源文件复制进 sample。

## 4. Kconfig 能力

常用驱动选项：

~~~text
SKYWALKER_DRIVER_MOTOR
  SKYWALKER_MOTOR_DJI / SKYWALKER_MOTOR_DM

SKYWALKER_IMU
  SKYWALKER_IMU_BMI088
  SKYWALKER_IMU_RECEIVER
  SKYWALKER_IMU_HEATER
  SKYWALKER_IMU_DM_PROTOCOL / SKYWALKER_IMU_DM_RS485
  SKYWALKER_ATTITUDE_EKF
~~~

通用线性 Kalman 位于 lib/control，由调用者提供缓冲并显式初始化，不需要设备树节点。当前 BMI088 QuaternionEkf 是另一套独立的实例对象算法，不调用线性 Kalman。旧 Kalman 驱动开关及 binding 已移除。

常用库选项：

~~~text
SKYWALKER_LIB_CONTROL
SKYWALKER_LIB_KALMAN_FILTER（自动选择 CONTROL、MATRIX 和 CMSIS-DSP）
SKYWALKER_LIB_MOTOR_CONTROL
SKYWALKER_LIB_COMMUNICATION
  SKYWALKER_UART_TRANSPORT
  SKYWALKER_REMOTE_DR16 / SKYWALKER_REMOTE_RECEIVER
  SKYWALKER_REFEREE / SKYWALKER_INTERBOARD
  SKYWALKER_VISION / SKYWALKER_VISION_AB / SKYWALKER_VISION_RECEIVER
SKYWALKER_LIB_ROBOTICS
  SKYWALKER_ROBOTICS_COMMAND
  SKYWALKER_COMMAND_SERVICE
  SKYWALKER_ROBOTICS_SWERVE / SKYWALKER_ROBOTICS_GIMBAL
~~~

SKYWALKER_COMMAND_SERVICE 需要显式启用；默认仲裁周期 10 ms。它依赖 SKYWALKER_ROBOTICS_COMMAND。当前没有 SKYWALKER_ROBOTICS_SAFETY 单独管理器选项。具体依赖以 lib/Kconfig、drivers/imu/Kconfig 和目标应用 prj.conf 为准。

## 5. 数据与线程边界

### 电机链路

~~~text
CAN callback → CanBus I/O worker → Motor snapshot
  → VelocityMotor / PositionMotor → Motor staged command
  → CanBus.commit() → asynchronous CAN TX
~~~

同一物理 CAN 只创建一个 CanBus。Group 表达共同使能和停机的机械故障域，可以跨 CAN；共享 CAN 的多个业务写入方必须在应用层协调 setter/update 与 commit。

### 命令链路

~~~text
RemoteReceiver / VisionReceiver / RefereeReceiver
  → RemoteSource / VisionSource / RefereePermissionSource
  → CommandManager worker → CommandArbiter
  → non-consuming CommandSnapshot
  → execution threads / telemetry / InterBoardEndpoint
~~~

在 CommandManager::start() 前注册来源和权限源。start 返回 0 不代表输入有效；snapshot() 首次发布前返回 -EAGAIN。快照读取不会消费数据，也不会刷新消息时间戳。完整合同见[命令来源服务](../modules/robotics/command-service.md)。

### 应用线程

sentry_gimbal 的 CommandManager 有自己的仲裁 worker；RemoteReceiver 也有接收 worker。应用的 linkTask 消费命令快照并推进 InterBoardEndpoint，gimbalTask 调用 GimbalExecutor。sentry_chassis 将板间 poll 与 ChassisExecutor 分开运行。各静态对象及 DMA buffer 必须覆盖 worker 和 callback 的整个生命周期。

## 6. 单位和状态

位置/角度使用 rad，速度 rad/s，DJI effort 使用 A，DM MIT effort 使用 N·m。时间单位依 API 后缀使用 ms、us 或 s。MotorState、BusState、CommandSnapshot 中的命令决策和应用 RunStatus 各代表不同层级，不能互相替代。

架构概览见[机器人模块](../modules/robotics/robotics.md)和[双主控应用](../applications/dual-controller.md)。
