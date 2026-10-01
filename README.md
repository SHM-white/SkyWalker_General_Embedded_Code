# SkyWalker 通用电控框架

SkyWalker 是一个基于 Zephyr RTOS 的机器人电控代码仓库，以 Zephyr module 形式提供板级支持、CAN 电机驱动、控制算法、通信协议、机器人子系统算法和可直接上机的样例。

当前仓库的代码边界如下：

```text
应用 / 样例
├── samples/                    独立可构建的硬件验证项目
└── applications/               双主控底盘 / 云台应用骨架
        │
        ├── lib/                可复用算法、通信和机器人决策层
        └── drivers/            Zephyr 设备驱动与 CAN 电机驱动
                │
                └── Zephyr + STM32 HAL + CMSIS-DSP
```

## 从哪里开始

按任务选择入口，完整主题目录和资料使用说明见 [文档中心](docs/README.md)：

| 要做什么                   | 从这里开始                                                                                                          |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| 第一次配置、构建和烧录     | [快速开始](docs/getting-started/quickstart.md) → [样例索引](docs/getting-started/samples.md)                                              |
| 理解工程分层和硬件         | [架构与构建](docs/getting-started/architecture.md) → [板级支持](docs/getting-started/boards.md)                                                |
| 调电机与控制环             | [DJI](docs/modules/drivers/motor-dji.md) / [达妙](docs/modules/drivers/motor-dm.md) → [电机工作链路](docs/guides/motor-workflow.md) |
| 做遥控、板间通信和整机应用 | [通信](docs/modules/communication/communication.md) → [机器人算法](docs/modules/robotics/robotics.md) → [应用骨架](docs/applications/dual-controller.md)           |
| 查现场问题                 | [故障排查](docs/guides/troubleshooting.md)                                                                               |
| 查设计记录和专题分析       | [开发专题索引](docs/dev/README.md)                                                                                   |

交互式架构图的本地打开方法见 [架构浏览器](docs/architecture-browser/README.md)。

## 当前能力

### 板级支持

| board                   | MCU                | 当前仓库内主要用途                                       |
| ----------------------- | ------------------ | -------------------------------------------------------- |
| `dm_mc02/stm32h723xx` | STM32H723，480 MHz | 达妙 MC02，三路 FDCAN、BMI088、遥控/板间串口、电机应用   |
| `rm_typec`            | STM32F407，168 MHz | RoboMaster Type-C C 板，CAN1/CAN2、BMI088、USB CDC、串口 |

板卡、设备树和烧录器配置位于 `boards/`；详细外设和 runner 见 [03 板级支持](docs/getting-started/boards.md)。

### 驱动与控制

- DJI：M3508-C620、M2006-C610、GM6020 电流模式；支持反馈解码、输出轴角度、温度和总线分组发送。
- 达妙：DM-J4310-2EC V1.1；支持 MIT、位置-速度、速度三种原生 CAN 模式，以及 Enable/Disable/ClearError/SaveZero。
- 统一 CAN 电机驱动：`CanBus` 管物理传输，`Motor` 管端点，`Group` 声明联动停机；`VelocityMotor`、`PositionMotor` 复用纯 C 控制算法并暂存目标。
- 纯 C 控制库：PID、前馈、复合前馈 PID、斜坡限幅、角度工具、速度环和位置-速度串级。
- IMU：独立 ImuSource / ImuState / ImuReceiver；BMI088 可选四元数 EKF 与 PWM 温控，DM-IMU-L1 通过主动 RS485 帧接入统一快照。

### 通信与机器人算法

- `AsyncUart`：固定缓冲、异步 UART、溢出代际检测和单线程消费约定。
- DR16：18 字节帧解码、摇杆死区、拨杆、鼠标/键盘和在线判断。
- 裁判系统：RM2026 V1.3 profile、CRC8/CRC16、权限和功率快照。
- 板间协议：固定帧格式、CRC16、序列号、boot_id、resume generation 和四类底盘消息。
- 机器人算法：CommandArbiter 同步仲裁；CommandManager 注册来源并后台发布快照；GimbalAxis、SwerveChassis 和应用私有执行器分别完成子系统计算与本地恢复。

## 构建一个样例

命令从 west 工作区根目录执行；如果当前目录就是 `skywalker_code`，则可直接使用下面的源码路径。构建目录建议每个目标独立保存：

```bash
west build -p -b dm_mc02/stm32h723xx \
  -d build/dm_mit_velocity \
  samples/motor/dm_mit_velocity_control

west flash -d build/dm_mit_velocity
```

其他适合首次验证的目标：

```bash
# 不接电机，只观察 CAN 接收
west build -p -b dm_mc02/stm32h723xx -d build/can_smoke samples/motor/can_smoke

# 纯控制算法自检
west build -p -b dm_mc02/stm32h723xx -d build/control samples/control

# IMU、EKF、恒温和 VOFA+
west build -p -b dm_mc02/stm32h723xx -d build/imu samples/imu_test

# DJI 位置环
west build -p -b rm_typec -d build/dji_position samples/motor/dji_position_control
```

修改 `prj.conf`、overlay 或板卡配置后，首次验证建议使用 `-p` 做 pristine build。不要把多个样例共用一个构建目录。

## 目录地图

```text
skywalker_code/
├── boards/                     本仓库维护的 Zephyr boards
├── dts/bindings/               IMU、Kalman 等设备 binding
├── drivers/
│   ├── imu/                    独立 IMU source 与采集封装
│   ├── kalman_filter/          通用 Kalman 设备
│   └── motor/                  共享 CAN I/O、DJI / 达妙协议
├── include/                    公共头文件，按 drivers/lib 对应组织
├── lib/
│   ├── control/                纯 C 控制与 C++ 电机封装
│   ├── communication/          UART、DR16、裁判、板间协议
│   ├── robotics/               命令、安全、舵轮、云台算法
│   ├── matrix/                 CMSIS-DSP 矩阵封装和可选 Flash 存储
│   └── vofa/                   VOFA+ JustFloat
├── samples/                    单一功能和真实外设台架样例
├── applications/               sentry_chassis / sentry_gimbal 应用骨架
├── tests/motor/regression/     电机软件回归测试
├── docs/                       当前文档与原始 PDF 手册
├── west.yml                    Zephyr 与依赖 revision
└── zephyr/module.yml           module、board_root、dts_root 声明
```

## 重要约定

1. 设备树提供物理 CAN 与 UART 设备；电机型号、ID、限幅和 Group 关系在应用 C++ 中配置。
2. DJI 的统一 effort 单位是 A；DM 的 MIT effort 单位是 N·m，代码不会自动换算两者。
3. `motor::Feedback::position_rad` 是首帧归零的连续输出轴角度，可在禁用状态通过 `reseedPosition()` 建立已知坐标；GM6020 的 `absolute_position_rad` 是固定零点单圈角。
4. 同一物理 CAN 只创建一个 `CanBus`：各 `Motor` 暂存目标后，由总线 `commit()` 提交；只有机械联动的电机才放入同一个 `Group`。
5. `disable()` 立即撤销软件输出许可，安全帧异步发送；这不等于机械制动，也不切断板上动力电源。反馈恢复后仍需新的显式 `enable()`。
6. `samples/` 是已存在的验证入口；`applications/sentry_*` 是需要按真实机器人修改 overlay 和 `src/board_config.hpp` 的应用骨架，不应被描述为开箱即用整机固件。
7. `tests/motor/regression/` 有电机软件回归测试；控制、通信和安全链路仍需结合样例、日志与硬件台架验证。

电机调试前必须让机构悬空或脱离负载，准备物理断电手段，并先从低限幅开始。完整的安全检查见 [12 故障排查](docs/guides/troubleshooting.md)。

## 资料

`docs/` 中同时保留了 DJI 电机/电调、达妙电机/开发板和 RoboMaster C 板的 PDF 手册；下载地址见 [docs/datasheets/DOWNLOAD_LINKS.txt](docs/datasheets/DOWNLOAD_LINKS.txt)。
