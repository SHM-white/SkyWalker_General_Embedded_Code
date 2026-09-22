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

- 交互式查看双主控架构：[网页源码与本地打开说明](docs/architecture-browser/README.md) · [在线浏览（需访问权限）](https://skywalker-architecture-browser.docile-raven-1977.chatgpt.site/)

- 第一次配置环境： [docs/01-getting-started.md](docs/01-getting-started.md)
- 想理解项目分层： [docs/02-architecture.md](docs/02-architecture.md)
- 想先跑硬件： [docs/10-samples.md](docs/10-samples.md)
- 调 DJI 电机： [docs/04-drivers-motor-dji.md](docs/04-drivers-motor-dji.md)
- 调达妙电机： [docs/05-drivers-motor-dm.md](docs/05-drivers-motor-dm.md)
- 做双主控通信： [docs/13-communication.md](docs/13-communication.md)
- 做底盘 / 云台安全链路： [docs/14-robotics.md](docs/14-robotics.md)
- 运行 `sentry_*` 应用骨架： [docs/15-applications.md](docs/15-applications.md)
- 遇到问题： [docs/12-troubleshooting.md](docs/12-troubleshooting.md)

完整文档索引在 [docs/README.md](docs/README.md)。

## 当前能力

### 板级支持

| board | MCU | 当前仓库内主要用途 |
|---|---|---|
| `dm_mc02/stm32h723xx` | STM32H723，480 MHz | 达妙 MC02，三路 FDCAN、BMI088、遥控/板间串口、电机应用 |
| `rm_typec` | STM32F407，168 MHz | RoboMaster Type-C C 板，CAN1/CAN2、BMI088、USB CDC、串口 |

板卡、设备树和烧录器配置位于 `boards/`；详细外设和 runner 见 [03 板级支持](docs/03-boards.md)。

### 驱动与控制

- DJI：M3508-C620、M2006-C610、GM6020 电流模式；支持反馈解码、输出轴角度、温度和总线分组发送。
- 达妙：DM-J4310-2EC V1.1；支持 MIT、位置-速度、速度三种原生 CAN 模式，以及 Enable/Disable/ClearError/SaveZero。
- 统一电机封装：`MotorBackend`、`MotorRuntime`、`VelocityMotor`、`PositionMotor`，包含反馈新鲜度、真实 dt、超速/超温保护和有限重试恢复。
- 纯 C 控制库：PID、前馈、复合前馈 PID、斜坡限幅、角度工具、速度环和位置-速度串级。
- IMU：BMI088 + `skywalker,imu` + 四元数 EKF + 恒温 PWM；底层使用 Kalman 设备和 CMSIS-DSP。

### 通信与机器人算法

- `AsyncUart`：固定缓冲、异步 UART、溢出代际检测和单线程消费约定。
- DR16：18 字节帧解码、摇杆死区、拨杆、鼠标/键盘和在线判断。
- 裁判系统：RM2026 V1.3 profile、CRC8/CRC16、权限和功率快照。
- 板间协议：固定帧格式、CRC16、序列号、boot_id、resume generation 和四类底盘消息。
- 机器人算法：人工指令映射、命令管理、全局/局部安全、四轮舵向运动学、Yaw 云台和台架功率限幅。

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
├── dts/bindings/               IMU、Kalman、电机 binding
├── drivers/
│   ├── imu/                    IMU 设备驱动
│   ├── kalman_filter/          通用 Kalman 设备
│   └── motor/{dji,dm}/         DJI / 达妙 CAN 驱动
├── include/                    公共头文件，按 drivers/lib 对应组织
├── lib/
│   ├── control/                纯 C 控制与 C++ 电机封装
│   ├── communication/          UART、DR16、裁判、板间协议
│   ├── robotics/               命令、安全、舵轮、云台算法
│   ├── matrix/                 CMSIS-DSP 矩阵封装和可选 Flash 存储
│   └── vofa/                   VOFA+ JustFloat
├── samples/                    单一功能和真实外设台架样例
├── applications/               sentry_chassis / sentry_gimbal 应用骨架
├── application/                空的历史占位目录，不是当前应用入口
├── docs/                       当前文档与原始 PDF 手册
├── west.yml                    Zephyr 与依赖 revision
└── zephyr/module.yml           module、board_root、dts_root 声明
```

## 重要约定

1. 设备树负责实例化驱动；`status = "okay"` 的节点才会进入设备构建和运行时。
2. DJI 的统一 effort 单位是 A；DM 的 MIT effort 单位是 N·m，代码不会自动换算两者。
3. `motor::Feedback::position_rad` 是连续输出轴角度，首个参考点由驱动/封装生命周期决定；GM6020 的 `absolute_position_rad` 才是固定零点单圈角。
4. 多个原生电机实例必须由同一物理 CAN 的一个 `Bus` 统一 `flush()`；统一 `DjiMotorBackend` / `DmMotorBackend` 是单电机独占封装。
5. 电机 `stop()` 是撤回命令/发送禁用或零输出，不等于机械制动，也不切断板上动力电源。
6. `samples/` 是已存在的验证入口；`applications/sentry_*` 是需要按真实机器人修改 overlay 和 `src/board_config.hpp` 的应用骨架，不应被描述为开箱即用整机固件。
7. 当前仓库没有独立 `tests/` 测试树，控制、通信和安全链路主要通过样例与日志进行台架验证。

电机调试前必须让机构悬空或脱离负载，准备物理断电手段，并先从低限幅开始。完整的安全检查见 [12 故障排查](docs/12-troubleshooting.md)。

## 资料

`docs/` 中同时保留了 DJI 电机/电调、达妙电机/开发板和 RoboMaster C 板的 PDF 手册；下载地址见 [docs/DOWNLOAD_LINKS.txt](docs/DOWNLOAD_LINKS.txt)。

