# 01 快速开始

本文只描述当前仓库真实存在的构建入口。命令默认在 west 工作区根目录执行；如果 `skywalker_code` 就是当前目录，路径可以直接写成 `samples/...`。

## 1. 准备环境

需要：

- Zephyr 所需的 Python、CMake、Ninja、设备树工具和 west。
- 与当前 Zephyr revision 匹配的 Zephyr SDK / ARM GNU 工具链。
- `west.yml` 中列出的依赖：Zephyr、CMSIS、CMSIS-DSP、STM32 HAL、MCUboot、Segger 等。
- 对应板卡的调试器和 OpenOCD；`rm_typec` 没有板载调试器。

仓库通过 `west.yml` 锁定 Zephyr commit，并将自身以 `skywalker_code` 路径挂入工作区：

```yaml
self:
  path: skywalker_code
```

如果还没有工作区，可以：

```bash
west init -m https://github.com/NUAAwyx/SkyWalker_General_Embedded_Code.git skywalker_ws
cd skywalker_ws
west update
```

已有工作区只需确认 manifest 已包含本仓库，然后执行 `west update`。不要只复制本目录而跳过 manifest 依赖，否则 CMSIS-DSP 或板卡搜索路径可能不完整。

## 2. 第一个构建

建议先编译不接电机的控制样例：

```bash
west build -p -b dm_mc02/stm32h723xx \
  -d build/control \
  samples/control
```

再选择一个需要 CAN 的样例：

```bash
west build -p -b dm_mc02/stm32h723xx \
  -d build/can_smoke \
  samples/motor/can_smoke
```

`-p` 是 pristine build；修改 `prj.conf`、`app.overlay`、设备树 binding 或切换 board 后应优先使用它。每个样例使用不同的 `-d` 目录，避免 CMake 缓存串用。

如果从 workspace 根目录构建，而仓库位于 `skywalker_code/`，源码路径写成：

```bash
west build -p -b dm_mc02/stm32h723xx \
  -d build/control \
  skywalker_code/samples/control
```

## 3. 烧录与串口

```bash
west flash -d build/control
```

当前两块板的 board.cmake 都把 OpenOCD 注册为默认 runner。MC02 可以选择：

```bash
west flash -d build/control
```

当前板卡默认 runner 是 OpenOCD；探针选择是在构建阶段传入：

```bash
west build -p -b dm_mc02/stm32h723xx \
  -d build/control-stlink \
  -DSKYWALKER_OPENOCD_PROBE=stlink \
  samples/control
```

烧录前先检查：

1. 调试器目标电压和地线已连接。
2. `build/<name>/zephyr/runners.yaml` 中的 `flash-runner` 与实际探针一致。
3. `rm_typec` 已连接外部 CMSIS-DAP / ST-Link；它没有板载调试器。
4. 串口终端使用对应板卡的 console UART，默认 115200 8N1，具体以 board DTS 和样例 overlay 为准。

## 4. 按目标选择样例

| 目标 | 构建入口 |
|---|---|
| 最小启动、VOFA 单通道 | `samples/hello` |
| 纯 C 控制算法 | `samples/control` |
| IMU + EKF + 加热 + VOFA | `samples/imu_test`，目前针对 `dm_mc02` |
| CAN 只收帧不发命令 | `samples/motor/can_smoke` |
| DJI 原生 / 统一速度位置环 | `samples/motor/dji_unified`、`dji_speed_control`、`dji_position_control`、`m2006_speed_control` |
| 达妙原生模式 | `samples/motor/dm_mit_control`、`dm_velocity_control`、`dm_position_control` |
| 达妙 MIT 软件速度/位置环 | `samples/motor/dm_mit_velocity_control`、`dm_mit_position_control` |
| UART 通信链路 | `samples/communication/dr16`、`referee`、`interboard` |
| 安全/底盘算法台架 | `samples/robotics/command_safety`、`yaw_gimbal`、`swerve` |
| 电机掉电恢复 | `samples/motor/recovery` |

详表和硬件前提见 [10 样例索引](10-samples.md)。

## 5. 新建一个最小应用

一个当前仓库风格的应用至少包含：

```text
my_app/
├── CMakeLists.txt
├── prj.conf
├── app.overlay              # 需要设备树实例时
└── src/main.cpp 或 main.c
```

`CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app)
target_sources(app PRIVATE src/main.cpp)
```

C++ 电机/通信样例通常需要：

```conf
CONFIG_CPP=y
CONFIG_STD_CPP20=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_MAIN_STACK_SIZE=8192
CONFIG_LOG=y
CONFIG_SERIAL=y
CONFIG_CAN=y
```

然后根据使用的功能增加 `CONFIG_SKYWALKER_*`。不要在业务代码里手动包含 `lib/*.cpp`；模块的根 CMake 会按 Kconfig 加入库。

## 6. 设备树配置顺序

以电机为例：

1. 在 `app.overlay` 中声明电机 compatible、CAN phandle、ID、量程和减速比。
2. 打开 `CONFIG_SKYWALKER_DRIVER_MOTOR=y` 以及对应的 `SKYWALKER_MOTOR_DJI` 或 `SKYWALKER_MOTOR_DM`。
3. 用 `DEVICE_DT_GET(DT_ALIAS(motor0))` 获取设备。
4. 先确认 `device_is_ready()`、反馈时间戳、状态和能力位，再 arm。
5. 修改 overlay 后重新 pristine build。

所有必填属性以 `dts/bindings/` 为准；不要从旧样例复制已经删除的节点名。

## 7. 第一次上电顺序

1. 机构悬空或卸载，手边放置物理断电手段。
2. 只给 MCU 供电，确认能看到启动日志。
3. 接好 CAN 总线，核对终端电阻、波特率、ID 和电机供电。
4. 先运行 `can_smoke` 或只读反馈逻辑。
5. 将电流/力矩/速度限幅设为很小，确认反馈方向和零点。
6. 最后才允许样例调用 `arm()` / `resume()`。

`stop()`、`suspend()` 和急停状态都不能替代机械制动或切断动力电源。

## 8. 下一步

- 先理解构建门控：[02 架构与构建](02-architecture.md)
- 先确认硬件：[03 板级支持](03-boards.md)
- 调电机：[04 DJI](04-drivers-motor-dji.md)、[05 达妙](05-drivers-motor-dm.md)
- 写整机应用：[13 通信](13-communication.md)、[14 机器人算法](14-robotics.md)、[15 应用骨架](15-applications.md)
