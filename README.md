# 巡天御风通用电控框架（SkyWalker General Embedded Code）

> 基于 [Zephyr RTOS](https://zephyrproject.org/) 的机器人通用电控代码框架，以 **west module** 形式组织，覆盖板级支持、设备驱动、控制算法库与可运行样例，用于电机控制、IMU 姿态解算、在线调试等电控日常开发。

---

## 特性

- **模块化 Zephyr 工程**：`west.yml` 锁定 Zephyr 版本，仓库自身作为 module（`zephyr/module.yml`）挂入构建；设备树 binding、板卡、驱动、库均可在模块内一站式维护。
- **DJI 电机驱动（C++）**：M3508-C620、M2006-C610、GM6020（电流环）三种 profile，多机共线管理（Bus / attach / arm / flush / stop），反馈自动解码、编码器连续角度展开、状态机与故障安全停机。
- **达妙 DM 电机驱动（C++）**：DM-J4310-2EC V1.1，支持 MIT / 位置-速度 / 速度三种控制模式，MIT 模式可给力矩；自带 Enable/Disable/ClearError/SaveZero 特殊命令与按 `master-id` 的反馈路由。
- **统一速度/位置封装（C++）**：`MotorBackend` 抽象品牌差异，`VelocityMotor` / `PositionMotor` 复用同一套速度/位置内核与 `MotorRuntime` 生命周期、dt 校验、超速/超温保护与故障停机。
- **IMU 姿态解算**：通用传感器解算器接口 + 内嵌 EKF（四元数 + 陀螺零偏在线估计 + 卡方抗扰动），并带恒温加热控制；底层复用 Kalman 滤波驱动与 CMSIS-DSP 矩阵库。
- **控制算法库（纯 C、零动态分配）**：PID（条件积分抗饱和、测量微分低通）、前馈（重力/静摩擦/速度/加速度补偿）、前馈+反馈复合控制器、非对称斜坡限幅、连续角度解包，以及速度/位置串级控制内核。
- **调试链路**：VOFA+ JustFloat 协议上位机绘图、UART `key=value` 命令行调参。
- **板级支持**：达妙 MC02（STM32H723）与大疆 RoboMaster Type-C C 板（STM32F407）。
- **完整文档**：`docs/` 下有按主题编排的框架文档与原始手册 PDF，见 [文档导航](docs/README.md)。

---

## 硬件支持

| 板卡 | board 名 | SoC | 主要外设（设备树） | 烧录 runner |
|---|---|---|---|---|
| [达妙 MC02](boards/damiao/dm_mc02) | `dm_mc02/stm32h723xx` | STM32H723VG，Cortex-M7 @480 MHz | FDCAN1/2/3 ×3、BMI088（SPI2，accel+gyro）、WS2812（SPI6）、USART×4/RS485、电源使能、flash 三分区（MCUboot 预留） | **openocd（默认）**、pyocd、stm32cubeprogrammer、jlink |
| [大疆 RoboMaster Type-C C 板](boards/rm_typec) | `rm_typec` | STM32F407IG，Cortex-M4 @168 MHz | CAN1/CAN2、BMI088（SPI1）、USB CDC-ACM、USART×3、CS43L22 音频、蜂鸣器 | openocd（外接探针） |

> `west flash` 的默认 runner 由 `board.cmake` 中**第一个注册**的 runner 决定；以构建目录下 `zephyr/runners.yaml` 的 `flash-runner` 为准（本仓库两块板默认都是 openocd）。探针用 `-DSKYWALKER_OPENOCD_PROBE=cmsis-dap|stlink|stlink-hla` 选择。
>
> 说明：`dm_mc02` 的 flash 容量与分区布局以设备树（1 MB 三分区）为准；board.yml 中记录的容量仅作 Zephyr 元数据。

---

## 软件架构

```
┌──────────────────────── 应用层 samples/ ─────────────────────────┐
│  hello / imu_test / control / motor/*                             │
│  ├─ C 应用：imu_fetch/imu_estimate、control_* 纯函数              │
│  └─ C++ 应用：VelocityMotor / PositionMotor + 品牌 Backend         │
└───────┬───────────────────────────────┬─────────────────────────┘
        │ Zephyr device API（C ABI）     │ C++ 类 / 纯函数 API
┌───────▼──────────────┐   ┌────────────▼──────────────────────────┐
│ drivers/（设备驱动）  │   │ lib/（算法与工具）                    │
│  imu（EKF+温控）      │   │  control · pid/feedforward/          │
│  kalman_filter       │   │            slew_rate/angle/          │
│  motor/dji（C++）     │   │            motor_velocity/position   │
│  motor/dm（C++）      │   │  matrix  · CMSIS-DSP 矩阵封装          │
│  pid（设备型 PID）    │   │  vofa    · VOFA+ 上位机协议            │
└───────┬──────────────┘   └───────────────────────────────────────┘
        │ DT 实例化（phandle/属性 → DEVICE_DT_DEFINE）
┌───────▼──────────────────────────────────────────────────────────┐
│ Zephyr 内核 / HAL（CAN、SPI、UART、PWM、传感器子系统 …）          │
└──────────────────────────────────────────────────────────────────┘
```

> `VelocityMotor` / `PositionMotor` 通过 `MotorBackend` 组合驱动：上层只认
> 「读反馈、写 effort」，DJI（电流 A）与 DM（MIT 力矩 N·m）各自实现后端。

### 仓库布局

```text
skywalker_code/                 # west module（module.yml：kconfig/cmake/board_root/dts_root）
├── boards/                     # 板级支持
│   ├── damiao/dm_mc02/         #   达妙 MC02
│   └── rm_typec/               #   大疆 C 板
├── dts/bindings/               # 设备树 binding（skywalker,* / dji,* / dm,*）
│   └── imu/  kalman_filter/  motor/
├── drivers/                    # 设备驱动（DT 实例化）
│   ├── imu/  kalman_filter/  pid/
│   └── motor/{dji,dm}/         #   DJI / 达妙电机（C++）
├── lib/                        # 算法/工具库
│   ├── control/  matrix/  vofa/
├── include/                    # 公共头文件（与 drivers/lib 一一对应）
│   ├── control/  drivers/  lib/
├── samples/                    # 可构建样例（含 motor/dm_common 共享支持代码）
├── application/                # 空占位模板目录（src/main.c 为空文件）
├── docs/                       # 主题文档 + 原始手册 PDF
├── west.yml                    # west manifest（锁定 Zephyr revision）
└── zephyr/module.yml
```

### 构建组织

- **manifest**：`west.yml` 只导入 `cmsis / cmsis-dsp / hal_st / hal_stm32 / mcuboot / segger` 等白名单子项目，Zephyr 版本以 `6085aade…` revision 锁定。
- **module**：`zephyr/module.yml` 声明 `board_root: .` 与 `dts_root: .`，因此 `boards/` 与 `dts/bindings/` 自动进入 Zephyr 的板卡/binding 搜索路径。
- **编译单元**：根 `CMakeLists.txt` 固定 C11/C++20，`zephyr_include_directories(include)`；`drivers/` 与 `lib/` 下每个子目录用**无参 `zephyr_library()`** 打包，是否参与构建由 `CONFIG_SKYWALKER_*` 门控（`add_subdirectory_ifdef`）。
- **Kconfig**：`drivers/Kconfig`（`SKYWALKER_DRIVER_KALMAN_FILTER / IMU / MOTOR`，电机族再分 `SKYWALKER_MOTOR_DJI / DM`）与 `lib/Kconfig`（`SKYWALKER_LIB_MATRIX / VOFA / CONTROL / MOTOR_CONTROL`）。公共前缀均为 `SKYWALKER_`。
- **文档**：主题文档在 `docs/`（见 [文档导航](docs/README.md)），原始手册 PDF 与下载直链也在该目录。

### 设备驱动（drivers/）

| 驱动 | compatible | 头文件 | 说明 |
|---|---|---|---|
| IMU | `skywalker,imu` | `drivers/imu/imu.h` | 通用解算器接口（`imu_fetch / imu_estimate / imu_heat_control`），内置 `"ekf"` 四元数解算器；通过 phandle 组合 accel/gyro/PWM/filter，并使用 `lib/control` 完成温控 |
| Kalman 滤波 | `skywalker,kalman_filter` | `drivers/kalman_filter/kalman_filter.h` | 通用 `F/H/R/X/P/Q/K` 滤波设备，CMSIS-DSP 实现，被 IMU-EKF 复用 |
| DJI 电机 | `dji,gm6020-current` / `dji,m3508-c620` / `dji,m2006-c610` | `drivers/motor/*.hpp`（C++） | 见下文 |
| 达妙 DM 电机 | `dm,j4310-2ec-v1-1` | `drivers/motor/dm_motor.hpp`（C++） | 见下文 |

### DJI 电机驱动（C++，`skywalker::motor::dji`）

- **Profile 表**：三种电机各自定义反馈 ID 基址、命令帧分组、电流量程与温度有效性。

  | 型号 | 反馈帧 | 命令帧 | 电流满幅 | 备注 |
  |---|---|---|---|---|
  | M3508-C620 | `0x200+id`（id 1–8） | `0x200` / `0x1FF` | 20 A | 温度有效 |
  | M2006-C610 | `0x200+id`（id 1–8） | `0x200` / `0x1FF` | 10 A | 无温度反馈 |
  | GM6020（电流环） | `0x204+id`（id 1–7） | `0x1FE` / `0x2FE` | 3 A | 温度有效；需固件 ≥1.0.11.2 |

- **对外接口**（Zephyr device + C++ 助手混合风格）：
  - 通用抽象 `skywalker::motor`：`State / Capability / Feedback / Api`，通过 `motor.hpp` 内联助手 `setCurrent / readFeedback / getState` 作用于 `const struct device *`。
  - `dji::describe()` 读取模型与描述符；`dji::Bus`：`init(can) → attach(motor) → arm() → flush()` 周期发送，失败自动清零进入 `Fault`，`stop()/recover()` 安全停机。
  - 协议层 `dji_protocol.hpp`：`decodeFeedback()`（大端 8 字节：编码器/RPM/电流/温度）与 `buildCommandFrame()`。
- **关键行为**：反馈回调做 8192 tick/圈编码器连续展开，并按 `gear-ratio-num/den` 换算到输出轴；命令发送受 armed 生命周期（epoch）与新鲜度约束；`current-limit-ma` 由设备树配置并软件钳位。DJI 族**只有电流模式**（`setTorque` 返回 `-ENOTSUP`）；GM6020 在减速比 1:1 时额外提供固定零点单圈角 `absolute_position_rad`。
- **配置项**：`CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS`（默认 20）、`COMMAND_TIMEOUT_MS`（10）、`MAX_MOTORS_PER_BUS`（12）、`MAX_BUSES`（3）、`MOTOR_INIT_PRIORITY`（90）。

### 达妙 DM 电机驱动（C++，`skywalker::motor::dm`）

- **Profile**：DM-J4310-2EC V1.1，`control-mode` 与电机持久化模式一致，取 `mit` / `position-velocity` / `velocity` 之一。

  | 模式 | 命令帧 ID | 载荷 |
  |---|---|---|
  | MIT（模式 1） | `0x000+id` | 位置 16 位 + 速度/KP/KD/力矩各 12 位打包 |
  | 位置-速度（模式 2） | `0x100+id` | 两个小端 float：位置、速度上限 |
  | 速度（模式 3） | `0x200+id` | 一个小端 float（DLC=4） |

- **协议量程**：设备树 `p-max-millirad` / `v-max-millirad-s` / `t-max-millinewton-meter`（对应调试助手的 PMAX/VMAX/TMAX），`torque-limit-millinewton-meter` 是软件力矩上限。
- **对外接口**：`dm::describe()`、`setMitCommand()`、`setPositionVelocity()`、`setVelocity()`、`readRawFeedback()`、`getDriveStatus()`；`dm::Bus` 提供 `init/attach/arm/flush/stop/recover/savePositionZero` 与 `TxReport`。
- **状态与故障**：反馈帧带驱动状态码（过压/欠压/过流/过温/通讯丢失/过载），故障会锁存并自动撤销 arm；`recover()` 发送 ClearError + Disable。
- **配置项**：`CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS`（50）、`COMMAND_TIMEOUT_MS`（20）、`MAX_MOTORS_PER_BUS`（8）、`MAX_MASTER_IDS_PER_BUS`（4）。
- **限制**：未实现协议模式 4（混合控制）；连续位置解包假设反馈在 ±PMAX 回绕。

### 统一速度/位置控制（C++，`skywalker::control`）

- `MotorBackend`（虚接口）负责品牌差异：`describe / prepare / read / arm / write / flush / stop`。
- `DjiMotorBackend`（effort 单位 A）与 `DmMotorBackend`（effort 单位 N·m，仅 MIT）。
- `MotorRuntime` 管生命周期（`Idle → Starting → Running → Stopped/Fault`）、真实 dt 校验、反馈新鲜度、超速/超温保护与故障停机。
- `VelocityMotor::update(rad/s)`、`PositionMotor::update(rad)`，位置支持 `StartupRelative` / `DriverContinuous` / `AbsoluteNearest` 三种参考。
- 约束：单电机、独占总线、单线程；对象需 static 存活到应用结束；多轴需自行实现共享 Bus 调度。

### 控制算法库（lib/control，纯 C，无动态分配）

| 头文件 | 模块 | 要点 |
|---|---|---|
| `pid.h` | PID | `kp/ki/kd` + 积分限幅/输出限幅（支持不对称）、死区、`freeze_integrator` 条件积分抗饱和、D 为测量值变化率的一阶低通、dt 越界返回 `-ERANGE` |
| `feedforward.h` | 前馈 | bias + 静摩擦（方向由速度/加速度过阈值判定）+ 速度项 + 加速度项 + 重力项（`SIN`/`COS` 模型） |
| `feedforward_pid.h` | 复合控制 | 前馈计算 + PID 加性注入一体化的 `validate/reset/step` |
| `slew_rate_limiter.h` | 斜坡限幅 | 非对称上升/下降速率，输出限幅值与实际速率 |
| `angle.h` | 角度处理 | 连续角度解包、最短角差（归一到 `[-π, π)`） |
| `motor_velocity.h` | 速度内核 | 测量低通 + 请求斜坡 + 软死区 + 复合前馈 PID，输出 effort（A 或 N·m） |
| `motor_position.h` | 位置内核 | 位置–速度串级，位置积分冻结，支持可选物理目标前馈 |

> `SKYWALKER_LIB_MOTOR_CONTROL` 打开时额外编译 `motor_control.cpp` 与品牌后端
> （`dji_motor_backend.cpp` / `dm_motor_backend.cpp`），即第 9 节的统一封装。

### 工具库（lib/）

- `matrix`：`Matrix` = `arm_matrix_instance_f32` 的宏封装 + `arm_mat_*` 映射；可选的 flash 矩阵存储后端（`SKYWALKER_LIB_MATRIX_STORAGE`）。IMU-EKF 与 Kalman 驱动的底层。
- `vofa`：TX 为 VOFA+ **JustFloat** 协议（float 小端 + `0x7F800000` 帧尾，DMA 发送）；RX 为 DMA + IDLE 行解析，`key=value\n` 触发 `vofa_cmd_handler`，用于在线调参。

---

## 快速开始

### 环境准备

1. 安装 Zephyr 依赖与 Zephyr SDK（参见 [Zephyr 官方文档](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)），当前依赖 Zephyr ~v4.4。
2. 创建 west 工作区并更新：

   ```bash
   west init -m https://github.com/NUAAwyx/SkyWalker_General_Embedded_Code.git skywalker_ws
   cd skywalker_ws && west update
   ```

   或对已有工作区：`west update`（revision 已锁定在 `west.yml`）。

### 构建与烧录（以样例为例）

```bash
# 在工作区根执行
west build -p -b dm_mc02/stm32h723xx -d build/imu_test samples/imu_test   # IMU+VOFA
west build -p -b dm_mc02/stm32h723xx -d build/dm_pos samples/motor/dm_mit_position_control
west build -p -b rm_typec -d build/dji_pos samples/motor/dji_position_control
west flash -d build/dm_pos        # 默认 openocd；探针可选 cmsis-dap/stlink/stlink-hla
```

> 样例目录内 `app.overlay` 自动生效；板卡专属 overlay 放 `<sample>/boards/<board>.overlay`。
> 需要指定构建目录时加 `-d build/<name>`；改过 overlay/prj.conf 后建议用 `-p` 重新 pristine 构建。
> 更多见 [快速开始](docs/01-getting-started.md)。

### 最小电机控制用法

1. 在样例 `app.overlay` 中声明电机（设备树实例化）：

   ```dts
   / {
       aliases { motor0 = &gm6020_1; };
       gm6020_1: motor-1 {
           compatible = "dji,gm6020-current";
           can-bus = <&can1>;
           motor-id = <1>;
           current-limit-ma = <100>;   /* 软件电流钳位，上限 3000 mA */
           gear-ratio-num = <1>; gear-ratio-den = <1>;
           encoder-zero-ticks = <0>;    /* GM6020 机械零位对应编码器 tick（必填） */
           current-loop-confirmed;      /* GM6020 电流环固件确认（必填） */
       };
   };
   ```

2. `prj.conf` 打开对应开关：`CONFIG_SKYWALKER_DRIVER_MOTOR=y`、`CONFIG_SKYWALKER_MOTOR_DJI=y`。

3. 速度/位置闭环使用 `VelocityMotor` / `PositionMotor`，按 `begin → update(目标)@周期 → stop` 调用；配置 `CONFIG_SKYWALKER_LIB_MOTOR_CONTROL=y` 和 `CONFIG_STD_CPP20=y`。达妙 MIT 与 DJI 分别选择对应后端，详见 [统一速度与位置控制](samples/motor/MOTOR_CONTROL.md)。

---

## 样例一览

| 样例 | 说明 |
|---|---|
| `hello/` | 最小工程 + VOFA 单通道（uptime，`usart1`） |
| `cpp_test/` | 最小工程冒烟（当前实际是 C 版 hello） |
| `control/` | `lib/control` 算法自检（复合前馈 PID / 斜坡 / 角度断言） |
| `imu_test/` | IMU+EKF+恒温加热+VOFA 上位机绘图（`dm_mc02`，overlay 在 `boards/`） |
| `motor/can_smoke/` | CAN 收帧冒烟测试（不发送命令） |
| `motor/dji_unified/` | GM6020 开环电流老炼/安全测试（限速限温保护停机） |
| `motor/dji_speed_control/` | `VelocityMotor` 速度环：复合前馈 PID + 斜坡限幅，正弦速度指令 |
| `motor/dji_position_control/` | **推荐参考**：`PositionMotor` 位置–速度串级，支持固定零点最短路径和连续相对目标；位置积分保持冻结 |
| `motor/m2006_speed_control/` | M2006 速度环派生样例（gear-ratio 36:1） |
| `motor/dm_mit_control/` | DM 原生 MIT 力矩（0.5 N·m，200 s，超速保护） |
| `motor/dm_velocity_control/` | DM 原生速度模式（0.5 rad/s） |
| `motor/dm_position_control/` | DM 原生位置-速度模式（0°/+90° 切换） |
| `motor/dm_mit_velocity_control/` | `VelocityMotor` + DM 后端，软件速度环输出力矩 |
| `motor/dm_mit_position_control/` | `PositionMotor` + DM 后端，连续正向 90° 位置序列 |

> `motor/dm_common/` 不是样例，而是达妙样例共享的支持代码（`enableMotorPower` 等）。
> `application/` 是空占位目录，`src/main.c` 为空文件，不属于模块构建。
> 完整样例说明见 [样例索引](docs/10-samples.md)。

---

## 常见坑位与约定

1. **PID 属于 control 层**：PID 是纯数值算法，不注册 Zephyr device；引用算法请使用 `control_pid_*` / `control_feedforward_pid_*` 与 `include/control/` 下的头文件。注意 `drivers/pid`（设备型）与 `lib/control/pid.h`（算法）是两套东西。
2. **GM6020 电流环**需电机固件 ≥1.0.11.2，且设备树节点必须声明 `current-loop-confirmed;`，`current-limit-ma` 上限 3000 mA。
3. **单位与模式**：DJI 只有电流模式（effort = A，`setTorque` 返回 `-ENOTSUP`）；DM 只有 MIT 模式支持力矩（effort = N·m），`setCurrent` 返回 `-ENOTSUP`。统一封装要求显式选择 `EffortUnit`，不做电流/力矩换算。
4. **多电机同总线**：DJI 多个轴必须共用一个 `dji::Bus` 统一 flush，不能各自建 Bus；DM 按 `master-id` 路由反馈。
5. **C++ 栈**：C++ 样例建议调大 `CONFIG_MAIN_STACK_SIZE`（如 8192）。
6. **断点会破坏控制时序**：软件闭环周期超过 `dt_max_s`（通常 20 ms）会触发停机，首次调试不要下断点。
7. **接线与安全**：电机上电前确认机构悬空、备好急停/断电；样例中均有 arm 前等待反馈、失败自动 `stop()` 清零的设计，实机调试请勿跳过。

---

## 文档导航

`docs/` 下有按主题编排的完整文档，建议从 [docs/README.md](docs/README.md) 进入：

| 文档 | 主题 |
|---|---|
| [01 快速开始](docs/01-getting-started.md) | 环境、构建、烧录、最小工程 |
| [02 架构与构建](docs/02-architecture.md) | 分层、module/Kconfig/CMake、数据流 |
| [03 板级支持](docs/03-boards.md) | `dm_mc02` / `rm_typec` 外设与 runner |
| [04 DJI 电机驱动](docs/04-drivers-motor-dji.md) / [05 达妙 DM 驱动](docs/05-drivers-motor-dm.md) | 协议、binding、Bus 生命周期 |
| [06 IMU 与 EKF](docs/06-drivers-imu.md) / [07 Kalman 与矩阵库](docs/07-kalman-matrix.md) | 姿态解算底层 |
| [08 纯 C 控制算法](docs/08-control-algorithms.md) / [09 统一控制封装](docs/09-motor-wrapper.md) | 算法库与速度/位置封装 |
| [10 样例索引](docs/10-samples.md) / [11 调试](docs/11-debugging.md) / [12 故障排查](docs/12-troubleshooting.md) | 使用与排障 |

---

## 参考资料

- 官方手册与下载直链：`docs/`（PDF）与 `docs/DOWNLOAD_LINKS.txt`（M3508/C620、M2006/C610、GM6020、DM-J4310 等）。
- 板卡默认配置见各 `board_defconfig` 与 `boards/<vendor>/<board>/` 设备树；binding 定义见 `dts/bindings/`。
- 统一封装的详细迁移说明：`samples/motor/MOTOR_CONTROL.md`；达妙示例：`samples/motor/DM_J4310_EXAMPLES.md`。

## 当前状态

> 本仓库处于持续迭代中（分支 `dev`）。`application/` 为占位目录；`samples/motor/` 是当前唯一的样例集合。README 与 `docs/` 将随框架演进同步更新。

