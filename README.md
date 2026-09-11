# 巡天御风通用电控框架（SkyWalker General Embedded Code）

> 基于 [Zephyr RTOS](https://zephyrproject.org/) 的机器人通用电控代码框架，以 **west module** 形式组织，覆盖板级支持、设备驱动、控制算法库与可运行样例，用于电机控制、IMU 姿态解算、在线调试等电控日常开发。

---

## 特性

- **模块化 Zephyr 工程**：`west.yml` 锁定 Zephyr 版本，仓库自身作为 module（`zephyr/module.yml`）挂入构建；设备树 binding、板卡、驱动、库均可在模块内一站式维护。
- **DJI 电机驱动（C++）**：M3508-C620、M2006-C610、GM6020（电流环）三种 profile，多机共线管理（Bus / attach / arm / flush / stop），反馈自动解码、编码器连续角度展开、状态机与故障安全停机。
- **IMU 姿态解算**：通用传感器解算器接口 + 内嵌 EKF（四元数 + 陀螺零偏在线估计 + 卡方抗扰动），并带恒温加热控制；底层复用 Kalman 滤波驱动与 CMSIS-DSP 矩阵库。
- **控制算法库（纯 C、零动态分配）**：PID（条件积分抗饱和、测量微分低通）、前馈（重力/静摩擦/速度/加速度补偿）、前馈+反馈复合控制器、非对称斜坡限幅、连续角度解包。
- **调试链路**：VOFA+ JustFloat 协议上位机绘图、UART `key=value` 命令行调参。
- **板级支持**：达妙 MC02（STM32H723）与大疆 RoboMaster Type-C C 板（STM32F407）。

---

## 硬件支持

| 板卡 | board 名 | SoC | 主要外设（设备树） | 烧录 runner |
|---|---|---|---|---|
| [达妙 MC02](boards/damiao/dm_mc02) | `dm_mc02` | STM32H723VG，Cortex-M7 @480 MHz | FDCAN1/2/3 ×3、BMI088（SPI2，accel+gyro）、WS2812（SPI6）、USART×4/RS485、SBUS、电源使能、flash 三分区（MCUboot 预留） | pyocd（默认）/ stm32cubeprogrammer / openocd / jlink |
| [大疆 RoboMaster Type-C C 板](boards/rm_typec) | `rm_typec` | STM32F407IG，Cortex-M4 @168 MHz | CAN1/CAN2、BMI088（SPI1）、USB CDC-ACM、USART×3、CS43L22 音频、蜂鸣器 | openocd |

> 说明：`dm_mc02` 的 flash 容量与分区布局以设备树（1 MB 三分区）为准；board.yaml 中记录的 512 KB 仅作 Zephyr 元数据，可能存在出入。

---

## 软件架构

```
┌──────────────────────── 应用层 samples/ ────────────────────────┐
│  hello / imu_test / dji_position_control / dji_speed_control …  │
└───────┬───────────────────────────────┬─────────────────────────┘
        │ Zephyr device API（C ABI）     │ 纯函数式算法 API
┌───────▼──────────────┐   ┌────────────▼──────────────────────────┐
│ drivers/（设备驱动）  │   │ lib/（算法库）                        │
│  imu  · ekf 姿态解算  │   │  control · pid / feedforward /       │
│  kalman_filter       │   │            slew_rate / angle          │
│  pid （设备型 PID）  │   │  matrix  · CMSIS-DSP 矩阵封装          │
│  motor/dji（C++）    │   │  vofa    · VOFA+ 上位机协议            │
└───────┬──────────────┘   └───────────────────────────────────────┘
        │ DT 实例化（phandle/属性 → DEVICE_DT_DEFINE）
┌───────▼──────────────────────────────────────────────────────────┐
│ Zephyr 内核 / HAL（CAN、SPI、UART、PWM、传感器子系统 …）          │
└──────────────────────────────────────────────────────────────────┘
```

### 仓库布局

```text
skywalker_code/                 # west module（module.yml：kconfig/cmake/board_root/dts_root）
├── boards/                     # 板级支持
│   ├── damiao/dm_mc02/         #   达妙 MC02
│   └── rm_typec/               #   大疆 C 板
├── dts/bindings/               # 设备树 binding（skywalker,* 与 dji,*）
├── drivers/                    # 设备驱动（DT 实例化）
│   ├── imu/  kalman_filter/  pid/
│   └── motor/dji/              #   DJI 电机（C++）
├── lib/                        # 算法/工具库
│   ├── control/  matrix/  vofa/
├── include/                    # 公共头文件（与 drivers/lib 一一对应）
├── samples/                    # 可构建样例
├── application/                # 空占位模板目录（供独立 west build 使用）
├── motor_docs/                 # 官方手册下载直链
├── west.yml                    # west manifest（锁定 Zephyr revision）
└── zephyr/module.yml
```

### 构建组织

- **manifest**：`west.yml` 只导入 `cmsis / cmsis-dsp / hal_st / hal_stm32 / mcuboot / segger` 等白名单子项目，Zephyr 版本以 `6085aade…` revision 锁定。
- **module**：`zephyr/module.yml` 声明 `board_root: .` 与 `dts_root: .`，因此 `boards/` 与 `dts/bindings/` 自动进入 Zephyr 的板卡/binding 搜索路径。
- **编译单元**：根 `CMakeLists.txt` 固定 C11/C++20，`zephyr_include_directories(include)`；`drivers/` 与 `lib/` 下每个子目录用**无参 `zephyr_library()`** 打包，是否参与构建由 `CONFIG_SKYWALKER_*` 门控（`add_subdirectory_ifdef`）。
- **Kconfig**：`drivers/Kconfig`（`SKYWALKER_DRIVER_KALMAN_FILTER / IMU / MOTOR`）与 `lib/Kconfig`（`SKYWALKER_LIB_MATRIX / VOFA / CONTROL`）。公共前缀均为 `SKYWALKER_`。

### 设备驱动（drivers/）

| 驱动 | compatible | 头文件 | 说明 |
|---|---|---|---|
| IMU | `skywalker,imu` | `drivers/imu/imu.h` | 通用解算器接口（`imu_fetch / imu_estimate / imu_heat_control`），内置 `"ekf"` 四元数解算器；通过 phandle 组合 accel/gyro/PWM/filter，并使用 `lib/control` 完成温控 |
| Kalman 滤波 | `skywalker,kalman_filter` | `drivers/kalman_filter/kalman_filter.h` | 通用 `F/H/R/X/P/Q/K` 滤波设备，CMSIS-DSP 实现，被 IMU-EKF 复用 |
| DJI 电机 | `dji,gm6020-current` / `dji,m3508-c620` / `dji,m2006-c610` | `drivers/motor/*.hpp`（C++） | 见下文 |

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
- **关键行为**：反馈回调做 8192 tick/圈编码器连续展开；命令发送受 armed 生命周期（epoch）与新鲜度约束；`current-limit-ma` 由设备树配置并软件钳位。当前**仅实现 `setCurrent`（电流/力矩），`setTorque` 为 nullptr**。
- **配置项**：`CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS`（默认 20）、`COMMAND_TIMEOUT_MS`（10）、`MAX_MOTORS_PER_BUS`（12）、`MAX_BUSES`（3）、`MOTOR_INIT_PRIORITY`（90）。

### 控制算法库（lib/control，纯 C，无动态分配）

| 头文件 | 模块 | 要点 |
|---|---|---|
| `pid.h` | PID | `kp/ki/kd` + 积分限幅/输出限幅（支持不对称）、死区、`freeze_integrator` 条件积分抗饱和、D 为测量值变化率的一阶低通、dt 越界返回 `-ERANGE` |
| `feedforward.h` | 前馈 | bias + 静摩擦（方向由速度/加速度过阈值判定）+ 速度项 + 加速度项 + 重力项（`SIN`/`COS` 模型） |
| `feedforward_pid.h` | 复合控制 | 前馈计算 + PID 加性注入一体化的 `validate/reset/step` |
| `slew_rate_limiter.h` | 斜坡限幅 | 非对称上升/下降速率，输出限幅值与实际速率 |
| `angle.h` | 角度处理 | 连续角度解包、最短角差（归一到 `[-π, π)`） |

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
# 在仓库根（或任何样本目录）执行
west build -p -b dm_mc02  samples/imu_test                    # IMU+VOFA 样例
west build -p -b rm_typec samples/motor/dji_position_control  # 电机位置环样例
west flash                # dm_mc02 默认 pyocd；rm_typec 用 openocd
```

> 样例目录内自定义设备树写在其 `app.overlay`（自动生效）；需要指定构建目录时加 `-d build/<name>`。

### 最小电机控制用法

1. 在样例 `app.overlay` 中声明电机（设备树实例化）：

   ```dts
   / {
       aliases { motor0 = &gm6020_1; };
       gm6020_1: motor-1 {
           compatible = "dji,gm6020-current";
           can-bus = <&can1>;
           motor-id = <1>;
           current-limit-ma = <100>;   /* 软件电流钳位 */
           gear-ratio-num = <1>; gear-ratio-den = <1>;
           current-loop-confirmed;      /* GM6020 电流环固件确认 */
       };
   };
   ```

2. `prj.conf` 打开对应开关：`CONFIG_SKYWALKER_DRIVER_MOTOR=y`、`CONFIG_SKYWALKER_MOTOR_DJI=y`。

3. 主循环按 `Bus → attach → arm → (setCurrent + flush)@周期` 流程运行，参考 `samples/motor/dji_position_control/src/main.cpp`。

---

## 样例一览

| 样例 | 说明 |
|---|---|
| `hello/`、`cpp_test/` | 最小工程 / C++ 冒烟 |
| `control/` | `lib/control` 算法自检（复合前馈 PID 输出与饱和位断言） |
| `imu_test/` | IMU+EKF+恒温加热+VOFA 上位机绘图（`dm_mc02`，overlay 在 `boards/`） |
| `motor/can_smoke/` | CAN 收帧冒烟测试 |
| `motor/dji_unified/` | GM6020 开环电流老炼/安全测试（限速限温保护停机） |
| `motor/dji_speed_control/` | GM6020 速度环：复合前馈 PID + 斜坡限幅，正弦速度指令 |
| `motor/dji_position_control/` | **推荐参考**：位置–速度串级闭环，位置外环 PID（含爬行积分与死区）→ 斜坡 → 纯 P 速度内环 → 软件电流钳 |
| `motor/m2006_speed_control/` | M2006 速度环派生样例（gear-ratio 36:1） |
| `motor_demo/` | 早期自包含综合 demo（自带电机封装与手写 PID），**非本框架驱动，仅留档参考** |
| `motor_demos/` | 另一框架（Breeze API）的样例集合，与本仓库 `drivers/` 无代码复用，勿混用 |

> `application/` 为空占位目录，不属于模块构建；如需自建主程序可 `west build -b dm_mc02 application` 后自行填充 `src/main.c`。

---

## 常见坑位与约定

1. **PID 属于 control 层**：PID 是纯数值算法，不注册 Zephyr device；引用算法请使用 `control_pid_*` / `control_feedforward_pid_*` 与 `include/control/` 下的头文件。
2. **GM6020 电流环**需电机固件 ≥1.0.11.2，且设备树节点必须声明 `current-loop-confirmed;`，`current-limit-ma` 上限 3000 mA。
3. **当前仅电流模式**：`setTorque` 未实现；开环测试请用 `setCurrent`。
4. **C++ 栈**：C++ 样例建议调大 `CONFIG_MAIN_STACK_SIZE`（如 8192）。
5. **接线与安全**：电机上电前确认机构悬空、备好急停/断电；样例中均有 arm 前等待反馈、失败自动 `stop()` 清零的设计，实机调试请勿跳过。

---

## 参考资料

- 官方手册直链：`motor_docs/DOWNLOAD_LINKS.txt`（M3508/C620、M2006/C610、GM6020、DM-J4310 等 PDF）。
- 板卡默认配置见各 `board_defconfig` 与 `boards/<vendor>/<board>/` 设备树；binding 定义见 `dts/bindings/`。

## 当前状态

> 本仓库处于持续迭代中（分支 `dev`），部分目录（`application/`、`motor_demo/`、`motor_demos/`）为占位或留档性质，README 将随框架演进同步更新。
