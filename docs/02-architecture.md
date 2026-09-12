# 02 架构与构建

本文说明框架的分层设计、仓库布局，以及 Zephyr module / Kconfig / CMake
如何把这些代码组织起来。理解这一层可以避免「改了代码但没进构建」和
「调用到了另一个同名 PID」这两类高频问题。

---

## 1. 分层视图

```text
┌──────────────────────── 应用层 samples/ ─────────────────────────┐
│  hello / imu_test / control / motor/*                            │
│  ├─ C 应用：直接调 imu_fetch/imu_estimate、control_* 纯函数       │
│  └─ C++ 应用：VelocityMotor / PositionMotor + 品牌 Backend        │
└───────┬───────────────────────────────┬─────────────────────────┘
        │ Zephyr device API（C ABI）     │ C++ 类 / 纯函数 API
┌───────▼────────────────┐   ┌───────────▼──────────────────────────┐
│ drivers/（设备驱动）    │   │ lib/（算法与工具）                    │
│  imu（EKF + 温控）      │   │  control · pid/feedforward/          │
│  kalman_filter         │   │            slew_rate/angle/          │
│  motor/dji（C++）       │   │            motor_velocity/position   │
│  motor/dm（C++）        │   │  matrix  · CMSIS-DSP 封装            │
│                         │   │  vofa    · 上位机协议                │
└───────┬────────────────┘   └───────────┬──────────────────────────┘
        │ DT 实例化（DEVICE_DT_DEFINE）  │
┌───────▼────────────────────────────────▼──────────────────────────┐
│ Zephyr 内核 / HAL（CAN、SPI、UART、PWM、sensor 子系统、flash …）   │
└───────────────────────────────────────────────────────────────────┘
```

关键设计：

- **驱动＝Zephyr device**：`imu`、`kalman_filter`、`motor/*` 都通过设备树
  实例化，应用用 `DEVICE_DT_GET()/DEVICE_DT_GET(DT_ALIAS(...))` 取句柄。
- **算法＝纯函数**：`lib/control/*` 不注册设备、不做动态分配，只吃
  `config/state/input`、吐 `output`，因此可以在任意线程/任务里复用。
- **品牌差异＝后端**：DJI 与 DM 的 CAN 协议不同，被 `MotorBackend` 抽象隔开；
  上层的 `VelocityMotor/PositionMotor` 只认「读反馈、写 effort」。
- **C++ 只在必要处**：电机协议层与统一封装是 C++20；IMU/Kalman 仍是 C。
  C 与 C++ 通过 `extern "C"` + Zephyr device API 交界。

---

## 2. 仓库布局

```text
skywalker_code/                 # west module
├── boards/                     # 板级支持（board_root）
│   ├── damiao/dm_mc02/         #   达妙 MC02
│   └── rm_typec/               #   大疆 C 板
├── dts/bindings/               # 设备树 binding（dts_root）
│   ├── imu/  kalman_filter/  motor/
├── drivers/                    # 设备驱动（DT 实例化）
│   ├── imu/  kalman_filter/
│   └── motor/{dji,dm}/
├── lib/                        # 算法/工具库
│   ├── control/  matrix/  vofa/
├── include/                    # 公共头文件（与 drivers/lib 一一对应）
│   ├── control/  drivers/  lib/
├── samples/                    # 可构建样例
├── application/                # 空占位模板（src/main.c 为空文件）
├── docs/                       # 文档 + 原始手册 PDF
├── west.yml                    # manifest（锁定 Zephyr revision）
└── zephyr/module.yml           # module 声明
```

> `application/` 目前是**空占位**：`CMakeLists.txt`、`prj.conf`、
> `src/main.c` 均为空文件，不属于模块构建，也没有可运行逻辑。

---

## 3. module 机制

`zephyr/module.yml`：

```yaml
name: "skywalker"
build:
  kconfig: Kconfig
  cmake: .
  settings:
    board_root: .
    dts_root: .
```

含义：

- `kconfig: Kconfig` → 根 `Kconfig`，引入 `drivers/Kconfig` 与 `lib/Kconfig`。
- `cmake: .` → 根 `CMakeLists.txt` 会作为 module 被 Zephyr 包含。
- `board_root: .` → `boards/` 进入板卡搜索路径，`dm_mc02`/`rm_typec` 可直接用。
- `dts_root: .` → `dts/bindings/` 进入 binding 搜索路径。

根 `CMakeLists.txt`（节选）：

```cmake
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
zephyr_include_directories(include)
add_subdirectory(drivers)
add_subdirectory(lib)
```

即所有公共头文件都在 `include/` 下，用 `<control/...>`、`<drivers/...>`、
`<lib/...>` 形式包含。

### west manifest

`west.yml` 用 `name-allowlist` 只导入必要子项目，并锁定 Zephyr revision。
新增依赖必须同时加进 allowlist，否则 `west update` 不会拉取。

---

## 4. Kconfig 门控

### drivers

```text
CONFIG_SKYWALKER_DRIVER_KALMAN_FILTER   # drivers/kalman_filter
CONFIG_SKYWALKER_DRIVER_IMU             # drivers/imu（select CMSIS_DSP_FASTMATH + SKYWALKER_LIB_CONTROL）
CONFIG_SKYWALKER_DRIVER_MOTOR           # drivers/motor（depends on CAN）
  ├── CONFIG_SKYWALKER_MOTOR_DJI        # DJI 族，default y
  └── CONFIG_SKYWALKER_MOTOR_DM         # 达妙族
```

电机相关参数（`drivers/motor/Kconfig`）：

| 配置项 | 默认 | 说明 |
|---|---|---|
| `SKYWALKER_MOTOR_INIT_PRIORITY` | 90 | 电机设备初始化优先级 |
| `SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS` | 20 | DJI 反馈新鲜度超时 |
| `SKYWALKER_DJI_COMMAND_TIMEOUT_MS` | 10 | DJI 命令缓存超时 |
| `SKYWALKER_DJI_MAX_MOTORS_PER_BUS` | 12 | 单 Bus 最大电机数 |
| `SKYWALKER_DJI_MAX_BUSES` | 3 | 最大 Bus 数 |
| `SKYWALKER_DM_FEEDBACK_TIMEOUT_MS` | 50 | 达妙反馈新鲜度超时 |
| `SKYWALKER_DM_COMMAND_TIMEOUT_MS` | 20 | 达妙命令缓存超时 |
| `SKYWALKER_DM_MAX_MOTORS_PER_BUS` | 8 | 单 Bus 最大电机数 |
| `SKYWALKER_DM_MAX_MASTER_IDS_PER_BUS` | 4 | 单总线不同 Master ID 数 |

### lib

```text
CONFIG_SKYWALKER_LIB_MATRIX            # select CMSIS_DSP + CMSIS_DSP_MATRIX
CONFIG_SKYWALKER_LIB_MATRIX_STORAGE    # 矩阵落 flash（select FILE_SYSTEM/ZMS/FLASH_*）
CONFIG_SKYWALKER_LIB_VOFA              # depends on SERIAL
CONFIG_SKYWALKER_LIB_CONTROL           # 纯 C 算法（始终可用）
CONFIG_SKYWALKER_LIB_MOTOR_CONTROL     # C++ 封装（depends on CPP + MOTOR + 某品牌）
```

### CMake 门控

`drivers/CMakeLists.txt` / `lib/CMakeLists.txt` 用
`add_subdirectory_ifdef(CONFIG_..., dir)` 决定子目录是否参与构建；
`lib/control/CMakeLists.txt` 再按 `CONFIG_SKYWALKER_LIB_MOTOR_CONTROL`
决定是否编译 `motor_control.cpp` 与品牌后端。

> **因此：只写代码、不开 Kconfig，对应文件不会进构建。**
> 新增一个后端要同时改 Kconfig、CMakeLists、binding 与驱动表。

---

## 5. 两条主要数据流

### 姿态解算

```text
BMI088 accel/gyro (sensor 子系统)
        │ imu_fetch()
        ▼
imu_data.accel/gyro/temp ──► imu_estimate(dt)
        │                         │ 查 estimator 表（"ekf"）
        │                         ▼
        │                  kalman_filter 设备（F/H/R/X/P/Q/K）
        │                  predict（陀螺积分 + 零偏）→ correct（加速度观测量）
        ▼
imu_data.angle[roll,pitch,yaw]
        │ imu_heat_control() ~10 Hz
        ▼
control_feedforward_pid_step → pwm_set(heat_dev, ch4, 20 ms 周期)
```

### 电机控制

```text
CAN 中断 → 驱动 rxCallback → 解码 → Feedback 缓存到电机设备 data
                                            │
应用线程 / 控制线程                          │ readFeedback()/MotorBackend::read()
        │                                   ▼
        │  control_pid_step / motor_velocity_step（纯 C）
        ▼
   effort（A 或 N·m）
        │ setCurrent()/setMitCommand()，或 MotorBackend::write() + flush()
        ▼
   驱动 Bus 组帧 → can_send()（周期 flush）
```

---

## 6. 命名与语义约定

- **别名前缀**：所有 Kconfig 均为 `SKYWALKER_`，binding 用 `skywalker,*`
  与 `dji,*` / `dm,*`。
- **单位**：`rad`、`rad/s`、`A`、`N·m`；设备树里毫秒/毫弧度/毫牛米字段在后缀标出。
- **状态**：`Offline → Ready → Fault`（电机设备级）；
  `Uninitialized → Safe → Armed → Fault`（Bus 级）；
  `Idle → Starting → Running → Stopped/Fault`（统一封装运行时）。
- **错误码**：`-EINVAL` 参数非法；`-ERANGE` 越界/超时；`-ENOTSUP` 缺能力；
  `-ESTALE`/`-EHOSTDOWN` 反馈失效；`-EBUSY`/`-EALREADY` 生命周期误用。
- **并发**：驱动与封装的业务 API 归**单线程**；CAN 回调只做解码与拷贝。

---

## 7. 一个高频陷阱：两个 PID

| | `drivers/pid` | `lib/control/pid.h` |
|---|---|---|
| 形态 | Zephyr 设备（`skywalker,pid`） | 纯 C 结构体函数 |
| 数值来源 | 设备树 string 属性 | C 结构体字段 |
| 调用 | device API | `control_pid_step()` |
| 用途 | 早期设备型 PID | 现在所有算法与封装的底座 |

**IMU 温控、`motor_velocity/position`、`VelocityMotor` 用的都是
`lib/control` 的纯函数版本**。改错文件不会报错，但完全不生效。

---

## 8. 相关文档

- [03 板级支持](03-boards.md)
- [08 纯 C 控制算法](08-control-algorithms.md)
- [09 统一速度/位置封装](09-motor-wrapper.md)
