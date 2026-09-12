# 01 快速开始

本文覆盖从零把仓库跑起来所需的全部步骤：环境、依赖更新、构建、烧录，
以及一个最小自建工程的骨架。

---

## 1. 前置条件

| 依赖 | 说明 |
|---|---|
| Zephyr SDK | 仓库当前按 Zephyr ~v4.4 开发，`west.yml` 已锁定 revision |
| `west` | manifest 工具，`west.yml` 已导入 `cmsis / cmsis-dsp / hal_st / hal_stm32 / mcuboot / segger` |
| CMake ≥ 3.20 | 根 `CMakeLists.txt` 要求 |
| C++20 编译器 | 电机驱动与统一封装是 C++（`CONFIG_STD_CPP20=y`） |
| 调试器 | `dm_mc02` 可用板载/外接调试器；`rm_typec` 需外接（CMSIS-DAP 或 ST-Link） |

Zephyr 环境变量（`ZEPHYR_BASE`、SDK 路径、`west` 命令）需按官方 Getting Started 配好。

---

## 2. 获取工作区

本仓库自身是一个 **west module**，需要放进 west 工作区里构建。

```bash
# 新建工作区（首次）
west init -m https://github.com/NUAAwyx/SkyWalker_General_Embedded_Code.git skywalker_ws
cd skywalker_ws
west update

# 已有工作区：revision 已锁定，直接拉齐
west update
```

工作区里会形成：

```text
skywalker_ws/
├── zephyr/            # 由 west.yml 拉取
└── skywalker_code/    # 本仓库（manifest 的 self.path）
```

> 本仓库作为 module 被 Zephyr 自动发现（`zephyr/module.yml` 声明
> `board_root: .` 与 `dts_root: .`），因此 `boards/`、`dts/bindings/`
> 无需额外配置即可被构建系统搜索到。

---

## 3. 构建

样例位于 `samples/`。构建命令必须在**工作区根**执行（或任意目录，只要
west 能定位到 manifest）。

```bash
# 板级 qualifier 可省略：dm_mc02 会自动解析为 dm_mc02/stm32h723xx
west build -p -b dm_mc02/stm32h723xx samples/imu_test

# 指定构建目录（推荐，便于区分多个样例）
west build -p -b dm_mc02/stm32h723xx -d build/imu_test samples/imu_test

# 大疆 C 板
west build -p -b rm_typec samples/motor/dji_position_control
```

要点：

- `-p`（pristine）先清空构建目录；改过设备树 overlay / prj.conf 后建议加 `-p`。
- 样例目录内的 `app.overlay` 会自动生效；`imu_test` 的 overlay 放在
  `samples/imu_test/boards/dm_mc02.overlay`（板卡专属写法）。
- 构建成功末尾会打印 FLASH/RAM 占用，例如 `dm_mit_position_control`
  约为 FLASH 11.5%、RAM 7.5%。

---

## 4. 烧录

### dm_mc02

`board.cmake` 中 **第一个注册的 runner 是 `openocd`**，因此 `west flash`
不带 `--runner` 时默认走 openocd（与早期 README 中“默认 pyocd”的说法不同，
以 `build/zephyr/runners.yaml` 的 `flash-runner: openocd` 为准）。

```bash
west flash                      # 默认 openocd
west flash -r pyocd             # 备选：pyocd（--target=stm32h723vgtx）
west flash -r stm32cubeprogrammer
west flash -r jlink
```

OpenOCD 的探针通过 CMake cache 变量选择，可选 `cmsis-dap`（默认）、
`stlink`、`stlink-hla`：

```bash
west build -p -b dm_mc02/stm32h723xx -DSKYWALKER_OPENOCD_PROBE=stlink-hla samples/imu_test
```

### rm_typec

无板载调试器，走外接 OpenOCD（`boards/rm_typec/board.cmake` 已接入
`boards/common/openocd.board.cmake`）。探针同样用 `SKYWALKER_OPENOCD_PROBE`
选择：

```bash
west flash -r openocd
west build -p -b rm_typec -DSKYWALKER_OPENOCD_PROBE=stlink samples/motor/dji_position_control
```

> WSL2 环境注意：ST-Link 需先 `usbipd attach --wsl --busid <id>` 透传，
> 否则 OpenOCD 报 `Error: open failed`。透传不持久，重启后要重做。

---

## 5. 跑一个最小样例

```bash
west build -p -b dm_mc02/stm32h723xx -d build/hello samples/hello
west flash -d build/hello
```

`hello` 只验证工具链与串口日志。要验证完整链路，按下面的顺序升级：

1. `samples/control` — 纯算法自检，无需硬件。
2. `samples/imu_test` — IMU + EKF + 温控 + VOFA（需 `dm_mc02`）。
3. `samples/motor/can_smoke` — 只收 CAN 帧，不驱动电机。
4. 具体电机样例 — **务必先悬空输出轴并准备物理断电**。

---

## 6. 最小自建工程骨架

推荐直接复制一个现有样例作为起点，例如 `samples/motor/dm_mit_position_control/`：

```text
my_app/
├── CMakeLists.txt     # find_package(Zephyr) + project() + target_sources
├── prj.conf           # Kconfig 开关
├── app.overlay        # 自定义设备树（电机节点 / alias）
└── src/main.cpp
```

`prj.conf` 打开所需模块（示例）：

```conf
CONFIG_CPP=y
CONFIG_STD_CPP20=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_CAN=y

# 选一族电机驱动
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DM=y          # 或 CONFIG_SKYWALKER_MOTOR_DJI=y

# 若要用统一速度/位置封装
CONFIG_SKYWALKER_LIB_MOTOR_CONTROL=y

# C++ 样例建议调大主栈
CONFIG_MAIN_STACK_SIZE=8192
```

`app.overlay` 声明电机节点并建 alias（统一封装通过 `DT_ALIAS(motor0)` 取设备）：

```dts
/ {
    aliases {
        motor0 = &dm_j4310_1;
    };

    dm_j4310_1: motor-1 {
        compatible = "dm,j4310-2ec-v1-1";
        status = "okay";
        can-bus = <&can1>;
        motor-id = <1>;
        master-id = <0x11>;
        control-mode = "mit";
        p-max-millirad = <12500>;
        v-max-millirad-s = <30000>;
        t-max-millinewton-meter = <10000>;
        torque-limit-millinewton-meter = <1000>;
    };
};
```

---

## 7. 上电与安全顺序

电机类样例的通用顺序，任何一步都不能跳：

1. **机械**：输出轴可靠悬空/脱离负载，人手可立即断电。
2. **电气**：确认 CAN 波特率（DJI/DM 均为 1 Mbps）、电机 ID、主机 ID、
   电源使能通道（MC02 需先开 XT30）。
3. **固件**：配置与电机实际参数一致（DM 的 PMAX/VMAX/TMAX、GM6020 的
   `current-loop-confirmed`）。
4. **上电**：先上控制板，再上电机电源；观察日志中的反馈握手。
5. **运行**：保持很低的限幅起步，每次只调一个参数。
6. **异常**：任何 `update()`/`begin()` 失败都要检查 `status().stop_error`，
   并物理断电。

---

## 8. 相关文档

- 目录与外设：[03 板级支持](03-boards.md)
- 构建机制：[02 架构与构建](02-architecture.md)
- 样例清单：[10 样例索引](10-samples.md)
- 出问题：[12 故障排查](12-troubleshooting.md)
