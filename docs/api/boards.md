# 板级：MC02 / RoboMaster Type-C：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

把物理引脚、外设、alias、时钟、DMA与电源连接交给Zephyr，业务参数保留在应用配置。

**接入状态：已有代码。** 两套板级存在；正式整车实物参数集中 calibration.hpp，connections_confirmed 默认 false；样例可另有 board_config。

## 职责与关联

DTS描述物理设备与连接，overlay选择应用实际使用的设备，Kconfig决定哪些实现进入构建，board_config.hpp提供电机ID/限幅/机械方向/链路身份。它们是四个不同层次。

输入 / 依赖：直接对接底层设备或算法。

消费者：[双主控应用与执行器](application.md)、[DJI 电机与共享 CAN 总线](motor-dji.md)、[达妙电机：MIT / 速度 / 位置速度](motor-dm.md)、[IMU：独立采集、姿态与加热](imu.md)、[异步 UART 与 DMA](uart.md)、[双主控板间通信](interboard.md)

## 接口契约

### 1. DEVICE_DT_GET(DT_NODELABEL(can1)); DEVICE_DT_GET(DT_ALIAS(remote_uart)); DEVICE_DT_GET(DT_ALIAS(accel0)); DEVICE_DT_GET(DT_ALIAS(gyro0))

```cpp
DEVICE_DT_GET(DT_NODELABEL(can1)); DEVICE_DT_GET(DT_ALIAS(remote_uart)); DEVICE_DT_GET(DT_ALIAS(accel0)); DEVICE_DT_GET(DT_ALIAS(gyro0))
```

取得Zephyr已描述设备的指针；node label指DTS节点标号，alias指应用连接名称。获取指针不启动业务驱动或允许电机输出。

| 参数 | 含义与边界 |
| --- | --- |
| `DT_NODELABEL(name)` | 直接选择节点label，例如物理can1/usart1。 |
| `DT_ALIAS(name)` | 选板或overlay约定的alias，DTS中横线在C宏里变下划线：remote-uart→remote_uart。 |

**返回 / 输出：** const device*；缺节点/未生成device通常是编译或链接错误，不是运行期空指针。

**线程 / 时序：** 构造/初始化阶段使用；先device_is_ready，再交给Motor/UART/IMU等模块。

**错误 / 边界：** Type-C的alias usart1指USART6，node label usart1指物理USART1，两者不可混淆；各应用overlay可能改alias。

### 2. bool device_is_ready(const device *dev)

```cpp
bool device_is_ready(const device *dev)
```

确认Zephyr驱动初始化成功，只证明设备层可用；不证明远端电机在线、UART协议正确或传感器姿态已初始化。

| 参数 | 含义与边界 |
| --- | --- |
| `dev` | 从DEVICE_DT_GET得到的设备。 |

**返回 / 输出：** true已就绪；false应用通常返回-ENODEV。

**线程 / 时序：** 初始化线程和必要的读取位置，无业务I/O。

**错误 / 边界：** 各模块还需要自己的start/init与反馈检查。

### 3. PWM_DT_SPEC_GET(DT_ALIAS(imu_heater))

```cpp
PWM_DT_SPEC_GET(DT_ALIAS(imu_heater))
```

取得加热PWM设备、通道、周期和极性，交给ImuHeater::Config::pwm。MC02为heat通道4，Type-C为通道1；默认PWM周期20ms。

| 参数 | 含义与边界 |
| --- | --- |
| `imu_heater` | 板级alias，不是自动温控算法。 |

**返回 / 输出：** pwm_dt_spec值。

**线程 / 时序：** 初始化阶段，真实写PWM由ImuHeater所有者线程完成。

**错误 / 边界：** PWM spec存在不表示温控已启动或有效温度已就绪；缺alias无法编译。

### 4. int regulator_enable(const device *dev)

```cpp
int regulator_enable(const device *dev)
```

MC02的power1/power2固定电源口默认regulator-boot-off，应用显式启用后才给相应XT30支路供电。

| 参数 | 含义与边界 |
| --- | --- |
| `dev` | DEVICE_DT_GET(DT_NODELABEL(power1)) 或power2，先device_is_ready。 |

**返回 / 输出：** 0成功；负值由Zephyr regulator驱动返回。

**线程 / 时序：** 启动线程，不能假设所有板卡都有power1/power2；用DT_NODE_EXISTS条件适配。

**错误 / 边界：** 确认支路连接后启用；开启电源不等于Motor软件enable，设备反馈与实际执行仍独立观测。

### 5. app.overlay + prj.conf + src/board_config.hpp

```cpp
app.overlay + prj.conf + src/board_config.hpp
```

三种应用配置入口：overlay声明物理资源/alias；prj.conf打开模块；board_config提供真实协议参数与机械约定。

| 参数 | 含义与边界 |
| --- | --- |
| `app.overlay` | 启用CAN/UART/传感器、设波特率/引脚、按应用绑定alias。 |
| `prj.conf` | CAN、UART API、CPP、FPU、业务模块Kconfig选项。 |
| `board_config.hpp` | 电机地址/模式/零点/量程，速度电流力矩限幅，传输角色和connections_configured。 |

**返回 / 输出：** 构建时配置，不是运行期接口。

**线程 / 时序：** 先把本应用连接参数填写完整，再构建；上车固件初始化应据connections_configured决定是否允许运动。

**错误 / 边界：** alias正确并不保证协议一致；修改overlay需pristine build。不要把电机型号/Group写成物理DTS驱动节点。

## 调用示例

### MC02：显式开启所需XT30，并取得物理CAN

```cpp
#include <zephyr/device.h>
#include <zephyr/drivers/regulator.h>
#include <drivers/motor/can_bus.hpp>
#include <cerrno>

int initializeBoardPower() {
#if DT_NODE_EXISTS(DT_NODELABEL(power1))
  // 仅在确认本车电机供电确实使用XT30_1后调用。
  const device *power=DEVICE_DT_GET(DT_NODELABEL(power1));
  if (!device_is_ready(power)) return -ENODEV;
  const int r=regulator_enable(power);
  if (r<0) return r;
#endif
  const device *can=DEVICE_DT_GET(DT_NODELABEL(can1));
  return device_is_ready(can) ? 0 : -ENODEV;
}

// 后续业务初始化仍需：构造长寿命Motor/CanBus，attach，start，
// 运行意图有效时持续enable/update/commit，设备独立恢复；不能因电源已开启直接写运动目标。

```

此例沿用现有DM和恢复样例的Zephyr调用；连接确认由应用负责。Type-C无power1节点时跳过MC02专有逻辑。board device pointer与CanBus/Motor对象生命周期是两个层次。

### 构建入口与物理外设配置

```cpp
// C++取设备；这是node label，始终选择物理can1。
const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
// UART按应用alias取；可能由app.overlay重新映射。
const device *telemetry = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
// MC02: telemetry=USART1，remote=UART5，console/shell=USART10。
// Type-C: telemetry=USART6，remote=USART3，console/shell=USART1。

```

板目标：dm_mc02/stm32h723xx、rm_typec。示例构建命令：west build -p -b dm_mc02/stm32h723xx -d build/dual_imu samples/imu/dual_imu。此任务仅展示命令，不运行固件构建或烧录。

## 调用顺序

1. 确定板卡与实际连接，选board目标。
2. overlay绑定物理设备和alias；prj.conf纳入模块并选择UART/FPU/缓存策略。
3. 应用取得device并检查ready；按实际供电需要显式开启MC02电源支路。
4. 依赖资源构造长寿命业务对象并按模块生命周期启动。
5. 机械方向、零点、量程、通信身份与控制增益确认后才解除应用connections_configured门槛。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `dm_mc02/stm32h723xx` | STM32H723，DTS CPU480MHz；CAN1/2/3，BMI088 SPI2，RS485 USART2/3，默认console USART10。 |
| `rm_typec` | STM32F407，DTS CPU168MHz；CAN1/2，BMI088，UART1/3/6，console USART1、telemetry USART6。 |
| `SKYWALKER_OPENOCD_PROBE` | board.cmake支持cmsis-dap、stlink、stlink-hla；烧录探针依板实际连接。 |
| `CONFIG_NOCACHE_MEMORY / CONFIG_FPU_SHARING` | MC02默认启用nocache区；含浮点工作线程的样例显式配置FPU共享。 |
| `connections_configured` | 应用/云台样例中的接线确认开关；应和真实硬件/协议/机械限制一致，不能用一次编译成功代替确认。 |

- MC02有can1/2/3，Type-C有can1/2；不能照抄第三路CAN或RS485 alias。
- 当前Type-C DTS的console/shell是USART1，telemetry是USART6；alias usart1也指USART6，不能从名字猜物理设备。
- MC02的XT30电源默认关闭；电机离线可能是供电未显式启用而非CAN解码问题。
- H7 DMA缓存一致性需__nocache缓冲和CONFIG_NOCACHE_MEMORY；业务对象不一定全部放nocache，DMA实际字节存储需按接口要求放置。
- 不同UART API/console/VOFA/协议收发器不能各自同时独占同一物理UART。
- DTS只负责硬件描述，Group、Motor型号和电机限幅仍由C++应用配置。

## 正文与源码

- [板级Markdown](../getting-started/boards.md)
- [DMA与缓存](../guides/uart-dma.md)

- [MC02设备树](../../boards/damiao/dm_mc02/dm_mc02.dts)
- [MC02默认配置](../../boards/damiao/dm_mc02/dm_mc02_defconfig)
- [MC02烧录配置](../../boards/damiao/dm_mc02/board.cmake)
- [Type-C设备树](../../boards/rm_typec/rm_typec.dts)
- [Type-C默认配置](../../boards/rm_typec/rm_typec_defconfig)
- [双IMU板级参数示例](../../samples/imu/dual_imu/src/board_config.hpp)
- [云台接线参数示例](../../samples/robotics/gimbal_control/src/board_config.hpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
