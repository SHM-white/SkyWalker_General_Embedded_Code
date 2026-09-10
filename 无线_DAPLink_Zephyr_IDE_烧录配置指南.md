# 无线 DAPLink 在 Zephyr IDE 中烧录 SkyWalker 的配置指南

## 1. 本次检查结论

这次换上的探针已被当前 Linux/WSL 环境识别为一只标准 CMSIS-DAP 设备：

```text
产品：Horco CMSIS-DAP
VID:PID：faed:4870
序列号：288485685614
调试接口：/dev/hidraw0
虚拟串口：/dev/ttyACM0
```

当前用户属于 `plugdev` 和 `dialout` 组，两个设备节点也允许相应用户组读写。因此，现阶段不需要先改 udev 权限。

仓库的真正配置冲突是：

- `boards/damiao/dm_mc02/support/openocd.cfg` 写死了 `interface/stlink-dap.cfg` 和 `dapdirect_swd`；
- `boards/rm_typec/support/openocd.cfg` 也写死了同一套 ST-Link 接口；
- 新探针不是 ST-Link，而是 CMSIS-DAP，所以必须使用 `interface/cmsis-dap.cfg` 和普通 `swd` transport。

但是，在改仓库配置前还有一个更早的硬件状态需要解决。本次只对探针本身执行了不连接目标、不复位、不烧写的查询，结果如下：

```text
pyOCD 0.45.1: [Errno 110] Operation timed out
OpenOCD CMSIS-DAP v2 bulk: CMSIS-DAP command CMD_INFO failed
OpenOCD CMSIS-DAP v1 HID:  CMSIS-DAP command CMD_INFO failed
```

OpenOCD 已经能按 `faed:4870` 找到 CMSIS-DAP v2 USB 接口，但探针没有返回最基本的 `CMD_INFO`。这发生在访问目标 MCU 之前，所以当前还不能靠修改 Zephyr 配置完成烧录。优先确认无线发射端与接收端已经配对、接收端已稳定供电。很多同类无线 DAP 在两端指示灯常亮时才表示链路已建立；具体灯态仍以手上型号的说明书为准。

推荐的数据链路是：

```text
Zephyr IDE 的“烧录”按钮
  -> Runner Profile: west-flash / openocd
  -> west flash -r openocd
  -> 当前构建目录的 zephyr/runners.yaml
  -> boards/<当前板>/support/openocd.cfg
  -> interface/cmsis-dap.cfg
  -> CMSIS-DAP 无线发射端 <~~2.4 GHz~~> 接收端
  -> SWDIO + SWCLK + GND（推荐再接 NRST）
  -> STM32H723 或 STM32F407
```

## 2. 哪些文件需要改

只修改当前实际使用的板卡配置，不要两块板一起盲改。

| Zephyr IDE 中选择的板 | 芯片 | 必须手工修改 |
| --- | --- | --- |
| `dm_mc02/stm32h723xx` | STM32H723VG | `boards/damiao/dm_mc02/support/openocd.cfg` |
| `rm_typec/stm32f407xx` | STM32F407 | `boards/rm_typec/support/openocd.cfg` |

以下文件与烧录探针类型无关，不需要修改：

- `*.dts`、`*.overlay`；
- `prj.conf` 和 Kconfig；
- 应用及驱动的 `CMakeLists.txt`；
- 电机、CAN、IMU 等业务源码；
- `.vscode/tasks.json`；
- 只做烧录时的 `.vscode/launch.json`。

`.vscode/zephyr-ide.json` 已经存在名为 `DM MC02 OpenOCD` 的 Runner Profile，其中 Flash 是 `west-flash/openocd`。对当前已经绑定此 Profile 的 `dji_speed_control` 和 `m2006_speed_control` 的 DM MC02 构建配置，不需要手改 JSON，只要在插件界面确认它仍是 Active Runner Profile。

## 3. 第一步：先恢复无线 DAPLink 自身链路

在进行任何目标连接测试之前：

1. 断开电机功率供电，保证机构不会因旧固件继续运行或复位而动作。
2. 给无线接收端稳定供电；不要只看到 PC 端 USB 枚举就认为接收端也已工作。
3. 确认发射端和接收端已配对，且两端状态灯进入厂商定义的“已连接”状态。
4. 缩短距离，先放在同一张桌面上，并避开强 Wi-Fi、蓝牙和遥控器干扰。
5. 如果运行在 WSL，USB 拔插后确认设备已重新 attach。当前成功 attach 时应能看到：

   ```bash
   ls -l /dev/hidraw* /dev/serial/by-id/
   ```

   预期包含：

   ```text
   /dev/hidraw0
   usb-Horco_Horco_CMSIS-DAP_288485685614-if02 -> ../../ttyACM0
   ```

6. 在仓库根目录用工作区 Python 环境检查探针，不会烧写目标：

   ```bash
   ../.venv/bin/pyocd list
   ```

只有不再出现 `[Errno 110] Operation timed out`，并能列出序列号 `288485685614`，才继续下一步。

如果仍超时，先处理接收端供电、配对和无线链路；此时不要继续改 DTS、Kconfig 或反复重编译。

## 4. 第二步：把当前板的 OpenOCD 接口改为 CMSIS-DAP

### 4.1 使用 DM MC02 / STM32H723 时

打开：

```text
boards/damiao/dm_mc02/support/openocd.cfg
```

把整个文件手工改成：

```tcl
source [find interface/cmsis-dap.cfg]

transport select swd

source [find target/stm32h7x.cfg]
adapter speed 1000
```

逐行含义：

- `interface/cmsis-dap.cfg`：让 OpenOCD 使用 CMSIS-DAP，而不是 ST-Link 驱动；
- `transport select swd`：选择 STM32 Cortex-M 使用的两线 SWD；
- `target/stm32h7x.cfg`：保留 STM32H7 的目标和 Flash 算法；
- `adapter speed 1000`：初次连接先降到 1 MHz，提高无线链路和飞线条件下的容错率。

不要保留以下 ST-Link 专属内容：

```tcl
source [find interface/stlink-dap.cfg]
transport select dapdirect_swd
```

第一次稳定烧录后，可以把 `adapter speed 1000` 逐步提高到 `2000` 或原来的 `4000`。每次只改一个档位；出现随机超时、校验失败或断连就退回上一个稳定值。

### 4.2 使用 RM Type-C / STM32F407 时

打开：

```text
boards/rm_typec/support/openocd.cfg
```

把整个文件手工改成：

```tcl
# STM32F407 OpenOCD configuration for RM Type-C board + CMSIS-DAP
source [find interface/cmsis-dap.cfg]

transport select swd

source [find target/stm32f4x.cfg]

adapter speed 1000
```

STM32F407 仍然使用 `target/stm32f4x.cfg`，不能照抄 DM MC02 的 `stm32h7x.cfg`。

### 4.3 可选的探针筛选参数

只有同时插着多只 CMSIS-DAP、OpenOCD 选错设备时，才在 `source [find interface/cmsis-dap.cfg]` 后加入：

```tcl
cmsis_dap_vid_pid 0xfaed 0x4870
adapter serial 288485685614
```

平时不推荐写死序列号，否则以后再次更换探针还要改仓库。

本机 OpenOCD 0.12.0 会自动先尝试 CMSIS-DAP v2 bulk，再尝试 v1 HID。正常情况下不用指定 backend。如果无线固件恢复通信后只有 HID 可用，可再临时加入：

```tcl
cmsis_dap_backend hid
```

如果只有 bulk 可用则写：

```tcl
cmsis_dap_backend usb_bulk
```

本次实测两个 backend 都在 `CMD_INFO` 阶段超时，所以现在强制其中任意一个都不能替代无线链路恢复。

## 5. 第三步：Zephyr IDE 中选择正确 Runner

### 5.1 DM MC02

1. 在 Zephyr IDE 中选中要烧录的项目和 `build/dm_mc02/stm32h723xx` 构建配置。
2. 打开 Runner Profile 面板。
3. 选择仓库已有的 `DM MC02 OpenOCD`。
4. 确认 Flash 为：

   ```text
   kind: west-flash
   runner: openocd
   ```

5. 点击普通“烧录”，先不要用 F5 调试来混合排查烧录与 GDB 问题。

当前 `boards/damiao/dm_mc02/board.cmake` 把 pyOCD 放在默认 runner 的第一位，但上述 Profile 明确指定了 `openocd`，所以推荐路径不需要修改 `board.cmake`。

如果不用 Zephyr IDE Profile，而是直接执行不带 `-r` 的 `west flash`，DM MC02 会默认走 pyOCD。为避免走错 runner，终端命令必须明确写：

```bash
../.venv/bin/west flash \
  --build-dir samples/motor/m2006_speed_control/build/dm_mc02/stm32h723xx \
  -r openocd
```

把构建目录替换成当前项目的实际目录。

### 5.2 RM Type-C

RM Type-C 的 `board.cmake` 本来就只有 OpenOCD runner。仍建议在 Runner Profile 中明确选择 `west-flash/openocd`，避免 Cortex-Debug 自动拼出错误的 ST-Link 接口参数。

## 6. 第四步：先连接检查，再烧录

完成 cfg 修改且无线链路恢复后，先运行无烧写连接测试。该命令会连接并暂停 MCU，可能改变程序的实时运行状态，因此必须先断开电机功率回路。

### 6.1 DM MC02

```bash
openocd \
  -f boards/damiao/dm_mc02/support/openocd.cfg \
  -c 'init' \
  -c 'reset halt' \
  -c 'mdw 0xE000ED00 1' \
  -c 'shutdown'
```

预期看到 STM32H7/Cortex-M7 被识别、`target halted`，且 CPUID 地址返回非零值；不应再出现 `CMSIS-DAP command CMD_INFO failed`。

### 6.2 RM Type-C

```bash
openocd \
  -f boards/rm_typec/support/openocd.cfg \
  -c 'init' \
  -c 'reset halt' \
  -c 'mdw 0xE000ED00 1' \
  -c 'shutdown'
```

预期看到 STM32F4/Cortex-M4 被识别、`target halted`，且 CPUID 非零。

连接检查成功后再执行烧录。现有构建的 `zephyr/runners.yaml` 保存的是板级 cfg 路径，OpenOCD 每次启动都会重新读取 cfg，所以只改接口内容通常不要求 pristine rebuild：

```bash
../.venv/bin/west flash --build-dir <当前构建目录> -r openocd
```

预期流程应越过目标识别和 halt，继续执行 STM32 Flash 擦除、写入、校验，并以退出码 0 结束。

## 7. 接线与电源检查

按信号名连接，不按排线颜色猜测：

```text
DAPLink GND    <-> 目标板 GND
DAPLink SWDIO  <-> STM32 PA13 / SWDIO
DAPLink SWCLK  <-> STM32 PA14 / SWCLK
DAPLink NRST   <-> STM32 NRST（推荐）
DAPLink VTref  <-> 目标 MCU 的 3.3 V I/O 参考电压（按产品定义确认）
```

注意：

- 有些无线接收端的 `3.3V/5V` 是电源输出，有些是目标电压采样或接收端供电输入，必须先看该型号引脚定义；
- 不要同时让目标板自供电和接收端电源输出互相回灌；
- 用万用表确认 MCU VDD 和 VTref，而不是只看软件日志；
- SWD 线尽量短，第一次使用 1 MHz；
- `/dev/ttyACM0` 是 DAPLink 的 CDC 虚拟串口，它是否能看到 Zephyr 日志还取决于接收端 UART TX/RX 是否接到了板上控制台 UART；它不是烧录必需项。

## 8. 常见报错如何判断

| 报错或现象 | 所在阶段 | 优先处理 |
| --- | --- | --- |
| 找不到 `/dev/hidraw*` | USB/WSL | 重新插拔或重新执行 WSL USB attach |
| `Permission denied` | Linux 权限 | 检查设备节点、`plugdev` 组和 udev；本次实测无此问题 |
| `CMD_INFO failed` / `Operation timed out` | 探针自身通信 | 接收端供电、无线配对、距离和干扰；尚未访问目标 MCU |
| `unable to find CMSIS-DAP device` | OpenOCD 接口选择 | 确认 cfg 使用 `interface/cmsis-dap.cfg`，必要时加 VID/PID |
| `unable to find a matching CMSIS-DAP device` 且插了多只探针 | 设备筛选 | 临时加 `adapter serial 288485685614` |
| `DP IDCODE`/`target examination failed` | SWD 电气链路 | 核对 GND、SWDIO、SWCLK、目标供电，降到 1000 kHz |
| reset/halt 超时 | 目标复位 | 接 NRST、检查目标供电和旧固件，必要时复位下连接 |
| 能烧录但 F5 失败 | GDB/调试配置 | 烧录已正常，另查 Runner 的 Debug 项和 Cortex-Debug 参数 |
| 随机校验失败或中途断开 | 无线质量/SWD 速度 | 拉近距离、稳定接收端供电、避开 2.4 GHz 干扰并降速 |

不要在不清楚后果时执行 mass erase、解除读保护或修改 Option Bytes。RDP 回退可能擦除整片 Flash，RDP Level 2 通常不可逆。

## 9. pyOCD 备选路线为什么暂不推荐

DAPLink 原生可供 pyOCD 使用，而且工作区已经安装 pyOCD 0.45.1，也内置了 `stm32h723xx` target。不过当前存在两点：

1. `pyocd list` 对这只无线探针实测直接超时；
2. DM MC02 的 `board.cmake` 当前传入 `--target=stm32h723vgtx`，而本机 pyOCD 列出的内置名称是 `stm32h723xx`。

所以本次优先使用已经配置好的 OpenOCD Runner，不同时引入 pyOCD target 名称问题。只有 OpenOCD 路径稳定后，才值得单独验证 pyOCD；不要把两条路径混在一次排查中。

## 10. 最终勾选清单

- [ ] 电机功率回路已断开，机构安全。
- [ ] 无线接收端已供电，发射端与接收端已成功配对。
- [ ] `../.venv/bin/pyocd list` 不再在探针查询阶段超时。
- [ ] 只修改了当前板对应的 `support/openocd.cfg`。
- [ ] cfg 使用 `interface/cmsis-dap.cfg` 和 `transport select swd`。
- [ ] 初始 `adapter speed` 为 1000 kHz。
- [ ] Zephyr IDE 的 Flash Runner 是 `west-flash/openocd`。
- [ ] 无烧写连接检查能识别 CPU、halt 并读到非零 CPUID。
- [ ] `west flash ... -r openocd` 能写入并校验成功。
- [ ] 稳定后才逐步提高 SWD 速度或启用 F5 调试。

## 11. 资料依据与本次边界

- OpenOCD 官方说明：CMSIS-DAP 支持 v2 USB bulk 和 v1 HID，默认自动选择 backend，并支持用 VID/PID 或序列号筛选探针：<https://openocd.org/doc-release/html/Debug-Adapter-Configuration.html>
- Arm CMSIS-DAP 官方说明：CMSIS-DAP 是主机与 Cortex 调试口之间的标准接口，v2 使用 USB bulk：<https://arm-software.github.io/CMSIS-DAP/latest/dap_firmware.html>
- Arm DAPLink 用户指南：DAPLink 提供调试、虚拟串口和可选拖放烧录接口：<https://github.com/ARMmbed/DAPLink/blob/main/docs/USERS-GUIDE.md>
- 同类无线 CMSIS-DAP 的公开手册说明了发射/接收两端必须先建立无线连接，并建议 Linux 使用 OpenOCD；具体灯态和供电定义仍须以手上 Horco 型号说明书为准：<https://github.com/wuxx/nanoDAP-HS-wireless>

本次只读检查了板级 runner、OpenOCD cfg、Zephyr IDE Profile、现有构建的 `runners.yaml`、本机 OpenOCD/pyOCD 支持情况和 USB 设备状态。仅执行了不连接目标、不复位、不擦除、不烧写的探针信息查询。未验证无线接收端的实际灯态、供电和接线，也未对目标 MCU 做连接测试。

依据仓库“古法编程模式”，本指南是本次唯一新增文件；业务源码、板级配置、VS Code 配置和构建产物均未修改。上面的配置修改需要用户手工完成。
