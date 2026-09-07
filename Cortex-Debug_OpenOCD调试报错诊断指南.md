# VS Code Zephyr IDE 调试达妙 MC02 报错诊断指南

## 1. 结论

本次“烧录并调试”失败的直接原因是：**OpenOCD 能打开 ST-Link，但无法识别 STM32H723 的 Cortex-M7 内核，于是拒绝 GDB 连接**。

用户日志末尾的：

```text
Remote connection closed
```

只是表层现象。用 VS Code 完全相同的命令复现后，OpenOCD 的真正错误是：

```text
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 4.739826
Error: [stm32h7x.cpu0] Cortex-M PARTNO 0x0 is unrecognized
Warn : target stm32h7x.cpu0 examination failed
```

GDB 连入时，OpenOCD 明确拒绝了连接：

```text
Error: Target not examined yet
Error: auto_probe failed
Error: Connect failed
Error: attempted 'gdb' connection rejected
```

故障链如下：

```text
Zephyr IDE 按钮
  → Cortex-Debug       成功
  → OpenOCD 0.12.0    成功
  → ST-Link USB       成功
  → SWD 调试端口       部分成功
  → STM32H723 内核     失败
```

当前首先要处理 **VTref/供电、SWD/NRST 接线或目标芯片状态**。其次，Zephyr IDE 自动生成的 OpenOCD 参数没有采用仓库为 MC02 准备的板级配置，也应在硬件链路恢复后修正。

## 2. 已排除的原因

| 检查项      | 实测结果                                 | 结论                        |
| ----------- | ---------------------------------------- | --------------------------- |
| ELF         | `zephyr.elf` 存在，符号正常读取        | 不是 ELF 缺失               |
| GDB         | Zephyr SDK 1.0.1 GDB 16.2 正常启动       | 不是 GDB 路径错误           |
| OpenOCD     | `/usr/bin/openocd` 0.12.0 正常启动     | 不是 OpenOCD 不存在         |
| ST-Link USB | WSL2 中可见`0483:3748`，OpenOCD 能打开 | 不是 USB 未挂载或 udev 权限 |
| SWD DAP     | 原生 DAP 模式读到`DPIDR 0x6ba02477`    | SWD 并非完全断路            |
| SWD 频率    | 降到 100 kHz 仍报`PARTNO 0x0`          | 不是单纯速度过高            |
| 复位下连接  | 使用`connect_assert_srst` 仍失败       | 仅换 cfg 不能修复当前状态   |
| 端口占用    | 50000/50001/50002 无旧进程占用           | 不是端口冲突                |

原生 DAP 还能读到 MEM-AP ID `0x84770001`，但读取 CoreSight ROM table 得到异常 CID，CPU CPUID 仍为 0。这表明 USB 传输在工作，但从 SWD DAP 访问目标内核的路径不正常。

## 3. 根因优先级

### 3.1 VTref 或目标供电异常

OpenOCD 连续报告目标电压约 `4.73–4.74 V`，这是当前最醒目的异常。STM32H723 的 SWD I/O 通常应与 MCU 的 3.3 V I/O 电源同基准。必须先用万用表核对，不能假定它是正常的“5 V 供电”。

不同 ST-Link 和仿制器上的 `3.3V`、`5V`、`VAPP`、`VTref` 含义可能不同：有的是电源输出，有的只是电平参考输入。未确认引脚定义时不要盲插，也不要让“目标板自供电”和“调试器电源输出”互相回灌。

### 3.2 SWD/NRST 接线、接触或共地异常

DPIDR 能读出，说明 SWDIO/SWCLK 不是完全开路，但仍可能有：

- GND 不可靠或排线方向错误。
- 转接板引脚定义与线色不一致。
- NRST 未连接、接触不良，或被外部电路持续拉低。
- 线过长，或外部电路占用了 PA13/PA14。

### 3.3 MCU 电源域或旧固件状态异常

芯片可能处于低功耗、复位未释放、供电不稳，或旧固件很快改变调试状态。这类情况需要可靠连接 NRST，再使用复位下连接。

### 3.4 读保护或硬件损坏

只有在供电、线序、NRST 和另一个已知正常的调试器都排除后，再考虑 RDP 或芯片损坏。RDP Level 1 回退到 Level 0 会全片擦除；RDP Level 2 通常不可逆。**未备份且未确认前，不要修改 RDP，也不要执行 mass erase。**

## 4. 现场恢复步骤

### 第 0 步：隔离电机功率回路

1. 断开电机功率电源或拔掉电机输出。
2. 只保留 MC02 逻辑供电和调试器。
3. 确保机构不会在意外恢复运行时动作。

`dji_speed_control` 是闭环电机样例，调试连接恢复后可能继续执行旧固件，因此这一步不能省略。

### 第 1 步：彻底断电并重插

1. 停止 VS Code 调试会话。
2. 拔掉 ST-Link USB，断开 MC02 逻辑电源。
3. 等板上电容放电后检查线序。
4. 若使用 WSL2 + usbipd，重插后在 Windows 侧重新 attach ST-Link。

### 第 2 步：用万用表测量

先不接 ST-Link 的 SWD 信号线，单独给 MC02 逻辑部分上电：

- MCU 3.3 V 对 GND 是否稳定在约 3.3 V。
- NRST 未按下时是否为高电平，按下时是否为低电平。
- 调试接口上准备用作 VTref 的引脚究竟是 3.3 V 还是 5 V。

如果 MCU I/O 是 3.3 V，但 OpenOCD 仍报 `Target voltage: 4.74 V`，先修正 VTref/供电接法，不要继续反复调试。

### 第 3 步：按信号名重新接线

```text
ST-Link GND    ↔ MC02 GND
ST-Link SWDIO  ↔ STM32 PA13 / SWDIO
ST-Link SWCLK  ↔ STM32 PA14 / SWCLK
ST-Link NRST   ↔ STM32 NRST
ST-Link VTref  ↔ MCU 实际 I/O 参考电压（通常为 3.3 V）
```

使用短线，并同时核对 MC02 与手上这款 ST-Link 的引脚定义；不要只按线色判断。

### 第 4 步：先做不烧录的连接自检

在仓库根目录运行：

```bash
openocd \
  -s /home/shm-white/.zephyr_ide/toolchains/zephyr-sdk-1.0.1/hosttools/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts \
  -f boards/damiao/dm_mc02/support/openocd.cfg \
  -c 'init' \
  -c 'reset halt' \
  -c 'mdw 0xe000ed00 1' \
  -c 'shutdown'
```

此命令会复位并暂停 MCU，但不会烧录。成功时应看到 Cortex-M7/STM32H7 被识别、target halted，且 `0xe000ed00` 读数不是 0。

若仍有 `Cortex-M PARTNO 0x0 is unrecognized`，不要改 C++、Kconfig 或 ELF；应继续检查电气连接，并换一个已知正常的 ST-Link/J-Link 及另一台主机交叉验证。

## 5. 硬件恢复后修正 Zephyr IDE

### 5.1 当前自动配置遗漏了板级 cfg

当前日志中的 Zephyr IDE 4.1.0 生成了：

```json
"configFiles": [
  "interface/stlink.cfg",
  "target/stm32h7x.cfg"
]
```

但 MC02 的板级配置是：

```text
boards/damiao/dm_mc02/support/openocd.cfg
```

其中除 ST-Link 和 STM32H7 外，还包含：

```tcl
adapter speed 4000
reset_config srst_only srst_nogate connect_assert_srst
```

自动生成路径丢失了“复位下连接”。Zephyr 的 OpenOCD west runner 会根据 `runners.yaml` 中的 `board_dir` 自动查找 `<board_dir>/support/openocd.cfg`，所以优先让 Zephyr IDE 通过 west debugserver 启动 OpenOCD。

### 5.2 推荐：Runner Profile 使用 west OpenOCD

1. 打开命令面板，执行 `Zephyr IDE: Open Runner Profile Panel`。
2. 新建配置，例如 `DM MC02 OpenOCD`。
3. Flash 选择 `west-flash`，runner 选择 `openocd`。
4. Debug 选择 `west-debug`，runner 选择 `openocd`。
5. 给 `dji_speed_control / build/dm_mc02/stm32h723xx` 选择这个 Active Runner Profile。
6. 再点击“烧录并调试”。

这样既保留 Zephyr IDE 按钮流程，也由 Zephyr runner 使用 MC02 板级 cfg。

### 5.3 备选：手写 launch.json

如果 west-debug 桥接仍有兼容问题，可在 `.vscode/launch.json` 手工添加：

```jsonc
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "DM MC02: OpenOCD under reset",
      "type": "zephyr-ide-cortex",
      "request": "launch",
      "servertype": "openocd",
      "serverpath": "/usr/bin/openocd",
      "configFiles": [
        "${workspaceFolder}/boards/damiao/dm_mc02/support/openocd.cfg"
      ],
      "searchDir": [
        "/home/shm-white/.zephyr_ide/toolchains/zephyr-sdk-1.0.1/hosttools/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts"
      ],
      "runToEntryPoint": "main",
      "showDevDebugOutput": "raw"
    }
  ]
}
```

然后让 Runner Profile 的 Debug 槽选择 `launch`，绑定 `DM MC02: OpenOCD under reset`。不要再额外添加 `interface/stlink.cfg` 或 `target/stm32h7x.cfg`，因为板级 cfg 已经 source 它们。

如果硬件确实没有 NRST，可在完全断电重启目标后暂时改用：

```text
boards/damiao/dm_mc02/support/openocd_stlink.cfg
```

但可靠的长期方案仍是接好 NRST 并使用 `openocd.cfg`。

## 6. 成功标准

`TERMINAL → gdb-server` 应出现目标核被识别、监听并接受 GDB 连接；不应再出现：

```text
Cortex-M PARTNO 0x0 is unrecognized
Target not examined yet
attempted 'gdb' connection rejected
Remote connection closed
```

之后 GDB 才会继续下载 ELF、设置断点并运行到 `main`。

## 7. 若仍失败

1. 保存新的 `TERMINAL → gdb-server` 完整输出，不要只复制 Cortex-Debug 摘要。
2. 换一个已知正常的 ST-Link/J-Link。
3. 在 Windows 原生 STM32CubeProgrammer 中选 Hardware reset/Under reset，先只连接和读取芯片 ID，不要直接擦除。
4. 若 CubeProgrammer 也读不到芯片 ID，返回检查 3.3 V、NRST、SWDIO/SWCLK 和芯片焊接/损坏。
5. 只有明确知道原固件设置了读保护，并接受丢失片上内容时，才考虑解锁或全片擦除。

## 8. 检查清单

- [ ] 电机功率回路已断开，机构安全。
- [ ] MCU 3.3 V 与 NRST 电平已实测。
- [ ] VTref 没有误接 5 V，也没有反向灌电。
- [ ] GND、SWDIO、SWCLK、NRST 已按信号名连接。
- [ ] WSL2 重插后已重新 attach ST-Link。
- [ ] 终端自检不再报 `PARTNO 0x0`。
- [ ] Runner Profile 使用 west OpenOCD，或 launch.json 使用板级 `openocd.cfg`。
- [ ] gdb-server 中目标被识别，GDB 不再被拒绝。

## 9. 本次诊断边界

- 已分析 Cortex-Debug 1.12.1 日志。
- 已检查 Zephyr IDE 4.1.0 配置、板级 cfg 和构建目录 `runners.yaml`。
- 已在当前硬件上复现目标探测失败及 GDB 被拒绝。
- 已测试 HLA SWD、原生 ST-Link DAP、100 kHz 和复位下连接，均未读到 Cortex-M7 CPUID。
- 尚未用万用表实测 MC02 3.3 V/NRST，也无法远程确认排线定义；具体电气异常仍需现场测量。
- 未执行烧录、全片擦除、选项字节修改、构建或测试。
- 未修改业务源码、板级配置、VS Code 配置或构建产物；本指南是唯一更新的文件。
