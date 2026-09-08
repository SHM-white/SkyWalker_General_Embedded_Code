# DM MC02 OpenOCD 烧录等待 halt 超时修正指南

## 1. 本次结论

当前失败不是 Zephyr 编译、HEX 文件、芯片型号或 VS Code Runner Profile 选错导致的。最新日志已经证明：

- OpenOCD 0.12.0 能打开 ST-Link V2；
- SWD 已读到 STM32H723 的 Cortex-M7 r1p2；
- west 正在加载正确的 `zephyr.hex`；
- 失败发生在 OpenOCD 等待内核进入 halted 状态时。

直接配置原因是 `boards/damiao/dm_mc02/support/openocd.cfg` 第 9 行：

```tcl
reset_config srst_only srst_nogate connect_assert_srst
```

该板级文件先加载了系统的 `/usr/share/openocd/scripts/target/stm32h7x.cfg`，随后又用上面一行覆盖目标脚本的复位设置。但是本机 OpenOCD 0.12.0 的 STM32H7 目标脚本在第 138～146 行明确说明：使用 ST-Link 的 HLA 驱动时，STM32H7 不支持 `connect_assert_srst`。SRST 保持有效时，HLA 通过 AP0/AXI 访问 DBGMCU 的路径不可用，因此可能识别出 Cortex-M7，却无法完成 halt，最终得到：

```text
Error: timed out while waiting for target halted
```

数据流如下：

```text
VS Code Zephyr 插件
  → west flash -r openocd
  → build/.../zephyr/runners.yaml
  → boards/damiao/dm_mc02/support/openocd.cfg
  → interface/stlink.cfg（HLA）
  → target/stm32h7x.cfg
  → 板级 cfg 再次启用不兼容的 connect_assert_srst
  → reset/halt 超时，尚未执行 flash write_image
```

## 2. 已核对且无需修改的配置

| 检查项 | 结果 | 结论 |
| --- | --- | --- |
| VS Code 工程 | `m2006_speed_control`、`dm_mc02/stm32h723xx` | 正确 |
| Active Profile | `DM MC02 OpenOCD`，flash 为 `west-flash/openocd` | 正确 |
| west runner | `/usr/bin/openocd` 0.12.0 | 正常 |
| runner 板级 cfg | 指向 `boards/damiao/dm_mc02/support/openocd.cfg` | 正确加载 |
| SoC | `CONFIG_SOC_STM32H723XX=y` | 正确 |
| ELF/HEX 起始地址 | `0x08000000` | 与当前非 MCUboot 构建一致 |
| HEX 文件 | 已存在，OpenOCD 在 halt 前失败 | 不是文件路径或格式问题 |
| 编译状态 | `ninja: no work to do` | 正常，不是报错 |

`runners.yaml` 中记录的 Zephyr SDK OpenOCD 搜索目录目前不存在，但 `/usr/bin/openocd` 仍从系统默认的 `/usr/share/openocd/scripts` 成功找到了 `interface/stlink.cfg` 和 `target/stm32h7x.cfg`。日志已经完成 Cortex-M7 识别，因此这不是本次阻断点。后续若更换 OpenOCD 安装方式，再重新 pristine build 以刷新工具路径即可。

## 3. 推荐修改（用户手工完成）

只改一个文件：

```text
boards/damiao/dm_mc02/support/openocd.cfg
```

删除第 8～9 行的错误注释和 `connect_assert_srst` 设置。修改后文件应为：

```tcl
source [find interface/stlink.cfg]

transport select hla_swd

source [find target/stm32h7x.cfg]
adapter speed 4000
```

不要修改系统的 `/usr/share/openocd/scripts/target/stm32h7x.cfg`。它已经包含适合 STM32H7 的默认复位配置 `reset_config srst_nogate`，并自带“初始 1800 kHz、复位初始化完成后切到 4000 kHz”的时钟策略。上面保留的板级 `adapter speed 4000` 会覆盖初始 1800 kHz；当前日志在 4 MHz 下已能识别 CPU，因此可先保留，若仍不稳定再按第 5 节降速。

仓库已有的 `boards/damiao/dm_mc02/support/openocd_stlink.cfg` 内容正好是不带错误复位覆盖的版本，可用来逐行对照，但 west 当前不会自动选它。

这项修改不要求重新编译：`runners.yaml` 保存的是 cfg 文件路径，OpenOCD 每次启动都会重新读取该文件。直接再次点击插件烧录即可。

## 4. 先做无烧写连接检查

电机控制样例可能在复位后立即驱动机构。先断开电机功率回路，只保留逻辑供电、GND、SWDIO、SWCLK，推荐同时连接 NRST。

修改前也可先用仓库已有的无 `connect_assert_srst` 配置验证根因：

```bash
openocd \
  -f boards/damiao/dm_mc02/support/openocd_stlink.cfg \
  -c 'init' \
  -c 'reset halt' \
  -c 'mdw 0xE000ED00 1' \
  -c 'shutdown'
```

该命令会复位并暂停 MCU，但不会擦除或烧写 Flash。成功时应看到：

- `Cortex-M7 r1p2 processor detected`；
- `target halted`；
- 地址 `0xE000ED00` 返回非零 CPUID（该内核通常为 `0x410fc271`）；
- 不再出现 `timed out while waiting for target halted`。

完成第 3 节修改后，再执行原命令：

```bash
west flash \
  --build-dir /home/shm-white/skywalker_ws/skywalker_code/samples/motor/m2006_speed_control/build/dm_mc02/stm32h723xx \
  -r openocd
```

预期会在 `target halted` 之后继续识别 STM32H7 Flash、写入并校验 `zephyr.hex`，最后 `shutdown command invoked`，退出码为 0。

## 5. 如果删除该设置后仍超时

按以下顺序排查，每次只改变一项：

1. 把板级 cfg 的 `adapter speed 4000` 临时改为 `adapter speed 1000`，重试第 4 节的无烧写检查。虽然当前 4 MHz 已能识别 CPU，但较低 SWD 时钟可排除信号完整性问题。
2. 确认 Windows STM32CubeProgrammer 已完全关闭；同一 ST-Link 不能同时被 Windows、WSL/usbipd 或另一个 GDB server 占用。
3. 断电后按信号名核对 GND、SWDIO、SWCLK、NRST，不按排线颜色猜测。尽量使用短线。
4. 在 Windows STM32CubeProgrammer 中选择硬件复位/复位下连接，只做连接测试；不要修改 Option Bytes，不要执行 mass erase。
5. 如果 Windows 能稳定 halt 而 Linux 仍失败，再比较两边 ST-Link 固件、连接模式和 SWD 频率。

本次日志中的 `Target voltage: 4.559555` 明显值得现场复核。用万用表直接测 MCU VDD/调试口 VTref 对 GND；若 MCU VDD 实测超过器件允许的 3.3 V 供电范围，应立即断电检查供电或 VTref 接法。Windows 能读出芯片 ID 只能证明当时 SWD 通信成功，不能证明这个电压读数安全，也不能消除当前复位配置冲突。

## 6. 与旧诊断指南的关系

仓库中的 `Cortex-Debug_OpenOCD调试报错诊断指南.md` 对应的是更早一次 `Cortex-M PARTNO 0x0 is unrecognized` 故障。当时内核尚未识别；本次最新日志已前进到 Cortex-M7 成功识别，因此旧指南中建议为 HLA 启用 `connect_assert_srst` 的部分不适用于本次状态，也与本机 OpenOCD 的 STM32H7 目标脚本冲突。以本指南为准。

## 7. 最终检查清单

- [ ] 已断开电机功率回路，机构不会意外动作。
- [ ] 已从 `openocd.cfg` 删除 `connect_assert_srst` 覆盖。
- [ ] 未修改系统 OpenOCD 的 `target/stm32h7x.cfg`。
- [ ] 无烧写测试能看到 `target halted` 和非零 CPUID。
- [ ] 原 `west flash -r openocd` 能继续执行写入和校验。
- [ ] MCU VDD/VTref 已用万用表复核，不依赖 ST-Link 日志猜测。
- [ ] 若仍失败，已先降到 1000 kHz，再排查 NRST、共地和工具占用。

## 8. 本次操作边界

- 已只读检查板级 runner、OpenOCD cfg、系统 OpenOCD 0.12.0 STM32H7 目标脚本、VS Code Zephyr Profile、构建配置与 ELF/HEX 地址。
- 未连接或操纵目标板，未执行烧录、擦除、复位、构建或测试。
- 未修改业务源码、板级配置、VS Code 配置或构建产物。
- 本 Markdown 指南是本次唯一新增文件；配置修改仍需用户按第 3 节亲手完成。
