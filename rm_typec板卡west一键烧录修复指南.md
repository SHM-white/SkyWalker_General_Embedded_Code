# rm_typec 板卡 west 一键烧录修复指南（ST-Link / OpenOCD）

> 目标：让 `samples/motor/dji_speed_control` 这类样例在 **rm_typec/stm32f407xx** 板上，能用
> Zephyr IDE / `west flash` 一键烧录，而不是报 `no runners.yaml found`。
> 本指南只涉及板级构建/调试配置文件与系统工具，**不修改任何业务源码**。

---

## 1. 当前状态与根因

复现现象：

```bash
west flash --build-dir samples/motor/dji_speed_control/build/rm_typec/stm32f407xx -r stlink
# west flash: rebuilding
# ninja: no work to do.
# no runners.yaml found in .../zephyr.
# Either board rm_typec/stm32f407xx doesn't support west flash, or a pristine build is needed.
```

诊断（已验证）：

1. `boards/rm_typec/` 下**没有 `board.cmake`**。对比 `boards/damiao/dm_mc02/board.cmake`
   注册了 pyocd/openocd/jlink 等 runner，`rm_typec` 一个都没注册。
   Zephyr 构建时只会把 `board.cmake` 里显式注册过的 runner 写进 `zephyr/runners.yaml`，
   所以该 build 目录根本没有 `runners.yaml`，任何 `west flash`（无论加不加 `-r`）都会失败。
2. 当前 Zephyr 快照（west.yml revision `6085aad…`）的 `boards/common/` 里**没有
   `stlink.board.cmake`**（只有 `stlink_gdbserver.board.cmake`），因此 west 的 `stlink`
   烧录 runner 在本工程注册不了；Zephyr 官方对 ST-Link 的烧录路线是
   **OpenOCD / STM32CubeProgrammer / pyocd**。
3. 环境是 WSL2：ST-Link 已通过 usbipd 透传（USB `0483:3748 STM32 STLink`），但
   `/dev/bus/usb` 设备节点权限是 `root:root 664`，普通用户无法读写 → pyocd/openocd
   都枚举不到探针（这也是 `pyocd list` 显示 “No available debug probes” 的原因之一；
   另外 pyocd 内置目标里也没有 stm32f407）。
4. 系统已装：`west`、`pyocd`；**未装 `openocd`**（apt 候选 0.12.0-3+b2）。
5. `boards/rm_typec/support/openocd.cfg` 目前写的是 **CMSIS-DAP** 接口，与你的
   **外接 ST-Link** 不符，需要改。

结论：补一个 `board.cmake` 注册 **openocd** runner + 把 `support/openocd.cfg` 改成
ST-Link 接口 + 安装 openocd + 修 USB 权限，即可一键烧录。

---

## 2. 涉及文件与系统改动

| # | 对象 | 动作 |
|---|------|------|
| 1 | `boards/rm_typec/board.cmake` | **新建**，注册 openocd runner |
| 2 | `boards/rm_typec/support/openocd.cfg` | **修改**，接口改为 ST-Link |
| 3 | 系统 | `sudo apt install openocd` |
| 4 | 系统（WSL USB 权限） | udev 规则 或 每次 `chmod 666` |

> ⚠️ 第 1、2 步会改动仓库内**板级构建文件**。按本仓库“古法编程模式”，这些改动
> 需要你亲手完成，本指南只给出可直接落地的内容与命令。

---

## 3. 文件 1：新建 `boards/rm_typec/board.cmake`

在仓库根目录执行（或在 VS Code 里新建文件后粘贴）：

```bash
cd /home/shm-white/skywalker_ws/skywalker_code
cat > boards/rm_typec/board.cmake <<'EOF'
# SPDX-License-Identifier: Apache-2.0

# RM Type-C 板无板载调试器，默认经 OpenOCD 使用外接调试器烧录。
# 若你同时保留 pyocd/stm32cubeprogrammer 等 runner，可仿照
# boards/damiao/dm_mc02/board.cmake 追加对应 include。
board_runner_args(openocd)

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
EOF
```

要点：

- `include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)` 会把 `openocd` 设为
  默认 flasher（`board_set_flasher_ifnset(openocd)`），并把 runner 写入 `runners.yaml`。
- openocd runner 会**自动使用** `boards/rm_typec/support/openocd.cfg`（默认约定），
  无需在 args 里再传 `--config`。
- 建议先只注册 openocd，减少变量；以后要加 pyocd/J-Link 再仿照 dm_mc02 追加。

---

## 4. 文件 2：修改 `boards/rm_typec/support/openocd.cfg`

现内容（CMSIS-DAP 版，与你的 ST-Link 不符）：

```tcl
source [find interface/cmsis-dap.cfg]
transport select swd
source [find target/stm32f4x.cfg]
adapter speed 8000
```

### 4.1 首选：ST-Link DAP 模式（推荐，需 ST-Link 固件 ≥ v2j24）

直接整体替换为（参考 Zephyr 上游 `weact/stm32f405_core`，F405 与 F407 同属 stm32f4x 家族）：

```tcl
# STM32F407 OpenOCD 配置：RM Type-C 板 + 外接 ST-Link（DAP 模式）
# 需要 ST-Link 固件 >= v2j24；旧固件请改用 4.2 的 hla 写法。
source [find interface/stlink-dap.cfg]
transport select dapdirect_swd

source [find target/stm32f4x.cfg]

adapter speed 4000
```

### 4.2 备用：ST-Link hla 模式（旧固件 ST-Link / 克隆版）

如果 4.1 在 `openocd -c "init"` 时报类似 “transport dapdirect_swd not supported /
your firmware is too old”，把接口段换成：

```tcl
source [find interface/stlink.cfg]
transport select hla_swd
```

其余行（`source [find target/stm32f4x.cfg]`、`adapter speed 4000`）保持不变。
OpenOCD ≥ 0.11 会打印 hla 弃用警告，但可正常烧录。

> 若你以后又想用回板载 CMSIS-DAP，只需把接口段改回
> `source [find interface/cmsis-dap.cfg]` + `transport select swd`。

---

## 5. 安装 openocd

```bash
sudo apt update
sudo apt install -y openocd
openocd --version   # 应显示 0.12.0
```

---

## 6. 修 WSL USB 权限（关键，否则 openocd 打不开 ST-Link）

当前设备节点是 `root:root 664`，普通用户只能读不能写。两种办法任选：

### 6.1 一劳永逸：udev 规则（推荐，WSL 需启用 systemd）

```bash
sudo tee /etc/udev/rules.d/99-stlink.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

验证 systemd/udev 是否生效（WSL 需在 `.wslconfig`/`/etc/wsl.conf` 开启 systemd）：

```bash
ls -l /dev/bus/usb/*/*
```

若节点仍是 `root:root`，说明 udev 没跑起来，用 6.2。

### 6.2 每次透传后临时放开（不依赖 systemd）

```bash
sudo chmod 666 /dev/bus/usb/*/*
```

> 注意：重新 `usbipd attach` 后节点可能重建，需要再执行一次 6.2。

---

## 7. 验证探针连通（烧录前必做）

```bash
cd /home/shm-white/skywalker_ws/skywalker_code
openocd -f boards/rm_typec/support/openocd.cfg -c "init; targets; shutdown"
```

预期输出含：

```
Info : STLINK V2J... 
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
```

若报 `Permission denied` → 回第 6 步；若报 `no device found` → 检查 Windows 侧
`usbipd list` / 重新 attach、SWD 接线与板卡供电。

---

## 8. 重新构建并一键烧录

在仓库根目录执行（沿用 Zephyr IDE 的 build 目录布局，会**清空重建**该目录并生成
`runners.yaml`）：

```bash
cd /home/shm-white/skywalker_ws/skywalker_code
west build -p always \
  -b rm_typec/stm32f407xx \
  samples/motor/dji_speed_control \
  -d samples/motor/dji_speed_control/build/rm_typec/stm32f407xx
```

自检：`runners.yaml` 应已生成且包含 openocd：

```bash
cat samples/motor/dji_speed_control/build/rm_typec/stm32f407xx/zephyr/runners.yaml
# 应看到 openocd: 相关条目
```

烧录（不指定 `-r`，west 会自动用 runners.yaml 里唯一的 openocd runner）：

```bash
west flash -d samples/motor/dji_speed_control/build/rm_typec/stm32f407xx
```

> 在 Zephyr IDE 里操作也一样：改完第 3、4 步后重新 **Build**（必要时用
> **Clean Build / pristine**），让 `runners.yaml` 重新生成；**Flash** 时如果 IDE 仍
> 自动带 `-r stlink`，请手动把 runner 改为 `openocd` 或去掉 runner 参数。

若你不想动仓库文件、只想临时烧一次，也可以绕过 west 直接：

```bash
openocd -f boards/rm_typec/support/openocd.cfg \
  -c "program samples/motor/dji_speed_control/build/rm_typec/stm32f407xx/zephyr/zephyr.elf verify reset exit"
```

（此命令同样依赖第 4～6 步的 cfg 与权限就绪。）

---

## 9. 自检点清单

- [ ] `boards/rm_typec/board.cmake` 已创建，内容含 `openocd.board.cmake` include
- [ ] `boards/rm_typec/support/openocd.cfg` 接口段已改为 ST-Link（4.1 或 4.2）
- [ ] `openocd --version` 可用（≥ 0.11）
- [ ] `/dev/bus/usb/*/*` 权限非 root-only（第 6 步）
- [ ] 第 7 步 `openocd ... -c "init; targets; shutdown"` 能找到 `stm32f4x.cpu`
- [ ] `west build -p always ...` 后 `zephyr/runners.yaml` 存在且含 openocd
- [ ] `west flash -d ...` 烧录成功，无 “no runners.yaml”

## 10. 安全提醒

- `dji_speed_control` 是**电机速度闭环样例**，上电运行会驱动电机。烧录本身无害，
  但烧完一旦复位运行，请先断开电机动力、把机构完全架空，再按
  `DJI电机PID速度控制示例实施指南.md` 第 11 节的上电顺序操作。
- 修改 `board.cmake`/`openocd.cfg` 只影响构建与烧录，不影响固件逻辑；本指南全程
  **未修改任何业务源码**。

## 11. 故障排查速查

| 现象 | 处理 |
|------|------|
| `no runners.yaml found` | 还没重建/重配；先跑第 8 步 `west build -p always` |
| `Permission denied` 打开 USB | 第 6 步 udev 或 `chmod 666` |
| openocd 找不到设备 | Windows 侧 `usbipd list`/重新 attach；查 SWD 接线与供电 |
| `dapdirect_swd` 不支持 | ST-Link 固件过旧，换 4.2 hla 写法 |
| Zephyr IDE Flash 仍带 `-r stlink` | IDE 里显式选 openocd runner，或命令行去掉 `-r` |
