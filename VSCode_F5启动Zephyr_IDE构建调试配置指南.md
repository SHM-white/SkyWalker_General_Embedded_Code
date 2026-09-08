# VS Code F5 启动 Zephyr IDE Build and Debug 配置指南

## 1. 目标、范围与结论

目标：在当前工作区中，未进入调试会话时按 `F5`，直接执行 Zephyr IDE 的 **Build and Debug**；进入调试会话后，`F5` 仍保持 VS Code 默认的“继续运行”。

本指南基于本机已安装的 **IDE for Zephyr / Zephyr IDE 4.1.0**。该版本中：

- **Build and Debug** 对应扩展命令 `zephyr-ide.build-debug`；
- 它不是 `launch.json` 中的一种 `request`，所以只增加一个普通调试配置并不能保证先构建；
- 命令会读取 Zephyr IDE 当前选中的项目、Build Configuration 和 Runner Profile，构建成功后再启动调试；
- 当前仓库已有 `DM MC02 OpenOCD` Runner Profile，其 Debug 槽使用 `cortex-debug + openocd`，可直接复用；
- `dm_mc02` 的 `board.cmake` 已向 OpenOCD 传入 `boards/damiao/dm_mc02/support/openocd.cfg`。

推荐做法是修改 VS Code **用户级** `keybindings.json`。VS Code 原生不读取仓库内的 `.vscode/keybindings.json`，因此不能仅靠工作区的 `launch.json` 完成同样可靠的命令绑定。

## 2. 执行链路

```text
未调试时按 F5
    |
    v
zephyr-ide.build-debug
    |
    +--> 读取当前 Active Project / Active Build
    |
    +--> 按该 Build 的参数执行 west build
    |       构建失败：停止，不启动调试
    |
    +--> 读取 Runner Profile 的 Debug 绑定
    |
    +--> Cortex-Debug + OpenOCD --> 下载 ELF --> 停在 main

已经进入调试后按 F5
    |
    v
VS Code 默认 Continue（继续运行）
```

这样拆分的原因是：Zephyr IDE 自己掌握活动项目、Build 目录、CMake 参数、ELF、GDB 和 `runners.yaml`；让扩展执行组合命令可避免在 `tasks.json`/`launch.json` 中重复并硬编码这些信息。

## 3. 必做配置

### 第一步：打开用户键盘快捷方式 JSON

在 VS Code 中按 `Ctrl+Shift+P`，执行：

```text
Preferences: Open Keyboard Shortcuts (JSON)
```

中文版命令通常显示为“首选项: 打开键盘快捷方式(JSON)”。这是用户配置文件，不是仓库的 `.vscode/launch.json`。

### 第二步：加入 F5 绑定

如果文件当前是空数组，改成：

```jsonc
[
    {
        "key": "f5",
        "command": "zephyr-ide.build-debug",
        "when": "!inDebugMode"
    }
]
```

如果文件中已经有其他快捷键，只在最外层数组末尾增加这个对象，并注意给前一个对象补逗号。例如：

```jsonc
[
    {
        "key": "ctrl+alt+b",
        "command": "workbench.action.tasks.build"
    },
    {
        "key": "f5",
        "command": "zephyr-ide.build-debug",
        "when": "!inDebugMode"
    }
]
```

`!inDebugMode` 是关键边界：调试尚未启动时拦截 `F5`；调试已经启动后不再匹配，于是 VS Code 原有的 `F5 = Continue` 仍然生效。

### 第三步：选择活动目标

在 VS Code 左侧打开 **Zephyr IDE** 面板，然后依次确认：

1. **Active Project** 是本次要调试的 sample，例如 `dji_speed_control`。
2. **Active Build** 是正确的板级构建，例如 `build/dm_mc02/stm32h723xx`。
3. Runner Profile 选择 **DM MC02 OpenOCD**。
4. Debug 槽显示为 OpenOCD/Cortex-Debug，而不是空绑定。

Zephyr IDE 默认可能随当前编辑文件自动切换 Active Project。按 `F5` 前应看一眼状态栏中的项目和构建名称，避免把另一个 sample 下载到板上。

### 第四步：重新加载并执行

第一次修改快捷键后可执行：

```text
Developer: Reload Window
```

连接调试器和目标板，未进入调试时按 `F5`。预期顺序为：

1. Zephyr IDE 输出窗口出现 Build 日志；
2. `west build` 成功；
3. OpenOCD 启动；
4. GDB 连接并下载当前 Build 的 `zephyr.elf`；
5. 程序在 `main` 处暂停（取决于当前调试配置/扩展默认行为）；
6. 再按一次 `F5`，程序继续运行，不会重复构建。

## 4. `launch.json` 如何处理

当前 `.vscode/launch.json` 已有三个 Zephyr IDE 条目。完成上面的快捷键绑定后，按 `F5` 走的是 `zephyr-ide.build-debug` 命令，因此**无需通过 `preLaunchTask` 拼接构建步骤，也不要求当前下拉框选中某个 launch 条目**。

当前第一个 `Debug with OpenOCD` 配置仍含占位路径：

```jsonc
"executable": "./bin/executable.elf"
```

它不是 Zephyr 的真实输出路径，不应选它来调试。若希望整理文件，可以手动将 `.vscode/launch.json` 精简为以下内容：

```jsonc
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Zephyr IDE: Debug Active Build (OpenOCD)",
            "type": "zephyr-ide-cortex",
            "request": "launch",
            "servertype": "openocd",
            "ask": "auto"
        },
        {
            "name": "Zephyr IDE: Attach Active Build (OpenOCD)",
            "type": "zephyr-ide-cortex",
            "request": "attach",
            "servertype": "openocd",
            "ask": "auto"
        }
    ]
}
```

字段含义：

- `type: zephyr-ide-cortex`：让 Zephyr IDE 从活动 Build 的 `runners.yaml` 解析 ELF、GDB 和 OpenOCD 参数，再转交 Cortex-Debug。
- `request: launch`：下载并启动程序；`attach` 只连接当前目标状态。
- `servertype: openocd`：使用原生 OpenOCD 调试路径。
- `ask: auto`：静默使用当前 Active Project 和 Active Build；若想每次选择目标，可改为 `askBoth`。

这个整理是可选项。它改善“运行和调试”面板中的手动启动入口，但不会代替前述 `F5 -> zephyr-ide.build-debug` 快捷键。

## 5. 可选的工作区建议项

为了让其他人打开仓库时收到正确的扩展安装提示，可手动在 `.vscode/extensions.json` 的 `recommendations` 中补充：

```jsonc
"mylonics.zephyr-ide",
"marus25.cortex-debug"
```

合并后的示例：

```jsonc
{
    "recommendations": [
        "mylonics.zephyr-ide",
        "marus25.cortex-debug",
        "ms-vscode.cpptools",
        "kylemicallefbonnici.dts-lsp"
    ],
    "unwantedRecommendations": []
}
```

## 6. 分步自检与故障排查

### 快捷键是否生效

打开“键盘快捷方式”，搜索 `F5`。未调试状态下应看到用户规则：

```text
f5 -> zephyr-ide.build-debug    when !inDebugMode
```

若按 F5 仍只启动普通 launch 配置：

1. 确认修改的是 `Preferences: Open Keyboard Shortcuts (JSON)` 打开的用户文件；
2. 确认 JSON/JSONC 没有漏逗号或括号；
3. 执行 `Developer: Reload Window`；
4. 在命令面板手动运行 `Zephyr IDE: Build and Debug`，验证扩展命令本身存在。

### 构建成功但不进入调试

依次检查：

1. Active Build 是否属于当前 Active Project；
2. Runner Profile 是否为 `DM MC02 OpenOCD`；
3. Build 输出目录中是否已经生成 `zephyr/runners.yaml` 和 `zephyr/zephyr.elf`；
4. VS Code 的 **Output** 面板中切换到 Zephyr IDE、Cortex-Debug 和 OpenOCD，查看第一条错误；
5. 确认已安装 `mylonics.zephyr-ide` 与 `marus25.cortex-debug`。

建议在 Zephyr IDE Terminal 中人工核对的命令如下；这些命令可能创建或更新构建产物，请由用户自行运行：

```bash
west build samples/motor/dji_speed_control \
  --build-dir samples/motor/dji_speed_control/build/dm_mc02/stm32h723xx

west debug \
  --build-dir samples/motor/dji_speed_control/build/dm_mc02/stm32h723xx \
  --runner openocd
```

如果换成其他项目，必须同时替换项目目录和 Build 目录，不能只改其中一个。

### OpenOCD 连接失败或等待 halt 超时

- 检查 ST-Link/J-Link 等探针是否被另一个 OpenOCD、GDB 或厂商工具占用；
- 检查 SWDIO、SWCLK、GND、VTref/3.3V 和 NRST；探针与目标板必须共地；
- 降低 SWD 速度后重试；
- 若固件很早关闭调试口、进入低功耗或时钟配置异常，尝试 connect-under-reset；
- 不要同时启动两个 Build and Debug 会话。

## 7. 硬件与安全边界

该仓库包含电机控制示例。调试下载或从断点继续时，PWM、CAN 控制帧和使能 GPIO 可能恢复输出。首次验证建议：

1. 断开电机动力电源，只保留控制板和调试器供电；
2. 架空机械负载，确保急停可用；
3. 确认当前 Active Project，避免误烧录其他控制程序；
4. 程序停在断点时不要假设执行器一定保持安全状态；
5. 确认下载和单步正常后，再按系统上电规程接通动力电源。

## 8. 最终勾选清单

- [ ] 用户 `keybindings.json` 已加入 `f5 -> zephyr-ide.build-debug`。
- [ ] 条件是 `!inDebugMode`，调试中的 F5 仍可继续运行。
- [ ] Zephyr IDE 4.1.0 和 Cortex-Debug 已启用。
- [ ] Active Project 与准备烧录的 sample 一致。
- [ ] Active Build 与目标板一致。
- [ ] Runner Profile 为 `DM MC02 OpenOCD`（或目标硬件对应的 Profile）。
- [ ] 首次验证时电机动力电源已断开。
- [ ] 按 F5 后先构建，构建成功后才启动 OpenOCD/GDB。
- [ ] 第二次按 F5 是继续运行，而不是重复构建。

## 9. 未验证假设

- 未在实体板上执行构建、烧录和调试，以免生成构建产物或触发电机相关硬件行为。
- 假设当前 VS Code 窗口使用本机检测到的 Zephyr IDE 4.1.0；其他版本的命令标识可能不同。
- 假设 `DM MC02 OpenOCD` Profile 对当前探针和目标板有效；如果实际使用 `rm_typec` 或其他探针，应选择对应 Build/Profile，不能照搬 DM MC02 的硬件参数。
