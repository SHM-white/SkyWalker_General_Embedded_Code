# DJI 位置控制示例：去掉启动 `a` 人工确认 + 主线程栈溢出分析与修复指南

> 适用仓库：当前 `skywalker_code`，分支 `dev`
>
> 适用样例：`samples/motor/dji_position_control/`
>
> 台架对象：`app.overlay` 中的 GM6020（ID=7，电流环，gear 1:1，与
> `samples/motor/dji_unified` 和 `samples/motor/dji_speed_control` 同一颗电机）
>
> 重要：本指南是「古法编程」产出物。以下所有代码改动请由你亲手完成；
> 当前业务源码**未被本指南自动修改**。

---

## 1. 事故现场与任务目标

串口日志（节选）：

```text
*** Booting Zephyr OS build v4.4.0-6527-g6085aadeb337 ***
Suspend the motor and prepare a physical power cut.
Press 'a' to move +250 mrad from the current output position.

a
[00:00:21.696,000] <err> os: ***** MPU FAULT *****
[00:00:21.696,000] <err> os:   Stacking error (context area might be not valid)
[00:00:21.696,000] <err> os:   Data Access Violation
[00:00:21.696,000] <err> os:   MMFAR Address: 0x200055f8
[00:00:21.696,000] <err> os: ...
[00:00:21.696,000] <err> os: >>> ZEPHYR FATAL ERROR 2: Stack overflow on CPU 0
[00:00:21.696,000] <err> os: Current thread: 0x200028a0 (main)
[00:00:22.866,000] <err> os: Halting system
```

任务：

1. 像 `dji_speed_control`（commit `9640dbf` 已做过同样处理）一样，去掉
   `dji_position_control` 启动时等待用户输入 `a` 的人工确认。
2. 分析上面的崩溃原因并给出修复。

### 关键假设

- 电机台架物理布局与速度样例一致（输出轴在 ARM 前必须保持悬空/可自由转动），
  因此「去确认」采用与 `dji_speed_control/src/main.cpp` **完全相同的自动启机
  模式**（反馈就绪 → 打印提示 → 立即 arm），不再保留 console 子系统。
- 下面第 2 节用到的内存数值取自你最近的构建产物：
  `samples/motor/dji_position_control/build/rm_typec/stm32f407xx/`，如果以后
  `prj.conf` 或配置变了，地址会变，但分析方法不变。

---

## 2. 崩溃原因分析：主线程栈溢出（不是控制代码写错）

### 2.1 硬件/系统给出的三条线索

| 日志内容                                             | 含义                                                                                                                                                                                                                             |
| ---------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ZEPHYR FATAL ERROR 2`                             | Zephyr 致命错误码 2 =`K_ERR_STACK_CHK_FAIL`，即“栈检查失败”。                                                                                                                                                                |
| `Current thread: ... (main)`                       | 出问题的是**main 线程**，不是 CAN 中断也不是 log 线程。                                                                                                                                                                    |
| `Stacking error (context area might be not valid)` | CPU 在**保存异常现场（压栈）时**自己又触发了 fault，说明当时的 SP 已经越界，压栈压到了无效区域；因此日志里的 `r0/r1/r15(pc)` 等寄存器**不可信**，别按 `pc=control_pid_validate` 去猜“为什么在 validate 里崩”。 |

### 2.2 用构建产物验证“栈确实用光了”

查看 `.config`：

```text
CONFIG_MAIN_STACK_SIZE=1024        ← main 线程栈只有 1 KB
CONFIG_ISR_STACK_SIZE=2048
CONFIG_DEBUG_OPTIMIZATIONS=y       ← 本次构建开了 -Og（调试优化）
```

用符号表查看 `z_main_stack`（main 线程的栈数组）：

```text
z_main_stack   @ 0x200055c0   size 0x440 (1088 B)
            栈区间约 [0x200055c0, 0x20005a00)，向低地址增长
```

而 fault 报告的 `MMFAR Address: 0x200055f8` 恰好位于 `z_main_stack` 的**最底部
附近**（比栈数组起始地址只高 0x38 = 56 字节，比可用栈底更低/贴着栈底）。

对照 Zephyr 的 Cortex-M 实现：`z_main_stack` 里有一部分低位字节用作 MPU
“栈保护区域(guard)”，真正的可用栈底在 0x20005600 附近；`0x200055f8 = 0x20005600 - 8`，正是**再往下压 8 字节就会踩进 guard 区域**的位置。

结论链：

```text
main 线程运行时 SP 一路向下增长
        │
        ▼
SP 走到 0x20005600（1 KB 栈基本耗尽）
        │
        ▼
异常(或函数调用)需要继续压栈 → 写 0x200055f8，命中 MPU guard
        │
        ▼
Data Access Violation + Stacking error
        │
        ▼
Zephyr 判定：FATAL ERROR 2 = Stack overflow on CPU 0, thread = main
```

**根因：main() 及其调用链在运行到最深时刻所需的栈超过 1024 B。**

### 2.3 为什么这个样例在“按了 a 之后”才炸、而 `dji_speed_control` 不炸

- 两者都只有 1 KB 主栈。位置样例的主函数 `main()` 常驻局部对象更多更大：
  `Descriptor`、`PositionController`（位置 PID + 斜率限制 + 前馈 PID 的 config
  与 state，约几百字节）、`FlushReport`、`Feedback`、每周期 `next_*` 状态拷贝、
  `PositionControlOutput` 等；
- 控制循环每 200 ms 打一条很长的 `printk`（含 `%llu` 和十余个实参），
  printk/LOG 的格式化链路在 main 上下文里又叠了一层；
- `-Og`（`CONFIG_DEBUG_OPTIMIZATIONS=y`）关闭了大部分函数内联，
  每个函数的栈帧明显变大；
- 位置样例的 `prj.conf` 还多了 console 子系统（`CONFIG_CONSOLE_SUBSYS` /
  `CONFIG_CONSOLE_GETCHAR`），以及输入 `a` 之前 `console_init()` 等路径，
  栈压力比速度样例更高。

因此速度样例恰好压在 1 KB 之下能跑，位置样例在最深调用点（arm 后运行中的
flush / 遥测 printk / 结束停机路径）**超出 1 KB**，于是主栈溢出。
崩溃时刻恰好紧跟在输入 `a` 之后，是因为 `a` 之后程序才进入“运行/停止”这些
最深的代码路径，而不是因为 `a` 本身做错了什么。

> 说明：`stacking error` 让现场寄存器失真，`pc` 指向 `control_pid_validate`
> 只是“栈里残留的旧返回地址”之类，不代表真正崩溃点；不要据此去查 validate。

### 2.4 修复方向

把 main 线程栈从 1 KB 提到足够大即可；顺手删掉不再需要的 console 子系统还能
再降一点栈压力。本指南推荐直接设 **`CONFIG_MAIN_STACK_SIZE=8192`**（SRAM 共
128 KB，8 KB 只占很小比例），给 `-Og` 调试构建留足余量；若想更省，4096 也
很可能够，但遇到再次溢出时再往上加会多一轮烧录验证。

---

## 3. 需要改的文件（共 2 个）

改动模板与 commit `9640dbf` 对 `dji_speed_control` 的处理完全一致。

### 3.1 `samples/motor/dji_position_control/src/main.cpp`

#### (1) 删除 console 头文件

文件开头：

```cpp
#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/console/console.h>      // ← 删除这一行
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
```

#### (2) 删除整个人工确认函数

把下面整个函数删掉（它位于 `readFreshPositionFeedback(...)` 之后、
`resetController(...)` 之前）：

```cpp
int waitForManualArmToken()
{
    int ret = console_init();
    if (ret < 0) {
        return ret;
    }

    printk("Suspend the motor and prepare a physical power cut.\n");
    printk("Press 'a' to move +250 mrad from the current output position.\n");
    for (;;) {
        const int ch = console_getchar();
        if (ch < 0) {
            return ch;
        }
        if (ch == 'a' || ch == 'A') {
            return 0;
        }
    }
}
```

#### (3) 在 `main()` 里替换“等人按 a”这段

找到（`dji_bus.attach` 之后）：

```cpp
    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("no fresh feedback before arm: %d", ret);
        return ret;
    }
    ret = waitForManualArmToken();
    if (ret < 0) {
        LOG_ERR("manual arm input failed: %d", ret);
        return ret;
    }
    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("feedback lost before arm: %d", ret);
        return ret;
    }
```

替换为（照抄速度样例的自动启机模式）：

```cpp
    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("no fresh feedback before arm: %d", ret);
        return ret;
    }

    LOG_INF("feedback ready: arming now; keep the GM6020 output "
            "suspended and hold a power cut");

    /* Re-check so a stale reading is not used as the reset baseline. */
    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("feedback lost before arm: %d", ret);
        return ret;
    }
```

后面的代码（读基准位置 → `resetController` → `dji_bus.arm` → 运行循环）都不动。
运行循环会在反馈就绪后自动开始：0.5 s 稳住 → 走 +250 mrad → 保持到 8 s 结束
→ 显式 0 A + `Bus::stop()`。

> 可选：如果希望自动启动前留几秒反应时间，可以在上面 `LOG_INF` 之后、
> 第二次 `waitForFreshFeedback` 之前加 `k_sleep(K_SECONDS(3));` 并打印倒计时。
> 注意 `dji_speed_control` 当前源码里并没有这个倒计时（只保留了提示日志），
> “和速度样例一样”的话就不加。

#### (4) 可选：加注释说明自动启机行为

建议在 `namespace {` 内、常量区上方加一段，解释为什么没有人工确认：

```cpp
/*
 * No console arm token: once fresh feedback is present the loop arms
 * automatically (feedback ready -> arm), runs the timed +250 mrad profile,
 * then sends 0 A and stops.
 * Keep the GM6020 output suspended (free to move) before power-on.
 */
```

### 3.2 `samples/motor/dji_position_control/prj.conf`

当前内容：

```text
CONFIG_CPP=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_CAN=y

CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

CONFIG_SERIAL=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_CONSOLE_SUBSYS=y        # ← 删除（只服务于 console_getchar）
CONFIG_CONSOLE_GETCHAR=y       # ← 删除（只服务于人工输入 a）

CONFIG_SKYWALKER_LIB_CONTROL=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS=20
CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS=20
```

修改后：

```text
CONFIG_CPP=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_CAN=y

CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

CONFIG_SERIAL=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y

CONFIG_SKYWALKER_LIB_CONTROL=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y
CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS=20
CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS=20

# 主线程栈默认仅 1024 B；-Og + C++ + 长 printk 下 main() 会栈溢出
# （实测 ZEPHYR FATAL ERROR 2 / thread=main / MMFAR 命中 z_main_stack 底部）。
# 8192 为调试构建预留余量；若想省内存可先试 4096。
CONFIG_MAIN_STACK_SIZE=8192
```

说明：删除 `CONFIG_CONSOLE_SUBSYS` / `CONFIG_CONSOLE_GETCHAR` 不影响
`LOG_*` 与 `printk` 从 UART 输出，这两个开关只负责“读串口输入”的子系统。

> 如果你平时仍习惯用 `-DCONFIG_DEBUG_OPTIMIZATIONS=y` 构建，保留它没问题，
> 8 KB 主栈已覆盖；`CONFIG_DEBUG_OPTIMIZATIONS=y` 不需要写进 prj.conf。

---

## 4. 自检点与验证步骤

每步完成后对照下面的现象；所有命令请在仓库根目录手动执行。

### 4.1 语法自检

只改代码后先确认没有残留引用：

```bash
cd ~/skywalker_ws/skywalker_code
grep -n "console\|waitForManualArmToken" samples/motor/dji_position_control/src/main.cpp
```

期望：**无输出**（两处都清干净）。

### 4.2 重新构建

```bash
west build -b rm_typec/stm32f407xx samples/motor/dji_position_control -p \
  --build-dir samples/motor/dji_position_control/build/rm_typec/stm32f407xx \
  -- -DBOARD_ROOT=$PWD
```

自检：构建成功；再次确认新配置生效：

```bash
grep -E 'CONFIG_MAIN_STACK_SIZE|CONFIG_CONSOLE_GETCHAR' \
  samples/motor/dji_position_control/build/rm_typec/stm32f407xx/zephyr/.config
```

期望看到 `CONFIG_MAIN_STACK_SIZE=8192`，且没有 `CONFIG_CONSOLE_GETCHAR`。

### 4.3 烧录并运行（注意 WSL/USB 透传，见仓库根 `rm_typec板卡west一键烧录修复指南.md`）

```bash
west flash -r openocd
```

打开串口（建议 115200 8N1），给电机上电并确认 CAN 连接。

**安全顺序（每次必做）**：

1. 机械上先保证 GM6020 输出轴悬空（无负载、可自由转动）；
2. 确认手边有可随时断电的物理开关；
3. 再上电运行程序。

**期望日志**：

```text
*** Booting Zephyr OS build ...
[00:00:00.xxx] <inf> dji_position_control: GM6020 ...            (如保留该日志)
[00:00:0x.xxx] <inf> dji_position_control: feedback ready: arming now; ...
[00:00:0x.xxx] <inf> dji_position_control: ...                   (arm 成功后)
target=250 pos=... pos_err=... vel_ref=... vel=... current=... sat=... age=... ms
...                                                              (每 200 ms 一条)
[00:00:09.xxx] <inf> dji_position_control: timed position-control test completed and motor stopped
```

- 现象：不再需要按 `a`；电机自动先稳 0.5 s，再平滑转到 +250 mrad 处保持到
  8 s 结束，最后 0 A 停机；全程无 `MPU FAULT` / `Stack overflow`。
- 如果**仍然**出现 `FATAL ERROR 2 ... (main)`：
  1. 把 `CONFIG_MAIN_STACK_SIZE` 从 8192 继续加大（如 16384）验证；
  2. 或在遥测 `printk` 里临时加一行量实际占用：
     ```cpp
     printk(" main_free=%u",
            (unsigned int)k_thread_stack_space_get(k_current_thread()));
     ```

     观察运行中主栈剩余最小值，据此确定最终取值。

### 4.4 回归确认

跑一次完整 8 s 流程不崩溃、位置能停在 +250 mrad 附近、结束后 `Bus::stop()`
成功（`zero_sent=1`），即视为通过。

---

## 5. 风险与安全说明

- 去掉 `a` 后，**从“反馈就绪”到“通电转动”之间不再有人工确认点**。程序
  会在上电后约 1–2 s 内自动 arm 并执行 +250 mrad 动作，请务必先保证输出轴
  悬空、手边有断电开关，再给电机上电。
- 位置环只给 ±0.30 A 软件限幅，但这是 **GM6020 电流环**上的持续电流，堵转时
  依然会发热；不要长时间用手反拧卡住输出轴。
- 本改动只涉及样例代码与样例 `prj.conf`，驱动层（`drivers/motor/`）与
  `lib/control/` 均未改动；`MAIN_STACK_SIZE` 只影响本样例构建。

---

## 6. 最终勾选清单

- [ ] `main.cpp`：删除了 `#include <zephyr/console/console.h>`
- [ ] `main.cpp`：删除了 `waitForManualArmToken()` 整个函数
- [ ] `main.cpp`：`main()` 中不再调用 `waitForManualArmToken()`，
  替换为 `LOG_INF("feedback ready: arming now; ...")` + 二次新鲜度复查
- [ ] `prj.conf`：删除 `CONFIG_CONSOLE_SUBSYS=y`、`CONFIG_CONSOLE_GETCHAR=y`
- [ ] `prj.conf`：新增 `CONFIG_MAIN_STACK_SIZE=8192`
- [ ] `grep -n "console\|waitForManualArmToken" src/main.cpp` 无输出
- [ ] `west build` 成功，`.config` 确认 `CONFIG_MAIN_STACK_SIZE=8192`
- [ ] 串口实测：自动 arm → +250 mrad → 8 s 结束 → 0 A 停机，全程无
  `MPU FAULT` / `Stack overflow` / `FATAL ERROR 2`
