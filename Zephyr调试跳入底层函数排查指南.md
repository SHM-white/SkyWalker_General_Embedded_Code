# Zephyr 调试跳入 `__ISB()` 等底层函数排查指南

## 结论

当前现象大概率不是 `__ISB()` 故障，而是调试器停在 Zephyr 的调度、空闲或中断上下文中。

本例有两个直接原因：

1. `main()` 的控制循环每轮执行 `k_sleep(K_MSEC(5))`。主线程睡眠时，CPU 会运行 Zephyr idle 线程或处理中断；随机暂停很容易落在内核底层。
2. 任意反馈、安全限制、命令或 CAN 发送错误都会经过 `stopAfterFailure()` 返回负错误码，随后 `main()` 直接返回。主线程一旦结束，再暂停时自然只剩 idle、调度器和中断相关调用栈。

`__ISB()` 是 ARM 的“指令同步屏障”，Zephyr 在锁定或恢复中断时会调用它。它被 CMSIS 声明为强制内联，即使当前工程使用适合调试的 `-Og -g`，源码单步映射也仍可能显示在这个头文件函数中。

## 当前执行路径

```text
main 控制循环
  -> 读取反馈及安全检查
  -> 设置扭矩
  -> 发送 CAN
  -> k_sleep(5 ms)
       -> 主线程阻塞
       -> Zephyr 调度器/idle/中断
       -> arch_irq_lock()/arch_irq_unlock()
       -> __ISB()

任何一步失败
  -> stopAfterFailure()
  -> 发送 Disable
  -> main() 返回负错误码
  -> main 线程终止
  -> 系统只运行 idle/系统线程/中断
```

## 先区分正常内核停点和真实 Fault

### 正常情况

调用栈包含以下名称时，通常只是暂停到了 RTOS 正常路径：

- `idle`、`z_idle`、`arch_cpu_idle`、`k_cpu_idle`
- `z_swap`、`PendSV_Handler`、`z_arm_exc_exit`
- `SysTick_Handler`、定时器或 CAN/UART ISR
- `arch_irq_lock`、`arch_irq_unlock`、`k_spin_lock`、`k_spin_unlock`
- `__ISB`、`__DSB`、`__WFI`

这时按 `Continue/F5` 即可，不应把 `__ISB()` 当成根因。

### 真实异常

调用栈出现以下名称，或串口打印 `***** HARD FAULT *****`，才需要按 Fault 排查：

- `z_arm_fault`
- `z_fatal_error`
- `k_sys_fatal_error_handler`
- `HardFault_Handler`、`MemManage_Handler`、`BusFault_Handler`、`UsageFault_Handler`

在 GDB 中读取：

```gdb
info registers pc lr sp xpsr msp psp control
bt
thread apply all bt
p/x *(unsigned int *)0xE000ED28
p/x *(unsigned int *)0xE000ED2C
p/x *(unsigned int *)0xE000ED34
p/x *(unsigned int *)0xE000ED38
```

地址依次对应 `CFSR`、`HFSR`、`MMFAR`、`BFAR`。保存这些数值、串口 Fault 输出以及完整调用栈，不要只记录当前显示的 `__ISB()` 行。

`xPSR` 低 9 位是当前异常号：0 表示线程模式；3/4/5/6 分别是 HardFault、MemManage、BusFault、UsageFault；14 是 PendSV，15 是 SysTick，大于等于 16 是外设中断。

## 本例推荐调试方法

不要从启动位置一路 `Step Into/F11`。在以下业务位置设置断点，然后使用 `Continue/F5`：

1. `main.cpp` 中 `readSafeFeedback()` 调用后的 `if (ret < 0)`。
2. `setTorque()` 调用后的错误分支。
3. `flush()` 调用后的错误分支。
4. `stopAfterFailure()` 的入口。

函数调用优先使用 `Step Over/F10`，尤其不要逐语句进入：

- `k_sleep()`
- `LOG_INF/LOG_ERR`
- `vofa_send()`
- CAN 驱动调用
- Zephyr 的锁、队列和调度 API

如果电机已经由绿灯变成长亮红灯，先重新启动调试会话，再让断点捕获第一次错误；此时 `main()` 很可能已经返回，继续暂停看不到业务调用栈。

建议额外设置三个函数断点：

```gdb
break z_arm_fault
break z_fatal_error
break k_sys_fatal_error_handler
```

它们都没有命中，而错误分支或 `stopAfterFailure()` 命中，说明是程序主动停止，不是 MCU 异常。

## 调试信息与固件一致性

当前 `dm_mit_control` 构建配置已有：

- `CONFIG_DEBUG_OPTIMIZATIONS=y`
- `CONFIG_DEBUG_THREAD_INFO=y`
- 编译参数 `-Og -g -gdwarf-4`
- ELF 内含 `.debug_info`、`.debug_line`、`.debug_frame` 和符号表

因此当前符号信息并未缺失。不过 `-Og` 仍允许内联和一定程度的代码重排，强制内联函数尤其容易出现“源码跳行”。如果必须进行逐语句教学式调试，可在 Zephyr IDE 新建专用构建配置，选择无优化构建，并确保不再同时传入 `CONFIG_DEBUG_OPTIMIZATIONS=y`。这会改变时序和镜像大小，只应用于诊断，不应用于最终电机控制验证。

每次调试前还要确认：

1. Zephyr IDE 当前活动项目确实是 `dm_mit_control`。
2. 调试器加载的 ELF 是 `samples/motor/dm_mit_control/build/dm_mc02/stm32h723xx/zephyr/zephyr.elf`。
3. 板上烧录的二进制与该 ELF 来自同一次构建。
4. 修改 `prj.conf`、overlay 或源码后做一次干净构建再下载。

ELF 与板上固件不一致时，PC 地址会被映射到错误源码行，调用栈也可能看起来全是无关底层函数。

## 建议操作顺序

1. 电机卸载，并准备可立即断电的急停手段。
2. 连接 USART10（PE2/PE3，115200）保存完整 Zephyr 日志。
3. 在 `stopAfterFailure()`、`z_arm_fault` 和 `z_fatal_error` 设置断点。
4. 重新下载与当前 ELF 匹配的固件并复位。
5. 使用 `Continue/F5` 运行，不进入内核 API。
6. 若先命中 `stopAfterFailure()`，记录 `original_error`；`-34` 是安全范围检查触发。
7. 若先命中 `z_arm_fault`，记录寄存器、Fault 状态寄存器和完整调用栈。
8. 若两者都未命中，只在手动暂停时看到 idle/`__ISB()`，则属于正常 RTOS 停点。

## 最终检查清单

- [ ] 断点设置在业务错误分支，而不是底层头文件。
- [ ] 使用 F5 运行、F10 越过系统调用。
- [ ] 已区分红灯长亮（失能）和红灯闪烁（电机故障）。
- [ ] 已确认 `main()` 是否已经因负错误码返回。
- [ ] 已检查是否命中 `z_arm_fault` 或 `z_fatal_error`。
- [ ] 调试 ELF 与板上固件来自同一次构建。
- [ ] 若发生 Fault，已保存 PC/LR/xPSR/CFSR/HFSR/MMFAR/BFAR。
