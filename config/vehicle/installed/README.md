# MC02 实车共享配置基线

日期：2026-10-25。此目录固定新 [独立上车工程](../../../samples/bringup/README.md) 和两个最终应用的板级资源、系统输出策略与驱动后端。当前内容依据仓库已有接线配置收敛，尚未实查 DAPLink 接线或完成实板验证。

每个工程的 `app.overlay` 只包含对应 `gimbal.overlay` 或 `chassis.overlay`；构建文件固定引用 `base.conf`、板级 conf 与工程本地 `prj.conf`。本地工程只选择所需模块能力，使用自己的 `main.cpp` 装配对象、启动接收器和执行控制循环。物理设备声明不意味着每个工程都会创建或使能该设备的执行器。

## 资源所有权

| 板卡 | 设备 | 代码基线与所有者 |
| --- | --- | --- |
| 云台 | UART5 / DBUS | DR16 遥控，保留板上电气通路 |
| 云台 | USART10 | 板间 UART，PE3 TX / PE2 RX，460800 8N1；DMAMUX 通道 3/4，TX/RX 请求 119/118 |
| 云台 | UART7 | `diagnostic-uart`，PE8 TX / PE7 RX，115200 8N1；移除 DMA，由诊断 worker 单独发送 |
| 云台 | CAN1 / CAN2 | 小 Yaw / Pitch 电机 |
| 云台 | CAN3 | 头部 DM IMU 独占，1 Mbps、主动四元数上报，从机 0x1 / 反馈 0x11 |
| 云台 | PC15 | 外部 5 V 供电使能，启动时启用；仍须核对实物供电 |
| 底盘 | CAN1 / CAN2 / CAN3 | 四舵向 / 四轮驱 / 大 Yaw，分别只有一个 CAN 所有者 |
| 底盘 | USART1 | 板间 UART，460800 8N1；沿用板级 DMA 通道 3/4 |
| 底盘 | UART7 | 与云台相同的单口诊断配置，无 DMA |

板载 SPI2 IMU 的 DMA1 通道 1/2 保留。云台 USART1 释放 DMA 3/4，再由 USART10 占用；不能同时启动两个共享这些 DMA 通道的串口异步链路。底盘保留 USART1 板间映射，不把 USART10 误认成当前板间口。

DAPLink 目标连接尚未实查：当前候选是 UART7 PE8 TX 接 DAPLink RX，双方共地，电气形式和接头须与实际板卡一致。模式切换不依赖 RX 命令，因此无需为切换模式新增 PE7 RX 接线。若 DAPLink 实际接到别的 UART，先解决与业务口的冲突，再一次性修改对应共享 overlay；不要让不同阶段用不同接线。

新共享配置不提供 `vision-uart` 别名，UART7 由诊断独占。后续视觉工程应先确定独立的物理资源、DMA 和接线，再添加其专用配置；不能把 UART7 同时交给视觉、console、AsyncUart 或 VOFA 接收器。USART3 仅保留裁判资源声明，真实裁判 TTL 与板载 RS485 仍需匹配的电气适配，声明不表示接收器已启动。

## 单变量选择日志或 VOFA

统一选择位于 [diagnostics_config.hpp](../../../include/robotics/vehicle/diagnostics_config.hpp)：

```cpp
inline constexpr DiagnosticOutput diagnostic_output = DiagnosticOutput::Log;
```

改为 `DiagnosticOutput::Vofa` 后，用原工程命令重新编译和烧录即可；无需改 Kconfig、额外 conf、串口命令或接线。模式编入二进制并在本次运行期间固定，复位继续使用该固件模式。电脑端同一时刻只让串口终端或 VOFA 中一个程序占用 DAPLink COM 口，波特率统一为 115200。

`base.conf` 关闭串口日志后台、console、shell 和启动 banner，启用专用日志后台。Log 模式把 `LOG_*` 与接入 Zephyr 日志的 `printk` 文本送入同一输出队列；Vofa 模式从初始化阶段丢弃这些文本，只发送 JustFloat，不因普通错误自动切回日志。以后新增错误路径也应使用这个输出策略，不能直接向 UART7 调用文本发送函数。

诊断队列固定为 16 项，每项最多 192 字节，生产方通过 `K_NO_WAIT` 提交。队列满或数据过长时丢弃观测并累计丢帧数，不阻塞控制线程。唯一诊断 worker 从队列读取后向 UART7 发送；日志、状态、波形不分别抢占串口。设备不可用时计数丢弃，也不改变电机授权或目标。VOFA 状态按 20 ms 预算提交，日志状态通常为 1 Hz，原因变化单独报告；共享通道、布局版本和单位见 [上车说明](../../../samples/bringup/README.md)。

无效测量由有效位与 NaN 标识，未知年龄为 -1 ms；大序号和丢帧计数拆成高低 16 位，避免 float 精度损失。诊断初始化失败、系统致命故障或执行中止时，数据流可能停止，主机应保留采集时间，不能把停流解释为最后测量值仍有效。普通 RAM 记录在复位/重新烧录后丢失，切回日志固件不会恢复上次 VOFA 的历史故障；波形应由电脑保存。

## 参数与版本

标定数据在 [calibration.hpp](../../../include/robotics/vehicle/calibration.hpp)，硬件/PID/控制策略由同目录的 `gimbal_profile.hpp`、`manual_profile.hpp`、`chassis_profile.hpp`、`swerve_profile.hpp` 和 `big_yaw_profile.hpp` 提供；这些都是无副作用配置，不隐藏静态电机、线程、总线提交或入口循环。

[diagnostics_config.hpp](../../../include/robotics/vehicle/diagnostics_config.hpp) 当前候选标识为硬件 `mc02-installed-20261025-candidate`、标定 `vehicle-20261025-unverified`；协议固定 v4。启动身份还包含应用、板角色和源码版本，便于区分本地未提交修改。配置修改后重建所有受影响阶段和最终应用；已烧录固件及旧通过记录不会自动更新。

中央确认标志当前保留连接、IMU 安装、发射约束为 true，功率模型为 false。它们是当前代码配置，不能代替现场核对。阶段工程只应使能其明确负责的机构；新最终云台暂不启动发射/视觉，新最终底盘使用低电流手动机械策略，功率实测与受约束执行仍需单独接入。
