# 12 故障排查

按“配置 → 构建 → 通信/CAN → 生命周期 → 控制”顺序排查。负值返回码通常是 errno 的负值。

## 1. 配置与构建

| 现象 | 检查 |
|---|---|
| 找不到 board | 从 west workspace 构建；确认 `zephyr/module.yml` 和 board identifier，使用 `dm_mc02/stm32h723xx` 或 `rm_typec` |
| 驱动源没进构建 | 在 `.config` 检查对应 `CONFIG_SKYWALKER_*`；根 CMake 只按 Kconfig 加入子目录 |
| DT binding 报 required property | 对照 `dts/bindings/`，特别是 CAN、电机量程、IMU phandle 和 `heat-*` 字符串 |
| `__device_dts_ord` 链接错误 | 节点未 `status = "okay"`、phandle 错误或驱动 Kconfig 没开；检查生成的 `zephyr.dts` |
| C++ 链接/栈问题 | 打开 `CONFIG_CPP=y`、`CONFIG_STD_CPP20=y`、`CONFIG_REQUIRES_FULL_LIBCPP=y`，并提高 `CONFIG_MAIN_STACK_SIZE` |
| 改 overlay 但现象不变 | 使用新的 `-d` 目录或 `west build -p`，不要复用错误 board 的 CMake cache |

## 2. 烧录与板卡

| 现象 | 处理 |
|---|---|
| `no runners.yaml found` | 重新 pristine build；检查 `build/<name>/zephyr/runners.yaml` |
| OpenOCD 打不开 | 核对 probe 选项、USB 权限、目标电压、SWDIO/SWCLK/GND；WSL 需要先透传调试器 |
| MC02 probe 不匹配 | 构建时选择 `-DSKYWALKER_OPENOCD_PROBE=cmsis-dap|stlink|stlink-hla` |
| `rm_typec` 无法连接 | 它没有板载调试器，需要外部 CMSIS-DAP 或 ST-Link |
| 日志没有输出 | 检查 chosen console、串口设备、波特率和是否被 VOFA/其他工具占用 |

## 3. CAN 与电机

| 现象 | 常见原因 |
|---|---|
| 反馈一直 Offline | CAN 未 start、收发线/地线/终端错误、bitrate 不同、motor-id/master-id 错误 |
| DJI 反馈数值存在但无法 arm | 反馈不新鲜、Bus 未 attach、GM6020 没有 `current-loop-confirmed`，或设备状态 Fault |
| DM arm 超时 | 电机未 Enable、master-id 路由错误、PMAX/VMAX/TMAX 不匹配、动力电源未开 |
| `-ERANGE` | effort/速度/温度/dt 超出边界，或设备树限幅超过协议边界 |
| 多电机互相影响 | 同一 CAN 没有共享原生 Bus，或 command ID/slot、DM `(master-id,motor-id)` 冲突 |
| 位置差一个减速比 | 驱动反馈已经换算到输出轴，不要再除一次 gear ratio |
| 位置零点不对 | GM6020 检查 `encoder-zero-ticks`；统一封装检查 `PositionReference` |

先运行 `samples/motor/can_smoke`，再运行原生模式样例，最后才进入软件闭环。

## 4. 统一电机封装

| 返回/状态 | 含义 |
|---|---|
| `-EACCES` | 未 configure、未 Ready/Active，或权限不允许 |
| `-EAGAIN` | 正在等反馈、CAN 恢复或稳定窗口 |
| `-ESTALE` / `-EHOSTDOWN` | 反馈过期或驱动状态不可用 |
| `-ERANGE` | dt、速度、温度、effort 或控制配置越界 |
| `-EALREADY` | 同一个 wrapper 重复 configure/begin |
| `telemetry.valid=false` | 本次 update 没有产生可用输出；不要沿用旧 effort |

运行状态正常路径是 `configure → poll → Ready → resume → Active → update`。故障后先 suspend/stop，再 poll；恢复成功且拿到新鲜反馈、新命令后才允许 resume。

## 5. 通信与安全

| 现象 | 检查 |
|---|---|
| DR16 offline | UART 100000、8E1、实际电平反相链路、18 字节帧和 `offline_timeout_ms` |
| 裁判 online 但没有权限 | `RefereeVersion` 是否明确为 `Rm2026V1_3`，权限帧 0x0201 是否通过 CRC |
| 板间反复 offline | TX/RX 交叉、共地、460800、角色不同、CRC 和序列号 |
| `-EOVERFLOW` 后安全不恢复 | 解析器必须 discardPartial；等待下一帧，不要拼旧缓冲 |
| 全局一直 Disable | 查看 `active_reasons`：CommandStale、PowerStale、FeedbackStale、RecoveryBoundary、EmergencyStop 等 |
| 急停清不掉 | 急停输入必须先释放，然后调用 `clearEmergencyStop(true)`；复位不会自动开始运动 |

## 6. IMU / 数学库

| 现象 | 检查 |
|---|---|
| 姿态不收敛 | 上电静止、传感器方向、dt、加速度幅值和 EKF 4/3 维度 |
| yaw 从 π 跳到 -π | 当前对外角度是包角；应用层自行解包 |
| 加热不工作 | PWM 通道 4、20 ms 周期、`heat-output-max` 不超周期、参数是 string |
| 栈溢出 | Kalman 临时 VLA 和 C++ 日志都可能增加栈，调大主/线程栈 |

## 7. 安全底线

任何电机排查都应：机构脱离负载、低限幅、手边物理断电。`stop()` 不是机械制动；CAN 恢复成功也不代表可以立即重新施加运动命令。先确认反馈、状态、权限和新命令，再逐步恢复。
