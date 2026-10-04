# 单舵轮双 CAN 遥控台架

本例程调试一个舵组：GM6020 接 CAN2、ID 2；M3508/C620 接 CAN1、ID 2。两个电机属于同一故障组，任一电机或总线故障均撤销整组输出。CAN 均为 1 Mbit/s。

## 配置与接线

默认主控为 DM MC02。DR16 接板级 `remote-uart`（UART5 / PD2 RX，100000 baud、8E1、RX DMA），接收机与主控共地。console 使用板级日志串口，115200 baud，仅用于观察。

本样例的接线已按本次台架设置启用，不再依赖整车 `connections_confirmed`。参数集中在 `src/board_config.hpp`：

| 参数 | 当前值 / 含义 |
| --- | --- |
| 舵向 | CAN2 / GM6020 ID 2，电流模式 |
| 轮驱 | CAN1 / M3508 ID 2，减速比 3591/187 |
| 电流上限 | 舵向 0.8 A，轮驱 0.5 A |
| 控制周期 | 每轮相对休眠 5 ms，控制器使用实际间隔；实际周期包含计算耗时 |
| CAN 位时序 | HSE 24 MHz，共用分频 1；两路均为 1 Mbit/s、87.5% 采样点、SJW 3 tq（125 ns）；启动输出驱动报告的时钟及配置 |
| 运动速度 | 指令矢量模长上限 0.1，沿用参考工程经验速度比例，斜推不会额外增速 |
| 等效换算半径 | `0.01 × (3591/187) ≈ 0.192032 m`，非实测轮半径 |
| 方向 | `steer_direction=+1`、`drive_direction=+1`，沿用参考工程 ID 2 |
| 舵向零点 | 固定 2421 ticks，沿用参考工程 ID 2（右前轮） |

`steer_current_mode=true` 是当前配置假设，程序不会自动识别或切换 GM6020 固件模式。运行前确认实际电机支持并处于电流模式；现有驱动不支持电压模式。

应用只在电机组 Active、控制计算和两个电流指令更新均成功后提交两路 CAN。启动、待机和故障停机的安全帧由 `CanBus` 自身发送，不依赖应用持续 `commit()`；运行中摇杆归中仍按控制周期提交零电流。这与 `dji_speed_control` 的提交条件一致。速度样例的持续运行只作为基本链路可用的对照，不作为 PID 参数已调好的依据。

本次日志在 `129052 ms` 显示 `drive_on=1`、轮驱电流指令 `-0.352 A`、轮速为零；说明该时刻对齐门已打开，但不能由命令值推断电流实际到达。`132117 ms` 的新组故障为 CAN2/6020 `TransportError/-ENETUNREACH`，随后整组撤销授权。两路发送错误计数均曾上升，故不能把这次停机归因于 PID 或周期检查。

**只把采样点从 75% 改为 87.5% 未能解决 Bus-off。** 最新复测在 `16094 ms` 时两路都已 ErrorPassive，CAN2/CAN1 的 TEC 分别为 151/159；轮驱已开门，电流指令 0.224 A、反馈 0.145 A、轮速为零。`16199 ms` 的 CAN2 普通目标帧 `0x1fe`（`purpose=0 seq=297`）报 `-ENETUNREACH`，随后整组停机。CAN1 在 `365 ms` 前已有四次恢复，最近一次为发送超时 `-116`，快照中有 22 次 ACK 错误；`pre_restart_tx` 中的旧 `-115` 是停止控制器产生的回调结果，不是该次超时的根因。此次 VOFA 入队为零，因此 VOFA 发包不是该故障的必要条件。

用户确认：此前分别运行速度样例时，另一台电机始终保持 CAN 接线和供电。单路持续运行没有验证双控制器同时启用的情况，也不证明 PID 已调好。代码路径中尚未找到两路发送缓冲或消息 RAM 重叠，Bus-off 根因仍未确定。

当前配置试修只作用于本样例：将 CAN1/2/3 的共用 FDCAN 时钟来源统一从 PLL1Q 120 MHz 切到 HSE 24 MHz，由 CAN1 显式设置共用 CCU 分频 1、旁路校准。CAN3 虽未由应用启动，其设备初始化也会写共用时钟选择，所以必须一并指定 HSE。应用启动前校验驱动报告的时钟等于 HSE，再计算 1 Mbit/s、87.5% 位时序，并把 SJW 设为相位段和硬件上限允许的最大值；本板应为 `prescaler=1 seg1=20 seg2=3 sjw=3`。配置失败则不启动电机总线。该调整包含时钟来源和 SJW 两项，**尚未实板确认有效，不能据此认定原来 120 MHz 配置非法或根因已修复**。PID、限幅、遥控授权和故障停机条件保持原值。时钟关系见 [ST FDCAN 时钟说明](https://community.st.com/stm32-mcus-60/faq-fixing-stm32-fdcan-communication-disruptions-apb-bus-kernel-and-time-quanta-clocks-141123)。

另修正独立的周期边界问题：入口使用三个控制环配置的共同 `dt` 范围（当前 1～20 ms），周期异常按可重新解锁的 `cycle` 停机处理；相对休眠避免延迟唤醒后追赶旧期限造成过短周期。以前只检查 20 ms 上限，低于 1 ms 时控制器返回 `-ERANGE`，会被当成需要手动清故障的硬故障。该缺陷不等于上述 Bus-off 的原因。

参数来源：[h7_framework-main / app_robot.c](https://github.com/SHM-white/h7_framework-main/blob/05e53b3ef0817a0bdeb91aca871516d4ad471371/Skywalker/Application/Src/app_robot.c)，版本 `05e53b3`。其 ID 1～4 零点依次为 3060/2421/2383/3097，本例使用 ID 2 的 2421，不套用 FL 的 3060。适用前提是当前组件就是原 ID 2 的机械安装，单纯给另一组改成 ID 2 不会继承其零点。原工程以 +Y 为前方，本框架以 +X 为前方；零点均表示机械正前方。

参考工程轮速换算为 `omega_rotor = v × 1 / 0.01`，其电机反馈直接从 rpm 转成电机轴 rad/s。当前驱动反馈已除以真实减速比 `G=3591/187`，故模块采用等效半径 `R=0.01×G`，得到 `omega_output=v/R`、`omega_rotor=G×v/R=100×v`。对齐且限速斜坡稳定后，最大指令 0.1 对应约 10 rad/s 电机轴转速、0.521 rad/s 输出轴转速。这保留参考工程速度比例，不能把日志中的速度指令当成已经标定的真实 m/s；实际轮径仍未知。测得轮半径后可直接用实际值替换 `wheel_radius_m`，同时确认总减速比。

默认固定零点。如果当前机械安装已变，可在上电前把轮子摆到希望的正前方，并设置 `capture_startup_zero=true`；首次反馈稳定后打印 `startup steer zero=...`，临时零点保持到主控重启，故障恢复和再次解锁不会重采。

电机供电由台架外部提供；例程不主动开启 MC02 的受控 XT30 电源口。首次调试将轮子架空，确认方向与实际轮径后再落地。

## 遥控操作

1. 左、右开关都拨到 Down，双摇杆及拨轮归中至少 0.5 秒。
2. 保持右开关 Down，将左开关拨到 Middle 解锁。
3. 左摇杆纵向控制前后、横向控制左右；程序先转舵，对齐后驱动轮子。左横杆向右对应负横向速度。
4. 摇杆归中后两轴卸力（Coast，不主动锁舵或制动）。左 Down 随时撤销运行，左 Up 同样停止本样例。

右摇杆、拨轮不产生运动指令，解锁时仍需归中。单模块没有整车旋转半径，因此不沿用四舵轮 FL 的拨轮偏航映射。大角度目标会使用既有最短转舵与轮速反向优化，例如后退可能保持舵角并让轮子反转。

遥控超过 100 ms 未更新、运行故障或失能后必须重新执行安全档归中解锁。清故障：先双 Down 归中 0.5 秒，保持左 Down，将右开关拨到 Up 并全部通道归中 1 秒；只清除一次，不自动运行。

## 构建与观察

### USB CDC / VOFA+

板载 USB CDC ACM 专门输出 VOFA JustFloat，console 日志继续走板级硬件 UART。将 MC02 的 USB 接到电脑，在 VOFA+ 中选择枚举出的 `SkyWalker Telemetry` 串口、115200 baud、**JustFloat** 协议；USB CDC 的波特率不决定实际 USB 速率。启动日志应显示 `VOFA USB device=VOFA init=0`（设备名由板级 `label` 决定）。

控制循环每轮相对休眠 5 ms，USB 遥测每 10 ms 检查一次主机 DTR；上位机打开串口并置 DTR 后才提交 16 通道（68 字节）。`vofa_connected=0` 时跳过入队，避免串口未打开时不断塞满队列并产生 `-ENOBUFS`；未连接不计入丢帧数。上位机需启用 DTR，且 DTR 置位本身不保证上位机及时读取。连接后发送队列满时仍丢弃当前帧、最多每秒报告一次错误；遥测初始化失败也不阻塞电机控制。执行暂停诊断期间遥测同步暂停。USB 口只输出二进制数据，不接收运动指令。该修正不代表 CAN 故障已解决。

JustFloat 不携带名称，按以下顺序配置曲线（通道从 0 开始）：

| 通道 | 内容 | 单位 / 说明 |
| --- | --- | --- |
| 0 | 前后速度指令 vx | 参考工程经验速度单位 |
| 1 | 横向速度指令 vy | 同上，非标定实际 m/s |
| 2 | 优化后目标舵角 | rad，包含翻转优化 |
| 3 | 实际舵角 | rad，已校零并施加方向符号 |
| 4 | 限速后的连续舵角参考 | rad，可跨圈 |
| 5 | 舵向对齐误差 | rad |
| 6 | 驱动目标转速 | 输出轴 rad/s |
| 7 | 驱动实际转速 | 输出轴 rad/s，已施加方向符号 |
| 8 | 6020 电流指令 | A，电机方向；组未 Active 时为 0 |
| 9 | 6020 电流反馈 | A，电机方向 |
| 10 | 3508 电流指令 | A，电机方向；组未 Active 时为 0 |
| 11 | 3508 电流反馈 | A，电机方向 |
| 12 | 遥控新鲜有效 | 0/1 |
| 13 | 电机组 Active | 0/1 |
| 14 | 两电机反馈健康 | 0/1，失效时反馈曲线可能保留旧值 |
| 15 | 驱动输出使能 | 0/1，要求 Active 且舵向对齐 |

电流指令表示本地控制输出，不代表总线已送达；实际状态结合电流反馈和健康标志观察。未运行或卸力时目标通道可能为 0，先看 Active、驱动使能再分析曲线。

### 编译与日志

VS Code 启动调试时，在“运行和调试”的下拉框选择 **MC02: Build and Run (no RTOS)**，再按 F5。Zephyr IDE 中的活动工程仍需选择本样例及 MC02；此启动项使用活动构建的 ELF，构建和下载指向同一目录，不固定使用下面的命令行构建目录。

OpenOCD 可执行文件和脚本目录使用本机 `${env:HOME}/zephyr-sdk-1.0.1/hosttools/...`。当前扩展的 `get-toolchain-path` 返回 SDK 内的 `gnu/arm-zephyr-eabi`，不能直接在其后拼接 `hosttools`；否则服务器尚未启动便会退出。以后更换 SDK 安装位置时，同时更新 `.vscode/launch.json` 的 `serverpath` 和 `searchDir`。

该启动项直接使用 Cortex-Debug，关闭 OpenOCD 的 RTOS 线程解析，SWD 速率降到 1000 kHz，下载、复位后自动运行，不再设置 `main` 临时断点。已有的手工断点和 HardFault 捕获仍然会暂停；需要观察初始化时可自行给 `main` 加断点。USB 栈在系统初始化阶段启动，VOFA 发送在 main 的循环中进行；持续收到新 JustFloat 帧表示主循环在运行，但不代表电机已解锁。此模式只能查看当前 CPU 上下文，不提供 Zephyr 全线程列表。原有 Zephyr IDE 调试项仍保留。

这是针对启动日志中 `No symbols for Zephyr` / `Only size_t of 4 bytes are supported` 的线程解析隔离措施，不代表 double fault 的固件根因已经确定。Zephyr IDE 4.1.0 的转接器自动加入 `rtos: Zephyr` 并过滤用户对 `rtos` 的覆盖，因此只在旧启动项里改该字段无效；需从下拉框使用新入口，不能继续点击 IDE 自动生成的 Debug 入口。

新入口启用 HardFault 向量捕获。如果仍在 `main` 前异常停止，保留该现场，在 Debug Console 执行下面的命令，并保存输出；不要连续重启清掉故障寄存器：

```text
info registers
bt
x/6wx 0xe000ed28
x/wx 0xe000ed08
```

前一组依次包含 CFSR、HFSR、DFSR、MMFAR、BFAR、AFSR，后一项为 VTOR。`reset.S` 中收到 SIGINT 只说明调试器报告复位入口停止，不能单凭该位置认定复位汇编有错误。相关线程解析报错来自 [OpenOCD Zephyr RTOS 插件](https://github.com/openocd-org/openocd/blob/master/src/rtos/zephyr.c)。

每秒输出一次 `alive ms=... loops=... vofa_queued=... vofa_drop=... can_err=.../...`，随后输出当前状态。`ms`、`loops` 持续递增表示控制循环运行；VOFA 入队计数不保证上位机收到，`vofa_drop` 表示被拒绝的帧数。其余原有详细日志同步降低到每秒一次，USB 遥测仍为 10 ms 一帧。

本次现场主线程在正常周期休眠、CFSR/HFSR 为 0；两条 CAN 均处于 Recovering，最近错误为 -114（此工具链的 `ENETUNREACH`，对应 Bus-off 路径）。CAN 恢复现改为重启一次后按重试间隔等待控制器真正退出 Bus-off，再允许发送；等待期间忽略不可执行的旧发送期限。连续即时工作每 8 轮休眠一个内核 tick，让低优先级遥控接收和日志线程获得运行机会。故障停机和重新解锁要求保持有效。

`can_err` 是最近历史错误，不会随着恢复自动归零，需结合 `can` 状态（0 未启动、1 Running、2 Recovering、3 配置阻塞）和反馈健康标志判断。若持续 Recovering，需要检查对应 CAN 的电机供电、CAN H/L、共地、1 Mbit/s 波特率和终端电阻；软件限速重试不会修复物理总线故障。多次手动复位或下载会截断尚未发完的 UART 日志并重新打印启动信息，不能单凭这些拼接的片段认定程序在自发重启。

### `active=0` 时看什么

先看每秒输出的 `arm=...`，它直接反映遥控解锁状态：

| `arm` | 当前条件与操作 |
| --- | --- |
| `rc_offline` | 遥控数据无效或超时，先恢复接收 |
| `return_both_down` | 未解锁；两开关回 Down 后重新开始，保持 Middle 不会重新解锁 |
| `center_controls` | 两开关已 Down，但有通道未归中；看 `raw_L`、`raw_R`、`wheel`，五个值均须在 ±33 内 |
| `hold_down_500ms` | 已归中，等待新遥控帧累计满 500 ms；`hold_ms` 为已保持时间 |
| `move_left_to_middle` | `arm_ready=1`，保持所有通道归中、右开关 Down，仅左开关拨 Middle |
| `unlocked` | 遥控授权已建立；若仍未 Active，继续看 `gate`、`pending`、反馈和 CAN 状态 |

开关日志采用解码后的枚举：1=Up、2=Middle、3=Down。因此 `switches=3/3` 为安全档，此时 `active=0` 正常；`switches=2/3` 才是本例运行档。拨档成功后再推左摇杆，先保持约一半行程几秒；当前速度起动阈值为 0.02，小幅推杆可能仍在 Coast。

`stops` 记录应用观察到的运行/使能中断次数，`last_stop/stop_err/stop_ms` 保留最后一次撤销的原因、错误及开机时间，即使随后已恢复为 `ready=1` 也不会丢失。`group_fault/motor/fault_ms` 是电机组保存的历史故障，需结合时间判断；`motor=6020_CAN2` 或 `3508_CAN1` 指出故障来源。恢复后的 `gate=command` 表示正在等待重新授权，不能据此覆盖之前的总线故障结论。

每次撤销运行还立即记录一组 `STOP` 日志，包括失败操作、实际 `dt_us`、两轴反馈字段/速度/电流/温度、撤销前的对齐误差与轮驱目标。先请求停机再打印。`operation=module_step` 与 `steer_current/drive_current/steer_commit/drive_commit` 区分控制计算、设电流和发布总线失败；`prerequisites` 表示周期、反馈、总线或已发生的电机故障检查。总线状态异常时保留总线错误码，不再统一记录成 `-ESTALE`。硬故障等待人工清除期间保留最初的恢复门原因。

Zephyr M_CAN 的 `can_start()` 会重置错误统计，恢复后的安全帧也会覆盖 `last_tx`，因此应用打印 `STOP` 时的实时计数可能已经属于重启后的控制器。本版在 `CanBus` 进入恢复、尚未停止/重启控制器时保存 `last_recovery`，每秒输出 `pre_restart` 和 `pre_restart_tx`；恢复成功后仍保留，直到下一次恢复覆盖。`count/ms` 标识本次恢复，`pre_restart_tx` 是进入恢复前最近一次完成的发送，并不保证它就是故障帧（例如接收溢出或尚未完成的发送超时）。快照由 I/O 线程采集，硬件自动 Bus-off 恢复可能已经开始；它不是中断瞬间的原子快照，也不保证 `TEC` 等于刚触发 Bus-off 时的数值。未启用 CAN 统计的其他样例会显示 `stats=0`。

周期日志新增 `drive_gate=inactive/coast/alignment/enabled`，并并排打印轮驱目标、实际速度、电流指令与反馈。`enabled` 且电流指令非零而轮速为零时，应继续区分电流反馈、发送结果和负载；不能再解释成轮驱没有使能。当前仍是每秒采样日志，不能据此排除两个采样点之间发生过短暂开门或运动。

`CAN2_6020` 和 `CAN1_3508` 分别输出底层控制器 `hw` 状态（0=ErrorActive、1=ErrorWarning、2=ErrorPassive、3=BusOff、4=Stopped）以及发送/接收错误计数 `TEC/REC`。`query_err=0` 时这些底层值有效；ErrorActive 是 CAN 的正常状态名称。`last_err`、`tx_err` 是历史结果，结合 `tx_ms`、控制器当前状态和新故障时间分析。遥控安全门、故障撤销和 VOFA 16 通道顺序保持原有行为。

启用 Zephyr CAN 统计后，同一行的 `bit0/bit1/stuff/crc/form/ack` 为驱动观察到的累计错误类型；看前后增量，不把历史非零值当成当前故障。Bit0 表示期望显性位却读回隐性位，Bit1 相反，ACK 表示未收到应答。M_CAN 在 Bus-off 恢复期间也可能报告 Bit0，因此需结合 `hw`、`TEC` 和时间段解释，不能仅凭 Bit0 累计量判定接线故障。延迟日志缓冲扩为 4096 字节，容纳整组诊断输出。

若日志先出现 `arm=unlocked active=1`，随后 `last_stop=transport`、`group_fault=8/-114`，说明 CAN Bus-off 导致整组停机。先确认物理总线：主控、电机及 USB/调试器供电全部断开，CAN 线保持连接，分别测量 CAN1、CAN2 的 H-L 电阻。每条独立总线两端各接 120Ω 时应约为 60Ω；约 120Ω 通常提示缺一端，约 40Ω 通常提示多接了一只，开路/很大提示未接终端或连线中断。电阻测量不能排除供电、共地、接触不良、收发器和位时序问题。

单电机作为总线末端时，GM6020 的第 4 位拨码 ON 接入内部终端；C620 有独立的 120Ω 终端开关。MC02 端是否已有终端需实测，不凭电机数量额外并联电阻。参考 [GM6020 官方说明书](https://rm-static.djicdn.com/tem/3724/RoboMaster%20GM6020%20Brushless%20DC%20Motor%20User%20Guide.pdf)、[C620 官方说明书](https://cdn-hz.robomaster.com/robomasters/public/document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.0.pdf) 和 [TI CAN 终端说明](https://www.ti.com/tool/TIDA-01238)。处理完成后重新上电、解锁，观察错误是否继续增长；软件恢复并不证明总线已修好。

在仓库根目录执行：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/swerve -d ../build/swerve_rc
west flash -d ../build/swerve_rc
```

日志包含固定零点、等效换算半径、初始化错误、遥控在线/授权、开关状态、两轴反馈新鲜度、驱动零点修正后的舵角（`raw_steer`，尚未施加方向符号/临时零点）、目标 XY 速度、舵向误差、对齐门控、翻转、卸力和电流。`init<0` 时先处理配置或总线启动错误；`rc=0` 检查遥控；`feedback=0/1` 或 `1/0` 检查对应电机的供电、ID 和 CAN 接线；`ready=1` 但 `run=0` 时执行解锁；`drive_on=0` 且有运动目标时观察舵向是否对齐。轮速方向反了可修改 `drive_direction`；数值速度准确性依赖真实轮径及减速比。

默认无故障注入。独立诊断构建仍支持场景 1（输入暂停）、2（执行暂停）：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/swerve -d ../build/swerve_rc_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

本台架每轮休眠单独设为 5 ms，其他底盘样例仍使用公共默认周期 2 ms；原始遥控时间戳、反馈恢复与重新解锁流程沿用原有例程。两路启动日志应为 `clock=24000000 bitrate=1000000 sample_permille=875 prescaler=1 seg1=20 seg2=3 sjw=3`，用它确认已下载本次配置。随后关注两路 TEC 和错误统计是否继续增长；有新故障时结合 `pre_restart` 的时间和计数判断。软件编译不能替代实板方向、控制参数与机械安装调试。
