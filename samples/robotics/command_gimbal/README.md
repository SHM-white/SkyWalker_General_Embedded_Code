# 仲裁接入小 Yaw / Pitch 实板样例

阶段 3 的独立入口：MC02 上真实 `RemoteReceiver → RemoteSource → CommandManager` 服务生成机械角速度命令，5 ms 执行线程消费快照，同时控制 GM6020 小 Yaw 与 DM4310 Pitch。`CommandManager` 按 10 ms 仲裁，遥控接收、执行、遥测分别运行。首版明确使用无裁判的空载台架权限策略，并关闭自动/视觉模式。

`connections_configured=false` 时仅观察命令和阻塞状态，不 attach/start 电机 CAN。该入口尚待实板验证；编译通过不代表方向、Pitch 限位或整车功能通过。

## 接线和标定

| 资源 | 默认分配 | 必须核对 |
| --- | --- | --- |
| 小 Yaw | GM6020 电流模式，CAN1，ID 7 | 电流模式、ID、供电、编码器零点、输出轴单位与方向 |
| Pitch | DM J4310 MIT，CAN2，ID 1，反馈 Master ID 0x11 | 已保存零点、PMAX/VMAX/TMAX、方向、实际行程及机械支撑 |
| DR16 | MC02 `remote-uart` / UART5 | 接线与接收端口独占 |
| VOFA | MC02 `telemetry-uart` / USART1，115200 | JustFloat 接收、文本控制输入，独立于控制台 |
| 控制台 | MC02 USART10 | 串口日志 |

配置集中在 [src/board_config.hpp](src/board_config.hpp)。沿用已有双轴台架校准：Yaw 驱动限幅 1.5 A、控制器 ±1.2 A，Pitch 驱动限幅 1.0 N·m、控制器 ±0.5 N·m。GM6020 编码器零点 5670 tick、机械端点 5670～7440 tick；两端各留 100 tick，安全行程 5770～7340 tick，即驱动坐标约 0.0767～1.281 rad。`yaw_center_rad` 是独立参数，初值为该安全行程中点约 0.679 rad，不等于编码器零角；后续大 Yaw 回中应使用同一标定产物。

Pitch 的 `-0.5～0.5 rad` 仍是占位值，必须替换为实物测量值。DM 量程默认 PMAX=12.5 rad、VMAX=30 rad/s、TMAX=10 N·m，必须匹配驱动器配置；轴参考使用电机已保存零点。Yaw 的机械参考使用校准单圈编码器坐标。禁用反馈稳定后，`GimbalAxis::poll()` 才准备参考，不将每次启动位置当成机械零点。

`ManualCommandMapper` 已将右摇杆横向映射为负 Yaw、纵向映射为正 Pitch，与原双轴样例的方向一致。`yaw_command_sign`、`pitch_command_sign` 默认均为 +1，表示仲裁后机械命令到当前驱动坐标的额外转换；若实测方向不同，应连同零点、限位与标定坐标一起确认。控制最大角速度先降为 Yaw 1 rad/s、Pitch 0.8 rad/s。

核对上述参数并支撑 Pitch 后才将 `connections_configured` 改为 `true`。将 `pitch_can` 改为 CAN1 可验证同总线拓扑；必须确认协议 ID 不冲突。

## 执行和恢复契约

应用静态创建两个 Motor、两轴 Group 和物理 CanBus，先 attach 两轴，再 start 全部总线，最后调用 `GimbalExecutor::begin()`。初次 start 的暂态传输错误按 100 ms 重试，配置/所有权错误保持阻塞。两轴为一个故障组，可以跨 CAN；该组不得额外加入其他机构。`GimbalExecutor` 持有 Motor/Group 引用和两个 `GimbalAxis`，负责参考、反馈、控制历史、共同启停和暂存本周期电流/扭矩。它不创建总线，也不调用 `commit()`。

应用的唯一执行线程在所有机构暂存输出后，每条已启动的物理 CAN 调用一次 `commit()`；同 CAN 配置只提交一次。任何提交错误立即调用 `suspend()` 撤销两轴输出许可。扩展摩擦轮/拨盘时，继续从此线程更新其他故障组，再统一提交，不让遥测或输入线程写电机目标。

左拨杆下位请求禁用，中位进入手动控制，上位因 `allow_auto=false` 保持禁用。右拨杆上位切换键鼠，其余有效档位使用摇杆；首次台架验证建议使用摇杆。该样例采用自动恢复，无需故障后重复安全拨杆循环：

1. 失效时撤销输出和使能握手，旧目标失去执行授权。
2. 等待总线、两轴反馈、参考、权限和 1～20 ms 控制周期恢复。
3. 在禁用状态 reset 两轴历史与目标，建立新的独立恢复代次。
4. 同时要求仲裁命令和原始输入的生产时间严格晚于准备边界，接受新命令后申请联合使能。
5. 等待联合握手；此期间仍检查新鲜性、权限、反馈、参考和控制周期，撤销条件出现即停止。

最终命令的仲裁时间不是原始输入时间。恢复门限同时检查 `CommandSnapshot.observed.remote.stamp`，因此 `CommandManager` 对恢复前缓存的遥控帧重新生成序号，不能授权使能。独立恢复代次来自 `RecoveryGate`，不使用 Motor 的 `enable_generation`。

反馈过期、命令过期、意外失能、使能超时、传输错误和接收队列溢出属于可恢复故障，条件恢复后按 100 ms 间隔尝试清除。驱动硬故障、非法目标、控制拒绝、超出校准行程和急停锁存要求显式 `clear=1`。释放急停后才可清除；仍必须重新准备参考并接收边界之后的输入。物理急停/复位可通过配置末尾的两个钩子接入。Pitch 失能可能下坠，所有断流和停止场景应保持机械支撑。

## 构建与操作

```sh
west build -b dm_mc02 samples/robotics/command_gimbal -d ../build/command_gimbal
west flash -d ../build/command_gimbal
```

先保持配置禁用上电，检查遥控源序号、源年龄、最终命令、运行状态和 VOFA。完成单机构方向与行程确认后，再使用支撑空载双轴，中位使能、轻推右摇杆验证运动和行程限幅；下位观察联合禁用。

VOFA 同一串口可发送换行结束的 ASCII `key=value`。回调只修改原子标志，执行线程应用启停动作。除急停锁存外，0 恢复对应生产者；清除请求使用 1。

| 操作 | 要验证的现象 |
| --- | --- |
| `input_pause=1` | 接收线程、仲裁和遥测继续；原始源序号冻结且年龄增加，最终命令随后禁用，两轴撤销输出 |
| `input_pause=0` | 得到真正新遥控帧、准备边界之后自动恢复；缓存旧帧不得使能 |
| `execution_pause=1` | 执行更新、提交和状态生产停止；命令与遥测继续。电机 I/O 按命令到期撤销整组，状态年龄超过 100 ms 后对外 ready=false |
| `execution_pause=0` | 超限周期被识别，旧上下文失效，参考与新输入重新准备后恢复 |
| `status_pause=1` | 执行和命令继续，仅冻结观测缓存；状态年龄增加，年龄检查禁止消费者认为旧 Active/Ready 有效 |
| `status_pause=0` | 重新发布真实生产时间，不刷新旧缓存时间来掩盖失效 |
| `estop=1` | 两轴停止，状态 Blocked；`estop=0` 只释放输入，保持锁存 |
| `clear=1` | 急停已释放且故障可清时请求复位；新恢复代次和新输入之后再使能 |
| 单轴断流或断电 | 两轴一起撤销输出；重新获得反馈与参考后按故障类型自动恢复或等待明确清除 |
| 一条 CAN 断开/恢复 | 云台组整体停止；总线恢复、新参考和新输入满足后再使能 |

VOFA 50 ms 一帧，16 通道依次为：原始源序号、源年龄 ms、最终命令序号、命令年龄 ms、RunState、WaitReason、恢复代次、状态生产年龄 ms、最后实际执行的命令序号、Yaw 实际角、Pitch 实际角、Yaw 目标角、Pitch 目标角、Yaw CAN 最近错误、Pitch CAN 最近错误、控制周期超限计数。无有效时间戳的年龄为 -1；同 CAN 时两个 CAN 错误通道相同。序号在 VOFA float 中仅适合短时间观察，完整 uint32 数值保留于控制台日志。

`RunState` 编号为 Disabled=0、Recovering=1、Active=2、Blocked=3；`WaitReason` 为 None=0、Command=1、Transport=2、Feedback=3、Reference=4、Configuration=5、Drive=6、Power=7、Cycle=8。控制台每秒记录原始源/命令序号、状态、等待原因、恢复代次、状态新鲜性、按年龄裁剪后的 ready、错误、执行耗时和周期超限数。原始缓存 Active 可继续显示，但 `wireFeedback(status, now, timeout)` 在状态过期时强制 ready=false、armed=false。

## 实板记录

| 项目 | 记录 |
| --- | --- |
| 固件版本、板号、CAN/UART 接线 | |
| 两轴型号、ID、模式、量程、方向、零点、限位 | |
| 小 Yaw 独立中心标定 | |
| 正常运动与限幅现象 | |
| 输入暂停到撤输出时间、状态年龄 | |
| 执行暂停到驱动停输出时间 | |
| 重启/断电恢复前后源序号和恢复代次 | |
| 急停释放与显式清除现象 | |
| 单轴/单总线故障的云台组影响范围 | |
| 周期超限数、执行耗时、两总线负载和线程栈余量 | |
| 编译通过 / 实板通过 / 待验证及操作者 | |

本入口只控制机械小云台。头部惯性参考、外置 IMU、视觉指向、大 Yaw 回中、共享摩擦轮/拨盘及实际发射仍属于后续独立阶段。

## 分阶段消息源配置

机械电机标定、连接确认、ID、方向、零点、独立中心、行程与限幅统一读取 `include/robotics/vehicle/calibration.hpp`，确认开关默认关闭。台架控制参数仍需实际调参，TODO 留在配置和参考会话接口处。

| 配置 | 接入链路 | 执行行为 |
|---|---|---|
| 默认 | 遥控 → CommandManager | 机械双轴控制 |
| `referee.conf` | 遥控 + RefereePermissionSource | 云台权限必须有效且新鲜 |
| `vision_observe.conf` | 遥控 + 裁判 + VisionSource | 接收、观测视觉；自动执行保持关闭 |
| `vision_execute.conf` | 遥控 + 裁判 + 视觉 + 头部 RS485 IMU | 所有云台目标通过 InertialGimbalAdapter，再进入机械执行器 |

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal_referee -- -DEXTRA_CONF_FILE=referee.conf
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal_vision_observe -- -DEXTRA_CONF_FILE=vision_observe.conf
west build -b dm_mc02/stm32h723xx samples/robotics/command_gimbal -d build/command_gimbal_vision_execute -- -DEXTRA_CONF_FILE=vision_execute.conf
```

MC02 的 USART1 接裁判、UART7 接视觉、RS485-2 接头部 IMU。裁判配置占用 USART1 时，VOFA 不初始化该串口，控制台继续输出源年龄、来源、仲裁原因、权限、参考及执行恢复代次；原有暂停/急停/清除变量可通过调试器操作。新增 `vision_pause`、`permission_pause`、`head_pause` 与原有 `input_pause`、`execution_pause`、`status_pause` 的含义分别为冻结该生产链路，原始时间不会刷新。USART2 RX 使用 DMA 通道 8，保留板载 SPI2 的通道 1/2，并避开遥控 UART5 的通道 6 和视觉 UART7 的通道 5/7。

TODO(实板验收)：先完成机械云台及头部 IMU 安装标定，再验收 `inertial_gimbal`；确认人工接管、目标过期、权限断流、参考变化和恢复后新输入边界。AB 协议没有参考代次字段，目前以明确的本地 `head_reference` 约定解码，IMU 参考变化后的旧视觉会话被撤销；需要与视觉端建立新会话后才能恢复。云台台架没有真实弹速/弹数测量，AB 反馈要求完整实测字段，因此反馈 TX 默认关闭，头部真实反馈已接入缓存，TODO 留待发射测量链路补齐。
