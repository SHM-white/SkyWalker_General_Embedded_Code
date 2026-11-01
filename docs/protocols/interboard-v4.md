# 板间协议 v4：字节布局、物理量与停止契约

日期：2026-10-25。本文描述本仓库现有 v4 codec、Link、Endpoint 和执行器的线协议，可供独立云台、底盘实现接入。未对另一套实现、现场方向和实板链路作兼容性确认；文末字节样例是依照源码布局计算的参考向量，尚未取得双方解码记录。

规范对应的源码入口：

- [消息定义](../../include/robotics/messages/interboard.hpp)、[枚举和时间戳](../../include/robotics/messages/common.hpp)。
- [Codec](../../lib/communication/interboard_codec.cpp)、[Parser 和 CRC](../../lib/communication/interboard_parser.cpp)、[Link](../../lib/communication/interboard_link.cpp)。
- [Endpoint](../../lib/communication/interboard_endpoint.cpp)、[大 Yaw 执行器](../../lib/robotics/big_yaw_executor.cpp)。
- [UART、RS485、CAN 传输封装](../modules/communication/interboard-transports.md)。本页中的“帧”指完整 v4 业务帧；RS485 和 CAN 在它外面还有传输封装。

## 1. 编码和完整帧

多字节整数全部小端；`f32` 是 IEEE 754 binary32，小端位表示。`u8/u16/u32/u64` 分别为无符号 8/16/32/64 位整数。布尔只接受 0、1。所有列出的保留字节发送为 0，接收非零时拒绝。所有浮点字段必须有限，禁止 NaN、正负无穷；功率和能量还必须非负。禁止直接传输 C/C++ 结构体内存布局。

| 整帧偏移 | 长度 | 字段 | 约束 |
| --- | ---: | --- | --- |
| 0 | 2 | magic | `A5 5A` |
| 2 | 1 | version | 固定 4；旧版本拒绝 |
| 3 | 1 | sender_role | 云台=1，底盘=2；0 不在线上使用 |
| 4 | 2 | message_id | 小端，见下一表 |
| 6 | 2 | payload_length=N | 最大128；已知消息必须等于固定长度 |
| 8 | 4 | frame_sequence | 每消息 ID 独立的传输序号 |
| 12 | N | payload | 各表偏移都从此处重新记为0 |
| 12+N | 2 | CRC16 | 覆盖整帧 `[0,12+N)`，结果小端 |

总长 `N+14`，最大142字节。CRC 参数：宽度16，poly=`0x1021`，init=`0xFFFF`，refin=false，refout=false，xorout=`0x0000`，即 CRC-16/IBM-3740（也常称 CCITT-FALSE）。每个输入字节先与 CRC 高8位异或，再做8次向左移位；高位为1时异或 `0x1021`，最终取低16位。CRC 自身不参与计算。

| ID | 名称 | 发送角色 | payload / 完整帧字节数 | Endpoint 最早发送间隔 |
| --- | --- | --- | ---: | ---: |
| `0x0001` | Heartbeat | 任一 | 20 / 34 | 20 ms |
| `0x0101` | ChassisControl | 云台 | 32 / 46 | 10 ms |
| `0x0102` | ChassisConstraint | 云台 | 20 / 34 | 20 ms |
| `0x0103` | ChassisFeedback | 底盘 | 32 / 46 | 20 ms |
| `0x0301` | OperatorControl | 云台 | 36 / 50 | 10 ms |
| `0x0402` | BigYawRequest | 云台 | 36 / 50 | 10 ms |
| `0x0403` | BigYawFeedback | 底盘 | 24 / 38 | 20 ms |

这些间隔是发送机会，背压、传输封装、调度或总线仲裁可以延长实际周期。Endpoint 单批最多240字节；成功交给本地传输后才推进周期。`send()` 成功不等于远端收到、接受或执行。

## 2. 枚举和位掩码

| 字段类型 | 数值 |
| --- | --- |
| BoardRole | 0 Unknown（不在线上使用），1 GimbalController，2 ChassisController |
| ChassisMode | 0 Disabled，1 BodyVelocity，2 Spin |
| ControlSource | 0 None，1 Remote，2 KeyboardMouse，3 Vision，4 Autonomous |
| SafetyAction | 0 Disable，1 Hold，2 Active |
| SafetyState | 0 Boot，1 Waiting，2 Ready，3 Active，4 Degraded，5 EmergencyStop，6 ConfigBlocked |
| ExecutionState | 0 Waiting，1 Recovering，2 Ready，3 Active，4 EStopLatched，5 ConfigBlocked |
| BigYawMode | 0 Disabled，1 FollowCenter |

`active_reasons` 是 `SafetyReason` 位或值，不是枚举序号：

| 位 / 值 | 名称 |
| --- | --- |
| 0 / `0x001` | OperatorDisabled |
| 1 / `0x002` | CommandStale |
| 2 / `0x004` | PowerDisabled |
| 3 / `0x008` | PowerStale |
| 4 / `0x010` | FeedbackStale |
| 5 / `0x020` | TransportUnavailable |
| 6 / `0x040` | EmergencyStop |
| 7 / `0x080` | InvalidConfiguration |
| 8 / `0x100` | RecoveryBoundary |
| 9 / `0x200` | PowerBudgetStale |

0表示没有已报告原因。codec不拒绝 `active_reasons` 的未知位；接入方应保留原值并显示未知位，不将其解释为授权。`ready/armed` 也不能替代独立操作授权。

## 3. 全部 payload 布局

### 3.1 `0x0001 Heartbeat`：20字节

| 偏移 | 类型 | 字段 | 含义 |
| ---: | --- | --- | --- |
| 0 | u8 | role | 必须与整帧 sender_role 一致 |
| 1 | u8 | safety_state | SafetyState |
| 2 | u8 | ready | 本板执行状态就绪 |
| 3 | u8 | sync_requested | 请求同步；Endpoint 在对端离线时置1 |
| 4 | u32 | active_reasons | SafetyReason 位掩码 |
| 8 | u32 | sender_uptime_ms | 发送方本次启动运行时间低32位，ms |
| 12 | u64 | sender_boot_id | 本次启动身份，必须非零 |

boot 身份不能用短周期序号或常量冒充。当前 Endpoint 启动时产生随机非零64位 ID。两板时钟不要求同步，不用 uptime 差值判断命令年龄。

### 3.2 `0x0101 ChassisControl`：32字节

| 偏移 | 类型 | 字段 | 含义 |
| ---: | --- | --- | --- |
| 0 | u8 | command.mode | ChassisMode |
| 1 | u8 | command.source | ControlSource |
| 2 | u8 | global_action | SafetyAction |
| 3 | u8 | reserved | 0 |
| 4 | f32 | command.vx_m_s | 底盘车体X轴线速度目标，m/s |
| 8 | f32 | command.vy_m_s | 底盘车体Y轴线速度目标，m/s |
| 12 | f32 | command.wz_rad_s | 底盘车体Z轴角速度目标，rad/s |
| 16 | u32 | active_reasons | SafetyReason 位掩码 |
| 20 | u32 | command.sequence | 命令生产序号；不是整帧序号 |
| 24 | u64 | receiver_boot_id | 目标底盘的当前启动身份 |

接收时间存入本地 `command.stamp.timestamp_ms`，线上只传生产序号。**本消息没有原始输入年龄字段**。接收方必须共同检查接收超时、生产序号、目标boot、`0x0301` 原始输入年龄/授权；不能描述成和大 Yaw 完全相同的年龄保护。

`BodyVelocity` 使用三个分量。当前 `SwerveChassis::step()` 对 `Spin` 强制忽略 vx、vy，只使用 wz；`Disabled` 进入停用路径。最终底盘应用还要求 `global_action==Active` 才接受运动；Hold 不等于 Active。需要停止时明确发送 Disabled，不能只把三个速度清零。

### 3.3 `0x0102 ChassisConstraint`：20字节

| 偏移 | 类型 | 字段 | 含义 |
| ---: | --- | --- | --- |
| 0 | u8 | output.valid | 裁判底盘输出许可是否有效 |
| 1 | u8 | output.enabled | 裁判底盘输出是否允许 |
| 2 | u8 | power_valid | 功率上限与能量数据有效 |
| 3 | u8 | reserved | 0 |
| 4 | f32 | power_limit_w | 非负功率上限，W |
| 8 | f32 | buffer_energy_j | 非负缓冲能量，J |
| 12 | u32 | output_age_ms | 原输出许可到编码时的年龄，ms |
| 16 | u32 | power_age_ms | 原功率约束到编码时的年龄，ms |

原始数据无效时年龄用 `UINT32_MAX` 表示。当前发送端 power_age 取功率上限时间戳与功率状态时间戳年龄的较大值；不能用新到的其中一个字段刷新另一个旧字段。上限/能量不是实测底盘功率。

### 3.4 `0x0103 ChassisFeedback`：32字节

| 偏移 | 类型 | 字段 | 含义 |
| ---: | --- | --- | --- |
| 0 | u8 | execution_state | ExecutionState |
| 1 | u8 | safety_state | SafetyState |
| 2 | u8 | ready | 就绪 |
| 3 | u8 | armed | 已使能诊断 |
| 4 | u32 | active_reasons | SafetyReason 位掩码 |
| 8 | u32 | last_command_sequence | 最近处理的行走命令生产序号 |
| 12 | u32 | valid_fields | 目前codec只允许低2位，其他位非零拒绝 |
| 16 | f32 | vx_m_s | 车体X轴实际线速度，m/s；须有有效位约定 |
| 20 | f32 | vy_m_s | 车体Y轴实际线速度，m/s；须有有效位约定 |
| 24 | f32 | wz_rad_s | 车体Z轴实际角速度，rad/s；须有有效位约定 |
| 28 | f32 | power_w | 实际功率，W且非负；须有有效位约定 |

当前消息定义只给出 `valid_fields` 数值，并未命名低2位分别对应什么。当前 Endpoint 的 `wireFeedback(RunStatus)` 只发布执行状态，**valid_fields=0，四个测量数值为默认0；不能当作实测零速或零功率**。在双方明确并实现有效位合同前，不使用这些字段闭环。启用实际反馈需补明位含义、测量来源及新鲜度，若改变既有双方合同则显式升级版本。

本消息也没有独立状态生产年龄字段。当前 Endpoint 在发送前以 `status_timeout_ms` 检查执行线程的原始 stamp；停产后发 Waiting、未就绪、未使能和 FeedbackStale，不能持续重发旧 Active。

### 3.5 `0x0301 OperatorControl`：36字节

| 偏移 | 类型 | 字段 | 含义 |
| ---: | --- | --- | --- |
| 0 | u8 | run_allowed | 原始操作输入允许运行 |
| 1 | u8 | emergency_stop | 请求急停 |
| 2 | u16 | reserved | 0 |
| 4 | u32 | source_sequence | 原始操作输入生产序号 |
| 8 | u32 | source_age_ms | 原始操作输入到编码时的年龄，ms |
| 12 | u32 | clear_event_id | 清除急停事件编号；0表示没有 |
| 16 | u32 | clear_event_age_ms | 原始清除事件年龄；没有事件时必须 `0xFFFFFFFF` |
| 20 | u64 | sender_boot_id | 云台当前启动身份，必须非零并匹配心跳 |
| 28 | u64 | receiver_boot_id | 目标底盘启动身份，必须非零 |

`run_allowed` 与 `emergency_stop` 不能同时为1。清除事件非零时，上述两值必须都为0。清除事件不授予运动；接收方按“发送方boot+事件ID”去重，不对同一次重发重复清除。事件必须绑定产生时的接收方boot，不能重试时绑定到刚重启的新底盘。

Endpoint 的 `operator_control_valid` 要求心跳在线、两个boot匹配，以及 `source_age_ms+本地接收后经过时间<=command_timeout_ms`。无效、未授权或急停时，底盘端 Endpoint 将行走和大 Yaw 请求降为 Disabled。

### 3.6 `0x0402 BigYawRequest`：36字节

| 偏移 | 类型 | 字段 | 含义 |
| ---: | --- | --- | --- |
| 0 | u8 | mode | BigYawMode |
| 1 | u8 | permission.valid | 输出许可有效 |
| 2 | u8 | permission.enabled | 输出许可允许 |
| 3 | u8 | reserved | 0 |
| 4 | f32 | target_rate_rad_s | 大 Yaw关节侧角速度目标，rad/s |
| 8 | u32 | source_sequence | 产生该请求的原始操作输入序号 |
| 12 | u32 | source_age_ms | 原始输入到编码时的年龄，ms |
| 16 | u32 | command_age_ms | 控制目标到编码时的年龄，ms |
| 20 | u64 | receiver_boot_id | 目标底盘本次启动身份 |
| 28 | u32 | permission_age_ms | 输出许可到编码时的年龄，ms |
| 32 | u32 | request.sequence | 大 Yaw请求生产序号；解码为 request.stamp.sequence |

原始输入序号、请求生产序号、整帧传输序号是三个不同的量。接收时 request.stamp.timestamp_ms 取本地收到完整帧的时间，不把该时间当作原始控制目标产生时间。

`Disabled+0` 请求失能；`FollowCenter+0` 是使能速度闭环的零速目标，两者不同。编码器只验证模式、布尔、保留字节和有限速度；执行器额外验证速率上限、权限、年龄、boot及运输状态。发送失能时统一清零速度并撤销permission.enabled，不能依赖接收方忽略残留非零值。

### 3.7 `0x0403 BigYawFeedback`：24字节

| 偏移 | 类型 | 字段 | 含义 |
| ---: | --- | --- | --- |
| 0 | u8 | execution_state | ExecutionState |
| 1 | u8 | ready | 就绪诊断 |
| 2 | u8 | armed | 使能诊断 |
| 3 | u8 | valid | 真实速度测量是否有效 |
| 4 | f32 | actual_rate_rad_s | 大 Yaw关节侧实际角速度，rad/s |
| 8 | u32 | active_reasons | SafetyReason 位掩码 |
| 12 | u32 | last_command_sequence | 最近处理的请求生产序号 |
| 16 | u32 | source_sequence | 本次执行反馈生产序号，Endpoint取执行者stamp.sequence |
| 20 | u32 | production_age_ms | 反馈从执行线程产生到编码时的年龄，ms |

这里 source_sequence 是反馈生产序号，与 `0x0402` 同名字段含义不同。接收方必须同时检查valid和production_age；valid=0时数值0只占位置，不表示真实零速。当前执行器只在电机反馈新鲜、包含 FeedbackVelocity 且数值有限时发布有效速度，禁止回填目标速度。

## 4. 物理含义、坐标与换算责任

底盘坐标合同建议固定为 X前、Y左、Z上。正wz绕+Z按右手规则，从上往下看为逆时针。当前运动学在轮位置 `(xi,yi)` 使用：

```text
vxi = vx - wz * yi
vyi = vy + wz * xi
轮缘速度 m/s → 除以轮半径 m → 轮轴 rad/s
转子 rpm = 轮轴 rad/s × 转子到轮轴总减速比 × 60/(2π)
```

`0x0101` 发送车体运动目标，由底盘解算舵角和轮速；线上不传轮电机转子rpm。当前 DJI Motor API 的 velocity_rad_s 已是输出轴速度，使用该API时不能再次乘内部减速比，见[DJI单位说明](../modules/drivers/motor-dji.md)。

大 Yaw是关节相对支撑侧的转动，不能当作头部IMU对地yaw角速度。`0x0402` 目标和 `0x0403` 实际值必须在同一关节侧坐标：

```text
电机接口目标 rad/s = 关节目标 rad/s × motor_to_joint_ratio × direction
关节实际值 rad/s = 电机接口反馈 rad/s × direction ÷ motor_to_joint_ratio
```

ratio定义为“当前Motor API代表的轴 / 大Yaw关节轴”的速度比，必须>0；direction只允许±1。如果API已代表关节轴，ratio=1。例：ratio=6、direction=+1，线上+0.2rad/s对应接口+1.2rad/s≈11.46rpm；反向反馈除以6。

当前中央big_yaw.gear_ratio=1、direction=+1；历史整车云台配置启用了 `CONFIG_VEHICLE_BIG_YAW_REVERSE`，云台在发请求前、消费反馈时各反号一次。因此目前线上正方向仍有既有对端适配，不能单方面宣布与建议坐标一致。重构后配置位置可改变，既有线上符号须保持，现场确认后再同步双方更新合同。

每个组合使用前填写：

| 项目 | 当前仓库基线 | 对接实现/现场记录 |
| --- | --- | --- |
| 车体XYZ及正yaw | X前/Y左/Z上，与现有运动学一致 | 待确认 |
| 大Yaw线上正方向，从何处看 | 保持既有云台反号适配 | 待贴方向标识并确认 |
| 大Yaw驱动反馈代表的轴 | Motor API接口轴 | 待确认是否已换算内部减速 |
| 外部传动比 | 中央配置当前为1 | 待实车确认 |
| 驱动安装符号 | 中央配置当前+1 | 待实车确认 |
| 换算责任 | 底盘BigYawExecutor；云台边界作既有线方向适配 | 填具体文件及版本 |

## 5. 时效、启动身份和停止

### 5.1 默认时限

| 条件 | 当前默认值 | 责任 |
| --- | ---: | --- |
| Endpoint 命令和原始操作输入 | 100 ms | Config.command_timeout_ms |
| Endpoint 对端心跳 | 100 ms | Config.heartbeat_timeout_ms；单独使用Link的默认值是200ms |
| Endpoint 本板状态/大Yaw反馈生产年龄 | 100 ms | Config.status_timeout_ms |
| 本地传输批次发送生命周期 | 60 ms | Config.tx_timeout_ms |
| Parser 半帧组装 | 50 ms | InterBoardParser默认构造参数 |
| BigYawExecutor 请求年龄 | 100 ms | command_timeout_ms |
| BigYawExecutor 原始输入年龄 | 100 ms | source_timeout_us=100000 |
| BigYawExecutor 输出许可年龄 | 300 ms | permission_timeout_ms |
| BigYawExecutor 相邻控制周期上限 | 20 ms | max_cycle_us=20000 |

实际应用可以固定为更保守的阶段策略，应把最终编入值登记在固件记录中。不要拿心跳在线替代命令、原始输入、许可或状态生产新鲜。

`forwardedFresh(stamp, age, now, timeout)` 等价于：stamp有效、now不早于stamp、age不大于timeout，且 `now-stamp.timestamp_ms <= timeout-age`。等于时限仍有效，超过才过期。无效/未来时间戳产生的发送年龄为 `UINT32_MAX`，累计年龄饱和到该值而不回绕。

例：线上command_age=30ms，接收后已过71ms，在100ms时限下失效；传输再重发同一生产序号不能把年龄清零。时间戳用各板本地单调时钟；双方只交换年龄，不交换绝对控制时间。

### 5.2 接收与重启规则

1. 未收到合法对端心跳前拒绝业务消息。拒绝本角色发送的帧、错误角色消息、错误版本/长度/CRC/枚举/保留字段。
2. 每个消息ID独立检查frame_sequence：`delta = next-previous`按u32模运算；仅 `0<delta<0x80000000` 为更新，支持正常回绕，半空间差值有歧义，拒绝。
3. 收到不同sender_boot_id的合法心跳，清空消息序号窗口以及命令、约束、反馈、授权缓存。目标执行前必须再匹配receiver_boot_id。
4. 对端心跳从在线变离线，Endpoint淘汰控制缓存。重新上线/重启时，发送端记录已有行走和大Yaw生产序号基线；运动要有新生产的请求，不能把旧目标自动重新绑定后复用。
5. 当前运行授权发布器遇到对端丢失或boot变化会撤销本地遥控解锁门；重新运行遵守当前应用的RC释放/解锁规则，不把“重新连上”当成自动运动授权。

### 5.3 重发与停止的区别

对于行走控制，只有新的command.sequence才更新接收缓存时间；相同生产序号的新传输帧不刷新命令年龄。行走消息缺少线上原始年龄，原始输入保护由 `0x0301` 补充；需要跨任意转发链保留独立原始命令年龄时须另行设计兼容升级。

对于大Yaw请求，只有新的request.sequence才替换请求和接收时间。**相同request.sequence的新frame_sequence可以把已有请求降为Disabled、速度0、许可撤销，但不刷新时间戳；后续相同生产序号不得恢复运行。** 这项特殊撤销规则不应机械套到所有消息。

当前云台Endpoint每10ms尝试发送大Yaw请求，包括停止请求。对端离线、原始授权停用/急停/过期、请求过期、重连后没有新请求、许可无效或速度无效时，改发Disabled+0，保留原生产序号并累计年龄。大Yaw目标发送不以反馈ready/armed/valid为前置条件；这些字段是观测结果或测量有效性，不能反过来阻断后续新目标。

断链时停止帧可能无法到达，接收方仍须独立依赖超时与授权检查关闭输出。恢复后各电机按自己的驱动/执行器恢复流程接受当前新鲜目标；v4没有resume_generation字段，不模拟已删除的恢复代次机制。

状态停产与通信断开不同：通信线程仍发送心跳时，过期RunStatus映射为Waiting和FeedbackStale；大Yaw反馈撤销valid/ready/armed并累计production_age。主机或对端不能因心跳还在就维持最后一次Active或真实速度有效。

## 6. 分阶段必须实现的消息

| 接入阶段 | 最少消息集合 | 执行约束 |
| --- | --- | --- |
| 无输出链路观察 | 双向Heartbeat；解码欲联调消息 | 不注册电机执行器；未提供状态生产者时不宣告ready |
| 大Yaw独立联调 | 双向Heartbeat、OperatorControl、BigYawRequest、BigYawFeedback | 同时落实授权/年龄/boot/速率上限/本地电机反馈检查 |
| 底盘行走 | 上述心跳与授权，加ChassisControl、ChassisFeedback | 车体单位、Spin语义、原始授权年龄明确；大Yaw保持独立 |
| 裁判/功率约束 | 加ChassisConstraint | 输出许可与功率约束分别判断有效性；真实功率另由本地传感输入提供 |

两套云台×两套底盘四个组合分别登记双方源码版本、协议版本、方向/单位适配、已通过阶段。先验证无输出解码，再验证一个执行机构；不能用其中一个组合通过替代其他组合的确认。

## 7. 固定字节参考与时序场景

下面完整帧依据当前codec的显式小端布局和当前CRC循环计算，不依赖结构体对齐。未运行实板或另一端解码；它们是可供双方录入的参考输入，**尚未登记为双方联调通过的金样**。

四个请求样例公共条件：sender=云台，ID=`0x0402`，frame_sequence=1；source_sequence=42，source_age=5ms，command_age=2ms，receiver_boot=`0x1122334455667788`，permission_age=3ms，request.sequence=7，permission.valid=1。运行请求permission.enabled=1；Disabled样例为0。

```text
FollowCenter +0.25 rad/s:
A5 5A 04 01 02 04 24 00 01 00 00 00
01 01 01 00 00 00 80 3E 2A 00 00 00 05 00 00 00
02 00 00 00 88 77 66 55 44 33 22 11 03 00 00 00
07 00 00 00 BE 0B

FollowCenter -0.25 rad/s:
A5 5A 04 01 02 04 24 00 01 00 00 00
01 01 01 00 00 00 80 BE 2A 00 00 00 05 00 00 00
02 00 00 00 88 77 66 55 44 33 22 11 03 00 00 00
07 00 00 00 21 87

FollowCenter 0 rad/s（使能下零速）:
A5 5A 04 01 02 04 24 00 01 00 00 00
01 01 01 00 00 00 00 00 2A 00 00 00 05 00 00 00
02 00 00 00 88 77 66 55 44 33 22 11 03 00 00 00
07 00 00 00 CB 21

Disabled 0 rad/s（失能）:
A5 5A 04 01 02 04 24 00 01 00 00 00
00 01 00 00 00 00 00 00 2A 00 00 00 05 00 00 00
02 00 00 00 88 77 66 55 44 33 22 11 03 00 00 00
07 00 00 00 AF 94
```

无效大Yaw反馈：sender=底盘，ID=`0x0403`，frame_sequence=1；Waiting，ready/armed/valid均0，速度占位0，active_reasons=FeedbackStale (`0x10`)，last_command_sequence=7，source_sequence=9，production_age=101ms。这个0不能被消费为真实零速。

```text
A5 5A 04 02 03 04 18 00 01 00 00 00
00 00 00 00 00 00 00 00 10 00 00 00 07 00 00 00
09 00 00 00 65 00 00 00 8F DF
```

各参考帧都独立使用frame_sequence=1，不能直接顺序注入同一个已有序号窗口来比较。联调重算后续帧序号和CRC。超时与重启是时序行为，不能仅凭某一帧payload证明：

| 场景 | 预期接收行为 | 双方实测记录 |
| --- | --- | --- |
| 请求年龄2ms，接收后99ms | 总年龄101ms，停止接受运动 | 待填写 |
| 同请求序号7重发运行帧 | 不刷新缓存接收时间；不延长请求生命 | 待填写 |
| 更新frame_sequence发送Disabled，请求仍为7 | 立即降为停止，不刷新请求年龄 | 待填写 |
| 再以请求7发送FollowCenter | 保持已降级状态，须生产更新请求 | 待填写 |
| 心跳boot从A变B | 淘汰A的命令/授权；receiver_boot仍A的运动不得执行 | 待填写 |
| 执行状态停产，通信仍工作 | 心跳继续，旧Active和反馈valid撤销 | 待填写 |

现有字段语义的澄清保留v4；改变任何已协商字段含义、位定义或布局时，由双方显式升级协议并同步部署。

## SkyWalker IMU 航向扩展（2026-10-28）

参考工程 `JJL0612/sentry_onlychassis@7c7e870` 的帧头同为 A5 5A、version=4、role、
LE16 ID/length、LE32 wire sequence，尾部为 CRC16-CCITT（初值 FFFF、多项式 1021）；
已逐项对照其 `alg_board_link.c` 与本工程 codec。共同的 0x0001/0101/0102/0103/0301/0402/0403
保留原字节布局。但这不等于整套行为或扩展兼容：参考只支持 BigYawMode 0/1，
且其 IMU 扩展为 **0x0404、32 字节**，没有发送方 Boot ID。

本工程使用独立的 **0x0405、40 字节**，不接收/重解释参考 0x0404。
两块板必须同步升级。旧固件仍能处理原有消息；开启新模式必须双端具备本扩展。

| Payload 偏移 | 类型 | 语义 |
| --- | --- | --- |
| 0 | u8 | valid，只允许 0/1 |
| 1..3 | 3 bytes | 必须为 0 |
| 4 | float LE | 软件启动零位相对 yaw，rad，[-π,π] |
| 8 | float LE | 实测 yaw 角速度，rad/s，绝对值不超过 100 |
| 12 | u32 LE | 源样本年龄 ms，发送等待需饱和累加，不刷新样本时间 |
| 16 | u32 LE | IMU 样本序号，允许自然回绕 |
| 20 | u32 LE | 软件零位代次，有效样本必须非零 |
| 24 | u64 LE | 生产时绑定的目标底盘 Boot ID，重试不可重新绑定 |
| 32 | u64 LE | 发送方 Boot ID，必须等于当前已接受心跳会话 |

完整帧 54 字节。所有浮点字段（含 invalid 帧）必须有限；有效帧源年龄不得超过 50ms。
Codec 严格检查角色、版本、长度、保留位及取值，失败不修改输出。Link 检查 wire 序号、
源会话、零位代次和 sample sequence。重复样本不能更新时间，invalid 可即时撤销相同样本。
Endpoint 再核对本地 Boot ID、源年龄加本地驻留时间、传输就绪及心跳在线。
发送者生产新绑定样本前，底盘单独重启不会使旧数据获得新身份。

Link 在本地运行期保留最多 8 个已退役的 peer Boot ID，拒绝已知旧心跳恢复会话。
退役表满后拒绝新的会话切换，需两板停机、共同重启；不会丢弃历史以重新接受旧会话。
该机制不是身份认证，也不能识别从未观察过的外部重放会话。

`BigYawMode::ImuHeadingHold=2` 扩展 0x0402 的 mode 值，payload 仍为 36 字节；
0/1 保持 Disabled/FollowCenter 含义。mode=2 的 target_rate 是操作员世界角速度，
只有选择航向锁定后端并确认 MIT/参考系的执行器可接收。旧固件遇到 mode=2 会拒绝。
航向、指令、授权独立计龄，IMU 数据不赋予运动权限。
