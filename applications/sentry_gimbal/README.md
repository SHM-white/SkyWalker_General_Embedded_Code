# 哨兵云台应用

当前双 Yaw 配合恢复到 `4dc29fb` 的编码器控制与偏角回中基线，包括原回中门控、请求发送顺序及底盘请求时效语义。视觉自瞄使用 CAN3 外置头部 IMU 的世界参考；板载 BMI088 仍单独为底盘 v4 `0x0404` 提供大 Yaw 载体航向/角速度。完整回滚范围、问题分析与后续分工见 [双 Yaw 分析](../../docs/dev/双Yaw回滚与控制分工分析.md)。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
west flash -d ../build/applications/sentry_gimbal
```

VS Code 的 `sentry_gimbal` 使用独立构建目录 `applications/sentry_gimbal/build/dm_mc02/stm32h723xx`；使用哪个目录构建，就从哪个目录烧录。双调试器同时连接时选 `DM MC02 Gimbal DAPLink`。

## 控制链

- 手动 Yaw：小 Yaw 关节速率来自 `DualYawMotionAllocator`（已含载体补偿、头部加速度窗与行程度制动包络），持续运动由大 Yaw 承担；分配器不可用时才退回“原始头部世界速率减载体实测速率”的旧公式。
- 大 Yaw 世界意图来自同一分配器的 `big.rate_rad_s`，经板间请求发给底盘；旧 `YawCenteringController` 已作废，不参与本应用。
- 视觉目标直接进入外置 IMU 的 Yaw 姿态外环，再以关节速度进入同一电机环。大角度目标保留，小 Yaw 仍受分配器的行程/制动余量约束。
- 视觉 Pitch 由外置头部四元数与实测 Pitch 关节计算，保留编码器闭环；不再混入载体 IMU 的独立世界零点。
- 本地控制每 5 ms 执行，板间请求和视觉反馈每 10 ms 发布。两条电机 CAN 每周期各提交一次。

外置 IMU 的姿态和角速度需在 20 ms 内，质量为 Tracking 或 Unknown，参考有效，头部 pitch 在惯性适配器范围内。失效时停止视觉控制和姿态回传，手动仍走旧编码器路径。板载载体 IMU 无效时禁止大 Yaw；板载平面 yaw 仍需上电静止标定。传感器内部归零若未在报文中暴露参考代次，需停止后重建视觉参考。

视觉 USB CDC 使用 AB：反馈 43 字节，四元数顺序 `w,x,y,z`，来自 `head.snapshot().sample.orientation`，保留采样时间，安装转换当前为单位四元数；不以板载 IMU 或编码器合成替代。接收 29 字节的 yaw/pitch 是同一世界参考下的绝对角，单位 rad，跨 ±π 按最短方向瞄准；不表达多圈转动。大角度目标需要持续发送，新鲜度超时即撤销。

按本次瞄准联调约定，未知弹速/弹量填 0；内部测量时间戳仍无效，不冒充已测值。协议默认仍拒绝缺少弹道数据的反馈，仅本应用显式启用该约定。

## 键鼠键位

遥控器解锁后键鼠即生效，**不再需要右拨杆上拨选择来源**。摇杆与鼠标按轴合并：两路换算成同一归一化角速度后取幅值更大的一路（相等时听摇杆），所以杆推得更猛时压过鼠标、鼠标甩得更快时压过杆；键盘是开关量，摇杆离开死区时该轴听摇杆。

| 输入 | 作用 |
| --- | --- |
| 左摇杆 / W、S、A、D | 底盘前后左右平移 |
| 右摇杆 / 鼠标 X、Y | 云台 Yaw / Pitch 角速度 |
| Shift（单点） | 锁存开关小陀螺；解锁撤销时自动清零 |
| Q / E（按住） | 小陀螺方向反转 / 正转，等同于把拨轮压到满幅；不按时仍按拨轮回中语义 |
| 右拨杆 Down | 拨杆版小陀螺（与 Shift 锁存并联） |
| 鼠标左键 | 发射：单击单发、快速连点连发、长按持续 |
| 鼠标右键 | 摩擦轮 |

摩擦轮（M3508 ID 1/2）和拨盘（M2006 ID 4）由云台板 CAN1 上的 `ShooterExecutor` 执行，与小 Yaw 共用一次 `commit`；速度、方向、每发分度在 `calibration.hpp`，电流/PID 在 `shooter_profile.hpp`。裁判热量门控尚未接入，当前 `require_heat=false`、`require_permission=false`，只保留执行器自身的反馈就绪、云台 Active 和拨盘位置参考约束。

## 拨杆与巡航模式

巡航（`Cruise`）指云台自动搜索：没有有效视觉目标时低速扫描，出现有效目标时沿用现有视觉跟踪；底盘小陀螺是与之并行的旋转选择。下表所有运动都要求已经解锁：未解锁时拨到巡航位置不会直接启动。

| 左拨杆 | 右拨杆 | 模式与输出 | 遥控掉线后 |
| --- | --- | --- | --- |
| Down | 任意合法位置 | 停止并撤销解锁 | 保持停止 |
| Up | Middle / Up | 现有手动及合并键鼠控制 | 撤销解锁 |
| Up | Down | 现有手动小陀螺控制 | 撤销解锁 |
| Middle | Middle | 普通导航 + 视觉；无目标时 Hold | 保持解锁与该模式 |
| Middle | Up | 云台扫描/视觉跟踪；小陀螺关闭 | 保持解锁与该模式 |
| Middle | Down | 云台扫描/视觉跟踪；小陀螺开启 | 保持解锁与该模式 |

- 解锁：左 Down 建立归中基线，右 Middle，摇杆与拨轮满足 `centered()`（原始值 ±33）后再左 Down→Middle。上电首帧还提供一次性快捷解锁：只要正式配置下的映射判定为“解锁后不会主动产生动作”即可（左上右上、左上右中均满足），不再固定要求右 Middle；右 Down/Shift/Q/E/有效拨轮、非零摇杆或鼠标机构请求、巡航档本身都会拒绝快捷解锁。快捷解锁资格一旦消耗不会因输入回零而重建。
- 掉线保持：只有“最后一帧已接受的有效输入处于左 Middle 且已经解锁”才保持自主运行。重连后新帧立即更新选择（左 Down 先停止），掉线期间的旧摇杆/鼠标/键盘值不参与自主运动。上电无遥控、从未解锁、手动档掉线、板间掉线或对端重启都不会被转换成自动授权。
- 巡航分工：两档的 `wz` 由本地小陀螺逻辑独占，不与导航 `wz` 相加。右下切右上是“停止请求旋转并按现有减速度回零”，不是物理转速瞬间为零；两档互切不重启视觉会话、不重新捕获扫描 Pitch、不重置减速状态。
- 扫描：使用头部世界 Yaw 角速度，经现有双 Yaw 分配器分配到大小 Yaw；Pitch 保持进入扫描时的实际关节角（不是回水平，也不是延续上一次视觉目标）。扫描速度集中在 `manual_profile.hpp` 的 `cruise_yaw_rate_rad_s`（0.3 rad/s ≈ 17.2°/s，方向与大小待实机确认）。
- 导航：巡航在无有效导航帧时平移为零（仍发合法零速 `BodyVelocity`），任一轴过期立即归零；普通导航（左中右中）的 `wz` 仍来自导航帧。
- 视觉失效：`VisionMissing/VisionStale/VisionStopped/WaitNewVision` 转入扫描；非法目标、世界参考不匹配、时钟倒退仍保留原拒绝/Hold/Disabled 语义，不用扫描掩盖故障。头部 IMU 不可用或裁判明确拒绝时仍停止输出。

## 配置与资源

| 资源 | 用途 |
| --- | --- |
| CAN1 | GM6020 小 Yaw + M3508×2 摩擦轮 + M2006 拨盘（0x1FF 与 0x200 控制帧、0x207 与 0x201/0x202/0x204 反馈互不冲突） |
| CAN2 | J4310 MIT Pitch |
| CAN3 / imu-can | 专供外置头部 CAN IMU，驱动独占控制器 |
| 板载 BMI088 | 大 Yaw 载体姿态与独立 gyro-z 航向 |
| UART5 | DR16 遥控 |
| USART10 | 底盘 v4 板间链路 |
| USART1 | 导航链路 |
| USB CDC | 视觉 AB 链路 |
| UART7 / diagnostic-uart | DAPLink 诊断，115200 8N1 |

外置 IMU 沿用 `headSensorConfig()`：主动上报四元数、feedback_id=0x11、slave_id=1，安装转换由 `head_sensor_to_body` 提供，参考为 `head_reference`。接收器每 1 ms 服务一次。已有 CAN 惯性样例的传感器设置继续适用，程序不会替用户改写 IMU 模块参数。

调参入口：

- `manual_profile.hpp`：`manualCenteringConfig()` 为旧回中增益、死区、速度和加速度；`sentryInertialConfig()` 的 Yaw P=3、阻尼=0.1 仅用于视觉姿态外环，世界目标超前角限制为 0（禁用）。
- `gimbal_profile.hpp`：小 Yaw/Pitch 电机 PID、速度参考斜率、电流/力矩上限。
- `calibration.hpp`：机械方向、原点、限位、IMU 安装参考；`vision_pitch_horizontal_rad` 为待实测水平零位。手动 Yaw 速度恢复 3 rad/s。
- `big_yaw_profile.hpp`：底盘侧旧航向 P/速度 PI；原生速度斜率仍为 20 rad/s²，区别于上游回中请求的 0.8 rad/s²。

保留遥控/键鼠、导航、视觉和裁判授权仲裁、变速小陀螺及发射机构。此次需要**同时重新构建并烧录云台和底盘**；编译不能证明实机跟踪性能。

## VOFA 诊断：I19=112，49 通道

同一诊断 UART 使用共享 `diagnostic_output` 选择 Log/Vofa；20 ms 输出一帧。I19=112 为接通巡航扫描与自主保持的版本。I0～I18 中 I8/I9 仍是小 Yaw 目标/反馈位置，I12 为控制电流；I10 改为外置头部姿态年龄。I17 为分配器中心误差，I18 为载体世界角速度。

I20～I25 保留载体测量语义。I38 为回中速度请求，I43 为到软件端点的最小距离（不再扣制动余量），I47 为小 Yaw 关节速度命令。外置诊断为：

| 通道 | 含义 |
| --- | --- |
| I26 | 头部原因：0 正常；1 初始化失败；2 未 Running；3 姿态过期；4 gyro 过期；5 质量不可用；6 参考无效；7 四元数非法；8 gyro 非法；9 pitch 超模型范围；10 参考变化重捕获 |
| I27 / I28 | 外置头部世界 yaw / pitch，rad；不可用时 NaN |
| I29 / I30 | 视觉 Yaw 外环误差 / 适配器 Pitch 观测误差，rad；后者不用于实际 Pitch 执行；手动时 NaN |
| I31 | 有效位：1 姿态新鲜，2 gyro 新鲜，4 质量允许，8 参考有效，16 头部可控制，32 惯性 Yaw 输出有效，64 惯性 Pitch 输出有效 |
| I32 | 头部 IMU I/O 错误计数低 16 位 |
| I33 | Pitch 关节位置，rad（电机编码器反馈） |
| I34 / I35 | 头部姿态 / gyro 年龄，ms |
| I36 | Pitch 关节速度，rad/s（电机编码器反馈） |
| I37 | 控制上下文位图：bit0 setup 未就绪，bit1 遥控不可用且不允许自主保持，bit2 未授权运行，bit3 对端离线，bit4 安装未确认，bit5 载体航向不新鲜，bit6 关节反馈无效，bit7 Yaw 输出无效，bit8 头部 IMU 不可控制，bit9 已解锁，bit10 遥控帧新鲜，bit11 遥控掉线自主保持，bit12..13 自主子模式（None/Navigation/Cruise/CruiseSpin），bit14..15 云台行为（None/Hold/Scan/Track），bit16 启动动作判定已知，bit17 判定为无动作，bit18 决策与本地行为匹配，bit19 扫描入口仍等待准备 |
| I46 | 头部世界速率原始请求（扫描时即 `cruise_yaw_rate_rad_s`），rad/s |
| I42 | 小陀螺 profile 实际输出，rad/s |
| I44 | Pitch 关节速度指令，rad/s |

I48：bit0=回中控制有效，bit4=大 Yaw 通道可用，bit9=视觉执行；bits5..8 为回中 WaitReason。底盘 I19=109，I46 为旧航向环输出的世界速度，I47、I48 均预留为 0。

本次不自动烧录；外置安装转换、方向、实际机械行程和带载响应仍需现场确认。
