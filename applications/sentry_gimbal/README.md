# 哨兵云台应用

当前双 Yaw 配合恢复到 `4dc29fb` 的编码器控制与偏角回中基线，包括原回中门控、请求发送顺序及底盘请求时效语义。视觉自瞄使用 CAN3 外置头部 IMU 的世界参考；板载 BMI088 仍单独为底盘 v4 `0x0404` 提供大 Yaw 载体航向/角速度。完整回滚范围、问题分析与后续分工见 [双 Yaw 分析](../../docs/dev/双Yaw回滚与控制分工分析.md)。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
west flash -d ../build/applications/sentry_gimbal
```

VS Code 的 `sentry_gimbal` 使用独立构建目录 `applications/sentry_gimbal/build/dm_mc02/stm32h723xx`；使用哪个目录构建，就从哪个目录烧录。双调试器同时连接时选 `DM MC02 Gimbal DAPLink`。

## 控制链

- 手动恢复编码器位置/速度环：小 Yaw 速度等于操作者世界速度减载体实测速度；Pitch 使用手动关节速度。
- 大 Yaw 使用 `YawCenteringController`，按小 Yaw 相对中心的偏角回中。恢复原来的 0.6 rad/s 请求上限、0.8 rad/s² 请求斜率与 0.5 回中增益。
- 视觉目标直接进入外置 IMU 的 Yaw 姿态外环，再以关节速度进入同一电机环。取消分配器、关节轨迹覆盖和 0.25 rad 世界目标截断；大角度目标保留，小 Yaw 仍受机械限位保护，大 Yaw 持续跟转。
- 视觉 Pitch 由外置头部四元数与实测 Pitch 关节计算，保留编码器闭环；不再混入载体 IMU 的独立世界零点。
- 本地控制每 5 ms 执行，板间请求和视觉反馈每 10 ms 发布。两条电机 CAN 每周期各提交一次。

外置 IMU 的姿态和角速度需在 20 ms 内，质量为 Tracking 或 Unknown，参考有效，头部 pitch 在惯性适配器范围内。失效时停止视觉控制和姿态回传，手动仍走旧编码器路径。板载载体 IMU 无效时禁止大 Yaw；板载平面 yaw 仍需上电静止标定。传感器内部归零若未在报文中暴露参考代次，需停止后重建视觉参考。

视觉 USB CDC 使用 AB：反馈 43 字节，四元数顺序 `w,x,y,z`，来自 `head.snapshot().sample.orientation`，保留采样时间，安装转换当前为单位四元数；不以板载 IMU 或编码器合成替代。接收 29 字节的 yaw/pitch 是同一世界参考下的绝对角，单位 rad，跨 ±π 按最短方向瞄准；不表达多圈转动。大角度目标需要持续发送，新鲜度超时即撤销。

按本次瞄准联调约定，未知弹速/弹量填 0；内部测量时间戳仍无效，不冒充已测值。协议默认仍拒绝缺少弹道数据的反馈，仅本应用显式启用该约定。

## 键鼠键位

遥控器解锁（左 Up 且右拨杆在中位）后键鼠即生效，**不再需要右拨杆上拨选择来源**。摇杆与鼠标按轴合并：两路换算成同一归一化角速度后取幅值更大的一路（相等时听摇杆），所以杆推得更猛时压过鼠标、鼠标甩得更快时压过杆；键盘是开关量，摇杆离开死区时该轴听摇杆。

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

## VOFA 诊断：I19=110，49 通道

同一诊断 UART 使用共享 `diagnostic_output` 选择 Log/Vofa；20 ms 输出一帧。I19=110 为此次回滚并接通视觉的版本。I0～I18 中 I8/I9 仍是小 Yaw 目标/反馈位置，I12 为控制电流；I10 改为外置头部姿态年龄。I17 为回中误差，I18 为载体世界角速度。

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
| I37 | 沿用原许可位图，新增 256=头部 IMU 不可控制 |
| I44 | Pitch 关节速度指令，rad/s |

I48：bit0=回中控制有效，bit4=大 Yaw 通道可用，bit9=视觉执行；bits5..8 为回中 WaitReason。底盘 I19=109，I46 为旧航向环输出的世界速度，I47、I48 均预留为 0。

本次不自动烧录；外置安装转换、方向、实际机械行程和带载响应仍需现场确认。
