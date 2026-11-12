# 哨兵云台应用

当前 `sentry_gimbal` 使用 CAN3 外置头部 IMU，运行小 Yaw + Pitch 的惯性姿态闭环。板载 BMI088 仍测量大 Yaw 载体，单独为双 Yaw 分配和底盘 v4 `0x0404` 提供载体航向/角速度；不能将板间数据替换成头部姿态。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
west flash -d ../build/applications/sentry_gimbal
```

VS Code 的 `sentry_gimbal` 使用独立构建目录 `applications/sentry_gimbal/build/dm_mc02/stm32h723xx`；使用哪个目录构建，就从哪个目录烧录。双调试器同时连接时选 `DM MC02 Gimbal DAPLink`。

## 控制链

- 操作者给出枪头世界角速度；视觉给出与外置头部 IMU 同参考的世界角轨迹。
- `DualYawMotionAllocator` 生成受速度、加速度、jerk 和小 Yaw 行程限制的可行 Yaw 世界参考。大 Yaw 意图只来自独立主动速度计划和编码器回中修正，行程保护导致的被动随动不反馈成新的主动意图。
- `InertialGimbalAdapter` 用外置 IMU 的四元数、角速度闭环跟踪 Yaw 参考和 Pitch 世界目标，输出关节速度。编码器仍负责电机内环、机械限位与回中。
- 惯性 Yaw 修正再次经过同一向外制动速度包络，超包络时清除旧关节超前目标。世界参考最多超前实测头部姿态 0.25 rad，避免受限时积累待追赶的转角。
- 本地控制每 5 ms 执行，板间大 Yaw/行走请求每 10 ms 发布。两条电机 CAN 每周期各提交一次。

外置 IMU 的姿态和角速度均需在 20 ms 内，质量为 Tracking 或驱动声明的 Unknown，四元数和角速度有限、参考有效，头部 pitch 在惯性适配器的有效范围内。头部 IMU 启动失败、断流或参考变化时撤销当前惯性输出和大 Yaw 请求；恢复后重新捕获当前姿态，不重放旧手动目标。视觉参考必须匹配头部参考，参考不匹配时不执行旧视觉命令。纯设备内部归零/重启如未通过报文暴露参考变化，仍需停止后重建控制会话。

板载载体 IMU 尚未就绪时大 Yaw 保持禁止；本地可由头部角速度和关节速度估计载体运动来做行程保护。该估计不发送给底盘。板载平面 yaw 仍要求上电静止标定，且长于 20 ms 的不可恢复采样缺口需重启重新标定。

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

- `include/robotics/vehicle/manual_profile.hpp` 的 `sentryInertialConfig()`：头部姿态外环 Yaw/Pitch P、陀螺阻尼和目标超前角。初值 P=3、阻尼=0.1。
- `include/robotics/vehicle/gimbal_profile.hpp`：两轴电机 PID、双 Yaw 分配、速度/加速度/jerk、行程和制动包络。`head_angle_lead_max_rad` 同时约束整车惯性参考超前角。
- `include/robotics/vehicle/calibration.hpp`：方向、原点、限位、IMU 安装转换和参考、操作者速度上限。Pitch 世界姿态目标不再由板载姿态和编码器几何估算。

保留现有遥控/键鼠、导航、视觉和裁判授权仲裁。上电保持未解锁并静止约 3 秒；双轴外置 IMU 数据可用且板载 I22=1 后，从中心附近小幅输入开始。外置安装转换、Pitch 正方向与实际机械行程必须使用现场标定；编译通过不代表实机稳向通过。

## VOFA 诊断：I19=106，49 通道

同一诊断 UART 使用共享 `diagnostic_output` 选择 Log/Vofa；20 ms 输出一帧。I19=104 为旧编码器/载体补偿版本，106 为本版。I0～I18 中 I8/I9 仍是小 Yaw 目标/反馈位置，I12 为控制电流；I10 改为外置头部姿态年龄。I17 为回中误差，I18 为载体世界角速度。

I20～I25 保留载体测量语义，I38～I48 保留大 Yaw 意图、小陀螺目标和分配状态语义。外置诊断为：

| 通道 | 含义 |
| --- | --- |
| I26 | 头部原因：0 正常；1 初始化失败；2 未 Running；3 姿态过期；4 gyro 过期；5 质量不可用；6 参考无效；7 四元数非法；8 gyro 非法；9 pitch 超模型范围；10 参考变化重捕获 |
| I27 / I28 | 外置头部世界 yaw / pitch，rad；不可用时 NaN |
| I29 / I30 | 惯性 Yaw / Pitch 跟踪误差，rad；对应轴输出无效时 NaN |
| I31 | 有效位：1 姿态新鲜，2 gyro 新鲜，4 质量允许，8 参考有效，16 头部可控制，32 惯性 Yaw 输出有效，64 惯性 Pitch 输出有效 |
| I32 | 头部 IMU I/O 错误计数低 16 位 |
| I33 | Pitch 关节位置，rad（电机编码器反馈） |
| I34 / I35 | 头部姿态 / gyro 年龄，ms |
| I36 | Pitch 关节速度，rad/s（电机编码器反馈） |
| I37 | 沿用原许可位图，新增 256=头部 IMU 不可控制 |
| I44 | Pitch 关节速度指令，rad/s |

I48 的行程限制位同时包含惯性姿态修正被制动包络裁剪的情况。底盘仍使用 I19=105，其协议和固件不因本次头部闭环接入而改变。

本次接入需重编译并烧录云台板；不自动烧录，也不声明外置 IMU 安装、链路和双轴带载响应已经实机验证。
