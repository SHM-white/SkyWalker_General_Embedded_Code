# gimbal_dual_yaw

本工程固定配未修改的参考底盘 `JJL0612/sentry_onlychassis@dcc8d4e`，不配本仓库的 `chassis_big_yaw`。本地小 Yaw/Pitch 使用电机编码器闭环，不启动或等待 CAN3 头部 DM IMU；大 Yaw 回中请求由小 Yaw 编码器相对机械中心的偏角生成。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_dual_yaw -d ../build/bringup/gimbal_dual_yaw
west flash -d ../build/bringup/gimbal_dual_yaw
```

这次适配只需重编译、烧录云台板，参考底盘保持现有 v4 固件。云台 USART10 与底盘 USART1 以 460800、8N1 互连；UART5 接 DR16，CAN1 接小 Yaw，CAN2 接 Pitch，CAN3 本入口暂不启动外置 IMU 接收。板载 BMI088 必须随大 Yaw 转动，安装变换沿用 `carrier_sensor_to_body`。参考固件版本、传感器安装方向及机械中心仍需现场确认。

## 控制关系

小 Yaw 编码器偏角为 `position_rad - vehicle::yaw_center_rad`，乘 `small_yaw.direction` 转为物理正方向；回中增益为 6.0，速度上限为 `min(bringup::big_yaw_rate_rad_s, 1.0)` rad/s，加速度上限为 4.0 rad/s²。偏角绝对值小于等于 0.003 rad（约 0.17°）时开始减速至零，大于等于 0.005 rad（约 0.29°）时重新跟随，防止中心附近频繁启停。中心沿用共享标定，不把参考云台的 0.765 rad 直接复制到当前不同的机械限位坐标。

这组参数将原 2.0 的回中增益提高到 6.0、加速度从 0.8 提高到 4.0，中心附近的停止/启动范围从 0.01/0.015 rad 缩小到 0.003/0.005 rad。忽略限速和加速阶段时，2° 偏角请求约 0.209 rad/s（12°/s），原来约 4°/s。参考底盘会将请求截断至 1 rad/s，抬高发送上限不能突破该限制。若诊断通道 7 enabled=0 或通道 8 target 始终为零，先结合 reason/detail 确认授权、IMU、执行状态和反馈条件；提高增益不能放行被撤销的请求。

不再使用 2.5 秒世界航向滑动平均。`0x0402` 的 `mode=1` 仍为参考底盘的世界系角速度请求，不额外乘大 Yaw 电机的 -1 安装符号。参考底盘持续执行自身的航向保持、角速度 PI 和底盘陀螺补偿，不以开启小陀螺为前提。

板载 BMI088 启动时采集 800 个静止样本估计陀螺偏置，之后固定这份偏置，避免把低速大 Yaw 转动误学成零偏。通过 `correct_gyro_bias=true`，姿态积分和发出的角速度使用同一份偏置；去偏角速度与姿态在同一次快照发布，初始化或计算失败时不回退发送原始角速度。该开关在参考底盘的两个 bringup 入口启用，整车 `applications/sentry_gimbal` 也采用相同配置，其他 BMI088 调用方保留原有默认配置。

每周期发送 v4 `0x0402` 大 Yaw 请求、`0x0404` 板载航向及去偏角速度、零速 `0x0101` 底盘控制。零速底盘控制只放行参考底盘的大 Yaw 执行，不请求轮子转动。对端还未上报实际速度时，只允许零速使能以获取反馈；实际反馈的接收期限为 250 ms，适应参考底盘的 100 ms 上报周期。板载航向、小 Yaw 编码器或小 Yaw 输出失效、授权撤销时，撤销大 Yaw 请求；外置 IMU 无回传与 Pitch 惯性输出不再作为回中门槛。

本地手动命令直接进入编码器闭环。有有效板载航向时，小 Yaw 速度减去载体世界角速度，再乘关节方向，实现大 Yaw 转动时的反向补偿；否则大、小 Yaw 一起转会保留相对编码器偏角，回中不能收敛。板载 BMI088 测量的是大 Yaw 载体，不能直接作为小 Yaw 头部实测姿态。板载航向无效时仍可做本地编码器手动控制，大 Yaw 保持 Disabled。Pitch 当前为编码器相对角控制，不进行外置 IMU 世界俯仰稳向。

视觉收发暂不加入此示例；整车未来的估计姿态与外置 IMU 接管记录在[后续方案](../../../docs/dev/云台视觉姿态回传与外置IMU接管方案.md)。

## 操作与诊断

上电保持机构静止至少 2 秒，左拨 Down、右拨 Middle，摇杆和拨轮全部归中至少 500 ms；左拨 Down→Middle 建立解锁，再拨 Up 运行。左 Down 停止，遥控失联、对端掉线或重启后重新解锁。先做零输入观察，再从机械中心两侧小幅验证回中方向；不要靠持续转到限位判断方向。

诊断沿用共享输出选择和固定 19 通道 JustFloat 布局，无需修改 `vehicle_diagnostics.cpp`：

| 通道（从 0 开始） | 本工程含义 |
| --- | --- |
| 5 / 6 / 7 | reason / 测量有效 / 回中请求有效 |
| 8 target | 发给参考底盘的世界系角速度请求 rad/s |
| 9 feedback | 将参考底盘原始电机速度按安装方向和传动比转换后的大 Yaw 关节速度 rad/s；底盘静止时可与 target 比较 |
| 10 age_ms | 对端反馈携带的生产年龄；有效性还计算本地接收后的年龄 |
| 12 effort | 本地小 Yaw 电流 A |
| 13 detail | 编码器回中控制器的 WaitReason |
| 14 error | 初始化或执行错误 |
| 17 extra0 | 带物理方向的小 Yaw 编码器中心偏角 rad |
| 18 extra1 | 实际发给参考底盘的板载去偏航向角速度 rad/s；无有效航向时 NaN |

`reason=6` 当前仅指板载航向无效，不再指 CAN3 无回传。reason：0 Boot、1 Ready、2 InitFailed、3 RemoteStale、4 Disarmed、5 FeedbackStale、6 ImuStale、7 PeerStale、8 Configuration、9 Permission、10 CommandStale。`valid=0` 时 feedback 为 NaN；中心偏角与目标速度应随左右操作改变符号。若 target 已归零而实际速度仍持续非零，记录 extra1，并检查参考底盘自身角速度和闭环积分。零输入时参考底盘仍是航向闭环，零请求不等于关闭电机。

仅软件适配，未烧录或实板验证。上电静止标定不能消除所有温漂；Pitch 需机械支撑。若 EKF 或 IMU 参考在运行中重置，现有航向引用保护会拒绝继续使用旧零点，应停机并按现场流程重新建立启动状态。
