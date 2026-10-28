# 哨兵 IMU 航向控制与坐标补偿

状态：软件实现并验证，实车待验收。大 Yaw 航向锁定、坐标转换、旋转补偿默认关闭，
本分支独立于 P0 舵轮分支，可分别审查、合并；完整上车能力仍需同时采用 P0。
基线 dev `8d67281`，参考 `JJL0612/sentry_onlychassis@7c7e870`。

## 控制责任与开关

CommandManager 继续选择命令来源并保留生产时间；云台应用转发底盘命令、滚轮请求和头部 IMU，
Endpoint 负责会话、年龄和批量 UART。底盘 ChassisMotionAdapter 在 SwerveKinematics 前完成
云台/世界到车体转换及可选角速度补偿；SwerveKinematics 仅接受车体速度。
BigYawExecutor 独立执行大 Yaw 的授权、时效、反馈及 MIT 输出管理，不借用轮组控制权限。

`motion_profile.hpp::chassisMotionConfig()`：

- `frame=Body` 为默认，保持当前车体坐标；World 为启动世界坐标，Gimbal 为头部 IMU 当前朝向。
- `compensation_enabled=false` 默认关闭 7ms 预测、旋转降速和角速度反馈。
- `heading_prediction_s=.007` 仅旋转时用于底盘 yaw 预测。
- `spin_forward_scale=.3`、`spin_lateral_scale=.4` 沿用参考“原输入有横移选 .4，否则有前进选 .3”的统一缩放，保持方向。
- `yaw_rate_feedback_kp=0` 未标定，不复制参考实车增益；无有效底盘 IMU 时省略反馈修正。
- `startup_heading_alignment_confirmed=false` 默认不宣称双 IMU 参考已校准；World/Gimbal 平移在确认前为零。

世界、云台和底盘均采用 +x 前、+y 左、+z 向上、逆时针正；world 表示共同启动前向而非地理北。
转换为 `v_body = R(head_yaw - predicted_body_yaw) * v_gimbal`；世界输入使用 head_yaw=0。
源样本年龄用于短期预测，原始命令的 stamp/source 不变，四轮运动学和限速仍由既有模块负责。
普通 BodyVelocity 支持旋转平移；Spin 沿用既有“仅旋转、忽略平移”的语义。

## 启动、失效和恢复

两板静止、云台/车头朝向共同前向，在 disarmed 状态运行至少 1 秒、连续 20 个不同有效静止样本
后，各自记录软件零位。复用现有安装变换与 core::euler；yaw rate 由已变换到机构坐标的陀螺率
按 Euler 运动关系换算，接近俯仰奇异位置时拒绝航向数据。没有向 IMU 写校准/保存/零位命令。

Gimbal 平移只有在未使能时取得两颗 IMU 的有效零位并绑定 Boot ID/代次后才允许。
短暂丢流不改变零位：有效数据恢复后沿原基准继续；云台单板重启、零位代次改变、
底盘参考变化或长于 500ms 的采样断流会锁住平移，需要重新摆正、停机并共同重启。
World/Gimbal 依赖的 IMU 失效时平移归零，原有受限旋转命令仍可执行；没有自动猜测或切换坐标系。
Body 输入不依赖绝对参考。头部 IMU 没有独立 Boot ID，短于保护窗口的传感器单独重启不能可靠识别，
现场仍须停机并共同重启建立基准。六轴 yaw 会长期漂移；不提供地理绝对航向。

## 大 Yaw MIT 航向锁定

`big_yaw_heading_lock_enabled`、`big_yaw_mit_mode_confirmed`、`startup_heading_alignment_confirmed`
默认均为 false。只有同时明确确认后才启用。原 FollowCenter/VelocityMotor 路径保留且默认使用。
选择新后端后不自动退回旧路径；失效要停机清历史。

滚轮积分目标世界 yaw，滚轮回中时锁定当前实测 yaw；误差取 ±π 最短分支并限 ±π/2，
外层 P 生成世界角速度目标，内层使用实测 IMU yaw rate PI 生成**关节力矩 N·m**，
经安装方向和外部传动比变换一次，再通过 Motor::setTorque（带 enable generation）进入 MIT。
没有复制参考的电机内部速度模式、0x206 帧、速度 PI 输出单位或增益。
PI 初始值沿用本工程已有 MIT 速度环的 .02/.15，仅作待调候选，力矩上限仍为原 .5 N·m。
饱和时条件积分，恢复/使能代次/零位或源启动代次变化首周期只重建参考、输出零。
权限撤销、命令过期、IMU 无效/超龄、电机反馈失效或传输故障清除目标与积分并停机。

可选模式会把遥控滚轮从底盘 wz 分配给大 Yaw 世界角速度（最大 ±1 rad/s）；
右拨杆 Down 单独请求 +.5 rad/s 底盘旋转，沿用 .3 m/s 平移上限。
只在新模式开启时生效；原遥控映射和跟随居中默认不变。

## 通信与上车验收

使用独立 0x0405/40 字节，保留参考 0x0404/32 字节的命名空间，新增发送方 Boot ID；
完整字节对照、时效及旧会话策略见 [v4 协议](../protocols/interboard-v4.md)。两个最终应用同时支持，
CAN/RS485 DM-IMU-L1 驱动未重写。当前最终应用复用已有 CAN 配置，不新增串口资源。

- 先完成 P0 单轮符号和总线验收；核对大 Yaw 实际 MIT、0x02/0x12 或现场 IDs，以及 PMAX/VMAX/TMAX。
- 在 IMU 观测工程确认两颗 IMU 的安装矩阵、正方向、姿态质量和时间戳，测量温漂。
- 双板摆正静止启动，确认软件零位，仅在真实验收后设置 alignment 标志。
- 先选择 Gimbal 坐标但关闭旋转补偿，验证车头/云台分别转 90° 时推前方向。
- 分别启用预测、降速和角速度反馈；最后一项需要单独调参，不与其他变量同时改变。
- 架空低力矩启用大 Yaw 航向锁定，验证滚轮回中、跨 ±π、连续多圈及停车后的新目标。
- 断头部 IMU、断 BMI088、暂停生产者、撤销授权、重启云台板/底盘板，验证拒绝旧帧且不追赶旧目标。
- 短断流测试基准不漂移，长断流/参考变化测试要求重新建立基准；记录不能识别的 IMU 单独重启场景。
- 落地低速验证后再调力矩、速度、几何和功率模型；没有实车记录前不写“已经调通”。

## 软件验证记录

固定 west.yml 的 Zephyr 6085aadeb337 / SDK 1.0.1；全部 10 个原生 testcase 工程共 143 项测试通过，
其中新增 4 项航向/执行器测试及 3 项 codec/link/双端通信测试。测试执行真实控制代码，
覆盖 ±π、多圈、滚轮回中、PI 限幅、源/目标重启、旧样本、CRC、长度/保留位、超龄与授权撤销。
`applications/sentry_chassis` 和 `applications/sentry_gimbal` 的 MC02 构建均通过。
本地 Twister 因 AF_UNIX 受限无法启动 multiprocessing Manager，采用逐工程 west build +
直接运行 native_sim/native/64 的 zephyr.exe；CI 仍由 Unit tests 自动发现全部 testcase。
GitHub Actions 状态以 PR 检查为准，软件验证不能替代实车验收。
