# 哨兵底盘应用

参考 v4 的 `0x0101` 平移以云台系定义：底盘在舵轮运动学之前，使用大 Yaw 原生电机反馈角 `native_position_rad` 及同坐标系的固定前向零位做车体系转换；参考小陀螺平移降速也在转换前执行。角度必须新鲜、原生角有效且为有限数；反馈中断时平移清零，在新鲜角度及有效行走命令恢复后重新放行。该转换不依赖连续位置展开参考，因此大 Yaw 掉线恢复后无需重新建立连续位置参考。BMI088 用于将大 Yaw 目标世界角速度换算为关节相对角速度。
大 Yaw 在板间只接受 `0x0402 mode=0/1`，不再接受 `mode=2` 线上扩展。可选 MIT 航向内环仍是本地驱动策略，不改变 v4 的速度语义。
原生速度配置的大 Yaw 关节速度上限由 `calibration.hpp` 的 `reference_big_yaw.velocity_limit_rad_s` 提供。世界速度经底盘旋转补偿后，按执行器使用的同一关节上限饱和，避免正常补偿超限触发反复失能。云台回中世界速度上限由云台入口单独设置；速度加速度限制为 4 rad/s²。
参数及双板启动、断流/重启规则见 [IMU 航向验收](../../docs/dev/哨兵IMU航向控制验收.md)。

固定的手动板间底盘执行入口。`src/main.cpp` 自行装配八个舵轮电机、CAN1 舵向、CAN2 轮驱、CAN3 大 Yaw、UART 板间端点及两个执行器，每个控制周期由一个执行线程各提交一次物理 CAN。无公共整车 `run()`、角色宏或场景宏。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_chassis -d ../build/applications/sentry_chassis
```

硬件使用 [chassis.overlay](../../config/vehicle/installed/chassis.overlay)，系统配置固定引用同目录 `base.conf` / `chassis.conf`。参数由 [calibration.hpp](../../include/robotics/vehicle/calibration.hpp)、[chassis_profile.hpp](../../include/robotics/vehicle/chassis_profile.hpp)、[swerve_profile.hpp](../../include/robotics/vehicle/swerve_profile.hpp) 和 [big_yaw_profile.hpp](../../include/robotics/vehicle/big_yaw_profile.hpp) 提供。不要通过角色参数或额外 conf 改成另一个上车应用；先运行 [无输出板间观察与大 Yaw 单项入口](../../samples/bringup/README.md)。

底盘以参考 v4 的新鲜 `0x0101` 命令及 `0x0402` 请求为运行依据，不再要求独立 `0x0301` 授权。左下档由发送端转为停止命令。行走命令必须满足对端在线、接收方 boot 绑定与命令新鲜度；大 Yaw 还独立检查原始输入/命令年龄、许可和本轴反馈。对端 boot 改变时，两执行器均先跨越恢复边界；停止、失联或授权失效时撤销输出，恢复行为由执行器的当前命令与恢复代次约束。

默认是受中央电流上限约束的手动机械链，不启用功率闭环，不以估算功率冒充实测，不宣称已实现裁判约束。当前轮半径、轮距、轴距和舵向零点仍需实车标定。大 Yaw 的 gear_ratio 指 Motor API 接口侧到关节侧速度比；线路目标/反馈均为关节 rad/s，见 [v4 契约](../../docs/protocols/interboard-v4.md)。

诊断用共享 `diagnostic-uart` 和 [diagnostics_config.hpp](../../include/robotics/vehicle/diagnostics_config.hpp) 的唯一输出选择，日志/VOFA 都通过 DAPLink。初始化失败不退出，持续报告步骤、阻塞原因和数据年龄。共享通道顺序与布局版本见 [上车入口说明](../../samples/bringup/README.md)。本应用保留固定 19 通道，运行诊断聚焦大 Yaw；状态灯仍综合四轮与大 Yaw 状态。

| 通道（从 0 开始） | 含义 |
| --- | --- |
| 5 | 当前大 Yaw DiagnosticReason |
| 6 / 7 | 大 Yaw 实际速度测量有效 / 大 Yaw 执行 Active；四轮运行不再影响通道 7 |
| 8 / 9 | 补偿并限幅后的关节速度目标 / 实测关节速度，rad/s |
| 10 | 大 Yaw 请求的本地接收年龄，ms；不包含请求携带的原始年龄，-1 表示无请求 |
| 11 | 最近一次大 Yaw 中断时的控制周期，µs；首次中断前显示当前周期 |
| 12 | 电机实测力矩，N·m；是否新鲜需结合通道 6 |
| 13 / 14 | 最近一次大 Yaw 中断的 WaitReason / 错误；首次中断前显示当前执行原因和错误 |
| 17 | 当前 MotorState：0 Offline、1 Disabled、2 Enabling、3 Active、4 Fault |
| 18 | 最近一次中断时的入口阻断条件；首次中断前显示当前条件，编号见下文 |

WaitReason：0 None、1 Command、2 Transport、3 Feedback、4 Reference、5 Configuration、6 Drive、7 Power、8 Cycle。通道 14 优先记录执行错误；中断时执行错误为零则保留电机 `last_fault.error`，该电机错误可能来自此前恢复，须结合通道 13/18 判断。常见 -34 为范围错误，-116 为数据过期，-110 为超时。

通道 18：0 表示入口没有主动改成 Disabled，继续根据通道 13 排查执行器；1 表示收到的请求为 Disabled；2 表示底盘行走命令被禁用，连带撤销大 Yaw；3 表示底盘 BMI088 陀螺无效或超过 50 ms；4 表示云台发来的 `0x0404` 载体航向样本无效；5 表示没有可用的大 Yaw 请求，包含接收端因原始年龄过期清空请求的情况。

每个控制周期捕获“请求撤回”或“Active 退出”事件，通道 11/13/14/18 保留该事件直至下一次中断或重启，使 2 ms 级的短暂中断不会被 20 ms 输出周期漏掉。左 Down 主动停止也会更新这份记录，因此应保存包含故障发生过程的波形。通道 5/6/7/8/9/10/12/17 始终反映当前值。

先看通道 18 再看 13/14：18=1 时看云台的 `yaw_running`、航向新鲜度和实际小 Yaw 是否越界；18=3/4 时分别检查底盘陀螺/板间 IMU 样本；18=0、13=3 时查 CAN3 反馈及电机状态；13=8 时结合通道 11 检查是否超过 20000 µs。云台侧原有通道 14=-34 且小 Yaw 实测角反复越过软件限位，是上游撤权的直接线索。

尚未进行实板验证；每个方向、停止时限和机构组合结论需现场单独记录。

舵轮新增最终优化角的 0.10/0.40 rad 驱动联锁和逐轮 cos³ 补偿；参考 2026-10-28
标定以可切换候选保存在 calibration.hpp，默认保留旧值。见
[符号、单位、故障与上车验收](../../docs/dev/哨兵参考工程移植验收.md)。
