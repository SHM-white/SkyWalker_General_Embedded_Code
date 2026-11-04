# 哨兵底盘应用

参考 v4 的 `0x0101` 平移以云台系定义：底盘在舵轮运动学之前，使用大 Yaw 电机编码器及固定前向零位做车体系转换；参考小陀螺平移降速也在转换前执行。BMI088 用于将大 Yaw 目标世界角速度换算为关节相对角速度。
大 Yaw 在板间只接受 `0x0402 mode=0/1`，不再接受 `mode=2` 线上扩展。可选 MIT 航向内环仍是本地驱动策略，不改变 v4 的速度语义。
参数及双板启动、断流/重启规则见 [IMU 航向验收](../../docs/dev/哨兵IMU航向控制验收.md)。

固定的手动板间底盘执行入口。`src/main.cpp` 自行装配八个舵轮电机、CAN1 舵向、CAN2 轮驱、CAN3 大 Yaw、UART 板间端点及两个执行器，每个控制周期由一个执行线程各提交一次物理 CAN。无公共整车 `run()`、角色宏或场景宏。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_chassis -d ../build/applications/sentry_chassis
```

硬件使用 [chassis.overlay](../../config/vehicle/installed/chassis.overlay)，系统配置固定引用同目录 `base.conf` / `chassis.conf`。参数由 [calibration.hpp](../../include/robotics/vehicle/calibration.hpp)、[chassis_profile.hpp](../../include/robotics/vehicle/chassis_profile.hpp)、[swerve_profile.hpp](../../include/robotics/vehicle/swerve_profile.hpp) 和 [big_yaw_profile.hpp](../../include/robotics/vehicle/big_yaw_profile.hpp) 提供。不要通过角色参数或额外 conf 改成另一个上车应用；先运行 [无输出板间观察与大 Yaw 单项入口](../../samples/bringup/README.md)。

底盘以参考 v4 的新鲜 `0x0101` 命令及 `0x0402` 请求为运行依据，不再要求独立 `0x0301` 授权。左下档由发送端转为停止命令。行走命令必须满足对端在线、接收方 boot 绑定与命令新鲜度；大 Yaw 还独立检查原始输入/命令年龄、许可和本轴反馈。对端 boot 改变时，两执行器均先跨越恢复边界；停止、失联或授权失效时撤销输出，恢复行为由执行器的当前命令与恢复代次约束。

默认是受中央电流上限约束的手动机械链，不启用功率闭环，不以估算功率冒充实测，不宣称已实现裁判约束。当前轮半径、轮距、轴距和舵向零点仍需实车标定。大 Yaw 的 gear_ratio 指 Motor API 接口侧到关节侧速度比；线路目标/反馈均为关节 rad/s，见 [v4 契约](../../docs/protocols/interboard-v4.md)。

诊断用共享 `diagnostic-uart` 和 [diagnostics_config.hpp](../../include/robotics/vehicle/diagnostics_config.hpp) 的唯一输出选择，日志/VOFA 都通过 DAPLink。初始化失败不退出，持续报告步骤、阻塞原因和数据年龄。共享通道顺序与布局版本见 [上车入口说明](../../samples/bringup/README.md)。本应用 `target` / `feedback` 分别为大 Yaw 请求/实测关节 rad/s，`valid` 为大 Yaw 测量有效性，`enabled` 为底盘或大 Yaw 有执行输出，`age_ms` 为行走命令年龄；不替代逐轮调试入口。

尚未进行实板验证；每个方向、停止时限和机构组合结论需现场单独记录。

舵轮新增最终优化角的 0.10/0.40 rad 驱动联锁和逐轮 cos³ 补偿；参考 2026-10-28
标定以可切换候选保存在 calibration.hpp，默认保留旧值。见
[符号、单位、故障与上车验收](../../docs/dev/哨兵参考工程移植验收.md)。
