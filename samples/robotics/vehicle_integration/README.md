# 双主控整车组合

这个 sample 与 `applications/sentry_gimbal`、`applications/sentry_chassis` 共用 `../common/vehicle_bench.hpp`。默认入口是云台板，`chassis.conf` 切换为底盘板。两板都使用 V2 大 Yaw 契约，并保留既有 V1 底盘契约。

| 板卡 | 物理资源 | 执行机构 |
| --- | --- | --- |
| 云台板 | CAN1 | 小 Yaw GM6020、两个 M3508 摩擦轮、一个 M2006 拨盘 |
| 云台板 | CAN2 | Pitch DM4310 |
| 底盘板 | CAN1 | 四个 GM6020 舵向 |
| 底盘板 | CAN3 | 四个 M3508 轮驱动 |
| 底盘板 | CAN2 | 大 Yaw DM4310 |
| 两板 | USART1，460800 | 板间 UART，TX/RX 交叉并共地 |
| 云台板 | UART5 | DR16 原始来源 |
| 云台板 | USART2 / RS485-2 | 随头部 Pitch 运动的外置 IMU |
| 云台板 | UART7 | 视觉接收 |
| 云台板 | USART3 / RS485-3 | 裁判数据，需要真实 TTL/RS485 电气适配或改到空闲 TTL 口 |

`vehicle_mc02.overlay` 的 DMA 使用 USART1 3/4、UART5 6、UART7 5/7、头部 USART2 8、裁判 USART3 9/10，保留板载 SPI2 的 1/2。USART3 RX/TX request 为 45/46，USART2 RX 为 43，UART7 RX/TX 为 79/80，取自本地 STM32H7 HAL 定义。RS485 接口不能直接作为裁判 TTL 接口使用。

应用创建 Motor、Group、CanBus 并完成 attach/start。每块板的电机目标只由一个执行线程写入（底盘 2 ms 绝对节拍，云台仍为 5 ms），周期末每条物理 CAN 提交一次。通信线程只处理协议和快照；控制台/遥测线程只处理观察及操作请求。底盘八电机、大 Yaw、云台双轴、摩擦轮对、拨盘分别拥有自己的故障范围和恢复上下文。

云台板运行真实 `CommandManager`，把头部 IMU 的惯性目标经 `InertialGimbalAdapter` 转成机械控制，再由 `GimbalExecutor` 执行。头部稳定后，小 Yaw 独立标定中心偏差经带死区、迟滞、限速和斜坡的外环产生大 Yaw 请求。底盘板大 Yaw 只执行自己的速度内环，拒绝目标 boot 或独立恢复代次不匹配的请求。版本不匹配、命令过期或状态生产停更时，大 Yaw 保持禁用；轮控按自身条件处理。

所有硬件标定使用 `include/robotics/vehicle/calibration.hpp`。默认连接、IMU 安装和发射约束确认标志均为 false。入口仍运行来源接收、协议和遥测，但不会输出电机目标。默认手动版本使用明确的悬空台架操作权限，保留头部惯性保持和大 Yaw 回中链路；实际地面运动之前必须完成前级 sample 验收。

```sh
# 手动版本：分别刷写两板
west build -b dm_mc02 samples/robotics/vehicle_integration -d ../build/vehicle_gimbal
west build -b dm_mc02 samples/robotics/vehicle_integration -d ../build/vehicle_chassis -DEXTRA_CONF_FILE=chassis.conf

# 云台板视觉观察或视觉执行（观察版本仍保持手动来源）
west build -b dm_mc02 samples/robotics/vehicle_integration -d ../build/vehicle_vision_observe -DEXTRA_CONF_FILE=vision_observe.conf
west build -b dm_mc02 samples/robotics/vehicle_integration -d ../build/vehicle_vision_execute -DEXTRA_CONF_FILE=vision_execute.conf

# 实际裁判权限与功率：两板使用一致配置
west build -b dm_mc02 samples/robotics/vehicle_integration -d ../build/vehicle_referee -DEXTRA_CONF_FILE=power.conf
west build -b dm_mc02 samples/robotics/vehicle_integration -d ../build/vehicle_power_chassis '-DEXTRA_CONF_FILE=chassis.conf;power.conf'

# 最后接入发射业务
west build -b dm_mc02 samples/robotics/vehicle_integration -d ../build/vehicle_shooting -DEXTRA_CONF_FILE=shooting.conf
```

默认 Pitch 锁定；`pitch_free.conf` 在头部 Yaw 惯性保持通过后释放 Pitch。视觉执行配置自动释放 Pitch，仍要求真实头部参考一致；AB 协议没有远端 epoch 元数据，TODO 保留显式视觉参考会话及真实弹速/弹数反馈。当前 AB 反馈 TX 默认禁用，不能用占位值启用发送。

底盘功率配置将真实转发预算送入 `ChassisExecutor`，实际测量通过 `run(IPowerMeasurementSource *, ...)` 注入。V1 预算包没有实测功率字段，默认 `PendingPowerSource` 返回无效数据并阻止功率模式运行。TODO 需要接真实母线电压/电流或其他实测功率源；估计回退默认关闭，功率控制完成标志保留关闭，台架输出使用显式安培上限（舵向 0.8 A、驱动 0.5 A），独立缩放默认均为 1；真实总功率约束仍同时作用于两类电机。

发射版本要求裁判许可、热量、摩擦就绪、头部稳定和云台状态同时有效。`run` 另外接受真实 `IShooterHeatSource` 与 `IDialHomeSource`；默认两个 Pending 源均无效，Loaded 拨盘不会按当前点伪造零位。TODO 接裁判热量解析/模型，以及真实拨盘 home/index 传感器。单发由新的鼠标按下边沿或控制台 `f` 产生独立事件编号与原始时间，恢复后旧事件被丢弃；`c` 请求连发，`h` 撤销。

控制台操作：`p` 暂停来源，`e` 暂停执行，`s` 暂停执行状态发布，`i` 冻结头部 IMU 消费快照，`m` 冻结实际功率测量消费，`!` 锁存本板急停，`r` 显式清除。通信线程保持运行，读取不会刷新旧状态或旧原始数据年龄。TODO 接实际物理急停并按接线同步两板锁存；本板控制台急停仅用于台架操作。

按手动整车 → 视觉观察 → 视觉执行 → 受约束发射推进。每级分别操作来源停产、执行停更、状态停更但心跳继续、单板重启、恢复代次变化、头部 IMU 断流/参考变化、单机构断流和物理 CAN 故障。记录原始来源年龄、状态年龄、独立恢复代次、头部稳定/回中误差、请求与实际速度、功率缩放、总线错误、控制耗时/超限次数、执行栈余量。TODO 保留真实停输出延迟和 I/O 队列峰值采集。

框架已实现，本轮未自行执行构建、测试或实板验证。中央 TODO 和 Pending 测量源需要实板标定后填写。
