# chassis_four_modules

固定前后直线阶段。只接受BodyVelocity的vx，vy和wz清零；Spin停用，不执行横移或自转。用于逐轮通过后比较四轮方向和线速度量纲。本入口直接装配四路舵向和四路轮驱，只启动CAN1/CAN2，不启动大Yaw。共享中央硬件、PID、零点与方向；阶段策略只施加更低输出上限。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/chassis_four_modules -d ../build/bringup/chassis_four_modules
```

先确认四个固定module工程，再确认无输出板间观察。使用已确认云台发送端和真实RC授权，经 `OperatorControl + ChassisControl` 提供输入；没有真实授权保持失能，不需要将遥控接收机移到底盘板。vx/vy分别限制0.1m/s、wz限制0.2rad/s，解算后各轮缘速度最多0.1m/s；舵向/轮驱电流最多0.3A。独立通信线程1ms轮询，控制线程2ms提交两路CAN各一次。命令/授权/心跳/boot失败停止；真实急停清除事件才解除本地急停锁存。

这些阶段使用手动低电流策略，未启用功率闭环；真实功率和裁判约束属于后续独立能力。四轮诊断每20ms轮换一轮，通道13标明当前轮；该轮反馈每80ms更新一次，不把多轮拼成同一时刻的测量。先以足够机械支撑和低速小幅输入逐项确认，保留可回退固件。

诊断只使用共享 `diagnostic-uart`（当前候选UART7，115200），不依赖USB。修改 [diagnostics_config.hpp](../../../include/robotics/vehicle/diagnostics_config.hpp) 的 `diagnostic_output` 后重新构建、烧录即可选择文本/VOFA，COM口同时只由一个电脑程序打开。

VOFA JustFloat布局版本1共19通道，按零起始编号：0布局版本；1/2采样序号低/高16位；3/4本板ms低/高16位；5原因；6轮速测量valid；7当前执行enabled；8轮轴速度目标rad/s；9轮轴实际rad/s（无效为NaN）；10原命令接收年龄ms（-1无效）；11控制dt微秒；12本轮驱动电流目标A；13模块序号（0FL/1FR/2RL/3RR）；14错误码；15/16诊断丢包数低/高16位；17优化后舵角目标rad；18舵角反馈rad（无效NaN）。原因枚举见 [diagnostics.hpp](../../../include/robotics/vehicle/diagnostics.hpp)，方向和轴侧见 [v4契约](../../../docs/protocols/interboard-v4.md)。队列满丢观测数据，不等待电脑。

本工程未进行实板验证。启动失败仍持续输出状态；未收到真实板间操作授权时保持失能。不要把心跳在线、原代码中的确认标志或编译成功作为现场已通过的记录。
