# chassis_module_rl

固定RL单舵轮上车工程。只注册本轮CAN1舵向和CAN2轮驱电机，不启动其他轮或大Yaw。FL/FR/RL/RR在中央数组中的序号为0/1/2/3，电机ID为1/4/2/3；真实ID、零点、减速比和方向只读取共享实车标定。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/chassis_module_rl -d ../build/bringup/chassis_module_rl
```

先在不执行电机的板间观察工程确认两端版本、boot、消息和原始输入年龄，再在云台发送端用真实遥控解锁并发送低速 `ChassisControl`。本入口从车体vx/vy/wz解算本轮目标，分别限制到0.1m/s、0.1m/s、0.2rad/s，最终轮缘速度不超过0.1m/s，舵向/轮驱控制电流均不超过0.3A。不使用底盘本地遥控，不伪造输出授权；撤销授权、命令超时、心跳失效、boot不匹配均停用该模块，急停须真实清除事件后才解除。

每次只接入/观察当前固定轮；当前轮未通过前不要进入四轮工程。失去舵向有效反馈时也撤销本轮驱动输出。所有电机目标与两路CAN提交由本入口控制线程拥有；独立通信线程1ms轮询，控制周期沿用中央2ms。

诊断只使用共享 `diagnostic-uart`（当前候选UART7，115200），不依赖USB。修改 [diagnostics_config.hpp](../../../include/robotics/vehicle/diagnostics_config.hpp) 的 `diagnostic_output` 后重新构建、烧录即可选择文本/VOFA，COM口同时只由一个电脑程序打开。

VOFA JustFloat布局版本1共19通道，按零起始编号：0布局版本；1/2采样序号低/高16位；3/4本板ms低/高16位；5原因；6轮速测量valid；7当前执行enabled；8轮轴速度目标rad/s；9轮轴实际rad/s（无效为NaN）；10原命令接收年龄ms（-1无效）；11控制dt微秒；12本轮驱动电流目标A；13模块序号（0FL/1FR/2RL/3RR）；14错误码；15/16诊断丢包数低/高16位；17优化后舵角目标rad；18舵角反馈rad（无效NaN）。原因枚举见 [diagnostics.hpp](../../../include/robotics/vehicle/diagnostics.hpp)，方向和轴侧见 [v4契约](../../../docs/protocols/interboard-v4.md)。队列满丢观测数据，不等待电脑。

本工程未进行实板验证。启动失败仍持续输出状态；未收到真实板间操作授权时保持失能。不要把心跳在线、原代码中的确认标志或编译成功作为现场已通过的记录。
