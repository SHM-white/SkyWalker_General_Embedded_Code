# 云台共享 CAN 空载组合

CAN1 同时 attach 小 Yaw GM6020、两个摩擦轮 M3508 和拨盘 M2006，Pitch DM4310 在 CAN2。三个故障组分别为小 Yaw/Pitch、两个摩擦轮、单拨盘；执行器只暂存目标，应用每 5 ms 在所有控制更新结束后各提交一次 CAN。连接与方向读取 `include/robotics/vehicle/calibration.hpp`，确认开关默认 false。

本入口为无弹空载资源验证。RC 仲裁控制云台和摩擦启动；console `f` 独立单发事件、`c` 连续分度、`h` 停拨盘、`!` 急停、`r` 清除、`p` 冻结命令快照、`x` 冻结执行。先确认摩擦达到目标和 dwell，再拨盘分度；恢复不重放旧事件，卡滞需清除。事件的编号和原始生产时间独立于仲裁序号。

```sh
west build -p always -b dm_mc02/stm32h723xx samples/robotics/gimbal_shared_can -d build/gimbal_shared_can
```

TODO：实测共享 CAN 负载、周期超限、故障范围、停输出延迟；校准摩擦方向/速度、拨盘索引、PID 和安装限位。loaded shooting 需要真实许可、热量与云台状态，不能沿用空载条件旁路。未进行实板验收。
