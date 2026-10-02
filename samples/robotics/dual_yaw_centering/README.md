# 双板双 Yaw 回中

云台角色默认构建，底盘角色只控制独立大 Yaw。输出确认默认关闭。先完成 `big_yaw` 与 `inertial_gimbal` 台架，在中央 calibration 中保存 ID、方向、减速比、限幅和独立小 Yaw 中心。头部保持有效且小云台实际 Active 后才开放回中。TODO(hardware)：死区/迟滞、回中方向、速度与斜坡、头部误差、无线输入停更和三种重启场景需要实板记录。

USART1 双板交叉 TX/RX 共地，日志用 console；云台小 Yaw/Pitch 分处 CAN1/CAN2，底盘大 Yaw 使用 CAN3。外置头部 IMU 使用 rs485-2。

```sh
west build -b dm_mc02 samples/robotics/dual_yaw_centering -d build/dual_yaw_gimbal
west build -b dm_mc02 samples/robotics/dual_yaw_centering -d build/dual_yaw_chassis -- -DEXTRA_CONF_FILE=chassis.conf
```

V1 心跳与底盘协议保持原含义。V2 capabilities、big-Yaw request/feedback 使用独立消息 ID 与帧版本。双方契约版本与能力必须相同，反馈生产年龄有效后才能绑定独立大 Yaw 恢复代次；轮控恢复代次不授权大 Yaw。原始输入年龄不会在转发时刷新。

控制台 `p` 停输入生产、`s` 停状态生产、`e` 停执行生产、`x` 急停、`r` 显式清除。通信线程独立继续运行，旧 Ready/Active 反馈过期后撤销。恢复后旧单次源和旧上下文请求不能使能。
