# gimbal_big_yaw_command

无本地电机的实际RC命令发送端。只生产大Yaw请求，配 `chassis_big_yaw` 或参考底盘 `sentry_onlychassis`；不启动轮控。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_big_yaw_command -d ../build/bringup/gimbal_big_yaw_command
west flash -d ../build/bringup/gimbal_big_yaw_command
```

## 发送内容

| 帧 | 触发 | 说明 |
|---|---|---|
| `0x0301` OperatorControl | 每周期 | 原始解锁/清故障事件，绑定对端 Boot ID |
| `0x0402` BigYawRequest | 每周期 | `mode=1` + 权限；`target_rate_rad_s = -normalize(右杆X) × bringup::big_yaw_rate_rad_s` |
| `0x0404` GimbalImuYaw（32 B） | 对端在线 | **板载 BMI088** 经四元数 EKF 得到的、相对软件零位的航向与世界角速度 |
| `0x0405` GimbalImuYaw（40 B） | 对端在线 | 本工程标准扩展，与 0x0404 同时发送 |
| `0x0101` ChassisControl | 每周期 | **零速 BodyVelocity**，只为让参考底盘执行 `RobotBigYawControl()`；不给轮子速度 |

内置 IMU 按 `samples/imu/dual_imu` 的 onboard 一路接：`accel0` / `gyro0`（板载 BMI088，SPI）+ `control::QuaternionEkf` + `imu::ImuReceiver{poll=500us, prio=5}`，安装变换用 `carrier_reference` / `carrier_sensor_to_body`（dual_imu 里该板对应 `large_yaw_carrier`）。零位由 `ImuHeadingReference` 在**解除授权**时采集：上电后要先双拨 Down 归中 500 ms 才可能拿到有效航向。

参考底盘的 gyro 大 Yaw 后端要求 `mode=1`、权限有效、以及 50 ms 内的 0x0404 航向样本，并且底盘自身 BMI088 就绪；缺任一项它只发 Disable。它把 `mode=1` 的 `target_rate_rad_s` 当作**头部世界角速度**，所以本样例右杆X量纲是世界角速度而不是关节相对速度。切回本仓库 `chassis_big_yaw` 时同一帧是关节相对速度，两种语义不可混用。

UART7/PE8-TX接DAPLink RX，共地，115200 8N1；云台USART10与底盘USART1互连为460800 8N1。当前映射沿用固件，未确认现场接线。修改共享diagnostics_config.hpp的diagnostic_output后重编译烧录选择文本或纯JustFloat，不发送切换命令。

VOFA布局1，固定19通道：layout、sequence低16、高16、uptime_ms低16、高16、reason、valid、enabled、target、feedback、age_ms、dt_us、effort、detail、error、dropped低16、高16、extra0、extra1。valid=0时feedback=NaN，age=-1未知。target/feedback为线上的目标与实际角速度rad/s；age为RC年龄；reason 依次判定 InitFailed→RemoteStale→PeerStale→ImuStale→Disarmed→Ready。

真实遥控双拨Down归中500ms，先把左Middle建立解锁、再拨左Up手动运行；左Down/输入过期/对端离线或重启均撤销授权，恢复后重新解锁。Disabled+0是失能，FollowCenter+0仍是零速闭环。目标限制不覆盖真实标定。先做[板间只观察](../gimbal_link_observe/README.md)，再支撑机构小幅操作。未做实板验证。
