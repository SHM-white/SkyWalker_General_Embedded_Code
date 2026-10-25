# gimbal_big_yaw_command

无本地电机的实际RC命令发送端。只生产±0.1rad/s大Yaw请求，配chassis_big_yaw；不启动轮控。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_big_yaw_command -d ../build/bringup/gimbal_big_yaw_command
west flash -d ../build/bringup/gimbal_big_yaw_command
```

UART7/PE8-TX接DAPLink RX，共地，115200 8N1；云台USART10与底盘USART1互连为460800 8N1。当前映射沿用固件，未确认现场接线。修改共享diagnostics_config.hpp的diagnostic_output后重编译烧录选择文本或纯JustFloat，不发送切换命令。

VOFA布局1，固定19通道：layout、sequence低16、高16、uptime_ms低16、高16、reason、valid、enabled、target、feedback、age_ms、dt_us、effort、detail、error、dropped低16、高16、extra0、extra1。valid=0时feedback=NaN，age=-1未知。target/feedback为线上的关节速度rad/s；age为RC年龄。

真实遥控双拨Down归中500ms、左Middle解锁；左Down/输入过期/对端离线或重启均撤销授权，恢复后重新解锁。Disabled+0是失能，FollowCenter+0仍是零速闭环。目标限制不覆盖真实标定。先做[板间只观察](../gimbal_link_observe/README.md)，再支撑机构小幅操作。未做实板验证。
