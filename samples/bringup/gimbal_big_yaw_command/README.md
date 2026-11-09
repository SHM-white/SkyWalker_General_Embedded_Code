# gimbal_big_yaw_command

不启动本地电机的实际 RC 大 Yaw 命令发送端，固定配未修改的参考底盘 `JJL0612/sentry_onlychassis@dcc8d4e`。这次适配只需重编译、烧录云台板，底盘保持现有 v4 固件。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_big_yaw_command -d ../build/bringup/gimbal_big_yaw_command
west flash -d ../build/bringup/gimbal_big_yaw_command
```

## 请求与反馈

`0x0402` 的 `mode=1` 请求世界系角速度：`-normalize(右杆X) × min(bringup::big_yaw_rate_rad_s, 1.0)` rad/s。右杆归中请求零速航向保持；左 Down 撤销授权并失能。参考底盘持续使用自己的陀螺补偿、板载航向和角速度 PI，不需要开启小陀螺才运行 IMU 闭环。

同时发送参考 v4 `0x0404`（32 字节）航向，以及零速 `0x0101` 底盘控制以放行大 Yaw 通道。没有额外的 `0x0405` IMU 帧，也不依赖 `0x0301` 遥控扩展作为运行前提。该世界系请求不能直接发给本仓库只接受关节相对速度的独立 `chassis_big_yaw`。

板载 BMI088 使用 SPI `accel0` / `gyro0`，必须安装在大 Yaw 随动体上。启动时用 800 个静止样本估计陀螺偏置，此后禁止在线学习偏置，避免误吸收真实慢转。`correct_gyro_bias=true` 使对外角速度与姿态积分使用相同的偏置；姿态和去偏角速度一起发布，初始化或计算失败不发送原始角速度作为替代。

航向有效且对端在线时才授权；远端实际速度未反馈时仅发送零速请求以启动反馈。反馈接收期限为 250 ms，匹配参考底盘 100 ms 上报周期；无效反馈不允许非零手动目标。UART7/PE8-TX 接诊断接收端 RX，共地，115200 8N1；云台 USART10 与底盘 USART1 为 460800 8N1。

## 操作与诊断

上电静置至少 2 秒，左 Down、右 Middle，摇杆及拨轮归中至少 500 ms，再左 Down→Middle→Up 进入手动。遥控失联、对端掉线或重启后重新解锁。先确认零输入停住，再小幅正反操作，避免碰撞或缠线。

固定 19 通道诊断：通道 8 target 为世界速度请求；通道 9 feedback 为按参考安装方向和传动比转换后的大 Yaw 关节速度（rad/s，底盘静止时可与 target 比较）；通道 17 extra0 为板载启动零点相对航向（rad）；通道 18 extra1 为发给底盘的去偏航向角速度（rad/s）。无有效航向时 extra0/extra1 为 NaN，valid=0 时 feedback 为 NaN。额外字段不再用于 ready/armed 标记。

单轴零输入仍持续转时，先记录 target、feedback、extra0、extra1，并观察参考底盘自身陀螺、航向误差和速度积分。这能区分发送侧偏差与参考底盘闭环偏差，不能把零请求直接理解为电机零输出。本次未烧录或实板验证，传感器安装、启动静止条件及温漂仍需现场确认。
