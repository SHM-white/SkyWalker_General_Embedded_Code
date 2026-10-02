# 三源命令管理台架

只接收命令并输出最终决策，不创建电机、CAN 或执行线程。支持 MC02。

```cpp
// 启动时依次检查返回码：注册遥控、视觉，绑定裁判，然后 start()。
robotics::CommandSnapshot frame{};
if (manager.snapshot(frame) == 0)
    telemetry.emit(frame);
```

CommandManager 内部线程完成采集和仲裁，默认周期 10 ms；同步算法位于 CommandArbiter。RemoteSource、VisionSource 与 RefereePermissionSource 在 main 中注册，所有对象和 DMA 存储保持静态生命周期。只支持启动前注册，每个角色一个来源，运行中不注销、不重启。

snapshot() 复制最新完整帧，不消费消息；current() 只复制最终 RobotCommand。首次发布前返回 -EAGAIN 且不改输出。读取成功不代表新帧、在线或允许运动，命令序号、原因和原始时间戳仍需按用途处理。采集时间与 decision 在同一快照中，错误存入 decision.error；读取不会刷新时间。Down 禁用；启动或遥控断流后必须重新完成安全档归中和解锁操作。

## 接线与构建

| 输入/输出 | MC02 接口 | 格式 |
| --- | --- | --- |
| DR16 遥控 | UART5 PD2 RX | 100000 8E1，确认 DBUS 反相链路 |
| 视觉 AB | UART7 PE7 RX / PE8 TX | 115200 8N1，仅 RX |
| 裁判 | USART1 PA10 RX / PA9 TX | 115200 8N1，Rm2026V1_3 |
| 终端 | USART10 PE3 TX / PE2 RX | 115200 8N1 |
| VOFA | USB CDC ACM | JustFloat，16 通道 |

外部 TX 接板端 RX 并共地。裁判设备与模拟发送器二选一。

```sh
west build -p always -b dm_mc02/stm32h723xx samples/robotics/command_manager -d build/command-manager-terminal
west build -p always -b dm_mc02/stm32h723xx samples/robotics/command_manager -d build/command-manager-vofa -- -DEXTRA_CONF_FILE=vofa.conf
west flash -d build/command-manager-terminal
```

只构建/烧录需要的版本。配置集中在 `src/board_config.hpp`，默认视觉参考为 `{1,1}`，遥控/视觉超时 100 ms，权限超时 300 ms。AB 不携带远端会话/序号，该参考是本地约定，不提供远端重放保护。

## 操作策略

左开关 Down=Safe、Middle=Manual、Up=Auto。先左 Down、右 Down，双摇杆和拨轮归中保持 500 ms，再拨左 Middle 解锁；已解锁后才可切 Auto。本例仅使用物理遥控器，右 Down 停摩擦轮、Middle 预热、Up 请求发射。Manual 忽略视觉；Auto 保留遥控底盘并由视觉控制云台。右摇杆映射幅度达到 0.15 时立即抢占；降到 0.05 以下输出零速率，持续 200 ms 后等待新视觉帧再交回。左摇杆不抢占云台。本例启用第五通道拨轮，拨轮控制底盘 wz；接收机必须输出该扩展。

进入 Auto 和解除抢占均要求收到后续新视觉帧。停止、超时、错误参考或非法目标使云台 Hold；遥控离线全停。裁判使用三个机构各自权限的时间戳逐机构否决，功率帧不能刷新权限。右 Middle/Up 授权摩擦轮，右 Up 请求手动发射；Auto 的视觉开火也必须保持右 Up。最终云台 Hold/Disabled 时禁止连续开火。本 sample 不判断实际摩擦轮就绪或热量。

## PC 输入与完整联调

串口发送需要 pyserial；`--hex-only` 不打开串口。

```sh
python3 samples/communication/vision/send_command.py --port /dev/ttyUSB_VISION --mode 1 --yaw 0.25 --pitch -0.20 --yaw-vel 0.10 --hz 50 --count 3000
python3 samples/robotics/command_manager/send_referee.py --port /dev/ttyUSB_REFEREE --flags 7 --hz 10 --count 600
```

1. 没有输入时观察全部 Disabled 和 RcUnavailable。
2. 开启上述两路发送、完成安全档解锁并进入遥控 Manual：左/右摇杆分别改变底盘/云台，视觉被忽略。
3. 切 Auto：后续视觉帧让云台变为 AbsoluteAngle/Vision。左摇杆继续控制底盘，右摇杆立即变为 Rate/Remote。
4. 右摇杆回中：quiet=1、两轴速率为零；200 ms 后先 Hold，再收到新帧回到 Vision。
5. 停止视觉发送，超过 100 ms 云台 Hold。发送 mode=0 同样 Hold。
6. 裁判 flags=5 禁底盘、6 禁云台、3 禁发射、0 全禁；停发裁判或只发送 `--power-only`，权限超过 300 ms 失效。
7. 关闭遥控，100 ms 后全停；重新连接后先回安全档归中并再次解锁。视觉 mode=2 需要右 Up 授权，云台禁用时不允许开火。
8. VOFA 不读取时丢帧计数可增长，仲裁继续。

终端每 100 ms 输出最终模式、来源、速度、目标、开火频率、各机构原因、输入年龄、UART 错误和 sample 返回状态。裁判 dropped=-1 表示 RefereeReceiver 未提供该计数；旧 ref_reset 列已移除。年龄 -1 表示无效或未来时间。枚举：底盘 Disabled=0/BodyVelocity=1；云台 Disabled=0/Hold=1/Rate=2/AbsoluteAngle=3；发射 Disabled=0/Ready=1/FireSingle=2/FireContinuous=3。原因位见 `include/robotics/command/command_inputs.hpp`；ManualOverride、OverrideQuiet、ValueLimited 是信息位。

| VOFA 通道 | 最终决策内容 |
| --- | --- |
| 0,1 | 操作模式、云台来源 |
| 2,3,4 | 底盘、云台、发射模式 |
| 5,6,7 | vx、vy、wz |
| 8,9 | yaw、pitch 角度 |
| 10,11 | yaw、pitch 速率 |
| 12,13 | 开火频率、目标弹速（0=未指定） |
| 14,15 | 原因低/高 16 位 |

原因重建为 `uint32(ch14) | (uint32(ch15) << 16)`。完整时间和序号留在终端。遥测从最终 command 输出；裁判否决后不会显示候选数值。接收器及 DMA 存储为静态生命周期，接收器自行推进 UART 重试；服务 start 返回负值时记录错误并停止启动，保持已启动接收器和 DMA 对象存活，不重复 start。

纯仲裁完整流程的主机检查见 `tests/command_manager/README.md`。

统一物理操作说明见 [遥控操作](../common/REMOTE_CONTROL.md)。本例观察仲裁结果，没有实际拨盘，单发边沿及摩擦就绪由发射执行样例验证。
