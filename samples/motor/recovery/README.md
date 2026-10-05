# 持续速度目标与电机自动恢复

本例固定目标由主循环每 5 ms 生产。按 `e` 后，电机缺席、掉电、CAN 恢复或驱动故障均不会撤销运行意图；底层独立恢复后执行最新目标，无需再次 `e`、手动 reset 或 clearFault。

| 按键 | 行为 |
| --- | --- |
| e | 请求持续运行；电机尚未上线也接受 |
| 空格 | 停止并取消旧命令 |
| ! | 停止并锁存用户急停 |
| r | 解除用户急停，保持停止 |

默认 M2006/C610，CAN1 ID1。`-DRECOVERY_DM=ON` 使用 DM J4310 MIT、ID1、Master ID0x11；MC02 DM 变体会打开 power1。PID 参数在 `src/board_config.hpp`。

日志区分 run、Motor.enabled_requested、实际状态、目标序号、输出有效性、等待原因和历史故障。`update()==0` 表示目标已接受，不保证设备已经执行。

```bash
west build -b dm_mc02/stm32h723xx samples/motor/recovery -d build/recovery_auto
west build -b dm_mc02/stm32h723xx samples/motor/recovery -d build/recovery_auto_dm -- -DRECOVERY_DM=ON
```

让电机输出轴空载，再在运行中断电、恢复电源；日志目标序号应持续增加，恢复后自动运动。离线期间按空格／急停，再恢复电源应保持停止。电源断开前保持控制目标是本例明确行为。
