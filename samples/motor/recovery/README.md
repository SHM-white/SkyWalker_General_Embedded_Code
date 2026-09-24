# 单电机断电恢复

一块 MC02 + 一台电机。默认 CAN1 上的 GM6020 电流模式 ID1，电流上限 0.5 A；可选 DM J4310 MIT ID1、Master 0x11、力矩上限 0.5 N·m。电机需独立供电、CAN 终端正确且与控制板共地。MC02 逻辑供电保持不断，控制台使用板定义的 USART10、115200 baud。

## 构建与刷写

```sh
west build -b dm_mc02/stm32h723xx samples/motor/recovery -d build/bench_recovery
west flash -d build/bench_recovery
```

DM MIT 配置：

```sh
west build -b dm_mc02/stm32h723xx samples/motor/recovery -d build/bench_recovery_dm -- -DRECOVERY_DM=ON
west flash -d build/bench_recovery_dm
```

`dm.overlay` 现为兼容性空 overlay；型号、CAN ID、限幅和时限在 `src/main.cpp` 中配置，控制器参数在 `src/board_config.hpp` 中配置。DM 的 PMAX/VMAX/TMAX 必须与上位机一致。DM 配置先启动 CAN，再使能 MC02 XT30_1 并等待 1.5 s。

## 操作与预期

启动仅接收反馈，无运动目标。电机 `ready` 后按 `e`，显式使能并以 2 rad/s 运行。空格立即撤销软件输出许可；`!` 还锁住再次使能。按 `r` 清除软件急停和可清故障，但不会启动电机；必须重新按 `e`。仅切断电机供电时，状态应退出 Active、旧目标作废；重新供电且准备就绪后，仍需按 `e` 发起新的使能，不会自动续跑。日志每 250 ms 显示状态、代次、故障、停机进度、速度和 effort。

目前仅做固件编译验证，尚未连接实物或刷写。
