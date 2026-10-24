# 哨兵云台应用

头部 DM-IMU-L1 默认使用 CAN 主动上报，安装参考与变换仍取中央 `include/robotics/vehicle/calibration.hpp`。CAN1/CAN2 为现有电机总线，IMU 独占云台板 CAN3；不改底盘板 CAN 分配。

## 当前主动模式接入

- 将 IMU CANH/CANL 接云台板 CAN3，按实际总线连接公共参考地与终端电阻。
- 厂家上位机选择 CAN、主动上报、1 Mbps，从机地址 `0x1`、主机地址 `0x11`。
- 开启 CAN 输出、四元数、角速度、加速度/温度（输出位图 `0x0D`），先设置 5ms 上报周期并保存。若启用温控，等待模块的温控输出门控解除。
- 非默认地址可修改 `CONFIG_SKYWALKER_IMU_DM_CAN_SLAVE_ID` 和 `CONFIG_SKYWALKER_IMU_DM_CAN_FEEDBACK_ID`；非默认速率修改设备树 `&can3` 的 `bitrate`。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d build/sentry_gimbal_can
```

每秒 `dm_imu_can` 应显示 `mode=active slave=1 feedback=11`，`rx/updates` 增长、`tx=0`。快照全部新鲜时 `fresh_mask=15`。接收断流时字段到期失效，不保活旧姿态。

## 切换模式

模块先在上位机设置并保存被动应答，再编译：

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d build/sentry_gimbal_can_passive -- -DEXTRA_CONF_FILE=imu_can_passive.conf
```

需要同时加载 Pitch 配置时使用 `-DEXTRA_CONF_FILE="pitch_free.conf;imu_can_passive.conf"`。主动模式默认生效，无须额外 fragment。切换配置不自动修改 IMU 内部模式。

恢复原 RS485 接线与驱动：

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d build/sentry_gimbal_rs485 -- -DEXTRA_CONF_FILE=imu_rs485.conf
```

CAN 两种模式的共用解码依据工作区 V1.3 手册；被动请求格式由厂家 MC02 示例补充。未进行硬件通信验证。安装标定、`imu_mounting_confirmed` 以及执行约束仍需按实际机构配置；通信接入不代表已完成惯性云台标定。

[CAN 协议、接口与诊断](../../docs/modules/drivers/imu.md#dm-imu-l1-can) · [无电机的双 IMU 观测样例](../../samples/imu/dual_imu/README.md#外置-imu-改用-can)
