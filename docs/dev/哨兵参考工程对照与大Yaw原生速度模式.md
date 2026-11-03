# 哨兵参考工程对照：大 Yaw 原生速度模式（2026-11-03）

状态：软件适配供审查，**默认关闭**；未烧录、未做实车测试。请勿仅因参考仓库能编译便宣称整车已调通。

## 固定参考

- 云台：[JJL0612/sentry_gimbal@3cfbe1c](https://github.com/JJL0612/sentry_gimbal/tree/3cfbe1ce53ade407812a327626112435a0834058)
- 底盘：[JJL0612/sentry_onlychassis@dcc8d4e](https://github.com/JJL0612/sentry_onlychassis/tree/dcc8d4ebfb5f91b0836849af7999e3b45849ed2e)
- 本工程起点：dev `529702e9ce9cfa3851e1ca742c5e39da5a2bb2a5`

参考 README 标注当前底盘仍有“四个驱动轮不转”待排查问题；云台最新跟随控制和 IMU 算法亦标注**尚未实车验证**。其文档包含历史实现描述，以最新版源代码和实际硬件读回为准。

## 已有能力（不重复移植）

- `SwerveKinematics / SwerveModule / SwerveChassis` 已有刚体运动学、四轮等比例限速、最优角翻转滞回、cos³、低速保角、舵向放行联锁及反馈超时保护。
- `ChassisMotionAdapter` 已有可选 IMU 坐标转换、预测、旋转降速；默认仍使用车身坐标，不能以未校准的两颗 IMU 直接做绝对差。
- `ImuHeadingLock` 已有可选 MIT 扭矩闭环，但必须确认电机 MIT 模式；不应误向速度模式电机发送 MIT 力矩。
- `dm_protocol.cpp` **原本**就支持原生 `0x200 + motor_id` 速度帧，DLC=4，小端 float；特殊使能/失能帧 DLC=8。之前缺少最终大 Yaw 执行器对 `setVelocity()` 的实际调用路径。

## 本次新增：原生速度后端（必须显式选择）

在 `include/robotics/vehicle/motion_profile.hpp` 中：

```cpp
inline constexpr bool big_yaw_native_velocity_profile_confirmed = false;
```

默认仍使用当前 `big_yaw` 的 0x02 / 0x12 / MIT 模式参数，**默认输出路径不变**。

现场确认 *电机实际配置* CTRL_MODE=3（原生速度模式）、基 ID=0x06、反馈 ID=0x11、CAN3 接线、安装方向、PMAX/VMAX/TMAX、实际传动比与机械行程后，才可以改为 `true`。选择后，`bigYawHardware()` 使用 `reference_big_yaw` 和 `j4310Velocity()`；`bigYawExecutionConfig()` 同步匹配对应 ID、安装方向及传动比，不混用 MIT/Motor 后端。两种后端与原先 MIT IMU 航向锁定互斥；改变此开关只改固件配置，**不会给电机写入硬件模式或永久设置**。

安全行为：

1. `BigYawExecutor::begin()` 检查实际 Motor capability 含 `CommandVelocity`，且不能同时启用 MIT 航向锁定；配错模式则拒绝启动。
2. 只有对端在线、Boot ID 一致、操作授权有效、输入年龄新鲜、电机 Active 且速度反馈有效时才发送速度目标。
3. 目标为 `target_joint_rate * motor_to_joint_ratio * direction`，以 motor-shaft rad/s² 斜率限幅；每次使能代次变化从零重建速度指令。
4. 超周期、电机失去 Active 状态时的反馈异常、权限或指令失效会撤销输出；不得在恢复后重放旧速度。电机本身依旧由已有 CanBus 安全停机流程负责。
5. 此模式目前执行**相对角速度跟随**，没有额外注入底盘陀螺补偿、IMU 世界航向 P/PI。参考的陀螺闭环及小 Yaw 世界跟随需独立实测，不直接复制增益。

## 上车前验证

- 先读取并记录 J4310 的当前 CTRL_MODE、CAN ID、MASTER_ID、驱动器零位、异常标志；速度模式与 MIT 模式不能仅凭源码推断。
- 架空机构、拆除可能碰撞的负载，低速确认速度帧为 CAN 0x206 / DLC4、速度正负方向正确，且 0x11 反馈持续更新。
- 验证使能→斜率启动→停机、故障、断线、超时、重新使能，观察每次控制停止后无旧命令追赶。
- 注意相对速度控制**不能单独抵消底盘自转**；启用自动瞄准或大 Yaw 自稳之前需要专门的底盘陀螺前馈、世界闭环及实车验收。
- 编译：`west build -b dm_mc02/stm32h723xx applications/sentry_chassis`；原生测试：`west build -b native_sim/native/64 tests/sentry_control`（在完整 west 工作区中设置 Zephyr 及额外模块路径）。

## 后续优先项

1. 完成四轮驱动实际出力排障和带载标定（几何、方向、机械零位、转子/轮轴单位），再谈更激进的驱动性能优化。
2. 按真实电机模式选用原生速度或 MIT 后端，记录闭环阶跃数据。
3. 小 Yaw 的外置 IMU 世界保持与大 Yaw 相对编码器回中可借鉴参考控制分工，但先确认两颗 IMU 安装在大/小 Yaw 的实际位置与参考基准，避免将独立零点相减。
4. 大 Yaw 编码器 25 rad 回绕在当前 Motor 层已有对应解包依据（2×PMAX），但**丢帧跨度超过半量程不能可靠还原历史圈数**，绝对角行进坐标不应在此情况下偷偷复用旧位置。
