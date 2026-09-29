# GimbalAxis 单轴封装实施记录

已先调整文档，再完成源码迁移。用户明确仅为本次任务禁用古法编程模式；不修改仓库政策，下一任务恢复。

## 本次范围

合并样例 Axis 与 YawGimbal 为 GimbalAxis，私有拥有 PositionMotor。提供 begin、poll、reset、update、updateRate、targetAngleRad、telemetry；AxisCommand 使用 target_rad/rate_rad_s。

Motor、Group、CanBus、命令来源/时间戳、安全拨杆、急停与重新授权保留在外层。本次不实现 Gimbal 或 DualYawCoordinator；旧分层方案中的对应内容为未来设计。

参考初始化采用 Preserve（默认）或 CalibratedFeedback（显式）。Limited 双轴样例沿用已校准反馈重建参考；Continuous 调用方使用 Preserve。poll 区分 ready_for_enable 与 feedback_healthy，记录 ready_since_ms，运行时不自动重建参考。只有成功 begin 后才允许准备参考。

## 实施顺序

1. 新建 gimbal_axis.hpp/.cpp，迁入单轴目标算法、健康检查与参考准备，保持私有 PositionMotor 地址稳定，删除复制/移动。
2. 迁移 gimbal_control、yaw_gimbal 台架、sentry_gimbal 三处调用方及配置，替换 CMake 源文件；保留现有安全和总线时序。
3. 移除旧 YawGimbal 接口，更新当前主题文档；历史指南标明过期，不冒充现行接口。
4. 完成功能后集中构建三个调用方及项目 applications/samples 门禁，适用时执行现有 motor 回归；不新增细粒度测试。

## 行为边界

保持 Rate/Hold/AbsoluteAngle、目标限速、有限角限位与连续角环绕。反馈检查统一拒绝过期、无效/非有限位置和 Limited 越界；参考来源选择同时校验 valid 与有限值，避免选择带有效位的 NaN。reset/update 错误由调用方撤销输出，不在类内自动使能或提交 CAN。

禁用期间 poll 可按显式策略 reseed；Active 时 ready 为 false 不表示反馈异常。恢复时间戳保持原语义；应用现有 > 与 >= 比较不改变。reset 不等于 reseed；控制方法由原执行器线程串行调用。

## 当前接口与调用示例

实现文件：[gimbal_axis.hpp](../../include/robotics/gimbal/gimbal_axis.hpp)、[gimbal_axis.cpp](../../lib/robotics/gimbal_axis.cpp)。旧 yaw_gimbal.hpp/.cpp 已移除，三个仓库内调用方全部迁移。

```cpp
static GimbalAxis yaw(yaw_drive, board_config::yawMotorConfig(), board_config::yaw);
// 所需总线 attach/start 成功后，检查 begin 返回值。
int ret = yaw.begin();
// 每个控制周期获取状态；授权与新帧检查仍在外层。
const auto state = yaw.poll(now_ms);
// 满足授权且 state.ready_for_enable 后：reset -> Motor/Group.enable。
// Active 后用 updateRate(rate, dt) 或 update(AxisCommand, action, dt)。
// 更新失败立即撤销输出；成功由应用协调 CanBus.commit。
```

| 接口 | 返回/副作用 |
| --- | --- |
| validate | 配置和位置参考类型校验；0、-EINVAL 或 -ENOTSUP |
| begin | 配置私有 PositionMotor，不启停电机；重复成功配置为 -EALREADY |
| poll | 返回状态副本；未配置 -EACCES；反馈过期 -EAGAIN、数据/参考缺失 -ENODATA、非有限位置 -EINVAL、Limited 越界 -ERANGE；参考重建错误透传 |
| reset | 先检查反馈，再清控制历史并建立目标；Active/Enabling 返回 -EBUSY，不重建驱动坐标 |
| update / updateRate | 更新目标并调用位置环；成功只暂存输出，失败由调用方停机；角度 rad、速率 rad/s、dt 秒 |
| telemetry / targetAngleRad | 返回位置控制器遥测副本或当前目标；目标角不是实时反馈角 |

poll 的 error 不是驱动锁存故障；Active 时 ready_for_enable=false 且 error=0 可以是正常状态。电机速度/温度等更完整的限制仍由 PositionMotor/Motor 校验，feedback_healthy 不代表所有输出许可都满足。

构造时按成员顺序保存 Motor 引用、构造 PositionMotor、复制云台配置。禁止复制/移动，保持绑定的控制器地址稳定；Motor 必须存活更久。主循环串行调用控制方法，没有增加线程或改变总线发布策略。

## 验证记录

- 三个迁移调用方在 dm_mc02/stm32h723xx 上编译链接通过：gimbal_control、yaw_gimbal、sentry_gimbal。
- 原有 tests/motor/regression：14 项通过。
- /tmp 整体流程模拟：沿用现有 Motor/CanBus 夹具，增加一个完整的 GimbalAxis 流程，15 项通过（原 14 项 + 该流程）。覆盖未配置拒绝、参考准备且不重复 reseed、reset、Active 下健康/ready 区分、Rate/Hold/AbsoluteAngle、越界拒绝、反馈丢失、重建参考、恢复后仍保持禁用。没有向仓库新增细粒度测试。
- 全量 applications/samples：24 个项目，15 个通过，9 个失败，未达到全绿门禁。失败均为未改动样例引用 USB CDC UART 时缺少 `__device_dts_ord_220`，对应 `/soc/usb@40040000/cdc_acm_uart0`；这些项目均未启用 CONFIG_SKYWALKER_ROBOTICS_GIMBAL。
- 从原始 HEAD `3fd4af1` 导出独立源码和模块，在 /tmp 单独构建 DR16，复现相同未定义符号。没有扩大范围修复 USB/板级配置。基线日志：/tmp/skywalker-gimbalaxis-baseline-dr16.log。
- git diff --check 与新文件空白检查通过。

验证产物位于 /tmp：skywalker-gimbalaxis-first-build.log、skywalker-gimbalaxis-gate-results.json、skywalker-gimbalaxis-motor-regression.log、skywalker-gimbalaxis-flow.log。临时完整流程源码在 /tmp/skywalker-gimbalaxis-flow-source，可用以下命令重跑：

```sh
west build -b native_sim/native/64 /tmp/skywalker-gimbalaxis-flow-source -d /tmp/skywalker-gimbalaxis-flow-build
/tmp/skywalker-gimbalaxis-flow-build/zephyr/zephyr.exe
```

模拟使用现有测试夹具注入驱动状态，不验证实际 CAN 时序或机械运动。未进行实机验证；机械零点、pitch 占位限位和世界坐标/大小 yaw 联动不在本次验证范围内。


全量门禁未通过的项目：

- `samples/communication/dr16`
- `samples/motor/dji_position_control`
- `samples/motor/dji_speed_control`
- `samples/motor/dm_mit_control`
- `samples/motor/dm_mit_position_control`
- `samples/motor/dm_mit_velocity_control`
- `samples/motor/dm_position_control`
- `samples/motor/dm_velocity_control`
- `samples/motor/m2006_speed_control`

本次只禁用古法编程模式以完成此重构，未修改 AGENTS.md 或技能政策；下一任务恢复原模式。
