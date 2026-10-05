# 统一速度与位置电机控制

VelocityMotor 和 PositionMotor 接收最新目标，Motor/CAN 负责每轴协议执行和自动恢复。Group 只批量启停。初始化使用 attach → CanBus::start → configure，随后每周期持续 enable/update/commit，不等待物理电机上线。

```cpp
if (run_requested) {
    const int enabled = drive.enable();
    const int accepted = axis.update(target_rad_s, dt_s);
    // enabled/accepted 的调用错误记录到诊断，不撤销其他轴。
} else {
    (void)drive.disable();
}
const auto published = bus.commit();
const auto telemetry = axis.telemetry();
```

update 返回 0 表示合法目标已接受。离线、故障、协议恢复和参考等待不拒绝目标，target_sequence 继续增加，output_valid 表示本周期计算是否有效。configure 校验控制能力、单位、PID 及数值范围，并绑定唯一 producer 身份；没有 MotorSafety、ControlFailurePolicy 或 preflight。

恢复后的首个可用周期从实际反馈自动初始化本轴 PID 并提交带执行代次的零 effort。计算输出使用同一份计算快照的 enable_generation；下一周期正常闭环。有限异常 dt 跳过本次积分，不锁存人工复位；负数及非有限 dt 返回调用错误。

| 位置模式 | 目标 | 参考要求 |
|---|---|---|
| AbsoluteNearest | 单圈绝对目标的最短路径 | 新鲜绝对角与速度，本地展开，不要求驱动多圈参考 |
| DriverContinuous | 驱动连续坐标中的位置 | 必须有可信连续参考 |
| StartupRelative | 首次可信原点加偏移 | 首次原点保留，恢复不能重采 |

reset 只处理本轴 PID 历史，不是输出准入，不能重定义 StartupRelative 原点。可信 reseedPosition 可恢复驱动连续参考；无依据不能猜圈数或自动置零。速度轴不要求位置参考。

保留 requested_velocity_abs_max、PID/effort 限幅与电机电流/力矩范围，删除重复软件温度和实测速锁停。显式停止取消旧命令，恢复不复活停止前目标；命令自身过期也不再输出。

共享 CAN 的所有轴先 update，再由统一发布者每周期 commit 一次。每个控制器一个执行写入线程，Motor/CanBus 对象必须覆盖异步工作线程生命周期。

完整接口、并发版本、逐轴恢复和人工验收见[实施指南](../../dev/电机持续指令与独立自动恢复重构实施指南.md)。样例见 [recovery](../../../samples/motor/recovery/README.md)、[单舵轮](../../../samples/robotics/swerve/README.md)。
