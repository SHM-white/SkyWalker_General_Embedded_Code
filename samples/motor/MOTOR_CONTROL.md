# 电机控制接口

`Motor` 保存用户运行意图与最新命令，实际电机状态通过快照观测。上层持续提交目标，协议握手、清错和恢复由对应物理 CAN 工作线程处理。

```cpp
int ret = bus.attach(drive);
if (ret == 0) ret = bus.start();
if (ret == 0) ret = axis.configure();
if (ret < 0) return ret; // 初始化配置错误。
for (;;) {
    if (requested) {
        recordCallError(drive.enable());
        recordCallError(axis.update(target, measured_dt_s));
    } else {
        recordCallError(drive.disable());
    }
    recordCallError(bus.commit().error);
    sleepUntilNextCycle();
}
```

示意辅助函数只记录错误和安排周期，不撤销请求。`enable()`、合法 setter 和控制器 `update()` 在离线期间继续接受请求。成功返回表示接受请求；实际输出通过快照和 `Telemetry::output_valid` 判断。

## 控制封装

`VelocityMotor`、`PositionMotor` 的配置保留 C 控制环参数和显式 `effort_unit`。Ampere 对应电流，NewtonMeter 对应力矩，范围不能超出电机配置。没有重复的温度／实测超速锁停，也没有 FailurePolicy、preflight 或 MotorSession。

第一次取得可执行反馈和恢复后的第一周期自动初始化本轴 PID，先输出零 effort，随后跟踪最新目标。过期反馈、缺字段或有限但不适合积分的周期只使本轴输出等待；下一正常周期自动继续。NaN、Inf、非法目标和配置是调用错误。

遥测 `target_valid/target_sequence` 表示目标接收，`output_valid` 表示本周期计算输出。不能把离线等待当作上层启动失败。

## 位置参考

- `AbsoluteNearest`：使用可信绝对角和速度，局部展开只服务当前连续反馈区间，恢复后自动重建计算坐标。
- `DriverContinuous`：使用真实连续位置；失联导致参考丢失时只等待本轴可信 `reseedPosition()`。
- `StartupRelative`：首次可信位置建立固定原点；停止、PID reset 或恢复均不重采原点。

`reset()` 是可选的本轴 PID 历史作废操作，启动和恢复不需要调用；它不会更换相对原点。可信多圈坐标不能从短暂缺失的反馈中凭空恢复。

## Group 与发布

Group 只提供批量 enable/disable 和成员状态统计，没有全员 ready、全员 active 准入或成员故障联动。可以单独控制组内成员。

每个物理 CAN 的控制发布者在一个周期内更新所有目标后 commit 一次；一个成员等待不能跳过其他成员。Recovering 时 commit 继续覆盖最新发布，恢复不重放历史队列。

显式停止、用户急停和输入源过期仍取消运动意图。电机掉线不撤销输入授权。
