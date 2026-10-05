# MotorSession 删除后的迁移说明

MotorSession、SessionState/Phase/Cause、RecoveryAction 和重复 ReferencePolicy 已从现行实现删除，不保留外壳或兼容 API。

原三个入口 dji_speed_control、dji_position_control 和 swerve/dual_speed 直接使用 Motor、控制器和 CanBus。启动完成 attach/start/configure 后持续提交 enable/update，每个物理 CAN 周期末 commit。演示目标时间轴不因设备异常归零。

Group 只负责批量操作；各 Motor 独立等待、清错和重新使能。目标接收与输出有效性分别观察，离线 update 返回 0，恢复首周期自动重置本轴历史。用户停止取消停止前目标。

dual_speed 的绝对中心仅在明确新启动时捕获，运行中恢复不重采。StartupRelative 的首次原点与 PID 历史分开，可信多圈参考丢失不能自动置零。

完整实施与调用方清单见[电机持续指令与独立自动恢复重构实施指南](电机持续指令与独立自动恢复重构实施指南.md)。旧 Session 教程不再适用于当前代码。
