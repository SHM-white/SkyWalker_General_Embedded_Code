# M2006 输出轴速度与 CAN 诊断

当前入口使用 M2006/C610、CAN1、ID1、36:1 减速比。输出轴目标启动前 100 ms 为 0，之后为 5 rad/s；控制循环相对休眠 5 ms，使用实际 dt，运行窗口为 300000000 ms（约 83.3 小时）。初始化后自行产生运行意图，不需要遥控解锁。修改目标/ID/电流上限应查 `src/main.cpp` 的实际构造和常量。

速度环 PID 为 kp=0.32、ki=0.3、kd=0.008，目标斜坡 100 rad/s²，软件/电机电流上限均为 10 A。这些是当前调试参数，不是任意机构可直接使用的标定结果。源码还留有已注释 M3508 ID3 构造；部分 DIAG 文本写着 M3508 ID3，不能据日志标题推断实际电机，实际构造为 M2006 ID1。

## 输出与判读

`telemetry-uart` 为 USB CDC ACM，文字日志走板级 console。VOFA JustFloat 使用下面的 **0 起始通道索引**，正常循环每次发一帧；USB 未连接或队列满不停止闭环。

| 通道 | 数据 |
|---:|---|
| 0 | 请求速度 rad/s |
| 1 | 斜坡参考速度 rad/s |
| 2 | 实际输出轴速度 rad/s |
| 3 | 滤波速度 rad/s |
| 4 | 速度误差 rad/s |
| 5 / 6 / 7 | PID 的 P / I / D |
| 8 | 前馈 |
| 9 | 本周期有效输出电流 A，无有效输出时为 0 |
| 10 | requested 且 output_valid，0/1 |
| 11 | 反馈年龄 ms |

DIAG 同时记录前后快照、原生 RPM/电流/编码器、反馈时间与 valid、实际 dt、ControlIssue、命令序号、enable_generation、TX/恢复记录、TEC/REC 和 CAN 错误统计。异常详细日志最多每 500 ms 一组，连续 100 ms 无异常后打印 recovered；正常摘要约每秒一次。rx_invalid_frames 增长也可能来自同毫秒重复反馈被拒绝，单独增长不作为控制故障触发。

先确认输入目标与参考，再比较反馈年龄和 dt，最后看 Motor 与 CAN 历史故障。BusState 已 Running 时，last_recovery 保留旧故障属于正常历史记录。Recovered 表示观测条件恢复，不证明 CAN 电气根因已解决。

## 构建

在仓库目录执行：

```sh
west build -p -b dm_mc02/stm32h723xx -d build/m2006-speed samples/motor/m2006_speed_control
```

持续 enable/update/commit 保留最新运行意图；一轴掉线后自行恢复，不使用 ready/clearFault/MotorSession。安全停机须撤销运行意图或物理断电；本样例没有交互式停止入口。电机单位、减速比和恢复语义见[电机工作流](../../../docs/guides/motor-workflow.md)。
