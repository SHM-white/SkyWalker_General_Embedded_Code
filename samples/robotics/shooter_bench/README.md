# 摩擦与拨盘台架

两个 M3508 摩擦轮组成独立 Group，M2006 拨盘独立 Group，应用统一提交 CAN1。默认空载模式，只允许无弹机械验收；所有电机的中央连接确认开关仍关闭。操控和线程关系复用 `../common/shooter_bench.hpp`：先安全档归中解锁，右 Middle 启动摩擦，Middle→Up 单发一次，保持 Up 500 ms 转连续，回 Middle 停供弹，右 Down 停摩擦；安全档手势清故障。console 不接收控制指令。

ShooterExecutor 已实现摩擦速度连续就绪、拨盘位置分度、独立事件去重、恢复丢弃、持续新鲜命令、许可/热量/云台状态约束、目标到达与卡滞超时。失效不排队重放单发。

```sh
west build -p always -b dm_mc02/stm32h723xx samples/robotics/shooter_bench -d build/shooter_bench
west build -p always -b dm_mc02/stm32h723xx samples/robotics/shooter_bench -d build/shooter_loaded -- -DEXTRA_CONF_FILE=loaded.conf
```

loaded.conf 关闭无弹旁路。TODO：接入真实 referee shooter_output 和 IShooterHeatSource，建立独立拨盘索引/机械零点，标定摩擦速度与弹速、热量和卡滞阈值；完整组合中的云台状态由真实执行器提供。未接这些数据时 loaded 配置保持禁止动作，不能通过常量伪造 ready/heat。未进行实板射击验收。

操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。诊断支持场景 1（命令快照暂停）与 2（执行暂停），`diagnostic.conf` 默认 1，真实遥控管理继续运行；普通构建不注入。事件在仲裁观察到对应 RC 帧之后一次消费，摩擦未就绪、供弹忙碌或任何许可不满足时丢弃，不补发。
