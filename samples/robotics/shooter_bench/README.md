# 摩擦与拨盘台架

两个 M3508 摩擦轮、M2006 拨盘分别组成摩擦 Group 和拨盘 Group，应用每轮统一提交 CAN1。默认无弹模式，连接/方向/速度读取现有 `include/robotics/vehicle/calibration.hpp`。

安全档左 Down/右 Down、双摇杆与拨轮归中保持500 ms，再拨左 Middle解锁 RC：右 Middle 预热，右 Up 直接请求连发，回 Middle 停供弹，Down 关闭摩擦。已解锁后左 Up 切入键鼠：按住右键开摩擦，左键普通点击立即请求单发；相邻按下小于500 ms从第二次点击起连发，点击间隙保持，最后一次按下满500 ms退出；持续按住达到250 ms后连发，释放退出。右键释放优先撤销，来源切换/失联/清故障后需先释放左键。console不接收控制指令。

快速连点、长按和 RC 共用同一射频。单发使用位置控制并按真实误差/速度/稳定时间判断到位；连发使用速度控制，正常撤销后受控减速，不积累旧位置目标。事件编号和原始戳去重，拒绝或过期不补发。预留次数和机械到位计数均不代表真实弹丸出弹。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/shooter_bench -d /tmp/skywalker-shooter-build
west build -b dm_mc02/stm32h723xx samples/robotics/shooter_bench -d /tmp/skywalker-shooter-loaded -- -DEXTRA_CONF_FILE=loaded.conf
```

loaded.conf关闭无弹旁路。尚需接入真实 referee shooter_output、IShooterHeatSource和独立拨盘参考；缺失数据保持禁止供弹。未进行实板射击验收，不自动刷写或运行固件。

调参注释与19通道 VOFA 布局见 [共享 CAN 样例](../gimbal_shared_can/README.md)，本入口没有云台，瞄准遥测中的云台测量无效。操作与清故障见 [统一遥控操作](../common/REMOTE_CONTROL.md)。诊断支持场景1/2，真实操作门控持续监督输入。
