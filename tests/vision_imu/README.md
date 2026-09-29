# 视觉与 IMU 主流程验收

功能完成后的单项整体验收，使用真实模块实现和一个模拟 PWM 设备。覆盖两实例 EKF 初始化/旋转/动态加速度/长间隔重置、手册 DM 已知帧和合成四元数流转、AB 拆包/粘包/错误恢复/停止命令、反馈编码、各字段独立过期与温度过期撤销 PWM。

~~~bash
west build -b native_sim/native/64 tests/vision_imu -d build/vision_imu_flow
build/vision_imu_flow/zephyr/zephyr.exe
~~~

不验证真实 UART、RS485 收发器、CAN、SPI、加热电路、安装方向或目标设备固件；板级编译和实物验收见两个新样例。
