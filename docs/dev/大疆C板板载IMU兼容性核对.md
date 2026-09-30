# 大疆 C 板板载 IMU 兼容性核对

日期：2026-09-30。根据后续授权，已将 MC02/C 板温控配置放入默认 DTS 并统一 IMU alias 调用。本次仅静态复核，未构建、烧录或实测。

## 结论与依据

标准 RoboMaster C 型板使用 BMI088 六轴 IMU，另有 IST8310 磁力计。当前 Bmi088Imu 可复用到 C 板，ImuReceiver 和 QuaternionEkf 不绑定达妙开发板。但当前 dual_imu 整个样例绑定 MC02，不能原样换目标板运行。

官方依据：[RoboMaster 开发板 C 型用户手册](https://rm-static.djicdn.com/tem/35228/RoboMaster%20%20%E5%BC%80%E5%8F%91%E6%9D%BF%20C%20%E5%9E%8B%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf)，六轴惯性测量单元与附表。适用假设为原版 C 型板或硬件连接一致的板卡。

## 项目现状

- boards/rm_typec/rm_typec.dts 已启用 SPI1 下的 bmi08x_accel、bmi08x_gyro，提供 accel0/gyro0 alias；加速度片选 PA4，陀螺仪片选 PB0。
- boards/rm_typec/rm_typec_defconfig 已开启 SENSOR、SPI、BMI08X。
- Bmi088Imu 构造接收两个 Zephyr sensor device 指针，不写死 SPI 控制器或引脚。C 板可沿用现有采样、温度读取和可选六轴 EKF。
- ImuReceiver 接收 ImuSource 引用及可选 ImuHeater 指针，start 启动后台采集；snapshot 只复制测量并计算有效期。实例、数据源与估计器必须长期存活，不能再从另一线程调用源的 init/service。
- 默认板级 DTS 已定义温控 PWM：C 板 TIM10_CH1/PF6、MC02 TIM3_CH4/PB1，周期均为 20ms，统一提供 imu-heater alias。两个 IMU 样例已移除 overlay 中重复的温控定义，并改用 PWM_DT_SPEC_GET(DT_ALIAS(imu_heater))。
- dual_imu 的 app.overlay、rs485-2、DMA 和 sample.yaml 均绑定 MC02；C 板遥测 alias 当前指向 USART6。
- 当前姿态估计只使用加速度和角速度，没有接入 IST8310，不提供磁航向或绝对 yaw，yaw 长期可能漂移。

## 后续验证与扩展

1. 两个 IMU 样例现已通过 DEVICE_DT_GET(DT_ALIAS(accel0)) 和 DEVICE_DT_GET(DT_ALIAS(gyro0)) 获取板载传感器。可先用 samples/imu_test 验证 C 板的单 IMU 路径；该入口直接调度 ImuSource。需要后台采集时沿用双 IMU 样例中的 ImuReceiver 组合方式。
2. 初始化后用 receiver.status() 查看 init_complete/init_error，读取 snapshot.fresh_mask 确认数据是否有效；start 返回 0 不表示已有有效姿态。静置等待 EKF 初始化，绕各轴转动，核对 sensor_to_body 安装变换；不能默认与 MC02 的板坐标一致。
3. C 板 TIM10_CH1/PF6 的 PWM 与 imu_heater 节点已经在默认 DTS 配置，无需再加温控 overlay。温控 PID、功率响应仍需在 C 板重新验证，不把 MC02 参数视为已验证值。
4. 样例按板拆分 overlay 与硬件配置；C 板输出使用自身 telemetry-uart alias。若还需外置 RS485，应单独确定 UART、外部收发器和 DMA，不能复用 MC02 的 rs485-2 接线假设。

预期数据流：C 板 BMI088 → Zephyr BMI08x sensor → Bmi088Imu/可选 EKF → ImuReceiver.snapshot → 应用；有效温度 → 可选 ImuHeater → C 板 PWM。

## 验收清单

- [ ] C 板目标构建成功；本次未执行构建。
- [ ] 静止加速度模长接近重力加速度、角速度接近零、温度合理。
- [ ] EKF 初始化后姿态有效，转动方向和安装变换符合实际。
- [ ] 检查采样间隔、栈余量和 F407 上 EKF 负载，确认实际频率。
- [ ] 启用温控前确认 PF6 接线与输出极性，实测升温、稳态和异常关断。

尚未验证：当前源码在 C 板上的构建与实物运行、温控参数、实际采样频率和安装方向。

## 两板统一 alias 的实施结果

用户已为 MC02 添加 accel0/gyro0，与 C 板保持一致。本次保留这些改动，并为两板新增 imu-heater alias：

```cpp
DEVICE_DT_GET(DT_ALIAS(accel0))
DEVICE_DT_GET(DT_ALIAS(gyro0))
PWM_DT_SPEC_GET(DT_ALIAS(imu_heater))
```

samples/imu/dual_imu/src/board_config.hpp 与 samples/imu_test/src/main.cpp 已使用以上入口。Bmi088Imu、ImuReceiver 和 ImuHeater 无需修改。温控硬件与周期由默认 DTS 提供，应用仍负责目标温度、PID 与开始更新；定义节点本身不启动温度闭环。

MC02 延续原温控 prescaler=99；C 板 TIM10 使用 prescaler=167。已静态核对 Zephyr 的定时器节点和引脚标签，未运行构建或进行实物温控验证。双 IMU 样例的外置 RS485/DMA 仍需针对 C 板另外适配。
