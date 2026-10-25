# gimbal_yaw_encoder

只在CAN1注册yaw一台电机、一个机械闭环；另一轴、头部IMU和底盘不参与。只接实际RC，双拨Down且摇杆归中500ms，再将左拨Middle解锁；左Down或遥控断流立即撤销目标。Pitch需机械支撑后低幅操作。

固定使用 [实车配置](../../../config/vehicle/installed/README.md)。独立入口只启动本阶段对象；UART7/PE8 TX 接 DAPLink RX，共地，115200 8N1，现场接线尚需确认。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_yaw_encoder -d ../build/bringup/gimbal_yaw_encoder
west flash -d ../build/bringup/gimbal_yaw_encoder
```

仅修改 `include/robotics/vehicle/diagnostics_config.hpp` 中 `diagnostic_output`，重编译烧录选择文本/VOFA。VOFA 使用 JustFloat，固定19通道、布局版本1：layout、sequence低16、高16、uptime_ms低16、高16、reason、valid、enabled、target、feedback、age_ms、dt_us、effort、detail、error、dropped低16、高16、extra0、extra1。valid=0时feedback为NaN，age=-1表示未知。reason编号见 [共享说明](../README.md)，未定义字段为0。

target/feedback=该关节目标/实际角(rad)，effort=电流A，extra0/1=目标/实际角速度(rad/s)，detail=MotorState，age=RC年龄。独立速度上限见bringup_policy.hpp，PID和真实零点限位仍用中央配置。

文本启动身份包含源码/配置/标定/协议版本。未上车验证；编译不代表接线、极性、机械限位已确认。现场确认结果记录到上车总表，日志/波形保存电脑；不需要目标板USB。
