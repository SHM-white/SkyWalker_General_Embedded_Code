# chassis_io

无执行器、无板间、无遥控接收线程的板级诊断入口。设备树可保留外设声明，不注册电机、不启用电机供电。

固定使用 [实车配置](../../../config/vehicle/installed/README.md)。独立入口只启动本阶段对象；UART7/PE8 TX 接 DAPLink RX，共地，115200 8N1，现场接线尚需确认。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/chassis_io -d ../build/bringup/chassis_io
west flash -d ../build/bringup/chassis_io
```

仅修改 `include/robotics/vehicle/diagnostics_config.hpp` 中 `diagnostic_output`，重编译烧录选择文本/VOFA。VOFA 使用 JustFloat，固定19通道、布局版本1：layout、sequence低16、高16、uptime_ms低16、高16、reason、valid、enabled、target、feedback、age_ms、dt_us、effort、detail、error、dropped低16、高16、extra0、extra1。valid=0时feedback为NaN，age=-1表示未知。reason编号见 [共享说明](../README.md)，未定义字段为0。

target/feedback保持0；valid表示本地循环在运行。

文本启动身份包含源码/配置/标定/协议版本。未上车验证；编译不代表接线、极性、机械限位已确认。现场确认结果记录到上车总表，日志/波形保存电脑；不需要目标板USB。
