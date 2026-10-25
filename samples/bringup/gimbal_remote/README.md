# gimbal_remote

仅观察DR16，不注册任何执行器。安全档归中解锁状态机运行但不产生动作。

固定使用 [实车配置](../../../config/vehicle/installed/README.md)。独立入口只启动本阶段对象；UART7/PE8 TX 接 DAPLink RX，共地，115200 8N1，现场接线尚需确认。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_remote -d ../build/bringup/gimbal_remote
west flash -d ../build/bringup/gimbal_remote
```

仅修改 `include/robotics/vehicle/diagnostics_config.hpp` 中 `diagnostic_output`，重编译烧录选择文本/VOFA。VOFA 使用 JustFloat，固定19通道、布局版本1：layout、sequence低16、高16、uptime_ms低16、高16、reason、valid、enabled、target、feedback、age_ms、dt_us、effort、detail、error、dropped低16、高16、extra0、extra1。valid=0时feedback为NaN，age=-1表示未知。reason编号见 [共享说明](../README.md)，未定义字段为0。

target=右X、feedback=右Y、effort=拨轮、extra0/1=左X/Y，均为原始DR16单位。detail=左开关枚举×4+右开关枚举，Unknown/Up/Middle/Down=0/1/2/3。

文本启动身份包含源码/配置/标定/协议版本。未上车验证；编译不代表接线、极性、机械限位已确认。现场确认结果记录到上车总表，日志/波形保存电脑；不需要目标板USB。
