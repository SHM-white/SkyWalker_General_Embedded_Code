# 12 故障排查速查

按「构建 → 链接 → 烧录 → 运行 → 硬件」顺序排查。每类给出**现象 / 原因 /
处理**三列。

---

## 1. 构建与配置

| 现象 | 原因 | 处理 |
|---|---|---|
| 改了某驱动却完全没生效 | 对应 `CONFIG_SKYWALKER_*` 未开，源文件没进构建 | 在 `prj.conf` 打开 Kconfig，并在 `build/.../zephyr/.config` 里确认 |
| 设备树节点被忽略 | `status` 非 `okay`，或 compatible 不在 binding | 检查 overlay 与 `dts/bindings/` |
| binding 编译报错 | 属性类型/必填不符 | 对照 `dts/bindings/motor|imu|kalman_filter/*.yaml` |
| IMU 温控参数无效 | `heat-*` 写成了整型 `<...>` | 必须写成字符串 `"..."` |
| `west build` 找不到板卡 | 不在工作区根执行，或 module 未挂载 | 在 `skywalker_ws/` 下构建；确认 `zephyr/module.yml` 生效 |
| CMake 报找不到源文件 | `CMakeLists.txt` 里的文件名与实际不一致 | 核对 `zephyr_library_sources(...)` 列表 |

---

## 2. 链接与 C++

| 现象 | 原因 | 处理 |
|---|---|---|
| `undefined reference to __device_dts_ord_N` | `DT_INST_FOREACH_STATUS_OKAY` 宏放在 C++ namespace 内，符号被名称修饰 | 设备实例化宏放**全局作用域**，宏体内用全限定名引用 namespace 内的类型 |
| C++ 样例链接 `-lstdc++`/异常相关失败 | 未开 C++/libcpp 配置 | `CONFIG_CPP=y`、`CONFIG_STD_CPP20=y`、`CONFIG_REQUIRES_FULL_LIBCPP=y` |
| 找不到 `arm_mat_*`/`arm_sqrt_f32` | CMSIS-DSP 未选 | `CONFIG_SKYWALKER_LIB_MATRIX=y` / `DRIVER_IMU=y`（会自动 select） |

---

## 3. 烧录

| 现象 | 原因 | 处理 |
|---|---|---|
| `no runners.yaml found` | 板卡缺 `board.cmake`（历史问题，现已修复） | 确认 `boards/*/board.cmake` 存在；重建构建目录 |
| 以为 `dm_mc02` 默认 pyocd，实际用 openocd | `board.cmake` 第一个注册的是 openocd | 以 `build/zephyr/runners.yaml` 的 `flash-runner` 为准；需要 pyocd 用 `-r pyocd` |
| OpenOCD `Error: open failed` | WSL 下调试器未透传 | `usbipd attach --wsl --busid <id>`（透传不持久，重启后重做） |
| 普通用户打不开 USB 设备 | `/dev/bus/usb` 权限 | 配置 udev 规则，或临时 `sudo chmod 666 /dev/bus/usb/*/*` |
| ST-Link 连不上 | 探针配置不匹配固件版本 | 用 `-DSKYWALKER_OPENOCD_PROBE=stlink` 或 `stlink-hla` |
| `rm_typec` 无法烧录 | 无板载调试器，需外接 | 用 CMSIS-DAP 或 ST-Link + `west flash -r openocd` |

详见 [01 快速开始](01-getting-started.md) 与 [03 板级支持](03-boards.md)。

---

## 4. 运行期（通用）

| 现象 | 原因 | 处理 |
|---|---|---|
| `FATAL ERROR ... Stack overflow`，`thread=main` | 默认主栈 1024 太小（C++ / 长 `%llu` 打印） | `CONFIG_MAIN_STACK_SIZE=8192` |
| `device_is_ready()` 为 false | 依赖设备未就绪/未初始化 | 检查 overlay 的 phandle 与 Kconfig |
| 电机一直 Offline | 没收到反馈帧 | 核对 CAN 波特率、ID、接线、`can-bus` 节点 |
| `setCurrent` 返回 `-EHOSTDOWN` | 反馈超时 | 提高 `FEEDBACK_TIMEOUT_MS` 或修总线 |
| 返回 `-EACCES` | 未 arm | 先 `arm()`/`begin()` |
| `begin()` 返回 `-ERANGE` | 设备树限幅超协议满幅/超温上限 | 改小 `current-limit-ma` / `torque-limit` |
| 重复 `begin()` 返回 `-EALREADY`，第二个实例 `-EBUSY` | 生命周期设计 | 一个 backend 只能绑定一个运行实例；失败不自动恢复 |
| 更新返回 `-ERANGE` | dt 超范围（如断点暂停 > 20 ms）或越界 | 去掉断点；检查控制周期与 `dt_max_s` |
| 遥测 `valid=false` | 首次 update 尚未成功，或已失败/stop | 不要使用 stop 后的旧输出 |

---

## 5. DJI 电机

| 现象 | 原因 | 处理 |
|---|---|---|
| GM6020 设备初始化失败 | 未声明 `current-loop-confirmed;` 或固件 <1.0.11.2 | 补属性 + 升级固件 |
| 多电机互相打架 | 每个电机各自建 Bus | 同一 CAN 上的多个电机共用一个 `dji::Bus` |
| M2006 开温度保护后启动失败 | M2006 无温度反馈 | 关闭温度保护 |
| 位置首帧为 0、不是机械零点 | `position_rad` 是相对量 | 固定零点用 GM6020 的 `absolute_position_rad` |
| 电流被拒 `-ERANGE` | 超 `current-limit-ma` | 调大（≤ 协议满幅）或降低指令 |
| 速度/位置数值差一个减速比 | 重复除减速比 | 反馈**已换算到输出轴**，不要再除 |

---

## 6. 达妙 DM 电机

| 现象 | 原因 | 处理 |
|---|---|---|
| 上电绿灯一秒后失能 | 电机未收到使能/反馈握手 | 先 CAN 后 XT30；参考 `DM-J4310绿灯一秒后失能排查指南.md` |
| 反馈收不到 | `master-id` 与电机配置不一致 | 与达妙调试助手核对 `motor-id`/`master-id` |
| 命令返回 `-ENOTSUP` | `control-mode` 与调用不匹配 | MIT 才能 `setMitCommand`/`setTorque`，速度用 `setVelocity` |
| 位置/速度/力矩数值差很多 | PMAX/VMAX/TMAX 与电机实际不符 | 用调试助手读取并写回 overlay |
| 多圈位置跳变 | 反馈非 ±PMAX 回绕 | 确认固件回绕行为；相邻消费间隔运动要小于 PMAX |
| MC02 上电机不上电 | XT30_1 未开 | 样例传入 `enableMotorPower`，CAN 初始化后再开 |
| 想用模式 4 混合控制 | 驱动未实现 | 暂不支持，改用现有三模式 |

详见 [05 达妙 DM 电机驱动](05-drivers-motor-dm.md) 与
`samples/motor/DM_J4310_EXAMPLES.md`。

---

## 7. IMU / EKF

| 现象 | 原因 | 处理 |
|---|---|---|
| 姿态漂移/不收敛 | 未静止、震动大 | 上电静止数秒；加速度 LPF 与卡方抗扰动需稳态 |
| 运行越界/数据错乱 | `filter-dev` 维度不是 4/3 | 改 `state-dim`/`measure-dim` |
| 加热不工作 | 通道/周期/参数错误 | 通道 4、周期 20 ms；`heat-output-max ≤ 20000000` ns |
| yaw 突然从 π 跳 -π | 输出是 wrap 的 | 应用层自行解包（内部 `YawTotal` 未对外） |
| 栈不足 | EKF/矩阵用 VLA | 调大主栈/相关线程栈 |

详见 [06 IMU 与 EKF](06-drivers-imu.md)、[07 Kalman 与矩阵库](07-kalman-matrix.md)。

---

## 8. 文档与路径

| 现象 | 原因 | 处理 |
|---|---|---|
| README 里 `motor_docs/DOWNLOAD_LINKS.txt` 打不开 | 旧路径 | 实际在 **`docs/DOWNLOAD_LINKS.txt`**（本次已修正） |
| 找不到 `motor_demo/`、`motor_demos/` | 目录已不存在 | 早期留档目录已移除；以 `samples/motor/` 为准 |
| `application/` 里没内容 | 它是空占位模板 | 自行填充 `src/main.c`；不属于模块构建 |

---

## 9. 仓库内的专题排查指南

根目录另有几篇针对具体问题的实战记录，可与本文互补：

- `DM-J4310绿灯一秒后失能排查指南.md`
- `XT30输出开启条件分析指南.md`
- `Zephyr调试跳入底层函数排查指南.md`
- `达妙电机接口实现指南.md`
- `SkyWalker_舵轮控制架构_分阶段施工指南.md`

---

## 10. 安全提醒

任何电机故障排查都必须先满足：**输出轴悬空 / 负载脱开，且人手能立刻物理
断电**。`stop()` 只是撤销命令，不是机械制动，也不切断动力电源。
