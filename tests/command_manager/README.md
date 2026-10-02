# CommandArbiter 单元测试与耗时

这里使用标准 Zephyr ztest / Twister 测试入口。原来的 `scenario.cpp` 独立 `main/assert` 测试已经迁入 `src/main.cpp`，完整有状态仲裁流程仍由一个独立用例连续执行。

测试对象是 `ManualCommandMapper` 和同步 `CommandArbiter` 算法，不启动 `CommandManager` 工作线程，也不需要遥控器、串口、裁判系统或电机硬件。

正确性用例覆盖：

- 摇杆死区、归一化、方向、饱和及键鼠映射，使用解析期望值判断数值结果。
- 手动运动限幅、逐机构时间戳与序号、离线/安全/非法输入。
- 遥控器与输出许可的超时临界点、未来时间戳、逐机构否决，以及整体裁判在线不能刷新机构许可。
- 自动模式必须等待新视觉帧；视觉参考、有限数值、序号、时间与速率/加速度限幅。
- 手动抢占、安静等待、交回视觉控制、键鼠开火授权和逐机构禁用的完整状态流。
- 无效配置、时钟倒退及 `reset()` 恢复。

计时用例分别测量 mapper、手动仲裁和视觉仲裁，每项预热 64 次，再运行 2048 次（16 个批次）。公共 `tests/common/benchmark.hpp` 输出批次统计的平均/最小/最大 ns/op；断言与日志位于计时区间之外。计时路径使用固定有效输入，统计包含算法调用与保存结果的开销。`native_sim` 上使用主机单调时钟，得到主机环境数据；板端使用 Zephyr timing API，实际目标还需提供可用的 timing 后端。

在已配置 Zephyr / west 环境的工作区执行：

```sh
west build -b native_sim/native/64 tests/command_manager -d /tmp/skywalker-command-manager
west build -d /tmp/skywalker-command-manager -t run
```

或者使用 Twister 发现并运行：

```sh
python "$ZEPHYR_BASE/scripts/twister" -T tests/command_manager -p native_sim/native/64 --inline-logs -O /tmp/skywalker-command-twister
```

真实接收和输出的台架验证仍参考 `samples/robotics/command_manager/README.md`。
