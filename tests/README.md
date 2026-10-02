# 测试入口

各目录包含独立 Zephyr 测试应用的 `CMakeLists.txt`、`prj.conf` 和 `testcase.yaml`，可单独 `west build` 或由 Twister 发现。

| 目录 | 范围 |
| --- | --- |
| [algorithms](algorithms/README.md) | EKF、线性卡尔曼、矩阵、控制及机器人算法的正确性与耗时 |
| [command_manager](command_manager/README.md) | 同步命令仲裁与手动映射的独立用例、完整场景及耗时 |
| [command_service](command_service/README.md) | 真实 CommandManager 后台线程、启动生命周期、来源与权限缓存、过期、恢复和双线程独立消费者 |
| [execution](execution/README.md) | 执行恢复边界、状态缓存、生产年龄与真实 InterBoardEndpoint 链路契约 |
| [vision_imu](vision_imu/README.md) | 视觉与 IMU 数据流集成 |
| [motor/regression](motor/regression/README.md) | 电机模块回归 |

```sh
west twister -T tests/algorithms -T tests/command_manager \
  -T tests/command_service -T tests/execution \
  -T tests/motor/regression -T tests/vision_imu \
  -p native_sim/native/64 --inline-logs -O build/twister-unit-tests
```

计时口径和各套件的独立构建命令见对应目录的 README。

独立的 [Unit tests 工作流](../.github/workflows/unit-tests.yml) 在相关源码、测试、构建配置或工作流发生 Push / PR 变更时运行，也支持手动触发。它使用 Ubuntu 24.04、Python 3.12 和宿主 GCC，按 `west.yml` 固定的 Zephyr 版本构建并执行以上六套测试。同步仲裁套件继续关闭服务线程，后台服务契约在独立套件中执行。

Actions 摘要展示逐套件通过数量及 `BENCH` 耗时，`unit-tests-native-sim-64` artifact 保存 Twister JSON / XML 报告和构建、执行日志，保留 14 天；测试失败时仍会生成摘要并尝试上传已有报告。CI 耗时用于观察宿主机执行情况，沿用各套件的计时口径。
