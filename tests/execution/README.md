# 执行恢复与状态生产契约

`RecoveryGate` 测试覆盖新原始输入边界、仲裁重复生产、命令服务停更、Enabling 撤销、独立机构恢复代次和显式清除边界。`SnapshotCache` 覆盖非消耗读取与失败时保留目标变量。真实 `InterBoardEndpoint` 通过立即复制的假传输，把 统一协议编码、解析与会话接通，覆盖执行生产停止但心跳继续时 ready/armed 撤销，以及新的状态生产恢复。

```sh
ZEPHYR_TOOLCHAIN_VARIANT=host/gnu west twister -T tests/execution \
  -p native_sim/native/64 --inline-logs -O build/twister-execution
```

此入口不启动电机，也不代表物理 CAN/UART 已通过实板验收。
