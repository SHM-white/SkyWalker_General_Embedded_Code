# CommandManager 后台服务测试

此套件启用真实 `CONFIG_SKYWALKER_COMMAND_SERVICE`，不替换后台仲裁线程。来源和 Manager 均具有静态生命周期，失败启动也不销毁或重启。

覆盖以下服务契约：

- 启动前快照不可用、重复角色与权限绑定、配置错误、缺失必需来源、来源或权限启动失败，以及失败后的注册和重启拒绝。
- 来源返回 `-EAGAIN` 时保留原始样本时间；后台线程继续发布命令并在来源过期后禁用输出。
- 两个真实消费者线程分别读取 `current()` 和 `snapshot()`，均可反复取得完整值，不竞争消费消息。
- 来源硬错误、错误 variant 与权限错误使缓存失效；新有效样本使仲裁输出恢复。
- 权限 `-EAGAIN` 不刷新原始时间，权限过期独立于仍然新鲜的操作输入。

```sh
west twister -T tests/command_service -p native_sim/native/64 \
  --inline-logs -O build/twister-command-service
```

这些测试验证内核线程和缓存契约；不代表实板执行器、电机或 UART 已通过验证。
