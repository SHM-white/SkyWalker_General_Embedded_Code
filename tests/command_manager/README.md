# Command manager 完整流程检查

纯值类型的主机功能检查，不依赖 Zephyr、串口或电机。覆盖手动、自动、新帧门禁、抢占和交回、视觉开火授权、逐机构否决、视觉停止/超时/序号倒退、权限失效、遥控离线及时间倒退。

```sh
c++ -std=c++20 -Wall -Wextra -Werror -Iinclude lib/robotics/command.cpp tests/command_manager/scenario.cpp -o /tmp/skywalker-command-scenario
/tmp/skywalker-command-scenario
```

真实接收和输出使用 `samples/robotics/command_manager/README.md` 的台架步骤验证。
