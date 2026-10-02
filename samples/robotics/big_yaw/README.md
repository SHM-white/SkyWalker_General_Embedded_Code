# 大 Yaw 连续速度样例

默认不输出。先完成 `include/robotics/vehicle/calibration.hpp` 的接线确认、DM MIT 参数、减速比、方向和速度环标定，再设置 `connections_confirmed`。大 Yaw 使用 CAN3，独立速度内环与恢复代次，不依赖固定绝对机械零点。

控制台 `a` 申请低速运行，`d` 禁用，`+`/`-` 改方向，`p` 暂停原始命令生产，`s` 暂停执行生产，`x` 急停，`r` 显式清除。停止生产后检查撤销延迟，恢复后需新生产的命令。TODO(hardware)：连续旋转接线、限速/力矩、反向、断流恢复和停机延迟需要台架记录。

```sh
west build -b dm_mc02 samples/robotics/big_yaw -d build/big_yaw
```
