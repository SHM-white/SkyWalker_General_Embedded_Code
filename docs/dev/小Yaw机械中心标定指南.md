# 小 Yaw 机械中心标定

## 当前情况与目标

当前 `include/robotics/vehicle/calibration.hpp` 用 `(yaw_min_rad + yaw_max_rad) * 0.5f` 作为 `yaw_center_rad`，因此大 Yaw 跟随的目标是**软件限位的中点**。改为现场测得的机械中心后，大 Yaw 在其他条件满足时会跟随小 Yaw，使小 Yaw 相对于大 Yaw 趋近该中心；这不是给小 Yaw 直接下一个机械位置目标。遥控松手也不表示小 Yaw 独立地向该位置转动。

数据流：机械中心标定值 → `yaw_center_rad` → `manualCenteringConfig().center_rad` → `YawCenteringController` 计算 `小 Yaw 反馈角 - 中心角` → 大 Yaw 世界角速度请求。正式应用还用板载载体角速度补偿小 Yaw；回中依赖实时遥控授权、板间连接、载体航向和电机反馈有效。

以下以当前仓库正式应用 `applications/sentry_gimbal`、GM6020、传动比 1 和现有软件限位为前提。指定的附件快照在当前环境不可读取，下面的行号和值来自工作区版本；如果实际固件的标定或应用不同，要以现场版本为准。

## 1. 测量机械中心

1. 撤销遥控运行授权并断开电机动力，固定或支撑机构。手动缓慢将云台对准你定义的机械中心，例如小 Yaw 头部与大 Yaw 载体的结构中心线重合；确认该位置在安全行程内。
2. 在保持电机不输出的条件下通电读取有效反馈。正式云台诊断 VOFA 通道 9 是小 Yaw `feedback.position_rad`，单位 rad；通道 6 `valid` 应为 1。记录稳定的读数 `机械中心反馈角`，以及通电启动位置和测量条件。不要把通道 8 目标角或通道 17 中心偏差当作反馈角。
3. 若只能取得 GM6020 原始编码器 tick，工作区当前的 `small_yaw.encoder_zero_ticks` 为 5670，单圈 8192 tick；当测得的 tick 与零点的差在 `[-4096, 4095]` 内，且与限位处于相同连续参考分支时，中心角为 `(实测 tick - 5670) * 6.283185307179586 / 8192` rad。例如**假设**测得 6500 tick，结果约为 0.6366 rad；6500 只是算例，不是本车实测值。驱动绝对角对差值按 8192 环绕到 `[-4096, 4095]`，有限位轴再以该绝对角建立连续位置参考，后续连续位置可能跨圈；跨圈、超过半圈或连续参考分支不确定时不要直接套用差值，应确认启动参考及整个安全行程的坐标一致。

当前软件限位为 tick 5770～7340，即约 0.0767～1.2809 rad；原中点是 6555 tick，约 0.6788 rad。它们是源码配置值，不代表实测的机械安全端点。如果机械中心不在现有软件限位内，先停机核实物理中心、参考分支及安全端点；不要为了通过编译盲目放宽限位。

## 2. 修改中心标定

在 `include/robotics/vehicle/calibration.hpp` 中，保留 `small_yaw.encoder_zero_ticks`、`yaw_low_ticks`、`yaw_high_ticks` 及它们算出的限位，将现有中心定义替换为测量结果，例如：

```cpp
// 实测机械中心的反馈角（rad）；以下数值仅为示例，必须替换为本车读数。
inline constexpr float yaw_center_rad = 0.6366f;
```

如果需要保留 tick 记录，可在确认参考分支后改为：

```cpp
// 6500 仅作演示，替换成机械中心处的实测 tick。
inline constexpr int yaw_mechanical_center_ticks = 6500;
inline constexpr float yaw_center_rad =
    (yaw_mechanical_center_ticks - static_cast<int>(small_yaw.encoder_zero_ticks)) * two_pi / 8192.0f;
```

现有 `static_assert` 要求中心严格处于 `yaw_min_rad` 和 `yaw_max_rad` 之间。不要只改编码器零点：它改变的是整个反馈坐标基准，软件限位也随之变化，无法单独表达回中目标。当前实现不需要修改 `manualCenteringConfig()` 或回中控制器；共享配置也会影响使用该中心的双 Yaw 上车样例。

## 3. 编译与上车确认

完成修改后，可由操作者在项目目录执行正式云台的构建、烧录：

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
west flash -d ../build/applications/sentry_gimbal
```

先支撑机构、清空活动范围并准备好停机；上电后先保持撤权，核对通道 9 在机械中心附近等于所填值。再按照正式应用的解锁流程，以小偏角分别从中心两侧观察：通道 17 应分别异号、在中心附近接近 0；大 Yaw 应使小 Yaw 相对中心的偏差减小。如果偏差增大、机构接近限位或有异常，立即撤权停机，检查安装方向、参考坐标和载体角速度补偿，不要直接扩大限位或提高增益。

当前正式应用的停止死区为 0.003 rad，重新跟随阈值为 0.005 rad，且速度有斜坡；因此停止位置可能落在中心附近，不保证精确等于某一个读数。若大 Yaw 不动，先查授权、板间反馈、载体航向和小 Yaw 反馈是否有效，再查中心参数。此指南没有实测、编译或烧录记录。

## 完成清单

- [ ] 机械中心定义及反馈角已在实车测量，并确认属于限位坐标分支。
- [ ] 将实测值写入 `yaw_center_rad`，确认中心在真实安全行程与软件限位之内。
- [ ] 编译烧录后，撤权核对反馈；从两侧小幅验证偏差收敛和停机。
