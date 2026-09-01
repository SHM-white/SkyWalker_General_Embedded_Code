# DJI 统一电机架构说明

## 1. 一句话结论

是的，`Bus` 负责汇总同一条物理 CAN 总线上的 DJI 电机指令并统一发送。

但更准确地说：

- `setCurrent(motor, A)` 只把某一台电机的目标电流写入该电机自己的缓存，不发送 CAN。
- `Bus::flush()` 一次性读取所有已挂载电机的缓存。
- `Bus::flush()` 按 DJI `command_id` 分组；每组最多 4 台电机，打包为一帧 8 字节 CAN 数据后发送。
- 任一电机的反馈或指令过期、状态不对、打包或发送失败，Bus 会进入 `Fault` 并尝试给所有已知分组发送全零帧。

## 2. 四层架构

```text
应用层（sample / 控制器）
  setCurrent(motor1, 1.0A)
  setCurrent(motor2, 2.0A)
  bus.flush()
            |
            v
公共电机接口层 motor.hpp
  屏蔽具体型号，提供 setCurrent/readFeedback/getState
            |
            v
DJI 电机 device 层 dji_motor.cpp + dji_profiles.cpp
  每台电机各有配置、反馈缓存、指令缓存、状态和锁
            |
            v
DJI Bus 层 dji_bus.cpp
  管理同一物理 CAN、生命周期、分组、集中提交和故障归零
            |
            v
协议层 dji_protocol.cpp
  raw 数组 <-> 8 字节 CAN 帧；反馈帧字节 <-> RawFeedback
```

对应文件：

- `include/drivers/motor/motor.hpp`：型号无关的应用接口。
- `drivers/motor/dji/dji_motor.cpp`：单台电机状态与收发缓存。
- `drivers/motor/dji/dji_profiles.cpp`：型号、CAN ID 和安培/raw 换算规则。
- `drivers/motor/dji/dji_bus.cpp`：同一 CAN 总线的唯一发送协调者。
- `drivers/motor/dji/dji_protocol.cpp`：CAN 帧编解码。
- `drivers/motor/dji/dji_*_instance.cpp`：从设备树生成不同型号的 Zephyr device。

## 3. 为什么必须有 Bus

DJI 电机协议不是“一台电机一条控制帧”。一条命令帧包含 4 个 16 位指令槽：

```text
CAN data[0..7]

+---------+---------+---------+---------+
| slot 0  | slot 1  | slot 2  | slot 3  |
| 2 bytes | 2 bytes | 2 bytes | 2 bytes |
+---------+---------+---------+---------+
```

例如 M3508/M2006 的电机 ID 1～4 通常映射为：

```text
command_id = 0x200

motor-id 1 -> slot 0 -> data[0], data[1]
motor-id 2 -> slot 1 -> data[2], data[3]
motor-id 3 -> slot 2 -> data[4], data[5]
motor-id 4 -> slot 3 -> data[6], data[7]
```

如果每台电机各自直接发送 `0x200`，后一台发送的帧会覆盖同组其他槽位的含义，而且多个对象会争用同一个 CAN ID。Bus 的职责就是确保同一命令帧只有一个组装者和发送者。

注意：Bus 是“按组一起提交”，不是保证多帧在物理线上同一时刻出现。如果挂载的电机跨多个 `command_id`，`flush()` 会按顺序发送多帧。

## 4. 一轮控制周期实际发生什么

应用代码通常这样写：

```cpp
setCurrent(motor1, current1_a);
setCurrent(motor2, current2_a);
setCurrent(motor3, current3_a);

FlushReport report{};
const int ret = bus.flush(report);
```

内部数据流是：

```text
1. setCurrent(motor1, 1.0A)
   校验 device/能力/状态/电流上限
   1.0A -> 型号对应的 int16 raw
   写入 motor1.command_raw 和时间戳
   不发送

2. setCurrent(motor2, 2.0A)
   同理，写入 motor2 缓存
   不发送

3. bus.flush()
   对每台 attached motor 调用 snapshotCommand()
   检查 armed、epoch、反馈新鲜度、命令新鲜度
   按 command_id 找组，按 command_slot 填数组
   buildCommandFrame() 把 4 个 int16 写成 8 字节大端数据
   can_send() 每个 command_id 一帧
```

伪代码：

```cpp
commands[group_count][4] = 全零;

for (motor : attached_motors) {
    snapshot = motor.snapshotCommand();
    group = groupIndex(snapshot.command_id);
    commands[group][snapshot.command_slot] = snapshot.command_raw;
}

for (group : groups) {
    frame = buildCommandFrame(group.command_id, commands[group]);
    can_send(frame);
}
```

没有挂载电机占用的槽位保持 0。例如只挂载 motor-id 1 和 3：

```text
0x200 = [motor1 raw] [0] [motor3 raw] [0]
```

## 5. 单台电机 device 负责什么

每台电机都有独立的 `DjiConfig` 和 `DjiData`。

### 静态配置 DjiConfig

来自设备树，包含：

- 所属 CAN device。
- 型号 profile。
- `motor-id`。
- 用户配置的电流上限。
- 减速比的分子和分母。

### 运行数据 DjiData

包含：

- 标准化反馈 `Feedback`。
- 原始反馈 `RawFeedback`。
- `command_id` 与 `command_slot`。
- 当前目标 `command_raw` 及其时间戳。
- armed、fault、epoch 等安全状态。
- 编码器累计值。
- 自旋锁，保护接收回调与控制线程之间的共享数据。

### `setCurrent()` 的边界

参数是安培 `float current_a`，成功返回 0。它会检查：

- 数值不是 NaN/Infinity，否则 `-EINVAL`。
- 不超过设备树电流上限，否则 `-ERANGE`。
- 反馈仍为 Ready，否则 `-EHOSTDOWN`。
- 电机已由 Bus arm 且无 fault，否则 `-EACCES`。

成功后只更新缓存和时间戳。这个“只缓存、不发送”是理解架构的关键。

## 6. Bus 负责什么、不负责什么

### Bus 负责

- 独占一条物理 CAN device，防止两个 Bus 同时管理它。
- `attach()` 电机，并拒绝不同 CAN 或 ID/槽位冲突。
- 记录需要发送的命令帧 ID 分组。
- 统一 arm、flush、stop、recover。
- 检查所有电机的反馈和命令是否新鲜。
- 组装共享命令帧并调用 `can_send()`。
- 失败时使整条 Bus 进入 Fault 并尽力发送全零帧。

### Bus 不负责

- 不计算 PID。
- 不决定目标电流是多少。
- 不主动定时调用 `flush()`；应用层必须每个控制周期调用。
- 不替应用层更新每台电机的命令缓存。
- 不直接解析反馈；反馈由每台电机注册的 CAN RX filter/callback 处理。

## 7. 接收路径与发送路径不同

### 接收反馈

```text
电机反馈 CAN 帧
 -> Zephyr CAN RX filter
 -> 单台电机 rxCallback()
 -> decodeFeedback()
 -> 更新该电机 raw/标准反馈和 last_rx_ms
 -> readFeedback()/getState() 供应用读取
```

每台电机有自己的反馈 CAN ID，因此接收天然适合按 device 分开处理。

### 发送命令

```text
应用为每台电机 setCurrent()
 -> 各自缓存
 -> Bus::flush() 汇总
 -> 按 command_id/slot 打包
 -> CAN 发送
```

命令帧是多电机共享的，因此发送必须由 Bus 统一处理。

## 8. Bus 生命周期

```text
Uninitialized
     |
     | init(can)
     v
   Safe <---------------- recover() 成功 ----------------+
     |                                                   |
     | attach(...)，等待所有反馈新鲜                     |
     | arm()：先发全零帧，再建立新 epoch                  |
     v                                                   |
   Armed                                                 |
     |  setCurrent(...), flush() 循环                    |
     |                                                   |
     +-- stop() 成功 -----------------------> Safe       |
     |                                                   |
     +-- 反馈/命令过期、状态错误、CAN 发送失败            |
     v                                                   |
   Fault ------------------------------------------------+
          先尝试发送全零帧；不能直接再次 arm
```

`epoch` 可以理解为一次 arm 会话的编号。旧会话缓存下来的命令不能混进新会话；若编号不匹配，`snapshotCommand()` 返回 `-ESTALE`。

重要错误码：

- `-EACCES`：Bus 状态不允许或电机未 arm。
- `-EHOSTDOWN`：反馈过期/电机离线。
- `-ESTALE`：命令过期或属于旧 arm 会话。
- `-EADDRINUSE`：反馈 ID 或命令槽位冲突。
- `-EXDEV`：试图 attach 属于另一条 CAN 的电机。
- `-EBUSY`：该物理 CAN 已被另一个 Bus 占用。

## 9. 当前 sample 为什么只有一台电机仍需要 Bus

`dji_unified` 目前只有一台 M3508，但它仍走 Bus，因为：

- 协议帧结构本身仍是 4 槽共享帧。
- 单电机和多电机使用同一套安全状态机。
- 以后添加电机只需设备树增加实例并 `attach()`，无需改发送架构。

当前 sample 的顺序：

```text
获取 motor0
 -> describe() 得到所属 CAN
 -> bus.init(can)
 -> bus.attach(motor0)
 -> 等待新鲜反馈
 -> 等待串口 a
 -> bus.arm()，先发全零帧
 -> 循环 setCurrent(motor0, 0A) + bus.flush()
```

这里 `current-limit-ma = <0>`，所以即使应用误把 `setCurrent()` 改成非零值，也会在单电机层返回 `-ERANGE`，不会进入发送缓存。

## 10. 多电机应用应怎样写

把下面结构放在应用控制循环，而不是驱动内部：

```cpp
// 初始化阶段：同一 can 上只创建一个 Bus
bus.init(can);
bus.attach(motor1);
bus.attach(motor2);
bus.attach(motor3);
// 确认全部反馈新鲜后
bus.arm(report);

// 每个控制周期
setCurrent(motor1, i1);
setCurrent(motor2, i2);
setCurrent(motor3, i3);
bus.flush(report); // 所有 setCurrent 完成后只调用一次
```

不要这样写：

```cpp
setCurrent(motor1, i1);
bus.flush(report);
setCurrent(motor2, i2);
bus.flush(report);
```

后者会造成同组电机并非在同一控制周期统一提交，还可能因为 motor2 的旧缓存超时而使整个 Bus 进入 Fault。

线程语义方面，电机缓存由自旋锁保护，因此 CAN 回调和控制线程共享数据是受保护的；但 `Bus` 自身没有给公开方法加互斥锁。应用应指定单一控制线程负责 `init/attach/arm/flush/stop/recover`，不要让多个线程并发调用同一个 Bus。

## 11. 自检与构建建议

阅读代码时可依次查看：

```bash
sed -n '1,220p' include/drivers/motor/motor.hpp
sed -n '1,520p' drivers/motor/dji/dji_motor.cpp
sed -n '1,430p' drivers/motor/dji/dji_bus.cpp
sed -n '1,180p' drivers/motor/dji/dji_protocol.cpp
sed -n '1,220p' samples/motor/dji_unified/src/main.cpp
```

修改多电机 sample 后建议执行：

```bash
west build -p always -b dm_mc02/stm32h723xx \
  samples/motor/dji_unified \
  --build-dir samples/motor/dji_unified/build/dm_mc02/stm32h723xx
```

硬件上电顺序：

1. 电机架空，先保持 `current-limit-ma = <0>`。
2. 确认每台电机 ID 唯一，CAN 波特率和终端电阻正确。
3. 上电后先观察所有反馈都进入 Ready。
4. 人工 arm，确认零帧发送成功。
5. 只验证 0 A；确认反馈、超时和 stop/fault 行为以后再逐步提高限流。

## 12. 最终检查清单

- [ ] 一条物理 CAN 只由一个 `Bus` 管理。
- [ ] 所有电机先 `attach()`，再 `arm()`。
- [ ] 每个周期先更新所有电机的 `setCurrent()`。
- [ ] 更新完成后只调用一次 `flush()`。
- [ ] 明白一帧最多 4 个槽，不等于一条 Bus 永远只发一帧。
- [ ] 控制周期小于反馈和命令 timeout。
- [ ] 同一个 Bus 的生命周期方法只由一个线程调用。
- [ ] 任意失败均按整条 Bus 故障处理，并确认全零帧结果。
