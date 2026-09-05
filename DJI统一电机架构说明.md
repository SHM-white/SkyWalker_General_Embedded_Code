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

## 13. 多线程并发：PID 在 flush 期间修改指令会怎样

### 13.1 结论

当前实现是“单电机读写安全”，不是“整批电机事务一致”。

- 不会读取到半个 `int16_t`、半个时间戳或互相不匹配的单电机字段；`setCurrentImpl()` 和 `snapshotCommand()` 都使用同一台电机的 `DjiData::lock`。
- 不会修改已经组装到 `commands[][]` 局部数组里的值。
- 可能让同一次 `flush()` 混合不同 PID 周期的结果，因为 Bus 逐台加锁、逐台读取，没有覆盖全部电机的总锁或批次编号。
- `Bus` 自身的 `state_`、motor/group 数组和生命周期没有公共互斥保护，因此 `flush()` 不能与 `arm()`、`stop()`、`recover()`、`attach()` 或另一个 `flush()` 并发。

### 13.2 具体时序

假设 Bus 按 motor1、motor2 的顺序取快照，两个 PID 线程正在生成第 101 轮指令：

```text
时间 --->

PID1:  setCurrent(motor1, round101) ----------------------
Bus:              snapshot(motor1) ---- snapshot(motor2) -- send
PID2:                                  setCurrent(motor2, round101)
```

如果 PID2 的更新发生在 `snapshot(motor2)` 之前，本帧是：

```text
motor1 = round101
motor2 = round101
```

如果 PID2 稍晚、发生在 `snapshot(motor2)` 之后，本帧可能是：

```text
motor1 = round101
motor2 = round100
```

motor2 的 round101 会留在缓存里，通常在下一次 `flush()` 发出。这是完整的旧值，不是损坏的数据。

如果某 PID 在 Bus 已经取完所有快照、正在 `can_send()` 时更新缓存：

```text
Bus 局部 commands[][]：保持不变，本帧继续发送旧快照
电机 DjiData 缓存：保存新指令，供下一次 flush 使用
```

### 13.3 现有锁能保证什么

`setCurrentImpl()` 在单电机自旋锁内一次性更新：

```text
command_raw
command_stamp_ms
command_generation
command_epoch
```

`snapshotCommand()` 持有同一把锁检查状态并复制：

```text
armed / fault
epoch
反馈时间戳
命令时间戳
command_id / command_slot / command_raw
```

所以同一台电机的快照具有内部一致性。自旋锁持有时间很短，锁内不调用 `can_send()`，不会因为硬件发送等待而长期阻塞 PID 线程。

但每台电机各有一把锁：

```text
lock(motor1) -> copy -> unlock
lock(motor2) -> copy -> unlock
lock(motor3) -> copy -> unlock
```

不存在下面这种整批锁：

```text
lock(all motors) -> copy all -> unlock(all motors)
```

因此无法保证所有电机来自同一个逻辑控制周期。

### 13.4 推荐方案：单一控制线程拥有 setCurrent + flush

最简单、最稳妥的模型是让一个控制线程完成整个周期：

```cpp
for (;;) {
    // 先取得同一轮传感器/目标快照
    const float i1 = pid1.update(...);
    const float i2 = pid2.update(...);
    const float i3 = pid3.update(...);

    // 再顺序写入所有电机缓存
    int ret = setCurrent(motor1, i1);
    if (ret < 0) { /* 统一故障处理 */ }
    ret = setCurrent(motor2, i2);
    if (ret < 0) { /* 统一故障处理 */ }
    ret = setCurrent(motor3, i3);
    if (ret < 0) { /* 统一故障处理 */ }

    // 全部写完后只提交一次
    FlushReport report{};
    ret = bus.flush(report);
    if (ret < 0) { /* 统一故障处理 */ }

    sleep_until_next_period();
}
```

即使多个算法希望并行计算，也不要让每个 PID 线程直接调用 `setCurrent()`。推荐结构是：

```text
PID worker 1 --写结果--> 应用层目标缓冲区 --+
PID worker 2 --写结果--> 应用层目标缓冲区 --+--> 控制线程取同一批结果
PID worker 3 --写结果--> 应用层目标缓冲区 --+    -> setCurrent(all)
                                                    -> flush(once)
```

这个“应用层目标缓冲区”应有批次/序列号或双缓冲语义。控制线程只有在一整批 PID 输出都就绪时才交换 active buffer；这样不会把 round100 和 round101 混合。

### 13.5 如果必须由多个 PID 线程直接更新

可以在应用层增加一个普通 mutex，覆盖“写入整批指令”和 `flush()` 的协调，但不要把可能阻塞的 `can_send()` 放进 Zephyr 自旋锁，也不要试图从外部获取每个 `DjiData::lock`。

最低限度规则：

```text
1. Bus 生命周期方法只有 Bus owner 线程能调用。
2. PID 线程只发布计算结果，不调用 Bus 方法。
3. owner 线程在明确的周期边界消费结果、调用所有 setCurrent，再 flush。
4. stop/fault 请求通过消息或事件通知 owner 线程，不从其他线程直接 bus.stop()。
```

单纯在每个 PID 线程外分别加 mutex 仍不够；如果每个线程独立获取/释放同一 mutex 后调用一次 `setCurrent()`，Bus 仍可能在两次更新之间获取 mutex 并 flush。锁必须保护“整批更新 + flush”的事务边界，或使用双缓冲加批次提交。

### 13.6 是否应该给 Bus 内部加锁

仅给 `Bus::flush()` 加 mutex 可以防止两个 Bus 方法互相踩状态，但不能自动解决 PID 批次一致性，因为 PID 修改的是各电机缓存，不是 Bus 自己的成员。

若未来扩展公共 API，更合适的方向是批量提交接口，例如概念上的：

```cpp
struct CurrentCommand {
    const struct device *motor;
    float current_a;
};

int Bus::commitCurrents(const CurrentCommand *commands,
                        std::size_t count,
                        FlushReport &report);
```

该接口需要在一次调用内完成全部校验、换算、批次标记、组帧和发送，并清晰定义“任何一个命令失败时整批不发送、转 Fault 并归零”。在当前 API 下不要假装一次 `flush()` 已具有这种强事务语义。

### 13.7 并发自检清单

- [ ] 同一个 Bus 只有一个 owner 线程调用生命周期方法。
- [ ] 不会并发执行两个 `flush()`。
- [ ] `flush()` 不会与 `stop()/recover()/arm()/attach()` 并发。
- [ ] PID worker 不直接操作 Bus。
- [ ] 若要求多轴同周期一致，使用批次号或双缓冲发布全部 PID 输出。
- [ ] 控制线程按“读取整批结果 → 所有 setCurrent → 一次 flush”执行。
- [ ] 控制周期和最坏线程调度延迟均小于 command timeout。
- [ ] 不在自旋锁内执行 PID、日志、sleep 或 `can_send()`。
