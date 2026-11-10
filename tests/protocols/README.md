# Protocol contract tests


裁判协议使用 **编译期 Kconfig 互斥选择**：`skywalker.protocols.contracts` 使用 RM2027 V1.0.0，`skywalker.protocols.contracts_rm2026` 使用 RM2026 V1.3.0。每次构建只链接对应年份的解析器；测试分别覆盖命令解码、跨版本长度拒绝、错误帧不污染已发布状态、CRC、输出许可/功率时间戳与缓冲能量。


Pure parser/codec tests for DR16 remote input, RM2026/RM2027 referee data, interboard UART wire messages and endian/stamp helpers. Hardware UART/CAN is intentionally not required.

- Golden-value remote channel decode; malformed switch and length must not mutate a previously published sample.
- Fragmented input, lost fragment, stale remote state, referee permission/power independence.
- Referee CRC8/CRC16, compile-time selected version, command decoding.
- Interboard frame metadata and CRC, invalid version/length, bounded queue behavior, session boot/replay rejection, encoder transactional failure.
- Producer-owned timestamps, forwarding age, sequence wraparound.

Run with `west twister -T tests/protocols -p native_sim/native/64`.
