# Protocol contract tests


裁判协议测试同时覆盖 RM2026 V1.3.0 和 RM2027 V1.0.0 的 `0x0201` 差异（13/17 字节、输出许可偏移、2027 初速度上限），以及两版共同的 `0x0202` 保留区、独立时间戳、CRC 和跨版本长度拒绝行为。


Pure parser/codec tests for DR16 remote input, RM2026 referee data, interboard UART wire messages and endian/stamp helpers. Hardware UART/CAN is intentionally not required.

- Golden-value remote channel decode; malformed switch and length must not mutate a previously published sample.
- Fragmented input, lost fragment, stale remote state, referee permission/power independence.
- Referee CRC8/CRC16, explicit version, command decoding.
- Interboard frame metadata and CRC, invalid version/length, bounded queue behavior, session boot/replay rejection, encoder transactional failure.
- Producer-owned timestamps, forwarding age, sequence wraparound.

Run with `west twister -T tests/protocols -p native_sim/native/64`.
