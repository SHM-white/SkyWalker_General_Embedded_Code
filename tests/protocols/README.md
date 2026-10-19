# Protocol contract tests

Pure parser/codec tests for DR16 remote input, RM2026 referee data, interboard UART wire messages and endian/stamp helpers. Hardware UART/CAN is intentionally not required.

- Golden-value remote channel decode; malformed switch and length must not mutate a previously published sample.
- Fragmented input, lost fragment, stale remote state, referee permission/power independence.
- Referee CRC8/CRC16, explicit version, command decoding.
- Interboard frame metadata and CRC, invalid version/length, bounded queue behavior, session boot/replay rejection, encoder transactional failure.
- Producer-owned timestamps, forwarding age, sequence wraparound.

Run with `west twister -T tests/protocols -p native_sim/native/64`.
