# Core contract tests

Independent native_sim Ztest coverage for header-only byte codec and timestamp/measurement contracts.

- Byte order against fixed on-wire vectors; offset access and sentinel preservation.
- IEEE-754 bit-pattern round trips, including signed zero, infinities and NaN payload.
- Timestamp freshness at exact deadline, future timestamps, invalid stamps and overflow boundary.
- Orientation-reference identity and default measurement state.

Run with `west twister -T tests/core_contracts -p native_sim/native/64 --inline-logs`.
