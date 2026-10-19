# Motor public API contracts

Tests the current Motor/Group/CanBus API on native_sim: fail-closed state, capabilities, command limits, atomic revisions, feedback prerequisites, single ownership and group status. Protocol golden frames are independently tested in `tests/motor_protocols`.

These do not cover CAN ISR completion, stop-frame interleavings, restart generations or bus-off recovery: the former regression suite used private fields and methods removed in the production refactor. A dedicated fake-CAN timing fixture (or HIL) is required for those invariants.
