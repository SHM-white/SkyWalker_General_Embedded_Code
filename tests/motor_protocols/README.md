# Motor CAN wire contracts

Uses the real DJI and Damiao protocol source on Zephyr native_sim; no physical CAN controller, motor state machine or motor simulator is used. Assert known byte layouts and integer/float encodings; reject bad frames and nonfinite/out-of-range controls without mutating the caller's output.

This complements `tests/motor/regression`, which retains multi-motor scheduling and recovery scenarios.
