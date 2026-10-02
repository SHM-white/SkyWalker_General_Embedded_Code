# Motor regression tests

Run against the pinned Zephyr workspace:

```sh
west build -b native_sim/native/64 tests/motor/regression -d /tmp/skywalker-motor-regression
/tmp/skywalker-motor-regression/zephyr/zephyr.exe
```

The suite links the real Motor / Group / CanBus and protocol implementations.
A registered CAN API substitute records sends without hardware; tests advance
private state at precise interleaving points rather than depending on thread
sleep timings. The test translation unit temporarily exposes private members;
there are no test-only entry points in the production API. Zephyr assertions and
spinlock validation remain enabled.

Coverage:

- Reject a stale nonzero safety candidate after another endpoint disables.
- Permit an already-authorized residual without acknowledging a later stop.
- Keep all slots and sequence from one publication across another commit.
- Detect DJI Active and DM Enabling feedback gaps before replacing timestamps.
- Restart fast-recovery stability for both brands without restoring position.
- Preserve a latched fault across transport errors until explicit clear.
- Reject completion from a canceled clear operation.
- Check Group authority at final submission, including multiple independent groups.
- Reject canceled DM Enable/Probe candidates.
- Ignore stale DM callback order without creating a false gap fault.
- Continue queued work, shorten waits to deadlines, and avoid spinning while
  waiting for a recovery retry or for another Group member to prepare.
- Run the real GimbalExecutor with DJI yaw and DM pitch on distinct fake CAN
  controllers: one-axis transport fault stops both, automatic recovery creates
  a new context, and old source input cannot authorize it.
- Cancel both pending gimbal enables when a command is disabled or permission
  is withdrawn; reject late protocol completion after cancellation.
- Revoke both axes when a mechanical reference generation changes.

The gimbal fixture also links the real GimbalAxis and PositionMotor loops.
Protocol-safe preparation and received feedback are injected through the same
private-state fixture convention as the other regressions. It does not start CAN
worker threads or claim to qualify physical enable-handshake latency.

These are deterministic software regressions, not a hardware timing, bus-off,
CAN cancellation, stack-headroom, or mechanical-stop qualification.
