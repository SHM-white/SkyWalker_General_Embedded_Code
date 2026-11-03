# Robot state and outer-loop safety contracts

Tests the production yaw-centering controller and run-status-to-wire conversion as standalone pure units. Covers configuration, deadband, acceleration-limited recovery, input age, power/reference revocation and prevention of stale-ready feedback.

Also covers `WorldYawWindowFollower` for reference-chassis world-frame big-Yaw control: rolling circular mean, static convergence to zero joint offset, wrap across ±π, bounded rate, and immediate fail-closed reset. No IMU/CAN/UART hardware is instantiated. No gimbal or chassis hardware is instantiated.
