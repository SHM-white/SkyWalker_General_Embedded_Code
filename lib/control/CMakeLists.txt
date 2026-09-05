zephyr_library()

zephyr_library_sources(
    pid.c
    feedforward.c
    feedforward_pid.c
    slew_rate_limiter.c
    angle.c
)