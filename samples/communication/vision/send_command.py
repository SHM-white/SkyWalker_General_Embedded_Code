#!/usr/bin/env python3
"""Send the user-selected 29-byte AB command to the receive-only bench."""
import argparse
import math
import struct
import time


def crc16(payload):
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB serial adapter connected to the board UART7 RX")
    parser.add_argument("--hex-only", action="store_true")
    parser.add_argument("--mode", type=int, choices=(0, 1, 2), default=1)
    parser.add_argument("--yaw", type=float, default=0.25, help="radians")
    parser.add_argument("--pitch", type=float, default=-0.2, help="radians")
    parser.add_argument("--yaw-vel", type=float, default=0.0, help="rad/s")
    parser.add_argument("--pitch-vel", type=float, default=0.0, help="rad/s")
    parser.add_argument("--yaw-acc", type=float, default=0.0, help="rad/s^2")
    parser.add_argument("--pitch-acc", type=float, default=0.0, help="rad/s^2")
    parser.add_argument("--hz", type=float, default=50)
    parser.add_argument("--count", type=int, default=100)
    args = parser.parse_args()
    values = (args.yaw, args.yaw_vel, args.yaw_acc, args.pitch, args.pitch_vel, args.pitch_acc)
    if not all(math.isfinite(x) for x in values) or not math.isfinite(args.hz) or not 0 < args.hz <= 100:
        parser.error("use finite angles/rates and a rate in (0, 100] Hz")
    if args.count < 1:
        parser.error("--count must be positive")
    try:
        payload = struct.pack("<2sB6f", b"AB", args.mode, *values)
    except (OverflowError, struct.error) as error:
        parser.error(str(error))
    frame = payload + struct.pack("<H", crc16(payload))
    if args.hex_only:
        print(frame.hex(" "))
        return
    if not args.port:
        parser.error("--port is required unless --hex-only is used")
    import serial  # Optional runtime dependency: python -m pip install pyserial
    with serial.Serial(args.port, 115200, timeout=0.1, write_timeout=1) as uart:
        deadline = time.monotonic()
        for _ in range(args.count):
            if uart.write(frame) != len(frame):
                raise OSError("short serial write")
            deadline += 1 / args.hz
            time.sleep(max(0, deadline - time.monotonic()))


if __name__ == "__main__":
    main()
