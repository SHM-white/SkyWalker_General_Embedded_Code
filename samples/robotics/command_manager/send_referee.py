#!/usr/bin/env python3
import argparse
import math
import struct
import time

def crc8(data):
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8C if value & 1 else 0)
    return value

def crc16(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value

def encode(flags, sequence, power_only):
    if power_only:
        # 只刷新0x0202状态与缓冲能量，不产生/刷新输出许可。
        command_id = 0x0202
        payload = bytes(8) + struct.pack('<HHH', 30, 0, 0)
    else:
        command_id = 0x0201
        payload = struct.pack('<BB5HB', 3, 1, 100, 100, 10, 200, 80, flags)
    header = b'\xA5' + struct.pack('<HB', len(payload), sequence & 0xFF)
    header += bytes([crc8(header)])
    body = header + struct.pack('<H', command_id) + payload
    return body + struct.pack('<H', crc16(body))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port')
    parser.add_argument('--flags', type=int, choices=range(8), default=7)
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--count', type=int, default=600)
    parser.add_argument('--power-only', action='store_true')
    parser.add_argument('--hex-only', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.hz) or not 0 < args.hz <= 100 or args.count < 1:
        parser.error('use 0 < hz <= 100 and count >= 1')
    if args.hex_only:
        print(encode(args.flags, 0, args.power_only).hex(' '))
        return
    if not args.port:
        parser.error('--port is required unless --hex-only is used')
    import serial
    with serial.Serial(args.port, 115200, timeout=0.1, write_timeout=1) as uart:
        deadline = time.monotonic()
        for sequence in range(args.count):
            frame = encode(args.flags, sequence, args.power_only)
            if uart.write(frame) != len(frame):
                raise OSError('short serial write')
            deadline += 1 / args.hz
            time.sleep(max(0, deadline - time.monotonic()))

if __name__ == '__main__':
    main()
