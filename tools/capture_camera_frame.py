#!/usr/bin/env python3
"""Capture one CEU RGB565LE frame from the CPU0 debug UART and save it as BMP."""

from __future__ import annotations

import argparse
import struct
import sys
import time
import zlib
from pathlib import Path


MAGIC = b"CEU56501"
FORMAT_RGB565_LE = 1
HEADER_REMAINDER = struct.Struct("<HHIII")
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024


def read_exact(port, size: int) -> bytes:
    data = bytearray()
    last_report = time.monotonic()

    while len(data) < size:
        chunk = port.read(min(4096, size - len(data)))
        if not chunk:
            raise TimeoutError(f"serial timeout after {len(data)}/{size} bytes")
        data.extend(chunk)

        now = time.monotonic()
        if now - last_report >= 2.0:
            print(f"Receiving: {len(data) * 100 // size}%", flush=True)
            last_report = now

    return bytes(data)


def wait_for_magic(port) -> None:
    matched = 0
    while matched < len(MAGIC):
        byte = port.read(1)
        if not byte:
            raise TimeoutError("serial timeout while waiting for CEU56501 header")

        if byte[0] == MAGIC[matched]:
            matched += 1
        else:
            matched = 1 if byte[0] == MAGIC[0] else 0


def rgb565le_to_bmp(payload: bytes, width: int, height: int, output: Path) -> None:
    row_bytes = width * 3
    row_stride = (row_bytes + 3) & ~3
    image_bytes = row_stride * height
    pixel_data = bytearray(image_bytes)

    for dst_y, src_y in enumerate(range(height - 1, -1, -1)):
        src_row = src_y * width * 2
        dst_row = dst_y * row_stride

        for x in range(width):
            src = src_row + (x * 2)
            pixel = payload[src] | (payload[src + 1] << 8)
            r5 = (pixel >> 11) & 0x1F
            g6 = (pixel >> 5) & 0x3F
            b5 = pixel & 0x1F
            dst = dst_row + (x * 3)
            pixel_data[dst] = (b5 << 3) | (b5 >> 2)
            pixel_data[dst + 1] = (g6 << 2) | (g6 >> 4)
            pixel_data[dst + 2] = (r5 << 3) | (r5 >> 2)

    file_size = 54 + image_bytes
    bitmap_header = struct.pack("<2sIHHI", b"BM", file_size, 0, 0, 54)
    dib_header = struct.pack(
        "<IiiHHIIiiII",
        40,
        width,
        height,
        1,
        24,
        0,
        image_bytes,
        2835,
        2835,
        0,
        0,
    )
    output.write_bytes(bitmap_header + dib_header + pixel_data)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture one 640x480 RGB565LE CEU frame from UART9."
    )
    parser.add_argument("port", help="Serial port, for example COM7")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=120.0, help="Serial timeout in seconds")
    parser.add_argument("-o", "--output", type=Path, default=Path("camera_frame.bmp"))
    parser.add_argument("--raw", type=Path, help="Also save the exact RGB565LE payload")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("Missing dependency: install it with 'py -m pip install pyserial'", file=sys.stderr)
        return 2

    print(f"Opening {args.port} at {args.baud} baud")
    print("Now enter the firmware SETTINGS page; one frame takes about 55 seconds.")

    try:
        with serial.Serial(args.port, args.baud, timeout=args.timeout) as port:
            port.reset_input_buffer()
            wait_for_magic(port)
            header = read_exact(port, HEADER_REMAINDER.size)
            width, height, pixel_format, payload_size, expected_crc = HEADER_REMAINDER.unpack(header)

            if pixel_format != FORMAT_RGB565_LE:
                raise ValueError(f"unsupported pixel format {pixel_format}")
            if payload_size != width * height * 2:
                raise ValueError(
                    f"invalid payload size {payload_size}; expected {width * height * 2}"
                )
            if payload_size > MAX_PAYLOAD_BYTES:
                raise ValueError(f"payload is unreasonably large: {payload_size}")

            print(
                f"Frame header: {width}x{height}, RGB565LE, "
                f"{payload_size} bytes, CRC32=0x{expected_crc:08X}"
            )
            payload = read_exact(port, payload_size)
    except (OSError, TimeoutError, ValueError) as exc:
        print(f"Capture failed: {exc}", file=sys.stderr)
        return 1

    actual_crc = zlib.crc32(payload) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        print(
            f"CRC mismatch: received 0x{actual_crc:08X}, expected 0x{expected_crc:08X}",
            file=sys.stderr,
        )
        return 1

    if args.raw is not None:
        args.raw.write_bytes(payload)
        print(f"Raw frame saved: {args.raw.resolve()}")

    rgb565le_to_bmp(payload, width, height, args.output)
    print(f"CRC verified. BMP saved: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
