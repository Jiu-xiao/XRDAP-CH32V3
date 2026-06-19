#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
from pathlib import Path


def crc32_wch(data: bytes) -> int:
    poly = 0xEDB88320
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ poly if (crc & 1) else (crc >> 1)
    return crc & 0xFFFFFFFF


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate 256-byte CH32 app seal page")
    parser.add_argument("input_bin")
    parser.add_argument("output_seal")
    args = parser.parse_args()

    image = Path(args.input_bin).read_bytes()
    crc = crc32_wch(image)
    record = struct.pack('<IIII', 0x4C414553, len(image), crc, (~crc) & 0xFFFFFFFF)
    Path(args.output_seal).write_bytes(record + b'\xFF' * (256 - len(record)))
    print(f'input={args.input_bin}')
    print(f'output={args.output_seal}')
    print(f'size={len(image)}')
    print(f'crc=0x{crc:08X}')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
