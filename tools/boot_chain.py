#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Build the boot chain image for the first installation on a CH32V307 board.

boot-chain.bin covers the flash from 0 to the end of bootloader slot A: stage 1 at 0, the
slot A bootloader at its base and slot A's record (common/boot_slots.hpp) with sequence
number 1 in the slot's last page; boot-chain.hex holds the same at the same addresses.
Write it with the chip's own USB loader (BOOT0) or a WCH-Link, then write the
application with tools/dfu_update.py.

  python tools/boot_chain.py --stage1 stage1.bin --bootloader bootloader-a.elf.bin --out boot-chain
"""

import argparse
import struct
import zlib

SEAL_MAGIC = 0x4C414553  # "SEAL"
RECORD_MAGIC = 0x534C4258  # "XBLS"
VERSION_TAG = b"XRDAP-BLVER:"


def libxr_crc32(data):
    """LibXR's CRC32 as the DFU seal uses it: reflected 0xEDB88320, start 0xFFFFFFFF, no
    final complement."""
    return zlib.crc32(data) ^ 0xFFFFFFFF


def image_version(image):
    """The version string after the first tag that is followed by one (common/boot_slots.hpp)."""
    start = 0
    while True:
        i = image.find(VERSION_TAG, start)
        if i < 0:
            return b""
        j = i + len(VERSION_TAG)
        if j < len(image) and image[j] != 0:
            end = image.find(b"\0", j)
            return image[j:end if end >= 0 else len(image)][:31]
        start = i + 1


def record(base, image, seq, record_size):
    crc = libxr_crc32(image)
    fields = struct.pack("<8I", SEAL_MAGIC, len(image), crc, crc ^ 0xFFFFFFFF, RECORD_MAGIC,
                         seq, seq ^ 0xFFFFFFFF, base)
    version = image_version(image).ljust(32, b"\0")
    return (fields + version).ljust(record_size, b"\xff")


def ihex(data, base):
    lines = []
    upper = None
    for offset in range(0, len(data), 16):
        address = base + offset
        if address >> 16 != upper:
            upper = address >> 16
            payload = struct.pack(">BHBH", 2, 0, 4, upper)
            lines.append(":" + payload.hex().upper() + "%02X" % (-sum(payload) & 0xFF))
        chunk = data[offset:offset + 16]
        payload = struct.pack(">BHB", len(chunk), address & 0xFFFF, 0) + chunk
        lines.append(":" + payload.hex().upper() + "%02X" % (-sum(payload) & 0xFF))
    lines.append(":00000001FF")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stage1", required=True)
    ap.add_argument("--bootloader", required=True, help="bootloader image linked for slot A")
    ap.add_argument("--out", required=True, help="output path without extension")
    ap.add_argument("--slot-base", type=lambda v: int(v, 0), default=0x1000)
    ap.add_argument("--slot-size", type=lambda v: int(v, 0), default=0xD000)
    ap.add_argument("--record-size", type=lambda v: int(v, 0), default=256)
    args = ap.parse_args()

    stage1 = open(args.stage1, "rb").read()
    image = open(args.bootloader, "rb").read()
    limit = args.slot_size - args.record_size
    if len(stage1) > args.slot_base:
        raise SystemExit(f"stage 1 is {len(stage1)} bytes, more than {args.slot_base:#x}")
    if not 0 < len(image) <= limit:
        raise SystemExit(f"the bootloader is {len(image)} bytes, slot A holds {limit}")
    if struct.unpack_from("<I", image, 4)[0] != args.slot_base:
        raise SystemExit("the bootloader is not linked for slot A")
    if not image_version(image):
        raise SystemExit("the bootloader has no version marker")

    chain = bytearray(b"\xff" * (args.slot_base + args.slot_size))
    chain[0:len(stage1)] = stage1
    chain[args.slot_base:args.slot_base + len(image)] = image
    rec = record(args.slot_base, image, 1, args.record_size)
    chain[args.slot_base + limit:args.slot_base + args.slot_size] = rec
    open(args.out + ".bin", "wb").write(chain)
    open(args.out + ".hex", "w").write(ihex(bytes(chain), 0))
    print(f"{args.out}.bin: stage 1 {len(stage1)} B, slot A {len(image)} B, "
          f"version {image_version(image).decode()}")


if __name__ == "__main__":
    main()
