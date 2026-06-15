#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
import usb.core


def main() -> int:
    parser = argparse.ArgumentParser(description="Wait for a USB VID:PID to appear")
    parser.add_argument("--vid", type=lambda x: int(x, 0), required=True)
    parser.add_argument("--pid", type=lambda x: int(x, 0), required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        dev = usb.core.find(idVendor=args.vid, idProduct=args.pid)
        if dev is not None:
            print(f"FOUND {args.vid:04x}:{args.pid:04x} bus={dev.bus} addr={dev.address}")
            return 0
        time.sleep(0.1)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
