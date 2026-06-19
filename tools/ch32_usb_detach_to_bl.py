#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import usb.core


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send DFU DETACH to runtime app so it jumps back to bootloader"
    )
    parser.add_argument("--vid", type=lambda x: int(x, 0), default=0x0D28)
    parser.add_argument("--pid", type=lambda x: int(x, 0), default=0x2041)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--detach-ms", type=lambda x: int(x, 0), default=50)
    args = parser.parse_args()

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        dev = usb.core.find(idVendor=args.vid, idProduct=args.pid)
        if dev is None:
            time.sleep(0.1)
            continue

        try:
            dev.set_configuration(1)
        except usb.core.USBError:
            time.sleep(0.1)
            continue

        cfg = dev.get_active_configuration()
        target = None
        for intf in cfg:
            if intf.bInterfaceClass == 0xFE and intf.bInterfaceSubClass == 0x01:
                target = intf.bInterfaceNumber
                break

        if target is None:
            time.sleep(0.1)
            continue

        try:
            if dev.is_kernel_driver_active(target):
                dev.detach_kernel_driver(target)
        except Exception:
            pass

        # DFU DETACH: bmRequestType = 0x21, bRequest = 0x00, wValue = detach timeout (ms).
        dev.ctrl_transfer(0x21, 0x00, args.detach_ms, target, b"", timeout=1000)
        print(f"DETACH_SENT intf={target} detach_ms={args.detach_ms}")
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
