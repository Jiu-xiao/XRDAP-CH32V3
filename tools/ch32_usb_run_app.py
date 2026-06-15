#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
import usb.core


def main() -> int:
    parser = argparse.ArgumentParser(description="Send vendor RUN_APP to DFU bootloader")
    parser.add_argument("--vid", type=lambda x: int(x, 0), default=0x1209)
    parser.add_argument("--pid", type=lambda x: int(x, 0), default=0x3072)
    parser.add_argument("--request", type=lambda x: int(x, 0), default=0x5A)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        dev = usb.core.find(idVendor=args.vid, idProduct=args.pid)
        if dev is None:
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

        dev.ctrl_transfer(0x41, args.request, 0, target, b"", timeout=1000)
        print(f"RUN_APP_SENT intf={target}")
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
