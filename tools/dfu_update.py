#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Update the XR-DAP-V307 firmware over USB DFU (no dfu-util needed).

Steps: send DFU DETACH to the running application's DFU runtime interface, which
restarts the probe into the bootloader (holding the board's boot key while plugging the
probe in does the same); wait for the bootloader (1209:3072); download the image with
DFU 1.1 DNLOAD blocks; finish with an empty DNLOAD. The bootloader then writes the seal
record and starts the new application.

  python tools/dfu_update.py build/app-evt/xrdap.elf.bin
  python tools/dfu_update.py --detach-only

Needs pyusb and libusb-package (pip install pyusb libusb-package). On Windows the
probe announces WinUSB, so no driver installation is required.
"""

import argparse
import sys
import time

import libusb_package
import usb.core
import usb.util

APP_ID = (0x1D50, 0x6199)
BOOTLOADER_ID = (0x1209, 0x3072)

DFU_CLASS, DFU_SUBCLASS = 0xFE, 0x01
DFU_DETACH, DFU_DNLOAD, DFU_GETSTATUS, DFU_CLRSTATUS = 0, 1, 3, 4
STATE_DNLOAD_IDLE, STATE_MANIFEST_WAIT_RESET, STATE_ERROR = 5, 8, 10
MIN_POLL_MS = 50
OUT = usb.util.CTRL_OUT | usb.util.CTRL_TYPE_CLASS | usb.util.CTRL_RECIPIENT_INTERFACE
IN = usb.util.CTRL_IN | usb.util.CTRL_TYPE_CLASS | usb.util.CTRL_RECIPIENT_INTERFACE


def find(ids, serial=None):
    backend = libusb_package.get_libusb1_backend()
    for dev in usb.core.find(find_all=True, idVendor=ids[0], idProduct=ids[1], backend=backend):
        if serial is None or (dev.serial_number or "").upper().endswith(serial.upper()):
            return dev
    return None


def dfu_interface(dev):
    """Returns (interface number, wTransferSize) of the device's DFU interface."""
    # dev[0]: the descriptor of the only configuration. Windows cannot report the active
    # configuration of a composite device whose parent has no WinUSB driver.
    for intf in dev[0]:
        if intf.bInterfaceClass == DFU_CLASS and intf.bInterfaceSubClass == DFU_SUBCLASS:
            extra = bytes(intf.extra_descriptors)
            transfer = 64
            # DFU functional descriptor: bLength 9, bDescriptorType 0x21.
            for i in range(len(extra) - 8):
                if extra[i] == 9 and extra[i + 1] == 0x21:
                    transfer = extra[i + 5] | (extra[i + 6] << 8)
            return intf.bInterfaceNumber, transfer
    raise RuntimeError("no DFU interface")


def get_status(dev, intf):
    s = dev.ctrl_transfer(IN, DFU_GETSTATUS, 0, intf, 6, timeout=5000)
    poll_ms = s[1] | (s[2] << 8) | (s[3] << 16)
    return s[0], poll_ms, s[4]  # bStatus, bwPollTimeout, bState


def wait_idle(dev, intf):
    while True:
        status, poll_ms, state = get_status(dev, intf)
        if status != 0 or state == STATE_ERROR:
            raise RuntimeError(f"DFU error: status {status}, state {state}")
        if state == STATE_DNLOAD_IDLE:
            return
        # The device stalls a GETSTATUS that arrives before its poll deadline, and the
        # LibXR DFU backend reports 10-25 ms while a 4 KB CH32 block takes ~350 ms and
        # delays its USB bookkeeping; polling at no less than 50 ms is reliable.
        time.sleep(max(poll_ms + 5, MIN_POLL_MS) / 1000)


def wait_for(ids, serial, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        dev = find(ids, serial)
        if dev is not None:
            return dev
        time.sleep(0.2)
    return None


def detach(serial):
    app = find(APP_ID, serial)
    if app is None:
        return
    try:
        intf, _ = dfu_interface(app)
    except RuntimeError:
        print("the application has no DFU runtime interface: hold the boot key and replug")
        return
    print("application found, sending DFU DETACH")
    app.ctrl_transfer(OUT, DFU_DETACH, 1000, intf, None, timeout=2000)


def download(path, serial):
    image = open(path, "rb").read()
    dev = wait_for(BOOTLOADER_ID, serial, 30)
    if dev is None:
        raise RuntimeError("bootloader 1209:3072 not found")
    intf, transfer = dfu_interface(dev)
    usb.util.claim_interface(dev, intf)
    status, _, state = get_status(dev, intf)
    if state == STATE_ERROR:
        dev.ctrl_transfer(OUT, DFU_CLRSTATUS, 0, intf, None)
    started = time.monotonic()
    for block, offset in enumerate(range(0, len(image), transfer)):
        dev.ctrl_transfer(OUT, DFU_DNLOAD, block, intf, image[offset:offset + transfer],
                          timeout=10000)
        wait_idle(dev, intf)
        print(f"\r{min(offset + transfer, len(image))}/{len(image)} bytes", end="")
    print(f"\rdownloaded {len(image)} bytes in {time.monotonic() - started:.1f} s")
    # Empty DNLOAD starts the manifest phase: the bootloader writes the seal record and,
    # once the GETSTATUS reporting dfuMANIFEST-WAIT-RESET completes, starts the image.
    dev.ctrl_transfer(OUT, DFU_DNLOAD, 0, intf, None, timeout=10000)
    try:
        while True:
            status, poll_ms, state = get_status(dev, intf)
            if status != 0 or state == STATE_ERROR:
                raise RuntimeError(f"manifest failed: status {status}, state {state}")
            if state == STATE_MANIFEST_WAIT_RESET:
                break
            time.sleep(max(poll_ms + 5, MIN_POLL_MS) / 1000)
    except usb.core.USBError:
        pass  # already gone: the application is starting
    usb.util.dispose_resources(dev)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("image", nargs="?", help="application .bin")
    ap.add_argument("--serial", help="probe serial or its tail")
    ap.add_argument("--detach-only", action="store_true", help="only restart into the bootloader")
    args = ap.parse_args()
    detach(args.serial)
    if args.detach_only:
        ok = wait_for(BOOTLOADER_ID, args.serial, 10) is not None
        print("bootloader running" if ok else "bootloader did not appear")
        return 0 if ok else 1
    if not args.image:
        ap.error("image is required")
    download(args.image, args.serial)
    ok = wait_for(APP_ID, args.serial, 10) is not None
    print("application running" if ok else "application did not appear")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
