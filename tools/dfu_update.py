#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Update the XR-DAP-V307 firmware over USB DFU (no dfu-util needed).

Steps: send DFU DETACH to the running application's DFU runtime interface, which
restarts the probe into the bootloader (holding the board's boot key while plugging the
probe in does the same); wait for the bootloader (1209:3072, or --bootloader); download the image with
DFU 1.1 DNLOAD blocks; finish with an empty DNLOAD. The bootloader then writes the seal
record and starts the new application.

  python tools/dfu_update.py build/app-evt/xrdap.elf.bin
  python tools/dfu_update.py --detach-only

The v305 board keeps the bootloader of XRDAP-CH32V3 (1209:3070), which does not start
the application by itself; --run-app sends its RUN_APP request (vendor request 0x5A)
after the download, and --upload reads the image in the application window back first:

  python tools/dfu_update.py --bootloader 1209:3070 --upload old.bin --run-app xrdap.elf.bin

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
DFU_DETACH, DFU_DNLOAD, DFU_UPLOAD, DFU_GETSTATUS, DFU_CLRSTATUS, DFU_ABORT = 0, 1, 2, 3, 4, 6
RUN_APP = 0x5A  # XRDAP-CH32V3 bootloader: start the application (vendor, interface)
STATE_IDLE, STATE_DNLOAD_IDLE, STATE_MANIFEST_WAIT_RESET, STATE_ERROR = 2, 5, 8, 10
OUT = usb.util.CTRL_OUT | usb.util.CTRL_TYPE_CLASS | usb.util.CTRL_RECIPIENT_INTERFACE
IN = usb.util.CTRL_IN | usb.util.CTRL_TYPE_CLASS | usb.util.CTRL_RECIPIENT_INTERFACE


def find(ids, serial=None):
    backend = libusb_package.get_libusb1_backend()
    for dev in usb.core.find(find_all=True, idVendor=ids[0], idProduct=ids[1], backend=backend):
        if serial is None:
            if ids != APP_ID:
                return dev
            # Other LibXR devices share 1d50:6199; only an XR-DAP probe is meant.
            try:
                if (dev.product or "").startswith("XR-DAP"):
                    return dev
            except (ValueError, NotImplementedError, usb.core.USBError):
                pass
            continue
        try:
            number = dev.serial_number or ""
        except (ValueError, NotImplementedError, usb.core.USBError):
            continue  # another program holds this probe, so it cannot be the one asked for
        if number.upper().endswith(serial.upper()):
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
        # DFU 1.1: wait bwPollTimeout before the next GETSTATUS; the bootloader reports
        # the measured duration of its last flash step.
        time.sleep(poll_ms / 1000)


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


def open_bootloader(serial):
    dev = wait_for(BOOTLOADER_ID, serial, 30)
    if dev is None:
        raise RuntimeError(f"bootloader {BOOTLOADER_ID[0]:04x}:{BOOTLOADER_ID[1]:04x} not found")
    intf, transfer = dfu_interface(dev)
    usb.util.claim_interface(dev, intf)
    return dev, intf, transfer


def to_idle(dev, intf):
    # An interrupted download leaves the bootloader in dfuERROR or mid-download; start
    # from dfuIDLE either way.
    status, _, state = get_status(dev, intf)
    if state == STATE_ERROR:
        dev.ctrl_transfer(OUT, DFU_CLRSTATUS, 0, intf, None)
    elif state != STATE_IDLE:
        dev.ctrl_transfer(OUT, DFU_ABORT, 0, intf, None)
    status, _, state = get_status(dev, intf)
    if state != STATE_IDLE:
        raise RuntimeError(f"bootloader not idle: status {status}, state {state}")


def upload(path, serial):
    """Read the image in the application window back (DFU UPLOAD until a short block)."""
    dev, intf, transfer = open_bootloader(serial)
    to_idle(dev, intf)
    data = bytearray()
    block = 0
    while True:
        chunk = bytes(dev.ctrl_transfer(IN, DFU_UPLOAD, block, intf, transfer, timeout=5000))
        data += chunk
        block += 1
        if len(chunk) < transfer:
            break
    open(path, "wb").write(data)
    print(f"uploaded {len(data)} bytes to {path}")
    to_idle(dev, intf)
    usb.util.dispose_resources(dev)


def run_app(serial):
    dev, intf, _ = open_bootloader(serial)
    try:
        dev.ctrl_transfer(usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR |
                          usb.util.CTRL_RECIPIENT_INTERFACE, RUN_APP, 0, intf, None,
                          timeout=1000)
    except usb.core.USBError:
        pass  # the bootloader may leave before the status stage completes
    usb.util.dispose_resources(dev)
    print("RUN_APP sent")


def download(path, serial):
    image = open(path, "rb").read()
    dev, intf, transfer = open_bootloader(serial)
    to_idle(dev, intf)
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
            time.sleep(poll_ms / 1000)
    except usb.core.USBError:
        pass  # already gone: the application is starting
    usb.util.dispose_resources(dev)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("image", nargs="?", help="application .bin")
    ap.add_argument("--serial", help="probe serial or its tail")
    ap.add_argument("--detach-only", action="store_true", help="only restart into the bootloader")
    ap.add_argument("--bootloader", default="1209:3072",
                    help="VID:PID of the bootloader (1209:3070: XRDAP-CH32V3 on v305)")
    ap.add_argument("--upload", metavar="FILE", help="read the current image back first")
    ap.add_argument("--run-app", action="store_true",
                    help="send RUN_APP after the download (XRDAP-CH32V3 bootloader)")
    args = ap.parse_args()
    global BOOTLOADER_ID
    BOOTLOADER_ID = tuple(int(x, 16) for x in args.bootloader.split(":"))
    detach(args.serial)
    if args.detach_only:
        ok = wait_for(BOOTLOADER_ID, args.serial, 10) is not None
        print("bootloader running" if ok else "bootloader did not appear")
        return 0 if ok else 1
    if not args.image:
        ap.error("image is required")
    if args.upload:
        upload(args.upload, args.serial)
    download(args.image, args.serial)
    if args.run_app:
        time.sleep(0.5)
        run_app(args.serial)
    ok = wait_for(APP_ID, args.serial, 10) is not None
    print("application running" if ok else "application did not appear")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
