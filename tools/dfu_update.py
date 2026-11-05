#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Update the XR-DAP CH32 firmware over USB DFU (no dfu-util needed).

Steps: send DFU DETACH to the running application's DFU runtime interface, which
restarts the probe into the bootloader (holding the board's boot key while plugging the
probe in does the same); wait for the bootloader (1209:3072, or --bootloader); download the image with
DFU 1.1 DNLOAD blocks; finish with an empty DNLOAD. The bootloader then writes the seal
record and starts the new application.

  python tools/dfu_update.py build/app-evt/xrdap.elf.bin
  python tools/dfu_update.py --detach-only

With the A/B boot chain (evt, wdap, v305; bootloader/), --bootloader-dir also updates the
bootloader: the running bootloader names the slot it can write (DFU interface 1,
"XR-DAP bootloader slot A" or "... slot B"); when the version in the image for that slot
differs from the running one (USB product string), the image goes to interface 1, the
bootloader closes the slot and restarts in DFU mode under the new bootloader, and the
application follows:

  python tools/dfu_update.py --bootloader-dir build/bootloader-wdap build/app-wdap/xrdap.elf.bin

A v305 board with the earlier three-stage bootloader (1209:3070), which
does not start the application by itself, migrates to the A/B boot chain with
migrate-to-ab; --run-app sends its RUN_APP request (vendor request 0x5A) after the
download, and --upload reads the image in the application window back first:

  python tools/dfu_update.py --bootloader 1209:3070 --run-app build/bootloader-v305/migrate-to-ab.elf.bin

Needs pyusb and libusb-package (pip install pyusb libusb-package). On Windows the
probe announces WinUSB, so no driver installation is required.
"""

import argparse
import os
import sys
import time

import libusb_package
import usb.core
import usb.util

APP_ID = (0x1D50, 0x6199)
BOOTLOADER_ID = (0x1209, 0x3072)

DFU_CLASS, DFU_SUBCLASS = 0xFE, 0x01
DFU_DETACH, DFU_DNLOAD, DFU_UPLOAD, DFU_GETSTATUS, DFU_CLRSTATUS, DFU_ABORT = 0, 1, 2, 3, 4, 6
RUN_APP = 0x5A  # the earlier three-stage bootloader: start the application (vendor, interface)
STATE_IDLE, STATE_DNLOAD_IDLE, STATE_MANIFEST_WAIT_RESET, STATE_ERROR = 2, 5, 8, 10
APP_INTERFACE = "XR-DAP application"
SLOT_INTERFACE = "XR-DAP bootloader slot "
VERSION_TAG = b"XRDAP-BLVER:"
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


def interface_name(dev, intf):
    try:
        return usb.util.get_string(dev, intf.iInterface) if intf.iInterface else ""
    except (ValueError, usb.core.USBError):
        return ""


def dfu_interface(dev, name=None):
    """Returns (interface number, wTransferSize) of the device's DFU interface; with name,
    of the one whose interface string starts with it."""
    # dev[0]: the descriptor of the only configuration. Windows cannot report the active
    # configuration of a composite device whose parent has no WinUSB driver.
    for intf in dev[0]:
        if intf.bInterfaceClass == DFU_CLASS and intf.bInterfaceSubClass == DFU_SUBCLASS:
            if name is not None and not interface_name(dev, intf).startswith(name):
                continue
            extra = bytes(intf.extra_descriptors)
            transfer = 64
            # DFU functional descriptor: bLength 9, bDescriptorType 0x21.
            for i in range(len(extra) - 8):
                if extra[i] == 9 and extra[i + 1] == 0x21:
                    transfer = extra[i + 5] | (extra[i + 6] << 8)
            return intf.bInterfaceNumber, transfer
    raise RuntimeError("no DFU interface" + (f" named {name!r}" if name else ""))


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


def open_bootloader(serial, name=None):
    dev = wait_for(BOOTLOADER_ID, serial, 30)
    if dev is None:
        raise RuntimeError(f"bootloader {BOOTLOADER_ID[0]:04x}:{BOOTLOADER_ID[1]:04x} not found")
    if name is None and app_interface_present(dev):
        name = APP_INTERFACE
    intf, transfer = dfu_interface(dev, name)
    # A freshly enumerated DFU device may be held for a moment by another program (fwupd
    # probes every new DFU device on Linux); wait for it.
    deadline = time.monotonic() + 10
    while True:
        try:
            usb.util.claim_interface(dev, intf)
            return dev, intf, transfer
        except usb.core.USBError as e:
            if e.errno != 16 or time.monotonic() > deadline:
                raise
        usb.util.dispose_resources(dev)
        time.sleep(0.3)
        dev = wait_for(BOOTLOADER_ID, serial, 5) or dev


def app_interface_present(dev):
    return any(interface_name(dev, i).startswith(APP_INTERFACE) for i in dev[0])


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


def download(path, serial, name=None):
    image = open(path, "rb").read()
    dev, intf, transfer = open_bootloader(serial, name)
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
            if state in (STATE_MANIFEST_WAIT_RESET, STATE_IDLE):
                break  # dfuIDLE: the bootloader slot interface, which starts nothing
            time.sleep(poll_ms / 1000)
    except usb.core.USBError:
        pass  # already gone: the application is starting
    usb.util.dispose_resources(dev)


def image_version(image):
    """The version string after the first version marker followed by one (boot_slots.hpp)."""
    start = 0
    while True:
        i = image.find(VERSION_TAG, start)
        if i < 0:
            return ""
        j = i + len(VERSION_TAG)
        if j < len(image) and image[j] != 0:
            end = image.find(b"\0", j)
            return image[j:end if end >= 0 else len(image)].decode(errors="replace")
        start = i + 1


def update_bootloader(directory, serial, force):
    """Writes the bootloader image for the slot the running bootloader offers, when its
    version differs (or with force); returns True when the bootloader was replaced."""
    dev = wait_for(BOOTLOADER_ID, serial, 30)
    if dev is None:
        raise RuntimeError("bootloader not found")
    slot = None
    for intf in dev[0]:
        name = interface_name(dev, intf)
        if name.startswith(SLOT_INTERFACE):
            slot = name[len(SLOT_INTERFACE):len(SLOT_INTERFACE) + 1].lower()
    running = (dev.product or "").rsplit(" ", 1)[-1]
    usb.util.dispose_resources(dev)
    if slot not in ("a", "b"):
        raise RuntimeError("this bootloader cannot replace itself (it predates the A/B boot "
                           "chain): write boot-chain.hex with the chip's USB loader (BOOT0) "
                           "or a WCH-Link first")
    path = None
    for candidate in (f"bootloader-{slot}.elf.bin", f"bootloader-{slot}.bin"):
        full = os.path.join(directory, candidate)
        if os.path.isfile(full):
            path = full
            break
    if path is None:
        raise RuntimeError(f"no bootloader-{slot}.elf.bin in {directory}")
    new = image_version(open(path, "rb").read())
    if new == running and not force:
        print(f"bootloader {running} is current")
        return False
    print(f"bootloader {running} -> {new or '?'} into slot {slot.upper()}")
    download(path, serial, SLOT_INTERFACE + slot.upper())
    # The bootloader closes the slot, restarts in DFU mode and stage 1 starts the new one.
    time.sleep(1.0)
    dev = wait_for(BOOTLOADER_ID, serial, 15)
    if dev is None:
        raise RuntimeError("the new bootloader did not appear")
    now = (dev.product or "").rsplit(" ", 1)[-1]
    usb.util.dispose_resources(dev)
    if new and now != new:
        raise RuntimeError(f"bootloader still reports {now}, expected {new}")
    print(f"bootloader {now} running")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("image", nargs="?", help="application .bin")
    ap.add_argument("--serial", help="probe serial or its tail")
    ap.add_argument("--detach-only", action="store_true", help="only restart into the bootloader")
    ap.add_argument("--bootloader", default="1209:3072",
                    help="VID:PID of the bootloader (1209:3070: the earlier three-stage bootloader, before the migration)")
    ap.add_argument("--upload", metavar="FILE", help="read the current image back first")
    ap.add_argument("--run-app", action="store_true",
                    help="send RUN_APP after the download (the earlier three-stage bootloader)")
    ap.add_argument("--bootloader-dir", metavar="DIR",
                    help="also update the bootloader from DIR/bootloader-a|b.elf.bin (A/B boot chain)")
    ap.add_argument("--force-bootloader", action="store_true",
                    help="write the bootloader even when its version is the running one")
    args = ap.parse_args()
    global BOOTLOADER_ID
    BOOTLOADER_ID = tuple(int(x, 16) for x in args.bootloader.split(":"))
    # Check the arguments before the DETACH, which leaves the probe in the bootloader.
    if not args.image and not args.detach_only and not args.run_app and not args.bootloader_dir:
        ap.error("image is required")
    detach(args.serial)
    if args.bootloader_dir:
        update_bootloader(args.bootloader_dir, args.serial, args.force_bootloader)
        if not args.image:
            run_app(args.serial)
            ok = wait_for(APP_ID, args.serial, 10) is not None
            print("application running" if ok else "application did not appear")
            return 0 if ok else 1
    if args.detach_only:
        ok = wait_for(BOOTLOADER_ID, args.serial, 10) is not None
        print("bootloader running" if ok else "bootloader did not appear")
        return 0 if ok else 1
    if not args.image and args.run_app:
        run_app(args.serial)  # only start the application already in the window
        ok = wait_for(APP_ID, args.serial, 10) is not None
        print("application running" if ok else "application did not appear")
        return 0 if ok else 1
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
