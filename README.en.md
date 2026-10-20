# XRDAP-CH32V307

[中文](README.md)

A CMSIS-DAP v2 debug probe built on the CH32V307 with [libxr](https://github.com/xrobot-org/libxr). It provides a Bulk v2 DAP interface over USB high speed (480 Mbit/s) with 1024-byte packets, so Keil, pyOCD, probe-rs and OpenOCD use it as an ordinary CMSIS-DAP probe. A USB DFU bootloader sits at the start of the flash, so firmware updates need only a USB cable.

The firmware is built per board; two boards are supported:

| Board | Chip, crystal | Target interface | Bootloader key |
|---|---|---|---|
| `evt`: CH32V307 evaluation board (the board of [CH32V307_LibXR_Template](https://github.com/xrobot-org/CH32V307_LibXR_Template)) | CH32V307VCT6, 8 MHz | SWCLK PB13, SWDIO PB14, NRST PB12 | key on PB3 |
| `wdap`: the "高速无线DAP调试器" board | CH32V307RCT6, 12 MHz | 2×6 header: TCK/SWCLK (pin 3), TMS/SWDIO (pin 4), RST (pin 5) | KEY2 |

## 1. Getting started

### Wiring

Connect the target's SWCLK, SWDIO and GND to the pins above; connect one probe per target. Pins 1 (3V3) and 7 (5V) of the `wdap` header are power outputs of the board; leave them unconnected when the target has its own supply.

### First flashing

The first time, connect a WCH-Link or WCH-LinkE (RISC-V mode) to the board's two-wire debug port (SWDIO, SWCLK, GND) and write the bootloader at address 0, for example with the WCH OpenOCD shipped with MounRiver Studio:

```
openocd -f platform/wch-riscv.cfg -c init -c halt -c "flash erase_sector wch_riscv.flash 0 last" -c "program bootloader.elf.bin 0x00000000 verify" -c "reset run" -c shutdown
```

A board without a debug header (such as a `wdap` with the header unpopulated) can use the chip's built-in USB download mode instead: with BOOT0 pulled high at power-up the PC sees `4348:55E0`; in WCHISPTool_CH32Vxxx, shipped with MounRiver Studio, select the CH32V307 chip and USB download, write `bootloader.elf.bin`, and leave read protection off.

With an empty application area the bootloader stays in DFU mode and the PC sees the USB device `1209:3072` (XR-DAP-V307 DFU). Write the application as described in the next section.

### Writing and updating the application

```
pip install pyusb libusb-package
python tools/dfu_update.py xrdap.elf.bin
```

Besides the DAP interface the application has a DFU runtime interface. `dfu_update.py` sends it DFU DETACH and the probe restarts into the bootloader; with several probes connected, `--serial` selects one by its serial. Holding the board's bootloader key while plugging in USB also keeps the bootloader in DFU mode. `dfu_update.py` writes the image over standard USB DFU 1.1; the bootloader writes a check record (the seal) at the end and starts the new application. On Windows the bootloader announces WinUSB, so no driver installation is needed. From then on the bootloader starts the application at every power-up once the application area passes its check, and the PC sees `1D50:6199` (XR-DAP-V307 CMSIS-DAP) with the serial `XRobot-XRDAP-` followed by the chip UID.

`xrdap` from [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host) confirms the probe (its messages are in Chinese):

```
> xrdap -p usb:XRobot-XRDAP-68E3BE7B54507AAF39E339E3 info
连接：USB XRobot-XRDAP-68E3BE7B54507AAF39E339E3
探针：XRobot XR-DAP-V307，序列号 XRobot-XRDAP-68E3BE7B54507AAF39E339E3，固件 0.1.0
CMSIS-DAP：2.0.0，包长 1024 字节，最多 4 个包在途
```

## 2. Building

CMake 3.20 or newer and WCH RISC-V GCC 15.2 or newer are needed (`riscv32-wch-elf-gcc` on `PATH`, or `-DCOMPILER_PREFIX=<path>/riscv32-wch-elf-`). The bootloader and the application are two CMake projects; `-DXRDAP_BOARD=evt` or `wdap` selects the board:

```
git clone --recursive https://github.com/Jiu-xiao/XRDAP-CH32V307.git
cd XRDAP-CH32V307
cmake -S bootloader -B build/bootloader-evt -DXRDAP_BOARD=evt
cmake --build build/bootloader-evt
cmake -S . -B build/app-evt -DXRDAP_BOARD=evt
cmake --build build/app-evt
```

The images are `build/bootloader-evt/bootloader.elf.bin` and `build/app-evt/xrdap.elf.bin`.

## 3. Reference

### Flash layout

| Address | Content |
|---|---|
| `0x00000000`–`0x0000FFFF` | bootloader (64 KB, runs at 96 MHz) |
| `0x00010000`–`0x0003FEFF` | application (runs at 144 MHz) |
| `0x0003FF00`–`0x0003FFFF` | seal: application size and CRC32, written by the bootloader when a DFU download ends |

Both RAM splits (FLASH 192K + RAM 128K, FLASH 256K + RAM 64K) work; the firmware is linked for 64 KB of RAM.

### When the bootloader stays in DFU mode

- the bootloader key is held at power-up;
- the boot mailbox at the top of RAM (`0x2000FFF0`) holds a request, which survives a software reset;
- the application area has no seal, the seal's CRC does not match, or the image is not linked for the application address (the second word of the vector table is not `0x00010000`).

### Board files

`boards/<name>.cmake` gives the crystal frequency and `boards/<name>.hpp` the bootloader key, the LED and the target pins. A new board needs only these two files.

### libxr version

The `libxr` submodule is pinned to the `fix/usb-speed-after-reset` branch: on top of `feat/xrdap-serial-transport` it corrects the CH32 USBHS reading a high-speed link as full speed at reset (which otherwise makes Windows reject the configuration descriptor).

## License

Apache-2.0. The WCH startup file, peripheral library and FreeRTOS under `platform/` keep their own licenses.
