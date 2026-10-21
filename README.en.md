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

Besides the DAP interface the application has a DFU runtime interface. `dfu_update.py` sends it DFU DETACH and the probe restarts into the bootloader; with several probes connected, `--serial` selects one by its serial. Holding the board's bootloader key while plugging in USB also keeps the bootloader in DFU mode. `dfu_update.py` writes the image over standard USB DFU 1.1; the bootloader writes a check record (the seal) at the end and starts the new application. On Windows the bootloader announces WinUSB, so no driver installation is needed. dfu-util works as well (tested with 0.11; it warns that the image has no DFU suffix, which does not affect the download) and also sends DETACH first: `dfu-util -d 1d50:6199,1209:3072 -S XRobot-XRDAP-<UID> -a 0 -D xrdap.elf.bin`. From then on the bootloader starts the application at every power-up once the application area passes its check, and the PC sees `1D50:6199` (XR-DAP-V307 CMSIS-DAP) with the serial `XRobot-XRDAP-` followed by the chip UID.

`xrdap` from [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host) confirms the probe (its messages are in Chinese):

```
> xrdap -p usb:XRobot-XRDAP-68E3BE7B54507AAF39E339E3 info
连接：USB XRobot-XRDAP-68E3BE7B54507AAF39E339E3
探针：XRobot XR-DAP-V307，序列号 XRobot-XRDAP-68E3BE7B54507AAF39E339E3，固件 0.1.0
CMSIS-DAP：2.0.0，包长 1024 字节，最多 4 个包在途
```

## 2. Wireless debugging (wdap)

The SX1281 radio module on the `wdap` board joins two probes into one wireless probe: the host probe is plugged into the PC and the remote probe is wired to the target. The PC uses the host probe as an ordinary CMSIS-DAP probe; the host sends DAP commands over the radio to the remote, which executes them on its own SWD port and sends the answers back.

Both boards run the same firmware, and by default the role is decided automatically. A probe without a debug session of its own listens to the radio as a remote; when a tool sends DAP_Connect, the probe first pings its paired peer, and if the peer answers the session runs over the radio, otherwise the probe drives its own SWD port. So the board that is only powered is the remote and the board a tool opens is the host; without the remote, the host is an ordinary wired probe. When the peer drops out during a session the commands return errors rather than moving to the local SWD port; the next DAP_Connect decides again. SWJ_Clock, TransferConfigure and SWD_Configure sent before DAP_Connect are sent on to the remote when the session moves to the radio.

### Pairing

New boards share the same default radio parameters (2.44 GHz, sync word `XRDP`) and work together without pairing; several sets in one place need pairing. Hold KEY2 on both boards for 3 s, one after the other within 10 s: the boards exchange their chip UIDs and derive the parameters of the pair from the two UIDs: frequency (2403–2479 MHz), CRC seed and GFSK sync word. Other pairs match neither frequency nor CRC, so their packets are not received. The pairing is kept in flash across power cycles and firmware updates; pairing the same two boards again gives the same parameters.

### Settings

Role, modulation and pairing live in the settings area at the end of the application window (see Flash layout) and are read and changed with vendor command `0x9F` (see `app/radio_command.hpp`). `0x9F 0x30` hands a `0x9F` command over the radio to the peer, for example to change the modulation of both boards from the host.

- Role: automatic (the default), always wired, always host, always remote.
- Modulation (both boards must match):
  - GFSK (the default): 2 Mb/s, packets of up to 255 bytes, at most 5 per 1024-byte message.
  - FLRC: 1.3 Mb/s, packets of up to 127 bytes, 9 per message. The build option `XRDAP_RADIO_CODING_RATE` selects the coding rate: `0x04` without FEC by default, `0x02` for 3/4, `0x00` for 1/2.

### Link

When no answer arrives in time the host sends the command again with the same sequence number, waiting twice as long each time; the remote answers a repeated sequence number with its last answer instead of executing the command again. After 1 s without a message from the peer, the next command pings first and returns an error at once when the ping fails, instead of retrying for a full second.

Continuous receive on the SX1281 can lock up under heavy traffic (datasheet errata 16.1), and the radio was seen staying in receive without receiving anything; a radio that hears no packet for 100 ms enters receive again and is reset after 2 s. Received packets are judged by the error byte of GetPacketStatus (errata 16.3), and the radio stays in FS after each transmit or receive (SetAutoFs) for shorter mode switches. Single receive, which the datasheet recommends, and transmit ping-pong both limit packets to 127 bytes and measured slower; they remain build options `XRDAP_RADIO_RX_SINGLE` and `XRDAP_RADIO_TX_PINGPONG`, off by default (see `app/sx1281.hpp`). FLRC uses the 32-bit sync word `XRDP`, also after pairing: without a sync word the receiver takes noise for packets and the link stops; errata 16.4 does not publish the sync words to avoid at coding rate 1, and at coding rate 3/4 `XRDP` is outside the forbidden range.

A small command takes about 0.6 ms there and back, of which the two transmissions take about 0.3 ms. To shorten it, SPI transfers of up to 24 bytes go straight through the registers instead of DMA; in continuous receive only the packet interrupts reach DIO1, so a high DIO1 means a packet and the interrupt status is not read; the radio leaves receive ahead of the next transmit while the remote executes a command and after the host has its answer; the PA ramps up in 8 µs.

Measured debugging an STM32H723 (1 MiB flash) over the radio with both boards on one desk, at an SWD clock of 30 MHz (10 MHz for OpenOCD):

| Item | GFSK | FLRC | Wired |
|---|---|---|---|
| Block reads, 4 bytes per command | 3.1 KiB/s | 2.5 KiB/s | |
| Block reads, 1020 bytes per command | 87–110 KiB/s | 61–70 KiB/s | |
| OpenOCD programming 1 MiB | 87–88 KiB/s | 63 KiB/s | 118.7 KiB/s |
| probe-rs download of 128 KiB (with erase) | 6.9 s | | |

The wired 118.7 KiB/s is the H7's own flash speed; erase (about 6.8 s) and verify (OpenOCD computes a CRC32 on the target, about 980 KiB/s) are the same wired and wireless. Before the round trip was shortened, the same setup gave 2.4 KiB/s for 4-byte block reads, 84 KiB/s for OpenOCD programming and 7.7 s for the probe-rs download; 1020-byte block reads are bound by air time and did not change. Over 117,000 consecutive commands each, GFSK retried 1.4% and FLRC 1.8% of them and none failed in the end; the share of retries follows the 2.4 GHz interference around.

### Watchdog and safe mode

The application runs the independent watchdog and resets when its main loop has not run for about 8 s. After three watchdog resets in a row it starts in safe mode: the settings area is neither read nor written and the probe works as a wired probe, with DFU as usual; 30 s of normal running or a power cycle ends it.

## 3. Building

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

## 4. Reference

### Flash layout

| Address | Content |
|---|---|
| `0x00000000`–`0x0000FFFF` | bootloader (64 KB, runs at 96 MHz) |
| `0x00010000`–`0x0003F6FF` | application (144 MHz on `evt`, 216 MHz on `wdap`) |
| `0x0003F700`–`0x0003FEFF` | settings: role, modulation, pairing; kept across firmware updates |
| `0x0003FF00`–`0x0003FFFF` | seal: application size and CRC32, written by the bootloader when a DFU download ends |

Both RAM splits (FLASH 192K + RAM 128K, FLASH 256K + RAM 64K) work; the firmware is linked for 64 KB of RAM.

The `wdap` application runs at 216 MHz from its 12 MHz crystal times 18, above the CH32V307's rated 144 MHz. SWD is generated by GPIO in software, so its speed follows the core clock: block reads from an STM32G431 run at 1188 KiB/s (790 KiB/s at 144 MHz), and 1.3 GiB were read over 20 minutes without an error. Setting `XRDAP_APP_SYSCLK_HZ` in `boards/wdap.cmake` to `144000000` returns to the rated clock.

### When the bootloader stays in DFU mode

- the bootloader key is held at power-up;
- the boot mailbox at the top of RAM (`0x2000FFF0`) holds a request, which survives a software reset;
- the application area has no seal, the seal's CRC does not match, or the image is not linked for the application address (the second word of the vector table is not `0x00010000`).

### Board files

`boards/<name>.cmake` gives the crystal frequency and `boards/<name>.hpp` the bootloader key, the LED and the target pins. A new board needs only these two files.

### libxr version

The `libxr` submodule is pinned to the `fix/usb-speed-after-reset` branch, which fixes three problems on top of `feat/xrdap-serial-transport`: the CH32 USBHS read a high-speed link as full speed at reset (Windows rejected the configuration descriptor); the DFU classes carried their own MS OS 2.0 descriptor, so Windows could not start a composite with the DAP interface; and the USBHS sent a packet of a multi-packet IN transfer twice, which shifted DAP answers longer than 512 bytes.

## License

Apache-2.0. The WCH startup file, peripheral library and FreeRTOS under `platform/` keep their own licenses.
