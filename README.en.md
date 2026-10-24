# XRDAP-CH32V307

[中文](README.md)

A CMSIS-DAP v2 debug probe built on the CH32V307 with [libxr](https://github.com/xrobot-org/libxr). It provides a Bulk v2 DAP interface over USB high speed (480 Mbit/s) with 1024-byte packets, so Keil, pyOCD, probe-rs and OpenOCD use it as an ordinary CMSIS-DAP probe. A USB DFU bootloader sits at the start of the flash, so firmware updates need only a USB cable.

The firmware is built per board; three boards are supported:

| Board | Chip, crystal | Target interface | Bootloader key |
|---|---|---|---|
| `evt`: CH32V307 evaluation board (the board of [CH32V307_LibXR_Template](https://github.com/xrobot-org/CH32V307_LibXR_Template)) | CH32V307VCT6, 8 MHz | SWCLK PB13, SWDIO PB14, NRST PB12 | key on PB3 |
| `wdap`: the "高速无线DAP调试器" board | CH32V307RCT6, 12 MHz | 2×6 header: TCK/SWCLK (pin 3), TMS/SWDIO (pin 4), RST (pin 5) | KEY2 |
| `v305`: the CH32V305 board of XRDAP-CH32V3 (end of section 1) | CH32V305FBP6, 12 MHz | SWCLK PB13, SWDIO PA9, NRST PC8 | XRDAP-CH32V3's bootloader; IAP key PC7 |

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

`xrdap info` from [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host) confirms the probe: it lists the probe name, serial, firmware version, CMSIS-DAP packet size and the vendor extension's capabilities, then connects to the target and reads its DPIDR and core state.

### Status lights (wdap)

The 8 WS2812B on the `wdap` board show the debug state, with the colours of the status light rule shared by the XR-DAP family ([XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core)). A rainbow sweeps once at power-up. The chain is red while USB is connected to a computer, blue during a debug session over the radio (on the host, and on the remote while its remote session is open), otherwise purple; pairing blinks purple fast. While a tool reports connected through DAP_HostStatus the main colour fades to purple and back every 2 s, and while the target runs red and blue alternate every second; DAP_Disconnect, or 5 s without any command, returns to the main colour. Every DAP command lights the chain up and moves a white dot one LED on, about 15 times a second under steady traffic.

TIM1_CH3N (PB15) PWM with DMA drives the chain, with the high time of each bit set from the WS2812B-2020 datasheet timing; the CPU only fills a buffer every 20 ms.

### CH32V305 board (v305, this firmware not yet verified on hardware)

`v305` is the CH32V305 board of [XRDAP-CH32V3](https://github.com/Jiu-xiao/XRDAP-CH32V3) (CH32V305FBP6, 12 MHz crystal, the circuit of the WCH-LinkE). On it this firmware is a wired USB high-speed DAP only, without radio, with the product name `XR-DAP-V305` (USB product string `XR-DAP-V305 CMSIS-DAP`, USB ids of the family), and with the pins of the XRDAP-CH32V3 application:

| Function | Pin |
|---|---|
| SWCLK | PB13 (wired to PA14 on the board) |
| SWDIO | PA9 (bonded to PA13 in the package), one bidirectional pin, push-pull |
| Target reset | PC8 |
| LED | PC9 (bonded to PA8), lit when low |
| Held low after start-up | PA5, PB12 (target power switches) |

The board keeps the three-stage boot of XRDAP-CH32V3: stage 1 at address 0 jumps to the stage-3 DFU bootloader at `0x20000` (USB `1209:3070`); the application window is `0x2000`–`0x20000`, its last 256 bytes the seal. This project does not build that bootloader; `tools/build.sh v305` produces the application only. The application is updated over DFU only, not by writing the whole flash with a WCH-Link (that destroys stage 1 and the bootloader):

```
dfu-util -d 1d50:6199,1209:3070 -a 0 -D xrdap.elf.bin
```

dfu-util first sends DFU DETACH to the running application (which writes the mailbox at `0x20007FF0` at the top of RAM and resets); the bootloader stays in DFU and writes the image and the seal. While the board still runs the old XRDAP-CH32V3 application, `-d` is `0d28:2041,1209:3070`. The XRDAP-CH32V3 bootloader does not start the application by itself: after power-up and reset it stays in DFU, and `tools/ch32_usb_run_app.py` of XRDAP-CH32V3 (vendor request `0x5A`) starts the application. Holding the IAP key (PC7) at power-up also keeps it in DFU. Once the application runs, PA13 and PA14 are no longer a debug port and a WCH-Link cannot reach the chip.

The LED is lit while USB is connected to a computer, dark without a connection, and goes dark for a moment on every DAP command, so it flickers under DAP traffic; a single-colour LED cannot show the colours of the status light rule. The chip has only 32 KB of RAM: the DAP packet count is 2 (4 on `evt` and `wdap`) and the sampling ring buffer 2 KB (reported by vendor INFO). The application runs at 216 MHz as on `wdap`, **above** the CH32V305's rated 144 MHz (the XRDAP-CH32V3 application runs at 144 MHz); `XRDAP_APP_SYSCLK_HZ` set to `144000000` in `boards/v305.cmake` returns to the rated clock. The SWD timing is calibrated on this board with a cycle sweep: up to 8 MHz requested the actual clock is within -8 %/+8 % of the request, and the undelayed path (about 17–19 MHz) is used only from requests of about 19 MHz.

## 2. Wireless debugging (wdap)

The SX1281 radio module on the `wdap` board joins two probes into one wireless probe: the host probe is plugged into the PC and the remote probe is wired to the target. The PC uses the host probe as an ordinary CMSIS-DAP probe; the host sends DAP commands over the radio to the remote, which executes them on its own SWD port and sends the answers back. Only two paired boards use the radio; a probe without a pairing does not use it at all, is a wired probe whatever its role setting, and executes no command from the radio.

Both boards run the same firmware, and by default the role is decided automatically. A paired probe without a debug session of its own listens to the radio as a remote; when a tool sends DAP_Connect, the probe first pings its paired peer, and if the peer answers the session runs over the radio, otherwise the probe drives its own SWD port. So the board that is only powered is the remote and the board a tool opens is the host; without the remote, the host is an ordinary wired probe. When the peer drops out during a session the commands return errors rather than moving to the local SWD port; the next DAP_Connect decides again. SWJ_Clock, TransferConfigure and SWD_Configure sent before DAP_Connect are sent on to the remote when the session moves to the radio.

One transport owns a probe at a time (the session owner of [XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core)): while the remote serves a radio session, the radio peer owns it, and commands on the remote's own USB get busy (standard commands `command FF`, vendor commands `command 05`) while DAP_Info is answered as usual; the other way round, a remote held by its local USB answers busy to the radio. When the radio session ends, the sampling it started stops.

### Pairing

Two boards work together once paired. Hold KEY2 on both boards for 3 s, one after the other within 10 s (or send vendor command `0x9F 0x20`): on the default radio parameters (2.44 GHz, sync word `XRDP`) the boards exchange their chip UIDs and derive the parameters of the pair from the two UIDs: frequency (2403–2479 MHz), CRC seed and GFSK sync word. Other pairs match neither frequency nor CRC, so their packets are not received. The pairing is kept in flash across power cycles and firmware updates; pairing the same two boards again gives the same parameters. The parameters separate pairs; they do not authenticate them.

### Settings

Role, modulation and pairing live in the settings area at the end of the application window (see Flash layout) and are read and changed with vendor command `0x9F` (see `app/radio_command.hpp`). `0x9F 0x30` hands a `0x9F` command over the radio to the peer, for example to change the modulation of both boards from the host; without a pairing it and the temporary role override `0x9F 0x10` answer status 4 (not paired).

- Role: automatic (the default), always wired, always host, always remote.
- Modulation (both boards must match):
  - GFSK (the default): 2 Mb/s, packets of up to 255 bytes, at most 5 per 1024-byte message.
  - FLRC: 1.3 Mb/s, packets of up to 127 bytes, 9 per message, without FEC (coding rate 1).

### Link

The packet format and the session rules are written down in `app/link_protocol.hpp` and `app/link_session.hpp`, and `tests/` checks the code against them. When no answer arrives in time the host sends the command again with the same sequence number, waiting twice as long each time; the remote answers a repeated sequence number of the same session with its last answer instead of executing the command again. After 1 s without a message from the peer, the next command pings first and returns an error at once when the ping fails, instead of retrying for a full second.

Every session over the radio has a 16-bit session id, a new one for every session the host opens, carried in the pings. During a session the host sends a heartbeat ping after 1 s without a command; on DAP_Disconnect, or after 5 s without a command from the PC, it ends the session with a ping carrying id 0. The remote ends the session on id 0, on a new id, or after 3 s without any packet from the host: it drops the cached answer and a half-assembled command and stops the sampling the session started. Commands outside a session are not executed.

Continuous receive on the SX1281 can lock up under heavy traffic (datasheet errata 16.1); a radio that hears no packet for 100 ms enters receive again and is reset after 2 s. Received packets are judged by the error byte of GetPacketStatus (errata 16.3), and the radio stays in FS after each transmit or receive (SetAutoFs) for shorter mode switches. FLRC uses the 32-bit sync word `XRDP`, also after pairing: without a sync word the receiver takes noise for packets and the link stops; errata 16.4 does not publish the sync words to avoid at coding rate 1, and at coding rate 3/4 `XRDP` is outside the forbidden range.

A small command takes about 0.6 ms there and back, of which the two transmissions take about 0.3 ms. To shorten it, SPI transfers of up to 24 bytes go straight through the registers instead of DMA; in continuous receive only the packet interrupts reach DIO1, so a high DIO1 means a packet and the interrupt status is not read; the radio leaves receive ahead of the next transmit while the remote executes a command and after the host has its answer; the PA ramps up in 8 µs.

Typical results with the boards about 1 m apart, the host debugging a Cortex-M target with 1 MiB of flash over the radio, at an SWD clock of 30 MHz (10 MHz for OpenOCD):

| Item | GFSK | FLRC |
|---|---|---|
| Block reads, 4 bytes per command | 3.1 KiB/s | 2.5 KiB/s |
| Block reads, 1020 bytes per command | 87–110 KiB/s | 61–70 KiB/s |
| OpenOCD programming 1 MiB | 87–88 KiB/s | 63 KiB/s |
| probe-rs download of 128 KiB (with erase) | 6.9 s | |

Erase (about 6.8 s) and verify (OpenOCD computes a CRC32 on the target, about 980 KiB/s) are the same wired and wireless; wired speeds and the host tools compared are in section 5, Host tools measured. Over more than a hundred thousand consecutive commands, about 1.5% were retried in GFSK and in FLRC and none failed in the end; the share of retries follows the 2.4 GHz interference around.

### Watchdog and safe mode

The application runs the independent watchdog and resets when its main loop has not run for about 8 s. After three watchdog resets in a row it starts in safe mode: the settings area is neither read nor written and the probe works as a wired probe, with DFU as usual; 30 s of normal running or a power cycle ends it.

## 3. Variable access and sampling (XR vendor extension)

The probe supports the XR vendor extension (commands 0x80 to 0x85): reading and writing several addresses at once, following pointer chains, and sampling variables on the probe at a fixed period. The protocol is the one of [XRDAP-ESP](https://github.com/Jiu-xiao/XRDAP-ESP), and `xrdap read`, `write` and `watch` work as they are. The code comes from [XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core) (the `xrdap-core` submodule), with the protocol in section 3 of its `docs/protocol.md`; the sampling task and the TIM7 tick that wakes it are in `app/sampler_task.cpp`. With 64 KB of RAM on the CH32V307 the sampling buffer holds 4096 bytes (16384 on XRDAP-ESP); `xrdap info` reports it.

Over the radio the vendor commands are forwarded to the remote like DAP commands; sampling runs on the remote at its own ticks, the frames collect in the remote's buffer and the host fetches them in batches.

The SWD clock is made in software on GPIO: a setting at or above the speed of the no-delay path (about 14 MHz on `wdap`) takes that path, and any other setting takes the delayed step whose period is closest, within half a step of the setting, slightly faster or slightly slower. The steps are calibrated in CPU cycles (`kSwdTiming` in `app/app_main.cpp`). With SWD set to 25 MHz, wired, N 4-byte variables per frame, the shortest periods without a skipped tick or a dropped frame are typically 73 µs for 1 variable (13.7 kHz), 181 µs for 8 (5.5 kHz) and 670 µs for 32 (1.5 kHz); with many variables the SWD accesses set the period, about 11 µs each.

## 4. Building

CMake 3.20 or newer and WCH RISC-V GCC 15.2 or newer are needed (`riscv32-wch-elf-gcc` on `PATH`, or `-DCOMPILER_PREFIX=<path>/riscv32-wch-elf-`). The bootloader and the application are two CMake projects; `-DXRDAP_BOARD=evt`, `wdap` or `v305` selects the board (`v305` keeps the bootloader of XRDAP-CH32V3 and has no bootloader project):

```
git clone --recursive https://github.com/Jiu-xiao/XRDAP-CH32V307.git
cd XRDAP-CH32V307
cmake -S bootloader -B build/bootloader-evt -DXRDAP_BOARD=evt
cmake --build build/bootloader-evt
cmake -S . -B build/app-evt -DXRDAP_BOARD=evt
cmake --build build/app-evt
```

The images are `build/bootloader-evt/bootloader.elf.bin` and `build/app-evt/xrdap.elf.bin`. `tools/build.sh evt wdap` builds the bootloader and the application for both boards and collects the images in `build/images/<board>/`; `.github/workflows/build.yml` runs it in the `xrimage/xrimage-ch32-riscv` image pinned by digest (WCH GCC 15.2) and uploads the images.

The link protocol, the session rules, the pair identity, DAP forwarding, the settings log and the flash layout are pure logic without hardware and are tested on the build machine (CI runs them too):

```
cmake -S tests -B build/tests
cmake --build build/tests
ctest --test-dir build/tests
```

The firmware version is `project(... VERSION ...)` in `CMakeLists.txt`; the build adds the commit, e.g. `0.2.0+gc82e787` (`.dirty` at the end with uncommitted changes), and DAP_Info reports it to the host. A copy of the sources without `.git` gives the commit with `-DXRDAP_GIT_REV=<commit>`.

## 5. Reference

### Host tools measured

`wdap` wired to a Cortex-M7 target with 1 MiB of flash, SWD set to 30 MHz (about 14 MHz actual). Over the radio a second `wdap` is the host, GFSK. Programming includes erasing the 128 KiB sector; verify computes a CRC on the target. Typical results:

| Tool | Link | Erase + program 128 KiB | Verify | Read 128 KiB of flash |
|---|---|---|---|---|
| WCH OpenOCD 0.11 (bundled with MounRiver Studio) | wired | 1.97 s | 977 KiB/s | 871 KiB/s |
| | radio | 2.42 s | 854 KiB/s | 96 KiB/s |
| OpenOCD 0.12 (openocd-esp32 v0.12.0-esp32-20260831) | wired | 1.96 s | 977 KiB/s | 992 KiB/s |
| | radio | 2.29 s | 853 KiB/s | 97 KiB/s |
| probe-rs 0.31.0 (`download --verify`, `read`) | wired | 3.8 s (with verify) | | 0.35 s (with attach) |
| | radio | 8.3 s (with verify) | | 1.57 s (with attach) |

Programming all 1 MiB over the wire: OpenOCD 0.12 erases in 6.8 s and programs at 418 KiB/s; WCH OpenOCD 0.11 programs at 356–373 KiB/s, as it sends one packet at a time and waits for its answer. Writing RAM with OpenOCD 0.12 runs at 955 KiB/s and reading at 985 KiB/s. Programming is slower than writing RAM because the target writes flash while it receives the data and makes the probe wait while busy.

### Flash layout

| Address | Content |
|---|---|
| `0x00000000`–`0x0000FFFF` | bootloader (64 KB, runs at 96 MHz) |
| `0x00010000`–`0x0003F6FF` | application (144 MHz on `evt`, 216 MHz on `wdap`) |
| `0x0003F700`–`0x0003FEFF` | settings: role, modulation, pairing; kept across firmware updates |
| `0x0003FF00`–`0x0003FFFF` | seal: application size and CRC32, written by the bootloader when a DFU download ends |

The application window of `v305` is `0x00002000`–`0x0001FFFF` (the XRDAP-CH32V3 layout), its last 256 bytes the seal and the 2 KB in front of it the place of the settings, which the firmware does not write.

A DFU download in the bootloader can only write up to the settings: an image larger than `0x2F700` bytes fails to download and the settings stay. Both RAM splits (FLASH 192K + RAM 128K, FLASH 256K + RAM 64K) work; the firmware is linked for 64 KB of RAM, and the top 256 bytes of RAM hold the boot mailbox and the watchdog count (`common/reserved_ram.h`).

The `wdap` application runs at 216 MHz from its 12 MHz crystal times 18, **above** the CH32V307's rated 144 MHz: out-of-spec operation. It is done for the software SWD, whose speed follows the core clock: block reads run about 1.5 times as fast as at 144 MHz. The board files choose the clock (`XRDAP_APP_SYSCLK_HZ` in `boards/wdap.cmake`, implemented in `common/sysclk.c`); `144000000` returns to the rated clock. The bootloader runs at 96 MHz.

### When the bootloader stays in DFU mode

- the bootloader key is held at power-up;
- the boot mailbox at the top of RAM (`0x2000FFF0`) holds a request, which survives a software reset;
- the application area has no seal, the seal's CRC does not match, or the image is not linked for the application address (the second word of the vector table is not `0x00010000`).

### Board files

`boards/<name>.cmake` gives the crystal frequency, the application core clock, which parts the board has (`XRDAP_HAS_LED`, `XRDAP_HAS_RADIO`, `XRDAP_HAS_STATUS_LIGHTS`, `XRDAP_HAS_STATUS_LED`), whether it keeps another project's bootloader (`XRDAP_EXTERNAL_BOOTLOADER`, then also the application window and the boot mailbox magics) and the chip memory and buffer sizes (`XRDAP_RAM_SIZE`, `XRDAP_RTOS_HEAP_SIZE` and others; defaults and meaning in `cmake/ch32v307.cmake`), and `boards/<name>.hpp` the bootloader key, the LED, the status lights, the radio, the target pins, the SWD timing and the set-up before the pins are taken (`Board::Init()`). A new board needs only these two files.

### libxr version

The `libxr` submodule follows the `feat/xrdap-r2-swd-loop` branch, which carries the fixes this project needs: the CH32 USBHS read a high-speed link as full speed at reset (Windows rejected the configuration descriptor); the DFU classes carried their own MS OS 2.0 descriptor, so Windows could not start a composite with the DAP interface; the USBHS sent a packet of a multi-packet IN transfer twice, which shifted DAP answers longer than 512 bytes; the DFU poll deadline and clearing an EP0 stall; and checking a posted AP write by reading RDBUFF, so that the clock stops only once the write is done and fast programming of a busy target loses no data.

## License

Apache-2.0. The WCH startup file, peripheral library and FreeRTOS under `platform/` keep their own licenses.
