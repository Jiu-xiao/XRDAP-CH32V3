# XRDAP-CH32V3

[中文](README.md)

A CMSIS-DAP v2 debug probe built on the CH32V307 and the CH32V305 with [libxr](https://github.com/xrobot-org/libxr). It provides a Bulk v2 DAP interface over USB high speed (480 Mbit/s) with 1024-byte packets, so Keil, pyOCD, probe-rs and OpenOCD use it as an ordinary CMSIS-DAP probe. A USB DFU bootloader sits at the start of the flash, so firmware updates need only a USB cable.

The firmware is built per board; three boards are supported:

| Board | Chip, crystal | Target interface | Bootloader key |
|---|---|---|---|
| `evt`: CH32V307 evaluation board (the board of [CH32V307_LibXR_Template](https://github.com/xrobot-org/CH32V307_LibXR_Template)) | CH32V307VCT6, 8 MHz | SWCLK PB13, SWDIO PB14, NRST PB12 | key on PB3 |
| `wdap`: the "高速无线DAP调试器" board | CH32V307RCT6, 12 MHz | 2×6 header: TCK/SWCLK (pin 3), TMS/SWDIO (pin 4), RST (pin 5) | KEY2 |
| `v305`: the CH32V305 board of this repository's earlier application (end of section 1) | CH32V305FBP6, 12 MHz | SWCLK PB13, SWDIO PA9, NRST PC8 | IAP key PC7 |

## 1. Getting started

### Wiring

Connect the target's SWCLK, SWDIO and GND to the pins above; connect one probe per target. Pins 1 (3V3) and 7 (5V) of the `wdap` header are power outputs of the board; leave them unconnected when the target has its own supply.

### First flashing

The first write is the boot chain `boot-chain.bin`: stage 1 at address 0 and the board's first bootloader slot (slot A on the CH32V307, slot B on `v305`; see "Boot chain"; built by `tools/build.sh`). Connect a WCH-Link or WCH-LinkE (RISC-V mode) to the board's two-wire debug port (SWDIO, SWCLK, GND), for example with the WCH OpenOCD shipped with MounRiver Studio:

```
openocd -f platform/wch-riscv.cfg -c init -c halt -c "flash erase_sector wch_riscv.flash 0 last" -c "program boot-chain.bin 0x00000000 verify" -c "reset run" -c shutdown
```

The bootloader slots of `v305` lie above 128 KB, where the WCH OpenOCD, which checks addresses against 128 KB, cannot write; WCH-LinkUtility writes `boot-chain.bin` at `0x08000000` instead.

A board without a debug header (such as a `wdap` with the header unpopulated) can use the chip's built-in USB download mode instead: with BOOT0 pulled high at power-up the PC sees `4348:55E0`; write `boot-chain.bin` with WCHISPTool_CH32Vxxx, shipped with MounRiver Studio (chip CH32V307, USB download, read protection off), or with [wchisp](https://github.com/ch32-rs/wchisp) (`wchisp flash boot-chain.bin`).

A board with the earlier single bootloader (at address 0, application at `0x00010000`) needs no debugger: write `migrate-to-ab.elf.bin` with `dfu_update.py`; the old bootloader starts it as an application, it writes the boot chain and restarts, and the new bootloader stays in DFU mode. This takes about 2 s; a power cut in that time leaves no working bootloader, and `boot-chain.bin` then has to be written in one of the two ways above. The settings (pairing and the rest) stay.

A `v305` with the three-stage boot (USB `1209:3070`) migrates over DFU the same way, with the `migrate-to-ab.elf.bin` of `v305`:

```
python tools/dfu_update.py --bootloader 1209:3070 --run-app migrate-to-ab.elf.bin
```

It writes slot B first and checks every page; meanwhile the old stage 1 and bootloader stay as they are, and a failure restarts into the old bootloader for another try. Stage 1 comes last, the page with its entry point last of all, so only a power cut during the few milliseconds of that page needs a WCH-Link to recover.

With an empty application area the bootloader stays in DFU mode and the PC sees the USB device `1209:3072` (XR-DAP-V307 DFU, XR-DAP-V305 DFU on `v305`). Write the application as described in the next section.

### Writing and updating the application

```
pip install pyusb libusb-package
python tools/dfu_update.py xrdap.elf.bin
```

Besides the DAP interface the application has a DFU runtime interface. `dfu_update.py` sends it DFU DETACH and the probe restarts into the bootloader; with several probes connected, `--serial` selects one by its serial. Holding the board's bootloader key while plugging in USB also keeps the bootloader in DFU mode. `dfu_update.py` writes the image over standard USB DFU 1.1; the bootloader writes a check record (the seal) at the end and starts the new application. On Windows the bootloader announces WinUSB, so no driver installation is needed. dfu-util works as well (tested with 0.11; it warns that the image has no DFU suffix, which does not affect the download) and also sends DETACH first: `dfu-util -d 1d50:6199,1209:3072 -S XRobot-XRDAP-<UID> -a 0 -D xrdap.elf.bin`. From then on the bootloader starts the application at every power-up once the application area passes its check, and the PC sees `1D50:6199` (XR-DAP-V307 CMSIS-DAP) with the serial `XRobot-XRDAP-` followed by the chip UID.

The bootloader is updated together with the application: `--bootloader-dir` names the output directory of `tools/build.sh` (`build/images/<board>/`, which holds `bootloader-a.elf.bin` and `bootloader-b.elf.bin`).

```
python tools/dfu_update.py --bootloader-dir build/images/wdap build/images/wdap/xrdap.elf.bin
```

In DFU mode the bootloader has two interfaces: interface 0 (`XR-DAP application`) writes the application, interface 1 (`XR-DAP bootloader slot A` or `... slot B`) the other bootloader slot. `dfu_update.py` compares the version of that slot's image in the directory with the running version in the USB product string and writes the bootloader first when they differ: the bootloader closes the slot with its record, restarts in DFU mode, stage 1 starts the new bootloader, and the application follows. With the same version the bootloader is not written (`--force-bootloader` writes it anyway).

`xrdap info` from [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host) confirms the probe: it lists the probe name, serial, firmware version, CMSIS-DAP packet size and the vendor extension's capabilities, then connects to the target and reads its DPIDR and core state.

### Status lights (wdap)

The 8 WS2812B on the `wdap` board show the debug state, with the colours of the status light rule shared by the XR-DAP family ([XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core)). A rainbow sweeps once at power-up. The chain is red while USB is connected to a computer, blue during a debug session over the radio (on the host, and on the remote while its remote session is open), otherwise purple; pairing blinks purple fast. While a tool reports connected through DAP_HostStatus the main colour fades to purple and back every 2 s, and while the target runs red and blue alternate every second; DAP_Disconnect, or 5 s without any command, returns to the main colour. Every DAP command lights the chain up and moves a white dot one LED on, about 15 times a second under steady traffic.

The chain's data line PB15 is SPI2's MOSI: each data bit becomes a run of SPI bits after the WS2812B-2020 datasheet timing (SPI clock at most 18 MHz), a frame goes out by DMA, and the CPU encodes a frame every 20 ms.

### CH32V305 board (v305)

`v305` is the CH32V305 board of this repository's earlier application (tag `legacy-v305-app`) (CH32V305FBP6, 12 MHz crystal, the circuit of the WCH-LinkE). On it this firmware is a wired USB high-speed DAP only, without radio, with the product name `XR-DAP-V305` (USB product string `XR-DAP-V305 CMSIS-DAP`, USB ids of the family), and with the pins of the earlier application:

| Function | Pin |
|---|---|
| SWCLK | PB13 (wired to PA14 on the board) |
| SWDIO | PA9 (bonded to PA13 in the package), one bidirectional pin, push-pull |
| Target reset | PC8 |
| LED | PC9 (bonded to PA8), lit when low |
| Held low after start-up | PA5, PB12 (target power switches) |

The boot chain is the one of the CH32V307 boards (see "Boot chain"); the bootloader key is the IAP key (S1 on the WCH-LinkE schematic, on PC7 with an external 10K pull-up), and the LED on PC9 blinks while the bootloader is in DFU mode. The other key, ModeS (S2), sits on the crystal output PD1 through a 0 Ω resistor: pressing it stops the crystal and drops USB, and the firmware does not use it. The application window is the one of the earlier three-stage boot (`0x2000`–`0x20000`, settings and seal at its end), and the two bootloader slots lie above 128 KB (see "Flash layout"). The application and the bootloader are written and updated as on the other boards; a board that still has the three-stage boot migrates first as described under "First installation".

While the board still runs the earlier application (`0d28:2041`), which the tool's DETACH does not look for, hold the IAP key at power-up so that the bootloader of the three-stage boot stays in DFU, and write the migration image. Once the application runs, PA13 and PA14 are no longer a debug port and a WCH-Link cannot reach the chip.

The LED is lit while USB is connected to a computer, dark without a connection, and goes dark for a moment on every DAP command, so it flickers under DAP traffic; a single-colour LED cannot show the colours of the status light rule. The chip has only 32 KB of RAM: the DAP packet count is 2 (4 on `evt` and `wdap`) and the sampling ring buffer 2 KB (reported by vendor INFO). The application runs at 216 MHz as on `wdap`, **above** the CH32V305's rated 144 MHz (so did the earlier application: it kept the times-18 PLL setting written for an 8 MHz crystal, and this board has a 12 MHz one); at 216 MHz the chip can run too hot, and restarts and similar problems can occur. `v305-144` is the same board at the rated 144 MHz, everything else is as on `v305`. OpenOCD 0.12 reads 128 KiB of STM32F302 flash at SWD 30 MHz at about 1174 KiB/s at 216 MHz and about 795 KiB/s at 144 MHz. The SWD timing is calibrated on this board with a cycle sweep: up to 8 MHz requested the actual clock is within -8 %/+8 % of the request, and the undelayed path (about 17–19 MHz) is used only from requests of about 19 MHz.

## 2. Wireless debugging (wdap)

The SX1281 radio module on the `wdap` board joins two probes into one wireless probe: the host probe is plugged into the PC and the remote probe is wired to the target. The PC uses the host probe as an ordinary CMSIS-DAP probe; the host sends DAP commands over the radio to the remote, which executes them on its own SWD port and sends the answers back. Only two paired boards use the radio; a probe without a pairing does not use it at all, is a wired probe whatever its role setting, and executes no command from the radio.

Both boards run the same firmware, and by default the role is decided automatically. A paired probe without a debug session of its own listens to the radio as a remote; when a tool sends DAP_Connect, the probe first pings its paired peer, and if the peer answers the session runs over the radio, otherwise the probe drives its own SWD port. So the board that is only powered is the remote and the board a tool opens is the host; without the remote, the host is an ordinary wired probe. When the peer drops out during a session the commands return errors rather than moving to the local SWD port; the next DAP_Connect decides again. SWJ_Clock, TransferConfigure and SWD_Configure sent before DAP_Connect are sent on to the remote when the session moves to the radio.

That the probe is used over one link at a time (USB or the radio) is up to the user; the probe does not arbitrate ([XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core) `docs/protocol.md`). When the radio session ends, the sampling it started stops.

### Pairing

Two boards work together once paired. Hold KEY2 on both boards for 3 s, one after the other within 10 s (or send control request `0x20`): on the default radio parameters (2.44 GHz, sync word `XRDP`) the boards exchange their chip UIDs and derive the parameters of the pair from the two UIDs: frequency (2403–2479 MHz), CRC seed and GFSK sync word. Other pairs match neither frequency nor CRC, so their packets are not received. The pairing is kept in flash across power cycles and firmware updates; pairing the same two boards again gives the same parameters. The parameters separate pairs; they do not authenticate them.

### Settings

Role, modulation and pairing live in the settings area at the end of the application window (see Flash layout; LibXR's `DatabaseRawSequential`) and are read and changed with control port requests (XRDAP-Core `docs/protocol.md` 3.7.1 and 8.3), not DAP commands. Over USB they are EP0 vendor control transfers (`bmRequestType 0x41 / 0xC1`, request code 1 sends the request, 2 takes the answer), over the radio the `CONTROL` packet kind. Request `0x30` hands a control request over the radio to the peer, for example to change the modulation of both boards from the host; without a pairing it and the temporary role override `0x10` answer status 4 (not paired). Firmware before 0.6 keeps the settings in another format, so two boards updated from it are paired again.

- Role: automatic (the default), always wired, always host, always remote.
- Modulation (both boards must match):
  - GFSK (the default): 2 Mb/s, packets of up to 255 bytes, at most 5 per 1024-byte message.
  - FLRC: 1.3 Mb/s, packets of up to 127 bytes, 9 per message, without FEC (coding rate 1).

### Link

The packet format, the session rules and the radio sessions are implemented in XRDAP-Core's `xrdap/radio/` (`docs/protocol.md` section 8), whose host tests check the code against the rules. When no answer arrives in time the host sends the command again with the same sequence number, waiting twice as long each time; the remote answers a repeated sequence number of the same session with its last answer instead of executing the command again. After 1 s without a message from the peer, the next command pings first and returns an error at once when the ping fails, instead of retrying for a full second.

Every session over the radio has a 16-bit session id, a new one for every session the host opens, carried in the pings. During a session the host sends a heartbeat ping after 1 s without a command; on DAP_Disconnect, or after 5 s without a command from the PC, it ends the session with a ping carrying id 0. The remote ends the session on id 0, on a new id, or after 3 s without any packet from the host: it drops the cached answer and a half-assembled command and stops the sampling the session started. Commands outside a session are not executed.

Continuous receive on the SX1281 can lock up under heavy traffic (datasheet errata 16.1); a radio that hears no packet for 100 ms enters receive again and is reset after 2 s. Received packets are judged by the error byte of GetPacketStatus (errata 16.3), and the radio stays in FS after each transmit or receive (SetAutoFs) for shorter mode switches. FLRC uses the 32-bit sync word `XRDP`, also after pairing: without a sync word the receiver takes noise for packets and the link stops; errata 16.4 does not publish the sync words to avoid at coding rate 1, and at coding rate 3/4 `XRDP` is outside the forbidden range.

The radio driver is the xrobot-org/SX1281 Module, which has no thread of its own and is advanced by the main loop. To shorten the round trip, SPI transfers to the SX1281 are polled instead of using DMA; only TX done, RX done and CRC error reach DIO1, which is read by level; the radio leaves receive ahead of the next transmit while the remote executes a command and after the host has its answer; the PA ramps up in 8 µs.

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

CMake 3.20 or newer and WCH RISC-V GCC 15.2 or newer are needed (`riscv32-wch-elf-gcc` on `PATH`, or `-DCOMPILER_PREFIX=<path>/riscv32-wch-elf-`). The boot chain (`bootloader/`) and the application are two CMake projects; `-DXRDAP_BOARD=evt`, `wdap`, `wdap-144`, `v305` or `v305-144` selects the board:

```
git clone --recursive https://github.com/Jiu-xiao/XRDAP-CH32V3.git
cd XRDAP-CH32V3
cmake -S bootloader -B build/bootloader-evt -DXRDAP_BOARD=evt
cmake --build build/bootloader-evt
cmake -S . -B build/app-evt -DXRDAP_BOARD=evt
cmake --build build/app-evt
```

The boot chain project builds `stage1.bin`, `bootloader-a.elf.bin` and `bootloader-b.elf.bin`, with Python 3 also `boot-chain.bin`/`.hex` (`tools/boot_chain.py`) and `migrate-to-ab.elf.bin`; the application is `build/app-evt/xrdap.elf.bin`. `tools/build.sh evt wdap wdap-144 v305 v305-144` builds the boot chain and the application for these configurations and collects the images in `build/images/<board>/`; `.github/workflows/build.yml` runs it in the `xrimage/xrimage-ch32-riscv` image pinned by digest (WCH GCC 15.2) and uploads the images.

The flash layout and the bootloader slot rules are pure logic without hardware and are tested on the build machine (CI runs them too; the radio link's logic is tested in XRDAP-Core):

```
cmake -S tests -B build/tests
cmake --build build/tests
ctest --test-dir build/tests
```

The firmware version is `XRDAP_VERSION` in `cmake/version.cmake`; the build adds the commit, e.g. `0.2.0+gc82e787` (`.dirty` at the end with uncommitted changes). The application reports it in DAP_Info, the bootloader in its USB product string and its slot record. A copy of the sources without `.git` gives the commit with `-DXRDAP_GIT_REV=<commit>`.

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
| `0x00000000`–`0x00000FFF` | stage 1 (`bootloader/stage1.S`, never updated) |
| `0x00001000`–`0x0000DFFF` | bootloader slot A (52 KB, runs at 96 MHz; the last 256 bytes are the slot record) |
| `0x0000E000`–`0x0001AFFF` | bootloader slot B (likewise) |
| `0x0001B000`–`0x0003F6FF` | application (144 MHz on `evt` and `wdap-144`, 216 MHz on `wdap`) |
| `0x0003F700`–`0x0003FEFF` | settings: role, modulation, pairing; kept across firmware updates |
| `0x0003FF00`–`0x0003FFFF` | seal: application size and CRC32, written by the bootloader when a DFU download ends |

The layout of `v305`:

| Address | Content |
|---|---|
| `0x00000000`–`0x00000FFF` | stage 1 |
| `0x00002000`–`0x0001F6FF` | application (`v305` 216 MHz, `v305-144` 144 MHz) |
| `0x0001F700`–`0x0001FEFF` | settings (the firmware does not write them) |
| `0x0001FF00`–`0x0001FFFF` | seal |
| `0x00020000`–`0x0002CFFF` | bootloader slot A |
| `0x0002D000`–`0x00039FFF` | bootloader slot B |

The CH32V305FB data sheet gives 128 KB of flash; the flash goes on above it, and the earlier stage-3 bootloader ran at `0x20000` as well. The application sits in the zero-wait area of the first 128 KB, the bootloader slots above it.

A DFU download in the bootloader can only write up to the settings: an application image larger than `0x24700` bytes (`0x1D700` on `v305`) fails to download and the settings stay. The settings and the seal sit at the addresses of the earlier single-bootloader layout, so a migrated board keeps its settings. The firmware is linked for 64 KB of RAM; the application window reaches beyond 192 KB, so with the user option FLASH 256K + RAM 64K the whole application runs from the zero-wait area, with FLASH 192K + RAM 128K the part beyond 192 KB runs slower; and the top 256 bytes of RAM hold the boot mailbox and the watchdog count (`common/reserved_ram.h`).

The `wdap` application runs at 216 MHz from its 12 MHz crystal times 18, **above** the CH32V307's rated 144 MHz. It is done for the software SWD, whose speed follows the core clock. Measured wired on an STM32H723 target: OpenOCD 0.12 reads 1 MiB of flash at SWD 30 MHz at about 1000–1010 KiB/s at 216 MHz and about 680 KiB/s at 144 MHz; 1 KiB block reads sent one at a time, each after the previous answer, run at about 713 KiB/s at 216 MHz and 505–545 KiB/s at 144 MHz. Over the radio both run at about 84–88 KiB/s, bound by the air rate. At 216 MHz the chip can run too hot, and restarts and similar problems can occur. `wdap-144` is the same board at the rated 144 MHz, everything else is as on `wdap`. The board files choose the clock (`XRDAP_APP_SYSCLK_HZ` in `boards/wdap.cmake` and `boards/wdap-144.cmake`, implemented in `common/sysclk.c`). The bootloader runs at 96 MHz.

### Boot chain

At every reset stage 1 at address 0 (`bootloader/stage1.S`) reads the bootloader key, checks the slot records at the end of the two bootloader slots (`common/boot_slots.hpp`: magic, sequence number and its complement, size, CRC32 and its complement, slot address, version) and starts the valid slot with the larger sequence number; with the bootloader key held at power-up it starts the other valid slot, whose bootloader sees the key as well and stays in DFU mode, to go back to the previous bootloader. With no valid slot stage 1 stops, and only a WCH-Link or the chip's USB download mode (which `v305` lacks) can write `boot-chain.bin`.

The same bootloader code is linked once for each slot. The running bootloader writes only the other slot: a download first erases that slot's record, and once the image is written and checked the record is written last, with a sequence number one more than its own; a download that stops or a power cut leaves that slot without a record, and stage 1 keeps starting the slot it started before. Images linked for the other slot's address and images larger than `0xCF00` bytes are refused; the CH32V307 boards and `v305` have different slot addresses, so each refuses the other's bootloader images. The application's control request `0x17` (BootInfo, `app/boot_info.hpp`) reports the running slot and the sequence numbers and versions of both slots.

### When the bootloader stays in DFU mode

- the bootloader key is held at power-up;
- the boot mailbox at the top of RAM (`0x2000FFF0`, `0x20007FF0` on `v305`) holds a request, which survives a software reset (the application asks through it on DFU DETACH, the bootloader after it wrote the other slot);
- the application area has no seal, the seal's CRC does not match, or the image is not linked for the application address (the second word of the vector table is not `0x0001B000`, `0x00002000` on `v305`).

### Board files

`boards/<name>.cmake` gives the crystal frequency, the bootloader key (`XRDAP_BOOT_KEY_PORT`, `XRDAP_BOOT_KEY_PIN`, read by stage 1 as well), the application core clock, the bootloader's LED (`XRDAP_HAS_LED`), the application's composition (`XRDAP_BOARD_APP`, that is `boards/<name>.cpp`) and the sources of the parts it uses (`XRDAP_BOARD_SOURCES`), the flash layout and bootloader parameters of a chip that differs from the default CH32V307 (application window, bootloader slot addresses, first slot, heap and product string of the bootloader, boot mailbox magics; `boards/v305.cmake` is an example) and the chip memory and buffer sizes (`XRDAP_RAM_SIZE`, `XRDAP_C_HEAP_SIZE` and others; defaults and meaning in `cmake/ch32v3.cmake`), and `boards/<name>.hpp` the bootloader key, the LED, the status lights, the radio, the target pins, the SWD timing and the set-up before the pins are taken (`Board::Init()`), and `boards/<name>.cpp` constructs the common probe (`app/probe.hpp`) and the parts the board has on top (radio, status lights, ...; `app/part.hpp`) and runs the main loop. A new board needs these three files.

### libxr version

The `libxr` submodule follows the `release/0.6-candidate` branch, which carries the fixes this project needs: the CH32 USBHS read a high-speed link as full speed at reset (Windows rejected the configuration descriptor); the DFU classes carried their own MS OS 2.0 descriptor, so Windows could not start a composite with the DAP interface; the USBHS sent a packet of a multi-packet IN transfer twice, which shifted DAP answers longer than 512 bytes; the DFU poll deadline and clearing an EP0 stall; and checking a posted AP write by reading RDBUFF, so that the clock stops only once the write is done and fast programming of a busy target loses no data.

## License

Apache-2.0. The WCH startup file and peripheral library under `platform/` keep their own licenses.
