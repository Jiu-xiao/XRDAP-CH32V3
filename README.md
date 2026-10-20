# XRDAP-CH32V307

[English](README.en.md)

用 CH32V307 做的 CMSIS-DAP v2 调试探针，基于 [libxr](https://github.com/xrobot-org/libxr)。探针经 USB 高速（480 Mbit/s）接口提供 Bulk v2 DAP 接口，包长 1024 字节，Keil、pyOCD、probe-rs、OpenOCD 把它当普通的 CMSIS-DAP 探针。Flash 开头是一个 USB DFU bootloader，固件升级只需一根 USB 线。

固件按板子编译，目前有两种板子：

| 板子 | 芯片、晶振 | 目标接口 | bootloader 键 |
|---|---|---|---|
| `evt`：CH32V307 评估板（[CH32V307_LibXR_Template](https://github.com/xrobot-org/CH32V307_LibXR_Template) 用的板子） | CH32V307VCT6，8 MHz | SWCLK PB13，SWDIO PB14，NRST PB12 | PB3 按键 |
| `wdap`：“高速无线 DAP 调试器”板 | CH32V307RCT6，12 MHz | 2×6 排针：TCK/SWCLK（3 脚）、TMS/SWDIO（4 脚）、RST（5 脚） | KEY2 |

## 1. 准备

### 接线

目标的 SWCLK、SWDIO、GND 接到上表的引脚，一个目标只接一个探针。`wdap` 板排针的 1 脚（3V3）和 7 脚（5V）是板子的电源输出，目标自己有电源时不接。

### 第一次烧录

第一次用 WCH-Link 或 WCH-LinkE（RISC-V 模式）接板子的两线调试口（SWDIO、SWCLK、GND），把 bootloader 烧到地址 0，例如用 MounRiver Studio 带的 WCH OpenOCD：

```
openocd -f platform/wch-riscv.cfg -c init -c halt -c "flash erase_sector wch_riscv.flash 0 last" -c "program bootloader.elf.bin 0x00000000 verify" -c "reset run" -c shutdown
```

没有调试口的板子（如未焊排针的 `wdap`）可以用芯片自带的 USB 下载模式：上电时拉高 BOOT0，电脑上出现 `4348:55E0`，用 MounRiver Studio 带的 WCHISPTool_CH32Vxxx 选芯片 CH32V307、下载方式 USB，把 `bootloader.elf.bin` 写入，不开读保护。

应用区为空时 bootloader 停在 DFU 模式，电脑上出现 USB 设备 `1209:3072`（XR-DAP-V307 DFU），然后按下一节写入应用。

### 写入、升级应用

```
pip install pyusb libusb-package
python tools/dfu_update.py xrdap.elf.bin
```

应用除 DAP 接口外还有一个 DFU runtime 接口。`dfu_update.py` 向它发 DFU DETACH，探针重启进入 bootloader；接了多块探针时用 `--serial` 指定序列号。也可以按住板上的 bootloader 键插上 USB，bootloader 同样停在 DFU 模式。`dfu_update.py` 经标准 USB DFU 1.1 写入镜像，bootloader 在末尾写入校验记录（seal）并启动新应用。Windows 上 bootloader 声明 WinUSB，不用另装驱动。之后每次上电，bootloader 校验应用区通过就直接启动应用，电脑上出现 `1D50:6199`（XR-DAP-V307 CMSIS-DAP），序列号为 `XRobot-XRDAP-` 加芯片 UID。

用 [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host) 的 `xrdap` 可以确认探针：

```
> xrdap -p usb:XRobot-XRDAP-68E3BE7B54507AAF39E339E3 info
连接：USB XRobot-XRDAP-68E3BE7B54507AAF39E339E3
探针：XRobot XR-DAP-V307，序列号 XRobot-XRDAP-68E3BE7B54507AAF39E339E3，固件 0.1.0
CMSIS-DAP：2.0.0，包长 1024 字节，最多 4 个包在途
```

## 2. 自己编译

需要 CMake 3.20 以上和 WCH RISC-V GCC 15.2 以上（`riscv32-wch-elf-gcc` 在 `PATH` 里，或用 `-DCOMPILER_PREFIX=<路径>/riscv32-wch-elf-` 指定）。bootloader 和应用是两个 CMake 工程，用 `-DXRDAP_BOARD=evt` 或 `wdap` 选板子：

```
git clone --recursive https://github.com/Jiu-xiao/XRDAP-CH32V307.git
cd XRDAP-CH32V307
cmake -S bootloader -B build/bootloader-evt -DXRDAP_BOARD=evt
cmake --build build/bootloader-evt
cmake -S . -B build/app-evt -DXRDAP_BOARD=evt
cmake --build build/app-evt
```

产物为 `build/bootloader-evt/bootloader.elf.bin` 和 `build/app-evt/xrdap.elf.bin`。

## 3. 参考

### Flash 布局

| 地址 | 内容 |
|---|---|
| `0x00000000`–`0x0000FFFF` | bootloader（64 KB，运行在 96 MHz） |
| `0x00010000`–`0x0003FEFF` | 应用（运行在 144 MHz） |
| `0x0003FF00`–`0x0003FFFF` | seal：应用大小和 CRC32，由 bootloader 在 DFU 下载结束时写入 |

两种 RAM 配置（FLASH 192K + RAM 128K、FLASH 256K + RAM 64K）都可用，固件按 64 KB RAM 链接。

### bootloader 何时停在 DFU 模式

- 上电时按住 bootloader 键；
- RAM 顶部的启动信箱（`0x2000FFF0`）里有进入 bootloader 的请求，软件复位后保留；
- 应用区没有 seal、seal 的 CRC 不符，或镜像不是按应用地址链接的（向量表第二个字不等于 `0x00010000`）。

### 板子文件

`boards/<名字>.cmake` 写晶振频率，`boards/<名字>.hpp` 写 bootloader 键、指示灯和目标引脚。增加一种板子只需要这两个文件。

### libxr 版本

`libxr` 子模块固定在 `fix/usb-speed-after-reset` 分支：它在 `feat/xrdap-serial-transport` 之上修正了 CH32 USBHS 在复位时把高速连接读成全速的问题（否则 Windows 判定配置描述符无效）。

## 许可证

Apache-2.0。`platform/` 下 WCH 的启动文件、外设库和 FreeRTOS 保留各自的许可证。
