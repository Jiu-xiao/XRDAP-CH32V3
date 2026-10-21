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

应用除 DAP 接口外还有一个 DFU runtime 接口。`dfu_update.py` 向它发 DFU DETACH，探针重启进入 bootloader；接了多块探针时用 `--serial` 指定序列号。也可以按住板上的 bootloader 键插上 USB，bootloader 同样停在 DFU 模式。`dfu_update.py` 经标准 USB DFU 1.1 写入镜像，bootloader 在末尾写入校验记录（seal）并启动新应用。Windows 上 bootloader 声明 WinUSB，不用另装驱动。也可以用 dfu-util（0.11 实测可用，会提示镜像没有 DFU 后缀，不影响写入），它同样先发 DETACH 再写入：`dfu-util -d 1d50:6199,1209:3072 -S XRobot-XRDAP-<UID> -a 0 -D xrdap.elf.bin`。之后每次上电，bootloader 校验应用区通过就直接启动应用，电脑上出现 `1D50:6199`（XR-DAP-V307 CMSIS-DAP），序列号为 `XRobot-XRDAP-` 加芯片 UID。

用 [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host) 的 `xrdap` 可以确认探针：

```
> xrdap -p usb:XRobot-XRDAP-68E3BE7B54507AAF39E339E3 info
连接：USB XRobot-XRDAP-68E3BE7B54507AAF39E339E3
探针：XRobot XR-DAP-V307，序列号 XRobot-XRDAP-68E3BE7B54507AAF39E339E3，固件 0.1.0
CMSIS-DAP：2.0.0，包长 1024 字节，最多 4 个包在途
```

## 2. 无线调试（wdap）

`wdap` 板上的 SX1281 射频模块可以把两块探针连成一套无线探针：主机端插在电脑上，远端接目标。电脑把主机端当普通 CMSIS-DAP 探针使用，主机端把 DAP 命令经无线发给远端，远端在自己的 SWD 口上执行后把应答发回。

两块板烧同一个固件，角色默认自动决定。探针没有自己的调试会话时作为远端监听无线；上位机发出 DAP_Connect 时，探针先 ping 已配对的另一块，有回应则这次会话走无线，没有回应则驱动自己的 SWD 口。所以只供电的那块是远端，被上位机打开的那块是主机端；远端不在时，主机端就是一块普通的有线探针。会话中途对端掉线时命令返回错误，不会改用本地的 SWD 口，下一次 DAP_Connect 时重新判断。DAP_Connect 之前发来的 SWJ_Clock、TransferConfigure、SWD_Configure 在会话转到无线时补发给远端。

### 配对

两块新板使用相同的默认无线参数（2.44 GHz、同步字 `XRDP`），不配对也能一起用；同一处有多套时需要配对。在 10 s 内先后把两块板上的 KEY2 按住 3 s，两块板交换芯片 UID，由两个 UID 算出这一对的链路参数：频点（2403–2479 MHz）、CRC 种子和 GFSK 同步字。别的设备对频点和 CRC 都对不上，它们的包收不到。配对结果存在 flash 里，断电和固件升级后保留；同两块板重新配对得到相同的参数。

### 设置

角色、调制方式和配对结果存在应用窗口末尾的设置区（见 Flash 布局），用厂商命令 `0x9F` 查看和修改（见 `app/radio_command.hpp`）。`0x9F 0x30` 把一条 `0x9F` 命令经无线交给对端执行，例如在主机端上同时改两块板的调制方式。

- 角色：自动（默认）、总是有线、总是主机端、总是远端。
- 调制方式（两块板必须相同）：
  - GFSK（默认）：2 Mb/s，每包最长 255 字节，一条 1024 字节的消息最多 5 包。
  - FLRC：1.3 Mb/s，每包最长 127 字节，一条消息分成 9 包。编译选项 `XRDAP_RADIO_CODING_RATE` 选编码率，默认 `0x04` 不纠错，`0x02` 为 3/4，`0x00` 为 1/2。

### 链路

主机端等不到应答时用同一序号重发，等待时间逐次加倍；远端对重复的序号直接重发上一条应答，不再执行一遍命令。对端 1 s 没有消息后，下一条命令先 ping，ping 不通时立即返回错误，不必等完重发的 1 s。

SX1281 的连续接收在信道拥挤时可能卡死（数据手册勘误 16.1），实测也出现过停在接收状态却收不到包的情况；射频 100 ms 收不到任何包时重新进入接收，2 s 收不到时复位。收到的包按 GetPacketStatus 的错误字节判断 CRC（勘误 16.3）；收发结束后停在 FS 模式（SetAutoFs），缩短下一次收发的切换时间。手册推荐的单次接收和发射双缓冲都要求包不超过 127 字节，实测反而更慢，保留为编译选项 `XRDAP_RADIO_RX_SINGLE`、`XRDAP_RADIO_TX_PINGPONG`，默认关闭（见 `app/sx1281.hpp`）。FLRC 使用 32 位同步字 `XRDP`，配对后也不换：关掉同步字时接收端把噪声当成包，链路不通；勘误 16.4 对编码率 1 下应避开的同步字没有公开，编码率 3/4 下 `XRDP` 不在禁用范围内。

一条小命令的往返约 0.6 ms，其中两次空中传输约 0.3 ms。为缩短往返：24 字节以内的 SPI 传输直接读写寄存器，不走 DMA；连续接收时 DIO1 只接收包中断，拉高即有包，不再读中断状态；远端执行命令期间、主机端收到应答后，射频提前离开接收、准备发射；PA 爬坡时间 8 µs。

两块板放在同一张桌上，主机端经无线调试 STM32H723（1 MiB flash），SWD 时钟设为 30 MHz（OpenOCD 为 10 MHz）时的实测结果：

| 项目 | GFSK | FLRC | 有线 |
|---|---|---|---|
| 块读，每条命令 4 字节 | 3.1 KiB/s | 2.5 KiB/s | |
| 块读，每条命令 1020 字节 | 87–110 KiB/s | 61–70 KiB/s | |
| OpenOCD 烧写 1 MiB | 87–88 KiB/s | 63 KiB/s | 118.7 KiB/s |
| probe-rs 下载 128 KiB（含擦除） | 6.9 s | | |

有线烧写的 118.7 KiB/s 是 H7 写 flash 本身的速度；擦除（约 6.8 s）和校验（OpenOCD 在目标上计算 CRC32，约 980 KiB/s）有线、无线相同。缩短往返之前，同样条件下 4 字节块读为 2.4 KiB/s，OpenOCD 烧写 84 KiB/s，probe-rs 下载 7.7 s；1020 字节块读受空中时间限制，没有变化。各连续 11.7 万条命令中 GFSK 重发 1.4%、FLRC 重发 1.8%，没有一条最终失败；重发比例随周围 2.4 GHz 干扰变化。

### 看门狗与安全模式

应用开启独立看门狗，主循环约 8 s 没有运行即复位。连续三次看门狗复位后应用以安全模式启动：不读写设置区，只作为有线探针工作，DFU 照常可用；正常运行 30 s 或重新上电后恢复。

## 3. 自己编译

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

## 4. 参考

### Flash 布局

| 地址 | 内容 |
|---|---|
| `0x00000000`–`0x0000FFFF` | bootloader（64 KB，运行在 96 MHz） |
| `0x00010000`–`0x0003F6FF` | 应用（`evt` 运行在 144 MHz，`wdap` 运行在 216 MHz） |
| `0x0003F700`–`0x0003FEFF` | 设置区：角色、调制方式、配对，固件升级时保留 |
| `0x0003FF00`–`0x0003FFFF` | seal：应用大小和 CRC32，由 bootloader 在 DFU 下载结束时写入 |

两种 RAM 配置（FLASH 192K + RAM 128K、FLASH 256K + RAM 64K）都可用，固件按 64 KB RAM 链接。

`wdap` 的应用用 12 MHz 晶振 18 倍频跑在 216 MHz，超过 CH32V307 标称的 144 MHz。SWD 由 GPIO 软件产生，速度与主频成正比：读 STM32G431 的块读速度为 1188 KiB/s（144 MHz 时为 790 KiB/s），20 分钟连续读取 1.3 GiB 数据无误。板级文件 `boards/wdap.cmake` 的 `XRDAP_APP_SYSCLK_HZ` 改为 `144000000` 即回到标称频率。

### bootloader 何时停在 DFU 模式

- 上电时按住 bootloader 键；
- RAM 顶部的启动信箱（`0x2000FFF0`）里有进入 bootloader 的请求，软件复位后保留；
- 应用区没有 seal、seal 的 CRC 不符，或镜像不是按应用地址链接的（向量表第二个字不等于 `0x00010000`）。

### 板子文件

`boards/<名字>.cmake` 写晶振频率，`boards/<名字>.hpp` 写 bootloader 键、指示灯和目标引脚。增加一种板子只需要这两个文件。

### libxr 版本

`libxr` 子模块固定在 `fix/usb-speed-after-reset` 分支，它在 `feat/xrdap-serial-transport` 之上修正了三个问题：CH32 USBHS 在复位时把高速连接读成全速（Windows 判定配置描述符无效）；DFU 类自带一份 MS OS 2.0 描述符，与 DAP 组成复合设备时 Windows 无法启动；USBHS 在多包 IN 传输中把已发出的包再发一遍，超过 512 字节的 DAP 应答因此错位。

## 许可证

Apache-2.0。`platform/` 下 WCH 的启动文件、外设库和 FreeRTOS 保留各自的许可证。
