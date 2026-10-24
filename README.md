# XRDAP-CH32V3

[English](README.en.md)

用 CH32V307 和 CH32V305 做的 CMSIS-DAP v2 调试探针，基于 [libxr](https://github.com/xrobot-org/libxr)。探针经 USB 高速（480 Mbit/s）接口提供 Bulk v2 DAP 接口，包长 1024 字节，Keil、pyOCD、probe-rs、OpenOCD 把它当普通的 CMSIS-DAP 探针。Flash 开头是一个 USB DFU bootloader，固件升级只需一根 USB 线。

固件按板子编译，目前有三种板子：

| 板子 | 芯片、晶振 | 目标接口 | bootloader 键 |
|---|---|---|---|
| `evt`：CH32V307 评估板（[CH32V307_LibXR_Template](https://github.com/xrobot-org/CH32V307_LibXR_Template) 用的板子） | CH32V307VCT6，8 MHz | SWCLK PB13，SWDIO PB14，NRST PB12 | PB3 按键 |
| `wdap`：“高速无线 DAP 调试器”板 | CH32V307RCT6，12 MHz | 2×6 排针：TCK/SWCLK（3 脚）、TMS/SWDIO（4 脚）、RST（5 脚） | KEY2 |
| `v305`：CH32V305 板（本仓库早期应用的板子，见 1 节末尾） | CH32V305FBP6，12 MHz | SWCLK PB13，SWDIO PA9，NRST PC8 | 三段式 bootloader（`bootloader-ch32v3`）；IAP 键 PC7 |

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

用 [XRDAP-Host](https://github.com/Jiu-xiao/XRDAP-Host) 的 `xrdap info` 可以确认探针：它列出探针名、序列号、固件版本、CMSIS-DAP 包长和厂商扩展的能力，并连接目标读出 DPIDR 和内核状态。

### 状态灯（wdap）

`wdap` 板上 8 颗 WS2812B 显示调试状态，灯色按 XR-DAP 全系列共用的状态灯规则（[XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core)）。上电时彩虹扫过一次。USB 已连上电脑时为红色；调试会话走无线时为蓝色（主机端，以及远端会话打开期间的远端）；都没有时为紫色；配对进行中紫色快闪。上位机经 DAP_HostStatus 报告已连接时主色与紫色来回（2 s 一次），目标运行中红蓝来回（1 s 一次）；DAP_Disconnect 或 5 s 没有任何命令时回到主色。每条 DAP 命令让灯链亮一下，并让一颗白点前进一格，命令不断时约每秒 15 次。

灯由 TIM1_CH3N（PB15）的 PWM 加 DMA 驱动，每位的高电平宽度按 WS2812B-2020 数据手册的时序设置，CPU 每 20 ms 只填一次缓冲。

### CH32V305 板（v305，本固件尚未在硬件上验证）

`v305` 是本仓库早期应用（标签 `legacy-v305-app`）用的 CH32V305 板（CH32V305FBP6、12 MHz 晶振，电路与 WCH-LinkE 相同）。本固件在它上面只做有线 USB 高速 DAP，没有射频，产品名 `XR-DAP-V305`（USB 产品字符串 `XR-DAP-V305 CMSIS-DAP`，USB 标识与全系列相同），引脚与早期应用相同：

| 用途 | 引脚 |
|---|---|
| SWCLK | PB13（板上连到 PA14） |
| SWDIO | PA9（封装内与 PA13 合封），同一个引脚双向，推挽 |
| 目标复位 | PC8 |
| 指示灯 | PC9（与 PA8 合封），低电平点亮 |
| 上电后保持低电平 | PA5、PB12（目标供电开关） |

板子沿用早期应用的三段式启动：地址 0 的 stage1 跳到 `0x20000` 的 stage3 DFU bootloader（USB `1209:3070`），应用区为 `0x2000`–`0x20000`，最后 256 字节是 seal。stage1 和 bootloader 的源码在 `bootloader-ch32v3/`（自带它运行时的 libxr 版本，`tools/build.sh v305` 一并编译出 `stage1.bin`、`bootloader.bin`）。空片或 bootloader 损坏时，用 WCH-Link 把 `stage1.bin` 写到 `0x08000000`、`bootloader.bin` 写到 `0x08020000`，应用再经 DFU 写入；平时不要动这两段。升级应用只走 DFU，不用 WCH-Link 写整片（会破坏 stage1 和 bootloader）：

```
python tools/dfu_update.py --bootloader 1209:3070 --run-app xrdap.elf.bin
```

`dfu_update.py` 先向正在运行的应用发 DFU DETACH（应用写 RAM 顶部 `0x20007FF0` 的信箱后复位），bootloader 停在 DFU 并写入镜像和 seal，最后发 RUN_APP（厂商请求 `0x5A`）启动应用：这个 bootloader 默认不自动启动应用，上电和复位后都停在 DFU，要再发一次 RUN_APP：`python tools/dfu_update.py --bootloader 1209:3070 --run-app`（不带镜像，只启动应用）。`--upload 文件` 在写入前把应用区里原来的镜像读出来保存。板上还是早期应用（`0d28:2041`）时，它不响应本工具的 DETACH 查找，可按住 IAP 键（PC7）上电让 bootloader 停在 DFU，或用 `dfu-util -d 0d28:2041,1209:3070 -a 0 -D xrdap.elf.bin` 再 `--run-app`。应用运行后 PA13、PA14 不再是调试口，WCH-Link 连不上芯片。

指示灯在 USB 已连上电脑时亮，没有连接时灭，每条 DAP 命令灭一下，DAP 通信时闪烁；单色灯显示不了状态灯规则里的颜色。芯片只有 32 KB RAM：DAP 包数为 2（`evt`、`wdap` 为 4），采样环形缓冲区 2 KB（厂商 INFO 报告）。应用与 `wdap` 一样跑在 216 MHz，**超出** CH32V305 标称的 144 MHz（早期应用为 144 MHz）；`boards/v305.cmake` 的 `XRDAP_APP_SYSCLK_HZ` 改为 `144000000` 即回到标称频率。SWD 时序在本板上按周期数扫描标定：请求 8 MHz 及以下时实际时钟在请求值的 -8 %/+8 % 以内，不加延时路径（约 17–19 MHz）只在请求不低于约 19 MHz 时使用。

## 2. 无线调试（wdap）

`wdap` 板上的 SX1281 射频模块可以把两块探针连成一套无线探针：主机端插在电脑上，远端接目标。电脑把主机端当普通 CMSIS-DAP 探针使用，主机端把 DAP 命令经无线发给远端，远端在自己的 SWD 口上执行后把应答发回。只有配对过的两块板之间走无线；没有配对的探针不用射频，无论角色设置如何都是有线探针，也不执行无线收到的任何命令。

两块板烧同一个固件，角色默认自动决定。配对过的探针没有自己的调试会话时作为远端监听无线；上位机发出 DAP_Connect 时，探针先 ping 已配对的另一块，有回应则这次会话走无线，没有回应则驱动自己的 SWD 口。所以只供电的那块是远端，被上位机打开的那块是主机端；远端不在时，主机端就是一块普通的有线探针。会话中途对端掉线时命令返回错误，不会改用本地的 SWD 口，下一次 DAP_Connect 时重新判断。DAP_Connect 之前发来的 SWJ_Clock、TransferConfigure、SWD_Configure 在会话转到无线时补发给远端。

同一时间只有一条传输拥有一块探针（[XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core) 的会话主人）：远端执行无线会话时，无线对端拥有它，这期间远端自己 USB 上的命令回“忙”（标准命令 `命令号 FF`，厂商命令 `命令号 05`），DAP_Info 照常回答；反之远端被本地 USB 占用时，无线来的命令也回“忙”。无线会话结束时，它开始的采样随之停止。

### 配对

两块板配对后才能一起用。在 10 s 内先后把两块板上的 KEY2 按住 3 s（或发厂商命令 `0x9F 0x20`），两块板用默认无线参数（2.44 GHz、同步字 `XRDP`）交换芯片 UID，由两个 UID 算出这一对的链路参数：频点（2403–2479 MHz）、CRC 种子和 GFSK 同步字。别的设备对频点和 CRC 都对不上，它们的包收不到。配对结果存在 flash 里，断电和固件升级后保留；同两块板重新配对得到相同的参数。这些参数只区分不同的对，不做认证。

### 设置

角色、调制方式和配对结果存在应用窗口末尾的设置区（见 Flash 布局），用厂商命令 `0x9F` 查看和修改（见 `app/radio_command.hpp`）。`0x9F 0x30` 把一条 `0x9F` 命令经无线交给对端执行，例如在主机端上同时改两块板的调制方式；没有配对时它和临时改角色的 `0x9F 0x10` 回状态 4（未配对）。

- 角色：自动（默认）、总是有线、总是主机端、总是远端。
- 调制方式（两块板必须相同）：
  - GFSK（默认）：2 Mb/s，每包最长 255 字节，一条 1024 字节的消息最多 5 包。
  - FLRC：1.3 Mb/s，每包最长 127 字节，一条消息分成 9 包，不纠错（编码率 1）。

### 链路

包格式和会话规则写在 `app/link_protocol.hpp`、`app/link_session.hpp`，`tests/` 按它们检查代码。主机端等不到应答时用同一序号重发，等待时间逐次加倍；远端对同一会话里重复的序号直接重发上一条应答，不再执行一遍命令。对端 1 s 没有消息后，下一条命令先 ping，ping 不通时立即返回错误，不必等完重发的 1 s。

每次走无线的会话有一个 16 位会话号，主机端每开一次会话换一个，ping 里带着它。会话期间主机端 1 s 没有命令就发一次心跳 ping；DAP_Disconnect 或 5 s 收不到电脑的命令时，主机端发会话号为 0 的 ping 结束会话。远端在收到会话号 0、收到新的会话号或 3 s 收不到主机端任何包时结束会话：清掉缓存的应答和拼了一半的命令，并停止这次会话启动的采样。会话之外收到的命令不执行。

SX1281 的连续接收在信道拥挤时可能卡死（数据手册勘误 16.1）；射频 100 ms 收不到任何包时重新进入接收，2 s 收不到时复位。收到的包按 GetPacketStatus 的错误字节判断 CRC（勘误 16.3）；收发结束后停在 FS 模式（SetAutoFs），缩短下一次收发的切换时间。FLRC 使用 32 位同步字 `XRDP`，配对后也不换：关掉同步字时接收端把噪声当成包，链路不通；勘误 16.4 对编码率 1 下应避开的同步字没有公开，编码率 3/4 下 `XRDP` 不在禁用范围内。

一条小命令的往返约 0.6 ms，其中两次空中传输约 0.3 ms。为缩短往返：24 字节以内的 SPI 传输直接读写寄存器，不走 DMA；连续接收时 DIO1 只接收包中断，拉高即有包，不再读中断状态；远端执行命令期间、主机端收到应答后，射频提前离开接收、准备发射；PA 爬坡时间 8 µs。

两块板相距约 1 m，主机端经无线调试一块 1 MiB flash 的 Cortex-M 目标，SWD 时钟设为 30 MHz（OpenOCD 为 10 MHz）时的典型结果：

| 项目 | GFSK | FLRC |
|---|---|---|
| 块读，每条命令 4 字节 | 3.1 KiB/s | 2.5 KiB/s |
| 块读，每条命令 1020 字节 | 87–110 KiB/s | 61–70 KiB/s |
| OpenOCD 烧写 1 MiB | 87–88 KiB/s | 63 KiB/s |
| probe-rs 下载 128 KiB（含擦除） | 6.9 s | |

擦除（约 6.8 s）和校验（OpenOCD 在目标上计算 CRC32，约 980 KiB/s）有线、无线相同；有线速度和各上位机的对比见第 5 节“上位机工具实测”。连续十余万条命令中 GFSK、FLRC 各约 1.5% 的命令重发过，没有一条最终失败；重发比例随周围 2.4 GHz 干扰变化。

### 看门狗与安全模式

应用开启独立看门狗，主循环约 8 s 没有运行即复位。连续三次看门狗复位后应用以安全模式启动：不读写设置区，只作为有线探针工作，DFU 照常可用；正常运行 30 s 或重新上电后恢复。

## 3. 变量读写与采样（XR 厂商扩展）

探针支持 XR 厂商扩展（命令 0x80 到 0x85）：一次读写多个地址、沿指针链解引用，以及在探针上按固定周期采样变量。协议与 [XRDAP-ESP](https://github.com/Jiu-xiao/XRDAP-ESP) 相同，`xrdap` 的 `read`、`write`、`watch` 直接可用。代码来自 [XRDAP-Core](https://github.com/Jiu-xiao/XRDAP-Core)（`xrdap-core` 子模块），协议见其 `docs/protocol.md` 第 3 节；采样任务和唤醒它的 TIM7 节拍在 `app/sampler_task.cpp`。CH32V307 只有 64 KB RAM，采样缓冲为 4096 字节（XRDAP-ESP 为 16384 字节），`xrdap info` 会报告。

无线调试时厂商命令和 DAP 命令一样转发给远端，采样在远端按自己的节拍进行，帧攒在远端的缓冲里，由主机端成批取回。

SWD 由 GPIO 软件产生：设定值不低于不加延时路径的速度（`wdap` 上约 14 MHz）时走这条路径，其余设定在带延时的档位里取周期最接近的一档，与设定值相差不超过半档，可能略快也可能略慢。档位按 CPU 周期标定（`app/app_main.cpp` 的 `kSwdTiming`）。SWD 设为 25 MHz、有线、每帧读 N 个 4 字节变量时，不跳拍、不丢帧的最短周期典型为：1 个变量 73 µs（13.7 kHz），8 个 181 µs（5.5 kHz），32 个 670 µs（1.5 kHz）；变量多时周期由 SWD 访问决定，每次访问约 11 µs。

## 4. 自己编译

需要 CMake 3.20 以上和 WCH RISC-V GCC 15.2 以上（`riscv32-wch-elf-gcc` 在 `PATH` 里，或用 `-DCOMPILER_PREFIX=<路径>/riscv32-wch-elf-` 指定）。bootloader 和应用是两个 CMake 工程，用 `-DXRDAP_BOARD=evt`、`wdap` 或 `v305` 选板子（`v305` 沿用 XRDAP-CH32V3 的 bootloader，没有 bootloader 工程）：

```
git clone --recursive https://github.com/Jiu-xiao/XRDAP-CH32V3.git
cd XRDAP-CH32V3
cmake -S bootloader -B build/bootloader-evt -DXRDAP_BOARD=evt
cmake --build build/bootloader-evt
cmake -S . -B build/app-evt -DXRDAP_BOARD=evt
cmake --build build/app-evt
```

产物为 `build/bootloader-evt/bootloader.elf.bin` 和 `build/app-evt/xrdap.elf.bin`。`tools/build.sh evt wdap` 一次编译两种板子的 bootloader 和应用，镜像收集在 `build/images/<板子>/`；`.github/workflows/build.yml` 在按摘要固定的 `xrimage/xrimage-ch32-riscv` 镜像（WCH GCC 15.2）里运行它并上传镜像。

链路协议、会话规则、配对身份、DAP 转发、设置日志和 Flash 布局是不依赖硬件的纯逻辑，在编译机上测试（CI 也运行）：

```
cmake -S tests -B build/tests
cmake --build build/tests
ctest --test-dir build/tests
```

固件版本写在 `CMakeLists.txt` 的 `project(... VERSION ...)`，编译时加上提交号，例如 `0.2.0+gc82e787`（有未提交改动时末尾加 `.dirty`），经 DAP_Info 报告给上位机。没有 `.git` 的源码副本用 `-DXRDAP_GIT_REV=<提交号>` 指定。

## 5. 参考

### 上位机工具实测

`wdap` 接一块 1 MiB flash 的 Cortex-M7 目标，SWD 设为 30 MHz（实际约 14 MHz）。无线时另一块 `wdap` 作主机端，GFSK。烧写含擦除 128 KiB 扇区；校验在目标上算 CRC。典型结果：

| 工具 | 方式 | 擦除+烧写 128 KiB | 校验 | 读 128 KiB flash |
|---|---|---|---|---|
| WCH OpenOCD 0.11（MounRiver Studio 自带） | 有线 | 1.97 s | 977 KiB/s | 871 KiB/s |
| | 无线 | 2.42 s | 854 KiB/s | 96 KiB/s |
| OpenOCD 0.12（openocd-esp32 v0.12.0-esp32-20260831） | 有线 | 1.96 s | 977 KiB/s | 992 KiB/s |
| | 无线 | 2.29 s | 853 KiB/s | 97 KiB/s |
| probe-rs 0.31.0（`download --verify`，`read`） | 有线 | 3.8 s（含校验） | | 0.35 s（含连接） |
| | 无线 | 8.3 s（含校验） | | 1.57 s（含连接） |

整片 1 MiB 有线烧写：OpenOCD 0.12 擦除 6.8 s、烧写 418 KiB/s；WCH OpenOCD 0.11 烧写 356–373 KiB/s，它每次只发一个包，等到应答再发下一个。写 RAM 时 OpenOCD 0.12 为 955 KiB/s，读为 985 KiB/s。烧写比写 RAM 慢，是因为目标边收数据边写 flash，忙时让探针等待。

### Flash 布局

| 地址 | 内容 |
|---|---|
| `0x00000000`–`0x0000FFFF` | bootloader（64 KB，运行在 96 MHz） |
| `0x00010000`–`0x0003F6FF` | 应用（`evt` 运行在 144 MHz，`wdap` 运行在 216 MHz） |
| `0x0003F700`–`0x0003FEFF` | 设置区：角色、调制方式、配对，固件升级时保留 |
| `0x0003FF00`–`0x0003FFFF` | seal：应用大小和 CRC32，由 bootloader 在 DFU 下载结束时写入 |

`v305` 的应用窗口为 `0x00002000`–`0x0001FFFF`（三段式启动的布局），最后 256 字节为 seal，前面 2 KB 为设置区位置，固件不写设置。

bootloader 的 DFU 下载只能写到设置区之前：超过 `0x2F700` 字节的镜像下载失败，设置区不受影响。两种 RAM 配置（FLASH 192K + RAM 128K、FLASH 256K + RAM 64K）都可用，固件按 64 KB RAM 链接；RAM 顶部 256 字节留给启动信箱和看门狗计数（`common/reserved_ram.h`）。

`wdap` 的应用用 12 MHz 晶振 18 倍频跑在 216 MHz，**超出** CH32V307 标称的 144 MHz，属于超规格运行。这样做是为了软件 SWD：速度与主频成正比，块读比 144 MHz 时快约 1.5 倍。时钟由板级文件选择（`boards/wdap.cmake` 的 `XRDAP_APP_SYSCLK_HZ`，实现在 `common/sysclk.c`），改为 `144000000` 即回到标称频率；bootloader 运行在 96 MHz。

### bootloader 何时停在 DFU 模式

- 上电时按住 bootloader 键；
- RAM 顶部的启动信箱（`0x2000FFF0`）里有进入 bootloader 的请求，软件复位后保留；
- 应用区没有 seal、seal 的 CRC 不符，或镜像不是按应用地址链接的（向量表第二个字不等于 `0x00010000`）。

### 板子文件

`boards/<名字>.cmake` 写晶振频率、应用主频、板上有哪些部件（`XRDAP_HAS_LED`、`XRDAP_HAS_RADIO`、`XRDAP_HAS_STATUS_LIGHTS`、`XRDAP_HAS_STATUS_LED`）、是否沿用别的工程的 bootloader（`XRDAP_EXTERNAL_BOOTLOADER`，此时还写应用窗口和启动信箱魔数）以及芯片内存和各缓冲区大小（`XRDAP_RAM_SIZE`、`XRDAP_RTOS_HEAP_SIZE` 等，默认值和含义见 `cmake/ch32v3.cmake`），`boards/<名字>.hpp` 写 bootloader 键、指示灯、状态灯、射频、目标引脚、SWD 时序和占用引脚前的初始化（`Board::Init()`）。增加一种板子只需要这两个文件。

### libxr 版本

`libxr` 子模块跟随 `feat/xrdap-r2-swd-loop` 分支，本项目用到的修正都在这个分支上：CH32 USBHS 在复位时把高速连接读成全速（Windows 判定配置描述符无效）；DFU 类自带一份 MS OS 2.0 描述符，与 DAP 组成复合设备时 Windows 无法启动；USBHS 在多包 IN 传输中把已发出的包再发一遍，超过 512 字节的 DAP 应答因此错位；DFU 轮询期限和 EP0 解除 STALL；检查已投递的 AP 写时读 RDBUFF，等写入完成再停时钟，目标忙时高速烧写不丢数据。

## 许可证

Apache-2.0。`platform/` 下 WCH 的启动文件、外设库和 FreeRTOS 保留各自的许可证。
