// SPDX-License-Identifier: Apache-2.0
#pragma once

// CH32V305 board (CH32V305FBP6 in QFN28, 12 MHz crystal, the circuit of the WCH-LinkE):
// the board of the earlier application of this repository (tag legacy-v305-app). Pins as
// in that application (User/app_main.cpp, User/ch32_fast_gpio.hpp) and its bootloader
// (bootloader-ch32v3/bootloader_main.cpp); package facts from CH32V307DS0.
//
// In the QFN28 package PA9 is bonded to PA13, PA1 to PA5 and PC9 to PA8 (CH32V307DS0
// p.34 note 6, p.35 notes 15/16), and on the board PB13 is wired to PA14. The target SWD
// lines therefore reach the chip's own debug pins PA13/PA14: Board::Init() turns SWJ off
// so that PA13/PA14 stay inputs while PA9/PB13 drive the target, and a WCH-Link cannot
// reach the chip while the application runs (updates go through the DFU bootloader).
//
// CH32V305 板（QFN28 封装的 CH32V305FBP6，12 MHz 晶振，电路同 WCH-LinkE），即
// 本仓库早期应用（标签 legacy-v305-app）的板子。引脚同该应用（User/app_main.cpp、
// User/ch32_fast_gpio.hpp）和它的 bootloader（bootloader-ch32v3/bootloader_main.cpp）。
// QFN28 封装内 PA9 与 PA13、PA1 与 PA5、PC9 与 PA8 合封，板上 PB13 连到 PA14，所以目标
// SWD 线也连着芯片自己的调试引脚 PA13/PA14：Board::Init() 关掉 SWJ，PA13/PA14 保持输入，
// 由 PA9/PB13 驱动目标；应用运行时 WCH-Link 连不上芯片（升级走 DFU bootloader）。

#include "board_pin.hpp"
#include "ch32v30x.h"

namespace XrDap::Board
{
/// 板名，进入 DAP_Info / Board name reported in DAP_Info.
inline constexpr char kName[] = "CH32V305-LINKE";
/// 产品名（DAP_Info 0x02）和 USB 产品字符串，按芯片 / Product name (DAP_Info 0x02) and
/// USB product string, by chip.
inline constexpr char kProductName[] = "XR-DAP-V305";
inline constexpr char kUsbProductName[] = "XR-DAP-V305 CMSIS-DAP";
/// USB bcdDevice，每种板子一个（XRDAP-Core Family::kBcdCh32V305）/ USB bcdDevice, one
/// per board (XRDAP-Core Family::kBcdCh32V305): Windows caches descriptors by VID, PID
/// and bcdDevice.
inline constexpr uint16_t kUsbBcd = 0x0600;

/// IAP 键：上电时按住，bootloader-ch32v3 停在 DFU；低电平有效 / IAP key: held
/// at power-up, bootloader-ch32v3 stays in DFU; active low.
inline constexpr BoardPin kBootKey{kPortC, 7};

/// 指示灯，低电平点亮（与 PA8 合封；bootloader 用它闪烁）/ LED, lit when low (bonded
/// to PA8; the bootloader blinks it).
inline constexpr BoardPin kLed{kPortC, 9};
inline constexpr bool kLedActiveHigh = false;

/// 目标 SWD：SWCLK = PB13（板上连 PA14），SWDIO = PA9（与 PA13 合封），同一个引脚双向，
/// 推挽 / Target SWD: SWCLK = PB13 (wired to PA14), SWDIO = PA9 (bonded to PA13), one
/// bidirectional pin, push-pull.
inline constexpr BoardPin kSwclk{kPortB, 13};
inline constexpr BoardPin kSwdio{kPortA, 9};
/// 目标复位，低电平有效 / Target reset, active low.
inline constexpr BoardPin kNreset{kPortC, 8};

/// 上电后保持低电平的两个控制脚（早期应用和原 WCH-LinkE 电路：SY6288D 目标
/// 供电开关，低电平打开）/ Two control pins held low after start-up (the earlier
/// application and the WCH-LinkE circuit: SY6288D target power switches, on when low).
inline constexpr BoardPin kPowerEnable3v3{kPortA, 5};
inline constexpr BoardPin kPowerEnable5v{kPortB, 12};

/// SWD 位翻转时序（LibXR SwdGpioTiming 的 b、a、c，周期数 x10），在本板
/// 216 MHz 下用 SysTick 周期计数扫描测得（F302CB 目标）：每个延时次数 N 每周期
/// 6.0 个周期（N >= 16 精确），N = 1..32 拟合 a = 25..27、b = 6.3..6.4，即 wdap 的
/// {63, 259}；不加延时路径每周期 11.45（写）、12.74（读）、14.13（SWJ 序列）个周期，
/// c 取最快的写路径，不加延时路径只在不比请求快时才用 / SWD bit-bang timing (b, a, c of
/// LibXR SwdGpioTiming, cycles x10), measured on this board at 216 MHz with a SysTick
/// cycle sweep (F302CB target): 6.0 cycles per SWCLK for each delay step N
/// (exact for N >= 16), fit over N = 1..32 a = 25..27, b = 6.3..6.4, i.e. wdap's {63,
/// 259}; the undelayed path costs 11.45 (write), 12.74 (read) and 14.13 (SWJ sequence)
/// cycles per SWCLK, and c is the fastest, the write path, so the undelayed path is used
/// only where it is not faster than requested.
inline constexpr uint32_t kSwdStepX10 = 63;
inline constexpr uint32_t kSwdInterceptX10 = 259;
inline constexpr uint32_t kSwdFastX10 = 114;

/// @brief 应用占用目标引脚之前调用 / Called before the application takes the target
///        pins.
/// SWJ off, so that PA13/PA14 (bonded or wired to the SWD pins) stay inputs (RM
/// AFIO_PCFR1 SWJ_CFG = 100b), and the two control pins low, in the order of
/// the earlier application.
inline void Init()
{
  RCC_APB2PeriphClockCmd(
      RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
  GPIO_InitTypeDef init{};
  init.GPIO_Mode = GPIO_Mode_Out_PP;
  init.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_ResetBits(kPowerEnable5v.Port(), kPowerEnable5v.Mask());
  init.GPIO_Pin = kPowerEnable5v.Mask();
  GPIO_Init(kPowerEnable5v.Port(), &init);
  GPIO_ResetBits(kPowerEnable3v3.Port(), kPowerEnable3v3.Mask());
  init.GPIO_Pin = kPowerEnable3v3.Mask();
  GPIO_Init(kPowerEnable3v3.Port(), &init);
}
}  // namespace XrDap::Board
