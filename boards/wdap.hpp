// SPDX-License-Identifier: Apache-2.0
#pragma once

// "高速无线DAP调试器" board (Schematic1, 2023-06-21): CH32V307RCT6, 12 MHz crystal,
// USBHS on PB6/PB7, target header P2 with 100 Ω series resistors, VG2392S240X0M2
// (SX1281) radio on SPI1. KEY1 shares its switch with BOOT0 (factory ISP), so the
// bootloader key is KEY2. The only LED besides the WS2812 chain is the power LED.
//
// “高速无线DAP调试器”板（Schematic1，2023-06-21）：CH32V307RCT6，12 MHz 晶振，USBHS 在
// PB6/PB7，目标接口 P2 串 100 Ω，SPI1 上接 VG2392S240X0M2（SX1281）射频模块。KEY1 与
// BOOT0（出厂 ISP）共用开关，所以 bootloader 键用 KEY2。除 WS2812 灯链外只有电源灯。

#include "board_pin.hpp"

namespace XrDap::Board
{
inline constexpr char kName[] = "XR-DAP-WDAP";

/// KEY2（SW2 接地），低电平有效，内部上拉 / KEY2 (SW2 to GND), active low, internal
/// pull-up.
inline constexpr BoardPin kBootKey{kPortC, 10};

inline constexpr bool kHasLed = false;
inline constexpr BoardPin kLed{kPortC, 0};

/// P2-3 TCK_O：SWD 时为 SWCLK / P2-3 TCK_O, SWCLK in SWD mode.
inline constexpr BoardPin kSwclk{kPortB, 3};
/// P2-4 TMS_O：SWD 时为 SWDIO / P2-4 TMS_O, SWDIO in SWD mode.
inline constexpr BoardPin kSwdio{kPortB, 5};
/// P2-5 RST_O / P2-5 RST_O.
inline constexpr BoardPin kNreset{kPortC, 1};
/// P2-6 TDI_O、P2-11 TDO_O / P2-6 TDI_O, P2-11 TDO_O.
inline constexpr BoardPin kTdi{kPortC, 2};
inline constexpr BoardPin kTdo{kPortC, 3};
}  // namespace XrDap::Board
