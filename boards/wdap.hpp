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

/// LED2..LED9：8 颗 WS2812B-2020 串成一链，DIN 接 PB15（TIM1_CH3N；wdap.cmake 设
/// XRDAP_HAS_STATUS_LIGHTS）/ LED2..LED9: a chain of 8 WS2812B-2020, DIN on PB15
/// (TIM1_CH3N; wdap.cmake sets XRDAP_HAS_STATUS_LIGHTS).
inline constexpr uint8_t kStatusLightCount = 8;
inline constexpr BoardPin kStatusLights{kPortB, 15};

/// P2-3 TCK_O：SWD 时为 SWCLK / P2-3 TCK_O, SWCLK in SWD mode.
inline constexpr BoardPin kSwclk{kPortB, 3};
/// P2-4 TMS_O：SWD 时为 SWDIO / P2-4 TMS_O, SWDIO in SWD mode.
inline constexpr BoardPin kSwdio{kPortB, 5};
/// P2-5 RST_O / P2-5 RST_O.
inline constexpr BoardPin kNreset{kPortC, 1};

/// VG2392S240X0M2 (SX1281) on SPI1 / VG2392S240X0M2（SX1281）接 SPI1.
inline constexpr BoardPin kRadioSck{kPortA, 5};
inline constexpr BoardPin kRadioMiso{kPortA, 6};
inline constexpr BoardPin kRadioMosi{kPortA, 7};
inline constexpr BoardPin kRadioNss{kPortC, 4};
inline constexpr BoardPin kRadioNreset{kPortC, 5};
inline constexpr BoardPin kRadioBusy{kPortB, 0};
inline constexpr BoardPin kRadioDio1{kPortB, 1};
/// Module RF switch: TXEN high to transmit, RXEN high to receive / 模块射频开关.
inline constexpr BoardPin kRadioTxEn{kPortB, 11};
inline constexpr BoardPin kRadioRxEn{kPortB, 12};
}  // namespace XrDap::Board
