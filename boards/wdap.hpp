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
/// 产品名（DAP_Info 0x02）和 USB 产品字符串，按芯片 / Product name (DAP_Info 0x02) and
/// USB product string, by chip.
inline constexpr char kProductName[] = "XR-DAP-V307";
inline constexpr char kUsbProductName[] = "XR-DAP-V307 CMSIS-DAP";

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

/// SWD 位翻转时序（LibXR SwdGpioTiming 的 b、a、c，周期数 x10）：由 216 MHz 下测得的半
/// 周期（约 60 ns + 每圈 14.6 ns，无延时路径约 14 MHz）换算；在 STM32H723 目标上验证：
/// 无延时路径 13.45 MHz、N=1 为 7.05 MHz，各 300 轮 16 KiB 无错 / SWD bit-bang timing
/// (b, a, c of LibXR SwdGpioTiming, cycles x10), from the half period measured at
/// 216 MHz (about 60 ns + 14.6 ns per loop, undelayed path about 14 MHz); checked on an
/// STM32H723 target: undelayed 13.45 MHz, N=1 7.05 MHz, 300 rounds of 16 KiB each without
/// an error.
inline constexpr uint32_t kSwdStepX10 = 63;
inline constexpr uint32_t kSwdInterceptX10 = 259;
inline constexpr uint32_t kSwdFastX10 = 154;

/// 应用占用目标引脚之前调用；本板无事可做 / Called before the application takes the
/// target pins; nothing to do on this board.
inline void Init() {}
}  // namespace XrDap::Board
