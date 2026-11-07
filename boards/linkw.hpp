// SPDX-License-Identifier: Apache-2.0
#pragma once

// WCH-LinkW-R0-1v1 (CH32V208GBU6 in QFN28, 32 MHz crystal, USB on PB6/PB7, the WCH-Link
// schematic, sheet WCH-LinkW-R0-1v1). The target header JP1 has SWCLK on PA5 and SWDIO on
// PA11, each through 100 Ω, and the same nets also reach the chip's own debug pins PA14
// and PA13; PA12 reaches SWDIO through 1 kΩ for single-wire debug. Board::Init() turns
// SWJ off, so that PA13/PA14 stay inputs while PA5/PA11 drive the target; a WCH-Link
// cannot reach the chip while the application runs (updates go through the DFU bootloader
// or the chip's ISP).
//
// WCH-LinkW-R0-1v1（QFN28 封装的 CH32V208GBU6，32 MHz 晶振，USB 在 PB6/PB7，WCH-Link
// 原理图的 WCH-LinkW-R0-1v1 页）。目标接口 JP1 的 SWCLK 接 PA5、SWDIO 接 PA11，各串
// 100 Ω，这两个网络也连着芯片自己的调试引脚 PA14、PA13；PA12 经 1 kΩ 也接 SWDIO，用于
// 单线调试。Board::Init() 关掉 SWJ，PA13/PA14 保持输入，由 PA5/PA11 驱动目标；应用运行
// 时 WCH-Link 连不上芯片（升级走 DFU bootloader 或芯片的 ISP）。

#include "board_pin.hpp"
#include "ch32_gpio.hpp"
#include "xrdap_chip.h"

namespace XrDap::Board
{
/// 板名，进入 DAP_Info / Board name reported in DAP_Info.
inline constexpr char kName[] = "WCH-LINKW";
/// 产品名（DAP_Info 0x02）和 USB 产品字符串，按芯片 / Product name (DAP_Info 0x02) and
/// USB product string, by chip.
inline constexpr char kProductName[] = "XR-DAP-V208";
inline constexpr char kUsbProductName[] = "XR-DAP-V208 CMSIS-DAP";
/// USB bcdDevice，每种板子一个（XRDAP-Core Family::kBcdCh32V208）/ USB bcdDevice, one
/// per board (XRDAP-Core Family::kBcdCh32V208): Windows caches descriptors by VID, PID
/// and bcdDevice.
inline constexpr uint16_t kUsbBcd = 0x0700;

/// ModeS 键（S2，按下接地，没有外部上拉）：上电时按住进入 bootloader（两个引导槽都有效时
/// 进旧的那个），运行中按住 3 s 开始配对 / ModeS key (S2, to GND when pressed, no
/// external pull-up): held at power-up to stay in the bootloader (the older one when both
/// bootloader slots are valid), held for 3 s while running to start pairing.
inline constexpr BoardPin kBootKey{kPortA, 4};

/// 蓝灯 D2（LED_CON），低电平点亮 / Blue LED D2 (LED_CON), lit when low.
inline constexpr BoardPin kLed{kPortC, 9};
inline constexpr bool kLedActiveHigh = false;

/// 绿灯 D3（LED_Mode），低电平点亮：会话走无线时亮，配对时闪 / Green LED D3 (LED_Mode),
/// lit when low: on while a session runs over the radio, blinking while pairing.
inline constexpr BoardPin kRadioLed{kPortC, 7};

/// 目标 SWD，同一个引脚双向，推挽 / Target SWD, one bidirectional pin, push-pull.
inline constexpr BoardPin kSwclk{kPortA, 5};
inline constexpr BoardPin kSwdio{kPortA, 11};
/// 目标复位，低电平有效 / Target reset, active low.
inline constexpr BoardPin kNreset{kPortC, 8};

/// 目标电源开关（CH217A，EN# 低电平打开）：JP1 的 3V3 和 5V / Target power switches
/// (CH217A, EN# on when low): the 3V3 and 5V of JP1.
inline constexpr BoardPin kPowerEnable3v3{kPortA, 0};
inline constexpr BoardPin kPowerEnable5v{kPortA, 1};

/// SWD 位翻转时序（LibXR SwdGpioTiming 的 b、a、c，周期数 x10）/ SWD bit-bang timing (b,
/// a, c of LibXR SwdGpioTiming, cycles x10).
inline constexpr uint32_t kSwdStepX10 = 63;
inline constexpr uint32_t kSwdInterceptX10 = 200;
inline constexpr uint32_t kSwdFastX10 = 144;

/// @brief 应用占用目标引脚之前调用 / Called before the application takes the target
///        pins.
/// SWJ off, so that PA13/PA14 (on the SWD nets) stay inputs (RM AFIO_PCFR1 SW_CFG =
/// 100b), and both target power switches on (low).
inline void Init()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
  static LibXR::CH32GPIO power_5v(kPowerEnable5v.Port(), kPowerEnable5v.Mask());
  static LibXR::CH32GPIO power_3v3(kPowerEnable3v3.Port(), kPowerEnable3v3.Mask());
  power_5v.Write(false);
  power_3v3.Write(false);
}
}  // namespace XrDap::Board
