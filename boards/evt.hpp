// SPDX-License-Identifier: Apache-2.0
#pragma once

// CH32V307 evaluation board (CH32V307VCT6, 8 MHz crystal): the board of
// CH32V307_LibXR_Template. The key on PB3 and the LED on PB4 are on the board;
// the SWD pins are free header pins chosen for this firmware.
//
// CH32V307 评估板（CH32V307VCT6，8 MHz 晶振），即 CH32V307_LibXR_Template 用的板子。
// PB3 按键和 PB4 指示灯是板载的；SWD 引脚是本固件选用的空闲排针。

#include "board_pin.hpp"

namespace XrDap::Board
{
/// 板名，进入 DAP_Info / Board name reported in DAP_Info.
inline constexpr char kName[] = "CH32V307-EVT";
/// 产品名（DAP_Info 0x02）和 USB 产品字符串，按芯片 / Product name (DAP_Info 0x02) and
/// USB product string, by chip.
inline constexpr char kProductName[] = "XR-DAP-V307";
inline constexpr char kUsbProductName[] = "XR-DAP-V307 CMSIS-DAP";

/// 上电时按住进入 bootloader，低电平有效，内部上拉 / Held at power-up to stay in the
/// bootloader; active low with the internal pull-up.
inline constexpr BoardPin kBootKey{kPortB, 3};

/// bootloader 中闪烁的指示灯，高电平点亮（evt.cmake 设 XRDAP_HAS_LED）/ LED blinking in
/// the bootloader, lit when high (evt.cmake sets XRDAP_HAS_LED).
inline constexpr BoardPin kLed{kPortB, 4};

/// 目标 SWD / Target SWD.
inline constexpr BoardPin kSwclk{kPortB, 13};
inline constexpr BoardPin kSwdio{kPortB, 14};
/// 目标复位，低电平有效 / Target reset, active low.
inline constexpr BoardPin kNreset{kPortB, 12};

/// SWD 位翻转时序（LibXR SwdGpioTiming 的 b、a、c，周期数 x10）/ SWD bit-bang timing (b,
/// a, c of LibXR SwdGpioTiming, cycles x10).
inline constexpr uint32_t kSwdStepX10 = 63;
inline constexpr uint32_t kSwdInterceptX10 = 259;
inline constexpr uint32_t kSwdFastX10 = 154;

/// 应用占用目标引脚之前调用；本板无事可做 / Called before the application takes the
/// target pins; nothing to do on this board.
inline void Init() {}
}  // namespace XrDap::Board
