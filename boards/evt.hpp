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
}  // namespace XrDap::Board
