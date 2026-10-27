// SPDX-License-Identifier: Apache-2.0
#pragma once

// The register-level GPIO for the SWD bit-bang loop and the radio pins: XRDAP-Core's
// F1Gpio (the CH32V3 and the STM32F1 share the register layout) plus the port clock this
// firmware enables on construction.
//
// SWD 位带循环和射频引脚用的寄存器级 GPIO：XRDAP-Core 的 F1Gpio（CH32V3 与 STM32F1 的
// 寄存器布局相同），加上本固件在构造时打开的端口时钟。

#include <cstdint>

#include "board_pin.hpp"
#include "xrdap/swd/f1_gpio.hpp"

namespace XrDap
{

template <uintptr_t PORT, uint8_t PIN>
class FastGpio : public F1Gpio<PORT, PIN>
{
 public:
  FastGpio() { RCC_APB2PeriphClockCmd(PortClock(PORT), ENABLE); }
};

/// The FastGpio for a board pin constant / 板级引脚常量对应的 FastGpio.
template <const BoardPin& P>
using FastGpioFor = FastGpio<P.port, P.pin>;

}  // namespace XrDap
