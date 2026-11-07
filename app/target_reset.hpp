// SPDX-License-Identifier: Apache-2.0
#pragma once

// 目标复位脚。复位线低电平有效，探针启动时不能把它拉低。引脚一配成推挽输出就输出
// 输出寄存器里的电平，而这个寄存器复位后是 0，所以要先把它置高，再配成输出。
//
// The target reset pin. The line is active low, and starting the probe must not pull it
// low. A pin drives the level of its output register as soon as it becomes a push-pull
// output, and that register resets to 0, so it is set high first.

#include "board_pin.hpp"
#include "ch32_gpio.hpp"

namespace XrDap
{

/// @brief 把引脚配成空闲为高的推挽输出 / Make the pin a push-pull output that idles high.
inline LibXR::CH32GPIO TargetResetOutput(BoardPin pin)
{
  // 写输出寄存器要先开端口时钟 / Writing the output register needs the port clock.
  RCC_APB2PeriphClockCmd(PortClock(pin.port), ENABLE);
  GPIO_SetBits(pin.Port(), pin.Mask());
  return LibXR::CH32GPIO(pin.Port(), pin.Mask());
}

}  // namespace XrDap
