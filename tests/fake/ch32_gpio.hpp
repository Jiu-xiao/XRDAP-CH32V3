// SPDX-License-Identifier: Apache-2.0
#pragma once

// 主机测试里代替 LibXR 的 CH32GPIO，只有默认的推挽输出。和真的驱动一样，构造时开端口
// 时钟，把引脚配成输出，不碰输出寄存器；这里记下引脚从那一刻起输出的电平。
//
// Stands in for LibXR's CH32GPIO in the host tests, as a push-pull output only. Like the
// real driver its constructor turns the port clock on and makes the pin an output,
// without touching the output register; here it records the level the pin drives from
// that moment.

#include <cstdint>

#include "ch32v30x.h"

namespace LibXR
{

class CH32GPIO final
{
 public:
  CH32GPIO(GPIO_TypeDef* port, uint16_t pin)
  {
    RCC_APB2PeriphClockCmd(Fake::PortClock(port), ENABLE);
    high_as_output_ = (Fake::output[port] & pin) != 0U;
  }

  CH32GPIO(const CH32GPIO&) = delete;
  CH32GPIO& operator=(const CH32GPIO&) = delete;

  bool HighAsOutput() const { return high_as_output_; }

 private:
  bool high_as_output_;
};

}  // namespace LibXR
