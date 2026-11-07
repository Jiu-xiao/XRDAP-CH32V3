// SPDX-License-Identifier: Apache-2.0
#pragma once

// 主机测试里代替 WCH 的 ch32v30x.h，只有被测代码用到的部分。端口时钟和输出寄存器
// 记录在 Fake 里，规则和芯片相同：时钟关着时写不进输出寄存器，输出寄存器复位后是 0。
//
// Stands in for WCH's ch32v30x.h in the host tests, with only what the tested code uses.
// Port clocks and output registers are recorded in Fake under the chip's rules: the
// output register takes no write while the port clock is off, and it resets to 0.

#include <cstdint>
#include <map>

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;

struct GPIO_TypeDef;

#define GPIOA_BASE 0x40010800U
#define GPIOB_BASE 0x40010C00U
#define GPIOC_BASE 0x40011000U
#define GPIOD_BASE 0x40011400U

#define RCC_APB2Periph_GPIOA 0x04U
#define RCC_APB2Periph_GPIOB 0x08U
#define RCC_APB2Periph_GPIOC 0x10U
#define RCC_APB2Periph_GPIOD 0x20U

namespace Fake
{
inline uint32_t apb2_clocks = 0U;
inline std::map<const GPIO_TypeDef*, uint16_t> output;

inline uint32_t PortClock(const GPIO_TypeDef* port)
{
  const auto address = reinterpret_cast<uintptr_t>(port);
  return RCC_APB2Periph_GPIOA << ((address - GPIOA_BASE) / 0x400U);
}

inline void Reset()
{
  apb2_clocks = 0U;
  output.clear();
}
}  // namespace Fake

inline void RCC_APB2PeriphClockCmd(uint32_t periph, FunctionalState state)
{
  if (state == ENABLE)
  {
    Fake::apb2_clocks |= periph;
  }
}

inline void GPIO_SetBits(GPIO_TypeDef* port, uint16_t pin)
{
  if ((Fake::apb2_clocks & Fake::PortClock(port)) != 0U)
  {
    Fake::output[port] |= pin;
  }
}
