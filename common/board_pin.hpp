// SPDX-License-Identifier: Apache-2.0
#pragma once

// A board pin as a compile-time constant: GPIO port base address plus pin number.
// Board headers in boards/ describe their pins with it; drivers turn it into the
// WCH register block and bit mask.
//
// 编译期常量形式的板级引脚：GPIO 端口基址加引脚号。boards/ 下的板级头文件用它描述
// 引脚，驱动再换算成 WCH 寄存器块和位掩码。

#include <cstdint>

#include "ch32v30x.h"

namespace XrDap
{

struct BoardPin
{
  uintptr_t port;
  uint8_t pin;

  GPIO_TypeDef* Port() const { return reinterpret_cast<GPIO_TypeDef*>(port); }
  uint16_t Mask() const { return static_cast<uint16_t>(1U << pin); }
};

inline constexpr uintptr_t kPortA = GPIOA_BASE;
inline constexpr uintptr_t kPortB = GPIOB_BASE;
inline constexpr uintptr_t kPortC = GPIOC_BASE;
inline constexpr uintptr_t kPortD = GPIOD_BASE;

/// APB2 clock enable bit of a GPIO port / GPIO 端口的 APB2 时钟使能位.
inline uint32_t PortClock(uintptr_t port)
{
  switch (port)
  {
    case kPortA:
      return RCC_APB2Periph_GPIOA;
    case kPortB:
      return RCC_APB2Periph_GPIOB;
    case kPortC:
      return RCC_APB2Periph_GPIOC;
    default:
      return RCC_APB2Periph_GPIOD;
  }
}

}  // namespace XrDap
