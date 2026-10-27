// SPDX-License-Identifier: Apache-2.0
#pragma once

// Register-level GPIO for the SWD bit-bang loop. The pin is a template argument, so
// every access compiles to a single store or load on the port registers; this is what
// lets LibXR::Debug::SwdGeneralGPIO reach tens of MHz on the CH32V307. It provides
// the GpioType interface SwdGeneralGPIO expects (Direction, Pull, SetConfig, Write,
// Read).
//
// SWD 位带循环用的寄存器级 GPIO。引脚是模板参数，每次访问编译成对端口寄存器的一次读写，
// LibXR::Debug::SwdGeneralGPIO 因此能在 CH32V307 上跑到几十 MHz。提供 SwdGeneralGPIO
// 需要的 GpioType 接口（Direction、Pull、SetConfig、Write、Read）。

#include <cstdint>

#include "board_pin.hpp"
#include "libxr_def.hpp"

namespace XrDap
{

template <uintptr_t PORT, uint8_t PIN>
class FastGpio
{
  static_assert(PIN < 16, "CH32 GPIO ports have 16 pins");

 public:
  enum class Direction : uint8_t
  {
    INPUT,
    OUTPUT_PUSH_PULL,
    OUTPUT_OPEN_DRAIN
  };
  enum class Pull : uint8_t
  {
    NONE,
    UP,
    DOWN
  };
  struct Configuration
  {
    Direction direction;
    Pull pull;
  };

  FastGpio() { RCC_APB2PeriphClockCmd(PortClock(PORT), ENABLE); }

  LibXR::ErrorCode SetConfig(Configuration config)
  {
    uint32_t mode = kInputFloating;
    switch (config.direction)
    {
      case Direction::OUTPUT_PUSH_PULL:
        mode = kOutputPushPull50MHz;
        break;
      case Direction::OUTPUT_OPEN_DRAIN:
        mode = kOutputOpenDrain50MHz;
        break;
      case Direction::INPUT:
        if (config.pull != Pull::NONE)
        {
          mode = kInputPull;
          // In pull mode the output latch selects up (1) or down (0).
          Write(config.pull == Pull::UP);
        }
        break;
    }
    volatile uint32_t& cfg = (PIN < 8) ? Regs()->CFGLR : Regs()->CFGHR;
    cfg = (cfg & ~(0xFU << kShift)) | (mode << kShift);
    return LibXR::ErrorCode::OK;
  }

  inline void Write(bool high)
  {
    if (high)
    {
      Regs()->BSHR = kMask;
    }
    else
    {
      Regs()->BCR = kMask;
    }
  }

  inline bool Read() const { return (Regs()->INDR & kMask) != 0U; }

 private:
  static constexpr uint32_t kInputFloating = 0x4U;
  static constexpr uint32_t kInputPull = 0x8U;
  static constexpr uint32_t kOutputPushPull50MHz = 0x3U;
  static constexpr uint32_t kOutputOpenDrain50MHz = 0x7U;
  static constexpr uint32_t kShift = (PIN % 8U) * 4U;
  static constexpr uint32_t kMask = 1U << PIN;

  static GPIO_TypeDef* Regs() { return reinterpret_cast<GPIO_TypeDef*>(PORT); }
};

/// The FastGpio for a board pin constant / 板级引脚常量对应的 FastGpio.
template <const BoardPin& P>
using FastGpioFor = FastGpio<P.port, P.pin>;

}  // namespace XrDap
