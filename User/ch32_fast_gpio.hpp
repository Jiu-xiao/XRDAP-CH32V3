#pragma once

#include <cstdint>

#include "ch32v30x.h"
#include "libxr_def.hpp"

namespace LibXR
{

namespace CH32FastGPIO
{
static constexpr uint32_t CFG_INPUT_FLOATING = 0x4u;
static constexpr uint32_t CFG_INPUT_PULL = 0x8u;
static constexpr uint32_t CFG_OUTPUT_PP_50M = 0x3u;
static constexpr uint32_t CFG_OUTPUT_OD_50M = 0x7u;

inline void EnablePortClock(GPIO_TypeDef* port)
{
  if (port == GPIOA)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  }
  else if (port == GPIOB)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  }
}

inline void ApplyInputPull(GPIO_TypeDef* port, uint16_t pin_mask, bool pull_up)
{
  if (pull_up)
  {
    port->BSHR = pin_mask;
  }
  else
  {
    port->BCR = pin_mask;
  }
}

}  // namespace CH32FastGPIO

class CH32GPIOSWC
{
 public:
  enum class Direction : uint8_t { INPUT, OUTPUT_PUSH_PULL, OUTPUT_OPEN_DRAIN };
  enum class Pull : uint8_t { NONE, UP, DOWN };
  struct Configuration { Direction direction; Pull pull; };

  CH32GPIOSWC() { CH32FastGPIO::EnablePortClock(GPIOB); }

  ErrorCode SetConfig(Configuration config)
  {
    const uint32_t base = GPIOB->CFGHR & ~CFG_MASK;
    if (config.direction == Direction::OUTPUT_PUSH_PULL)
    {
      GPIOB->CFGHR = CFG_OUT_PP | base;
      return ErrorCode::OK;
    }
    if (config.direction == Direction::INPUT)
    {
      if (config.pull == Pull::NONE)
      {
        GPIOB->CFGHR = CFG_IN_FLOAT | base;
      }
      else
      {
        GPIOB->CFGHR = CFG_IN_PULL | base;
        CH32FastGPIO::ApplyInputPull(GPIOB, GPIO_Pin_13, config.pull == Pull::UP);
      }
      return ErrorCode::OK;
    }
    return ErrorCode::ARG_ERR;
  }

  inline void Write(bool value)
  {
    if (value)
    {
      GPIOB->BSHR = GPIO_Pin_13;
    }
    else
    {
      GPIOB->BCR = GPIO_Pin_13;
    }
  }

 private:
  static constexpr uint32_t CFG_SHIFT = 20u;
  static constexpr uint32_t CFG_MASK = static_cast<uint32_t>(0xFu << CFG_SHIFT);
  static constexpr uint32_t CFG_OUT_PP = static_cast<uint32_t>(CH32FastGPIO::CFG_OUTPUT_PP_50M << CFG_SHIFT);
  static constexpr uint32_t CFG_IN_FLOAT = static_cast<uint32_t>(CH32FastGPIO::CFG_INPUT_FLOATING << CFG_SHIFT);
  static constexpr uint32_t CFG_IN_PULL = static_cast<uint32_t>(CH32FastGPIO::CFG_INPUT_PULL << CFG_SHIFT);
};

class CH32GPIOSWD
{
 public:
  enum class Direction : uint8_t { INPUT, OUTPUT_PUSH_PULL, OUTPUT_OPEN_DRAIN };
  enum class Pull : uint8_t { NONE, UP, DOWN };
  struct Configuration { Direction direction; Pull pull; };

  CH32GPIOSWD() { CH32FastGPIO::EnablePortClock(GPIOA); }

  ErrorCode SetConfig(Configuration config)
  {
    const uint32_t base = GPIOA->CFGHR & ~CFG_MASK;
    if (config.direction == Direction::OUTPUT_PUSH_PULL)
    {
      GPIOA->CFGHR = CFG_OUT_PP | base;
      return ErrorCode::OK;
    }
    if (config.direction == Direction::OUTPUT_OPEN_DRAIN)
    {
      GPIOA->CFGHR = CFG_OUT_OD | base;
      return ErrorCode::OK;
    }
    if (config.direction == Direction::INPUT)
    {
      if (config.pull == Pull::NONE)
      {
        GPIOA->CFGHR = CFG_IN_FLOAT | base;
      }
      else
      {
        GPIOA->CFGHR = CFG_IN_PULL | base;
        CH32FastGPIO::ApplyInputPull(GPIOA, GPIO_Pin_9, config.pull == Pull::UP);
      }
      return ErrorCode::OK;
    }
    return ErrorCode::ARG_ERR;
  }

  inline void Write(bool value)
  {
    if (value)
    {
      GPIOA->BSHR = GPIO_Pin_9;
    }
    else
    {
      GPIOA->BCR = GPIO_Pin_9;
    }
  }
  inline bool Read() const { return (GPIOA->INDR & GPIO_Pin_9) != 0u; }

 private:
  static constexpr uint32_t CFG_SHIFT = 4u;
  static constexpr uint32_t CFG_MASK = static_cast<uint32_t>(0xFu << CFG_SHIFT);
  static constexpr uint32_t CFG_OUT_PP = static_cast<uint32_t>(CH32FastGPIO::CFG_OUTPUT_PP_50M << CFG_SHIFT);
  static constexpr uint32_t CFG_OUT_OD = static_cast<uint32_t>(CH32FastGPIO::CFG_OUTPUT_OD_50M << CFG_SHIFT);
  static constexpr uint32_t CFG_IN_FLOAT = static_cast<uint32_t>(CH32FastGPIO::CFG_INPUT_FLOATING << CFG_SHIFT);
  static constexpr uint32_t CFG_IN_PULL = static_cast<uint32_t>(CH32FastGPIO::CFG_INPUT_PULL << CFG_SHIFT);
};

}  // namespace LibXR
