// SPDX-License-Identifier: Apache-2.0
#include "ws2812.hpp"

#include XRDAP_BOARD_HEADER
#include "ch32v30x.h"

namespace XrDap
{
namespace
{
constexpr uint32_t kBitNs = 1280;  // T0H+T0L = T1H+T1L >= 1.25 us
constexpr uint32_t kT0hNs = 300;   // 220..380 ns; T0L 980 ns <= 1 us
constexpr uint32_t kT1hNs = 650;   // 580 ns..1 us; T1L 630 ns >= 580 ns

// One compare value per bit, then a zero that holds the line low after the frame.
uint16_t dma_buffer[24U * Ws2812::kMaxLeds + 1U];
uint16_t t0h_ticks = 0;
uint16_t t1h_ticks = 0;

// TIM1 is on APB2, which runs at the core clock (system_ch32v30x.c, PPRE2 = DIV1).
uint16_t Ticks(uint32_t ns)
{
  return static_cast<uint16_t>(((SystemCoreClock / 1000U) * ns + 500000U) / 1000000U);
}

uint16_t* PutByte(uint16_t* out, uint8_t value)
{
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
  {
    *out++ = (value & mask) != 0U ? t1h_ticks : t0h_ticks;
  }
  return out;
}
}  // namespace

void Ws2812::Init()
{
  t0h_ticks = Ticks(kT0hNs);
  t1h_ticks = Ticks(kT1hNs);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | PortClock(Board::kStatusLights.port),
                         ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // PWM mode 1 with preload on channel 3: the output is high while CNT < CH3CVR, and a
  // value written by DMA takes effect at the next update event. Only CH3N is enabled
  // (CC3E = 0, CC3NE = 1), so it carries OC3REF unchanged (RM figure 14-3); MOE turns
  // the outputs on.
  TIM1->CTLR1 = 0U;
  TIM1->PSC = 0U;
  TIM1->ATRLR = static_cast<uint16_t>(Ticks(kBitNs) - 1U);
  TIM1->CH3CVR = 0U;
  TIM1->CHCTLR2 = (TIM1->CHCTLR2 & 0xFF00U) | TIM_OC3M_2 | TIM_OC3M_1 | TIM_OC3PE;
  TIM1->CCER = TIM_CC3NE;
  TIM1->BDTR = TIM_MOE;
  TIM1->SWEVGR = TIM_UG;
  TIM1->DMAINTENR = TIM_UDE;
  TIM1->CTLR1 = TIM_ARPE | TIM_CEN;

  DMA1_Channel5->CFGR = 0U;
  DMA1_Channel5->PADDR = reinterpret_cast<uint32_t>(&TIM1->CH3CVR);
  DMA1_Channel5->MADDR = reinterpret_cast<uint32_t>(dma_buffer);

  GPIO_InitTypeDef io = {};
  io.GPIO_Pin = Board::kStatusLights.Mask();
  io.GPIO_Speed = GPIO_Speed_50MHz;
  io.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(Board::kStatusLights.Port(), &io);
}

bool Ws2812::Busy()
{
  return (DMA1_Channel5->CFGR & DMA_CFGR1_EN) != 0U && DMA1_Channel5->CNTR != 0U;
}

bool Ws2812::Show(const Rgb* leds, uint8_t count)
{
  if (Busy())
  {
    return false;
  }
  if (count > kMaxLeds)
  {
    count = kMaxLeds;
  }
  uint16_t* out = dma_buffer;
  for (uint8_t i = 0; i < count; ++i)
  {
    out = PutByte(out, leds[i].g);
    out = PutByte(out, leds[i].r);
    out = PutByte(out, leds[i].b);
  }
  *out++ = 0U;

  // A request left pending while the channel was off is served at once; the value then
  // waits in the preload register for the next update event, so no bit is cut short.
  DMA1_Channel5->CFGR = 0U;
  DMA1->INTFCR = DMA_CGIF5;
  DMA1_Channel5->CNTR = static_cast<uint32_t>(out - dma_buffer);
  DMA1_Channel5->CFGR = DMA_DIR_PeripheralDST | DMA_MemoryInc_Enable |
                        DMA_PeripheralDataSize_HalfWord | DMA_MemoryDataSize_HalfWord |
                        DMA_Priority_Medium | DMA_CFGR1_EN;
  return true;
}
}  // namespace XrDap
