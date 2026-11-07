// SPDX-License-Identifier: Apache-2.0
#include "sampler_tick.hpp"

#include "ch32v30x.h"

namespace XrDap
{

namespace
{

constexpr uint32_t kMaxCounterUs = 65535U;

// TIM7 的时钟是 PCLK1，APB1 分频时翻倍（RM 时钟树）。HCLK 等于内核时钟，APB1 不分频
// 或二分频（common/sysclk.c），所以它等于内核时钟。 / TIM7 runs from PCLK1, doubled when
// APB1 is divided (RM, clock tree). HCLK is the core clock and APB1 is undivided or
// divided by 2 (common/sysclk.c), so TIM7 runs at the core clock.
uint32_t Tim7ClockHz() { return SystemCoreClock; }

}  // namespace

SamplerTick* SamplerTick::instance = nullptr;

SamplerTick::SamplerTick(DapVendor& vendor) : vendor_(vendor)
{
  instance = this;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  TIM_Cmd(TIM7, DISABLE);
  TIM7->CTLR1 = TIM_URS;  // 只有溢出产生中断，UG 不产生 / only overflows interrupt
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
  NVIC_EnableIRQ(TIM7_IRQn);
}

void SamplerTick::Start(uint32_t period_us)
{
  if (period_us == 0U)
  {
    period_us = 1U;
  }
  TIM_Cmd(TIM7, DISABLE);
  repeat_ = (period_us + kMaxCounterUs - 1U) / kMaxCounterUs;
  count_ = 0U;
  TIM7->PSC = static_cast<uint16_t>(Tim7ClockHz() / 1000000U - 1U);
  TIM7->ATRLR = static_cast<uint16_t>(period_us / repeat_ - 1U);
  TIM7->SWEVGR = TIM_PSCReloadMode_Immediate;  // UG 装入 PSC 和 ATRLR / UG loads them
  TIM7->CNT = 0U;
  TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
  TIM_Cmd(TIM7, ENABLE);
}

void SamplerTick::Stop() { TIM_Cmd(TIM7, DISABLE); }

void SamplerTick::OnInterrupt()
{
  count_ = count_ + 1U;
  if (count_ < repeat_)
  {
    return;
  }
  count_ = 0U;
  vendor_.OnTickIsr();
}

}  // namespace XrDap

extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void TIM7_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM7, TIM_IT_Update) == RESET)
  {
    return;
  }
  TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
  if (XrDap::SamplerTick::instance != nullptr)
  {
    XrDap::SamplerTick::instance->OnInterrupt();
  }
}
