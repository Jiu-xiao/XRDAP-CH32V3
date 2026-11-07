// SPDX-License-Identifier: Apache-2.0
#include "sampler_tick.hpp"

#include "xrdap_chip.h"

namespace XrDap
{

namespace
{

constexpr uint32_t kMaxCounterUs = 65535U;

// 节拍定时器在 APB1 上，时钟是 PCLK1，APB1 分频时翻倍（RM 时钟树）。HCLK 等于内核时钟，
// APB1 不分频或二分频（common/sysclk.c），所以它等于内核时钟。 / The tick timer is on
// APB1 and runs from PCLK1, doubled when APB1 is divided (RM, clock tree). HCLK is the
// core clock and APB1 is undivided or divided by 2 (common/sysclk.c), so the timer runs
// at the core clock.
uint32_t TickClockHz() { return SystemCoreClock; }

}  // namespace

SamplerTick* SamplerTick::instance = nullptr;

SamplerTick::SamplerTick(DapVendor& vendor) : vendor_(vendor)
{
  instance = this;
  RCC_APB1PeriphClockCmd(XRDAP_TICK_TIM_RCC, ENABLE);
  TIM_Cmd(XRDAP_TICK_TIM, DISABLE);
  XRDAP_TICK_TIM->CTLR1 =
      TIM_URS;  // 只有溢出产生中断，UG 不产生 / only overflows interrupt
  TIM_ITConfig(XRDAP_TICK_TIM, TIM_IT_Update, ENABLE);
  NVIC_EnableIRQ(XRDAP_TICK_IRQN);
}

void SamplerTick::Start(uint32_t period_us)
{
  if (period_us == 0U)
  {
    period_us = 1U;
  }
  TIM_Cmd(XRDAP_TICK_TIM, DISABLE);
  repeat_ = (period_us + kMaxCounterUs - 1U) / kMaxCounterUs;
  count_ = 0U;
  XRDAP_TICK_TIM->PSC = static_cast<uint16_t>(TickClockHz() / 1000000U - 1U);
  XRDAP_TICK_TIM->ATRLR = static_cast<uint16_t>(period_us / repeat_ - 1U);
  XRDAP_TICK_TIM->SWEVGR =
      TIM_PSCReloadMode_Immediate;  // UG 装入 PSC 和 ATRLR / UG loads them
  XRDAP_TICK_TIM->CNT = 0U;
  TIM_ClearITPendingBit(XRDAP_TICK_TIM, TIM_IT_Update);
  TIM_Cmd(XRDAP_TICK_TIM, ENABLE);
}

void SamplerTick::Stop() { TIM_Cmd(XRDAP_TICK_TIM, DISABLE); }

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

extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void XRDAP_TICK_IRQ_HANDLER(
    void)
{
  if (TIM_GetITStatus(XRDAP_TICK_TIM, TIM_IT_Update) == RESET)
  {
    return;
  }
  TIM_ClearITPendingBit(XRDAP_TICK_TIM, TIM_IT_Update);
  if (XrDap::SamplerTick::instance != nullptr)
  {
    XrDap::SamplerTick::instance->OnInterrupt();
  }
}
