// SPDX-License-Identifier: Apache-2.0
#include "sampler_task.hpp"

#include "ch32v30x.h"
#include "thread.hpp"

namespace XrDap
{
namespace
{
constexpr uint32_t kTaskStackBytes = 3072;  // one frame on the stack plus SWD calls
// TIM7 counts microseconds in a 16-bit counter.
constexpr uint32_t kMaxCounterUs = 65535U;

/// TIM6/TIM7 run from PCLK1, doubled when APB1 is divided (RM, clock tree).
uint32_t Tim7ClockHz()
{
  RCC_ClocksTypeDef clocks;
  RCC_GetClocksFreq(&clocks);
  const bool divided = (RCC->CFGR0 & RCC_PPRE1) != RCC_PPRE1_DIV1;
  return clocks.PCLK1_Frequency * (divided ? 2U : 1U);
}

// TIM7 counts at 1 MHz and raises an update interrupt per timer period. A tick period
// longer than the 16-bit counter is split into `repeat` equal timer periods and every
// repeat-th interrupt is a tick; the split rounds the period down by less than `repeat`
// microseconds (under 0.002 %).
class Tim7Tick final : public Vendor::TickTimer
{
 public:
  Tim7Tick()
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_Cmd(TIM7, DISABLE);
    TIM7->CTLR1 = TIM_URS;  // only overflows interrupt, not UG
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM7_IRQn);
  }

  void Start(uint32_t period_us) override
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
    TIM7->SWEVGR = TIM_PSCReloadMode_Immediate;  // UG loads PSC and ATRLR
    TIM7->CNT = 0U;
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_Cmd(TIM7, ENABLE);
  }

  void Stop() override { TIM_Cmd(TIM7, DISABLE); }

  /// TIM7 中断调用 / Called by the TIM7 interrupt.
  void OnInterrupt()
  {
    count_ = count_ + 1U;
    if (count_ < repeat_)
    {
      return;
    }
    count_ = 0U;
    if (vendor_ != nullptr)
    {
      vendor_->OnTickIsr();
    }
  }

  DapVendor* vendor_ = nullptr;

 private:
  volatile uint32_t repeat_ = 1U;
  volatile uint32_t count_ = 0U;
};

Tim7Tick* tick = nullptr;
}  // namespace

void StartSamplerTask(DapVendor& vendor, uint32_t priority)
{
  static Tim7Tick timer;
  static LibXR::Thread thread;
  timer.vendor_ = &vendor;
  tick = &timer;
  thread.Create<DapVendor*>(
      &vendor, [](DapVendor* v) { v->RunSamplerTask(*tick); }, "dap_sampler",
      kTaskStackBytes, static_cast<LibXR::Thread::Priority>(priority));
}
}  // namespace XrDap

extern "C" __attribute__((interrupt("WCH-Interrupt-fast"))) void TIM7_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM7, TIM_IT_Update) == RESET)
  {
    return;
  }
  TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
  if (XrDap::tick != nullptr)
  {
    XrDap::tick->OnInterrupt();
  }
}
