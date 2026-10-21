// SPDX-License-Identifier: Apache-2.0
#include "sampler_task.hpp"

#include "ch32v30x.h"
#include "thread.hpp"
#include "timebase.hpp"

namespace XrDap
{
namespace
{
constexpr uint32_t kTaskStackBytes = 3072;  // one frame on the stack plus SWD calls
// TIM7 counts microseconds; its 16-bit counter limits a timed period to this.
constexpr uint32_t kMaxTimerPeriodUs = 65535U;

DapVendor* tick_vendor = nullptr;

/// TIM6/TIM7 run from PCLK1, doubled when APB1 is divided (RM, clock tree).
uint32_t Tim7ClockHz()
{
  RCC_ClocksTypeDef clocks;
  RCC_GetClocksFreq(&clocks);
  const bool divided = (RCC->CFGR0 & RCC_PPRE1) != RCC_PPRE1_DIV1;
  return clocks.PCLK1_Frequency * (divided ? 2U : 1U);
}

// TIM7 (basic timer, RM chapter 16) counts at 1 MHz and raises one update interrupt per
// period. TIM7（基本定时器，RM 第 16 章）按 1 MHz 计数，每个周期产生一次更新中断。
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

  // Periods beyond the 16-bit counter go without the timer.
  bool Start(uint64_t start_us, uint32_t period_us) override
  {
    if (period_us > kMaxTimerPeriodUs)
    {
      return false;
    }
    TIM_Cmd(TIM7, DISABLE);
    TIM7->PSC = static_cast<uint16_t>(Tim7ClockHz() / 1000000U - 1U);
    TIM7->ATRLR = static_cast<uint16_t>(period_us - 1U);
    TIM7->SWEVGR = TIM_PSCReloadMode_Immediate;  // UG loads PSC and ATRLR
    const auto now = static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds());
    TIM7->CNT = static_cast<uint16_t>((now - start_us) % period_us);
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_Cmd(TIM7, ENABLE);
    return true;
  }

  void Stop() override { TIM_Cmd(TIM7, DISABLE); }
};
}  // namespace

void StartSamplerTask(DapVendor& vendor, uint32_t priority)
{
  static Tim7Tick tick;
  static LibXR::Thread thread;
  tick_vendor = &vendor;
  thread.Create<DapVendor*>(
      &vendor, [](DapVendor* v) { v->RunSamplerTask(&tick); }, "dap_sampler",
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
  if (XrDap::tick_vendor != nullptr)
  {
    XrDap::tick_vendor->WakeSamplerFromIsr();
  }
}
