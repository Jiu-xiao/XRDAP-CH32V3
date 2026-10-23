// SPDX-License-Identifier: Apache-2.0
#include "ch32v30x.h"
#include "sysclk.h"

void bootloader_main(void);

int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  SystemInit();
  (void)XrDapSysClockInit();
  SystemCoreClockUpdate();

  // LibXR's CH32 timebase reads SysTick as a free-running 64-bit HCLK counter (enabled,
  // HCLK, no auto-reload, counting up, no interrupt).
  SysTick->CTLR = 0;
  SysTick->SR = 0;
  SysTick->CNT = 0;
  SysTick->CTLR = (1U << 0) | (1U << 2);
  __enable_irq();

  bootloader_main();
  for (;;)
  {
  }
}
