// SPDX-License-Identifier: Apache-2.0
#include "sysclk.h"
#include "xrdap_chip.h"

void bootloader_main(void);
void __libc_init_array(void);

int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  SystemInit();
  (void)XrDapSysClockInit();

  // LibXR's CH32 timebase reads SysTick as a free-running 64-bit HCLK counter (enabled,
  // HCLK, no auto-reload, counting up, no interrupt).
  SysTick->CTLR = 0;
  SysTick->SR = 0;
  SysTick->CNT = 0;
  SysTick->CTLR = (1U << 0) | (1U << 2);
  __enable_irq();

  // C++ constructors of namespace-scope objects run once the clocks are set.
  __libc_init_array();
  bootloader_main();
  for (;;)
  {
  }
}
