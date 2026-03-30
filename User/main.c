#include "app_main.h"
#include "ch32v30x.h"

static void SysTickInit(void)
{
  NVIC_SetPriority(SysTick_IRQn, 0x80);
  NVIC_EnableIRQ(SysTick_IRQn);

  SysTick->CTLR = 0;
  SysTick->SR = 0;
  SysTick->CNT = 0;
  SysTick->CMP = SystemCoreClock / 1000;
  SysTick->CTLR = 0x0F;
}

int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  SystemInit();
  SystemCoreClockUpdate();
  SysTickInit();
  __enable_irq();

  app_main();

  while (1)
  {
  }

  return 0;
}
