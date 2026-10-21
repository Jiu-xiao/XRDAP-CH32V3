// SPDX-License-Identifier: Apache-2.0
#include "sysclk.h"

#include "ch32v30x.h"

#if (XRDAP_SYSCLK_HZ % HSE_VALUE) != 0
#error "XRDAP_SYSCLK_HZ must be a multiple of the crystal frequency"
#endif

// PLL multipliers of the CH32V30x D8C (RM, RCC_CFGR0 PLLMUL with EXTEN).
#if XRDAP_SYSCLK_HZ / HSE_VALUE == 8
#define XRDAP_PLL_MUL RCC_PLLMULL8_EXTEN
#elif XRDAP_SYSCLK_HZ / HSE_VALUE == 12
#define XRDAP_PLL_MUL RCC_PLLMULL12_EXTEN
#elif XRDAP_SYSCLK_HZ / HSE_VALUE == 18
#define XRDAP_PLL_MUL RCC_PLLMULL18_EXTEN
#else
#error "no PLL multiplier for this crystal and core clock"
#endif

int XrDapSysClockInit(void)
{
  RCC->CTLR |= RCC_HSEON;
  for (uint32_t i = 0; (RCC->CTLR & RCC_HSERDY) == 0U; ++i)
  {
    if (i == HSE_STARTUP_TIMEOUT)
    {
      return 0;
    }
  }

  RCC->CFGR0 = (RCC->CFGR0 & ~(RCC_HPRE | RCC_PPRE2 | RCC_PPRE1)) | RCC_HPRE_DIV1 |
               RCC_PPRE2_DIV1 | RCC_PPRE1_DIV2;
  RCC->CFGR0 &= ~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL);
  RCC->CFGR0 |= RCC_PLLSRC_HSE | RCC_PLLXTPRE_HSE | XRDAP_PLL_MUL;
  RCC->CTLR |= RCC_PLLON;
  while ((RCC->CTLR & RCC_PLLRDY) == 0U)
  {
  }
  RCC->CFGR0 = (RCC->CFGR0 & ~RCC_SW) | RCC_SW_PLL;
  while ((RCC->CFGR0 & RCC_SWS) != RCC_SWS_PLL)
  {
  }
  return 1;
}
