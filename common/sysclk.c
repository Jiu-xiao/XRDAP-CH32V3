// SPDX-License-Identifier: Apache-2.0
#include "sysclk.h"

#include "xrdap_chip.h"

#if (XRDAP_SYSCLK_HZ % XRDAP_PLL_INPUT_HZ) != 0
#error "XRDAP_SYSCLK_HZ must be a multiple of the PLL input"
#endif

#if XRDAP_SYSCLK_HZ / XRDAP_PLL_INPUT_HZ == 8
#define XRDAP_PLL_MUL XRDAP_PLL_MULTIPLIER(8)
#elif XRDAP_SYSCLK_HZ / XRDAP_PLL_INPUT_HZ == 12
#define XRDAP_PLL_MUL XRDAP_PLL_MULTIPLIER(12)
#elif XRDAP_SYSCLK_HZ / XRDAP_PLL_INPUT_HZ == 18
#define XRDAP_PLL_MUL XRDAP_PLL_MULTIPLIER(18)
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
  // 216 MHz 下，切换后紧接着的一小段指令可能出错：写内存丢失或写错，读 RCC 读错。
  // 空转这一段，之后才读写。 / At 216 MHz the instructions right after the switch can go
  // wrong: a store lost or wrong, a read of RCC wrong. Idle through them before any read
  // or write.
  __asm__ volatile(".option push\n.option norvc\n.rept 32\nnop\n.endr\n.option pop\n");
  SystemCoreClock = XRDAP_SYSCLK_HZ;
  return 1;
}
