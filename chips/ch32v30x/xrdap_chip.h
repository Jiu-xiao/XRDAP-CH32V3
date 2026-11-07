// SPDX-License-Identifier: Apache-2.0
#ifndef XRDAP_CHIP_H_
#define XRDAP_CHIP_H_

/*
 * What differs between the CH32 families and is not a matter of the board: the SDK
 * header, the PLL, the LSI and the tick timer. This is the CH32V30x's; the include path
 * of the board's family (chips/<family>/) selects it.
 *
 * CH32 各系列之间的差别中不属于板子的部分：SDK 头文件、PLL、LSI 和节拍定时器。这是
 * CH32V30x 的；板子所属系列的包含路径（chips/<系列>/）选中它。
 */

#include "ch32v30x.h"

/* The PLL multiplies the crystal directly (RM, RCC_CFGR0 PLLMUL with EXTEN). */
#define XRDAP_PLL_INPUT_HZ HSE_VALUE
#define XRDAP_PLL_MULTIPLIER(n) RCC_PLLMULL##n##_EXTEN

/* LSI, the clock of the independent watchdog (RM, clock tree). */
#define XRDAP_LSI_HZ 40000U

/* Sampling tick timer (app/sampler_tick.hpp): TIM7, a basic timer. */
#define XRDAP_TICK_TIM TIM7
#define XRDAP_TICK_TIM_RCC RCC_APB1Periph_TIM7
#define XRDAP_TICK_IRQN TIM7_IRQn
#define XRDAP_TICK_IRQ_HANDLER TIM7_IRQHandler

#endif /* XRDAP_CHIP_H_ */
