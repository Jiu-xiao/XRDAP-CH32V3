// SPDX-License-Identifier: Apache-2.0
#ifndef XRDAP_CHIP_H_
#define XRDAP_CHIP_H_

/*
 * What differs between the CH32 families and is not a matter of the board: the SDK
 * header, the PLL, the LSI and the tick timer. This is the CH32V20x's; the include path
 * of the board's family (chips/<family>/) selects it.
 *
 * CH32 各系列之间的差别中不属于板子的部分：SDK 头文件、PLL、LSI 和节拍定时器。这是
 * CH32V20x 的；板子所属系列的包含路径（chips/<系列>/）选中它。
 */

#include "ch32v20x.h"

/*
 * The PLL takes the 32 MHz crystal divided by 4, and multiplies by at most 18 (RM, clock
 * tree of the CH32V208; SDK SetSysClockTo144_HSE).
 * PLL 的输入是 32 MHz 晶振除以 4，最多乘 18。
 */
#define XRDAP_PLL_INPUT_HZ (HSE_VALUE / 4)
#define XRDAP_PLL_MULTIPLIER(n) RCC_PLLMULL##n

/* LSI, the clock of the independent watchdog (RM, clock tree). */
#define XRDAP_LSI_HZ 32000U

/* Sampling tick timer (app/sampler_tick.hpp): TIM4; this family has no basic timers. */
#define XRDAP_TICK_TIM TIM4
#define XRDAP_TICK_TIM_RCC RCC_APB1Periph_TIM4
#define XRDAP_TICK_IRQN TIM4_IRQn
#define XRDAP_TICK_IRQ_HANDLER TIM4_IRQHandler

#endif /* XRDAP_CHIP_H_ */
