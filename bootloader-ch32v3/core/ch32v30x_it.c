/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/03/05
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v30x_it.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

extern void libxr_systick_handler(void) __attribute__((weak));

/* Preserve last hard-fault context in top SRAM for post-reset inspection. */
#define CH32_HARDFAULT_TRACE_MAGIC 0x48465231u /* "HFR1" */
#define CH32_HARDFAULT_TRACE_BASE ((volatile uint32_t*)0x20007FC0u)
#if defined(CH32_APP_USE_CUSTOM_BOOTLOADER) && CH32_APP_USE_CUSTOM_BOOTLOADER
/* Request custom BL after reset so USB-only recovery remains possible. */
#define CH32_CUSTOM_BL_REQ_ADDR0 ((volatile uint32_t*)0x20007FF0u)
#define CH32_CUSTOM_BL_REQ_ADDR1 ((volatile uint32_t*)0x20007FF4u)
#define CH32_CUSTOM_BL_REQ_MAGIC0 0x5842424Cu /* "LBBX" */
#define CH32_CUSTOM_BL_REQ_MAGIC1 0x30335643u /* "CV30" */
#endif

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  volatile uint32_t* trace = CH32_HARDFAULT_TRACE_BASE;
  uint32_t mcause = 0;
  uint32_t mepc = 0;
  uint32_t mtval = 0;
  uint32_t mstatus = 0;
  uint32_t ra = 0;
  uint32_t sp = 0;
  uint32_t stk0 = 0;
  uint32_t stk1 = 0;
  uint32_t stk2 = 0;
  uint32_t stk3 = 0;

  __asm volatile("csrr %0, mcause" : "=r"(mcause));
  __asm volatile("csrr %0, mepc" : "=r"(mepc));
  __asm volatile("csrr %0, mtval" : "=r"(mtval));
  __asm volatile("csrr %0, mstatus" : "=r"(mstatus));
  __asm volatile("mv %0, ra" : "=r"(ra));
  __asm volatile("mv %0, sp" : "=r"(sp));

  if ((sp >= 0x20000000u) && (sp <= 0x20007FF0u - 16u))
  {
    volatile uint32_t* stk = (volatile uint32_t*)sp;
    stk0 = stk[0];
    stk1 = stk[1];
    stk2 = stk[2];
    stk3 = stk[3];
  }

  trace[0] = CH32_HARDFAULT_TRACE_MAGIC;
  trace[1] = mcause;
  trace[2] = mepc;
  trace[3] = mtval;
  trace[4] = mstatus;
  trace[5] = ra;
  trace[6] = sp;
  trace[7] = stk0;
  trace[8] = stk1;
  trace[9] = stk2;
  trace[10] = stk3;
#if defined(CH32_APP_USE_CUSTOM_BOOTLOADER) && CH32_APP_USE_CUSTOM_BOOTLOADER
  *CH32_CUSTOM_BL_REQ_ADDR0 = CH32_CUSTOM_BL_REQ_MAGIC0;
  *CH32_CUSTOM_BL_REQ_ADDR1 = CH32_CUSTOM_BL_REQ_MAGIC1;
#endif
  __asm volatile("fence iorw, iorw" ::: "memory");

  NVIC_SystemReset();
  while (1)
  {
  }
}

void SysTick_Handler(void)
{
  if (libxr_systick_handler != 0)
  {
    libxr_systick_handler();
  }
  SysTick->SR = 0;
}
