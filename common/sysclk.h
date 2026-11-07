// SPDX-License-Identifier: Apache-2.0
#ifndef XRDAP_SYSCLK_H_
#define XRDAP_SYSCLK_H_

/*
 * Core clock of each image, chosen by the board files (boards/<name>/board.cmake) instead
 * of the WCH system file, which is left at its reset clock (HSI). The PLL multiplies the
 * board's crystal (XRDAP_PLL_INPUT_HZ of xrdap_chip.h) up to XRDAP_SYSCLK_HZ; HCLK and
 * PCLK2 run at the core clock, PCLK1 at half of it. HSE stays on: the USBHS PHY PLL and
 * the CH32V208's radio use it as reference.
 *
 * Clocks used: the bootloader 96 MHz; the wdap-144, v305-144 and linkw applications
 * 144 MHz (the rated maximum of the CH32V307, CH32V305 and CH32V208); the wdap and v305
 * applications 216 MHz (12 MHz x 18), which is ABOVE the rated clock. That is an accepted
 * risk of these builds: bit-banged SWD runs 1.5x faster, and the chip can run too hot,
 * with restarts and similar problems.
 *
 * 各镜像的内核时钟由板级文件（boards/<名字>/board.cmake）选择，WCH 的系统文件保持复位时钟
 * （HSI）。PLL 把板上晶振（xrdap_chip.h 的 XRDAP_PLL_INPUT_HZ）倍频到 XRDAP_SYSCLK_HZ；
 * HCLK、PCLK2 等于内核时钟，PCLK1 为其一半。HSE 保持开启：USBHS PHY 的 PLL 和 CH32V208
 * 的射频以它为参考。
 *
 * 使用的时钟：bootloader 96 MHz；wdap-144、v305-144、linkw 应用 144 MHz（CH32V307、
 * CH32V305、CH32V208 的标称上限）；wdap 和 v305 应用 216 MHz（12 MHz x 18），**超过**标称
 * 时钟。
 * 这是这两种构建接受的风险：软件 SWD 快 1.5 倍，芯片可能温度过高，出现重启等问题。
 */

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Switch the core clock from HSI to the PLL at XRDAP_SYSCLK_HZ and set
   *        SystemCoreClock / 把内核时钟从 HSI 切到 XRDAP_SYSCLK_HZ 的 PLL，并设置
   *        SystemCoreClock.
   * @return 1 on success; 0 when the crystal did not start, the clock then stays on HSI /
   *         成功时为 1；晶振未起振时为 0，时钟保持 HSI。
   * @note Call once after SystemInit(), with interrupts off / 在 SystemInit() 之后调用
   *       一次，此时中断关闭。
   */
  int XrDapSysClockInit(void);

#ifdef __cplusplus
}
#endif

#endif /* XRDAP_SYSCLK_H_ */
