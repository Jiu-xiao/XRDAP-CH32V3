// SPDX-License-Identifier: Apache-2.0
#ifndef XRDAP_SYSCLK_H_
#define XRDAP_SYSCLK_H_

/*
 * Core clock of each image, chosen by the board files (boards/<name>.cmake) instead of
 * the WCH system file, which is left at its reset clock (HSI). The PLL multiplies the
 * board's crystal (HSE_VALUE) up to XRDAP_SYSCLK_HZ; HCLK and PCLK2 run at the core
 * clock, PCLK1 at half of it. HSE stays on: the USBHS PHY PLL uses it as reference.
 *
 * Clocks used: the bootloader 96 MHz (its flash driver needs SYSCLK <= 120 MHz); the
 * evt application 144 MHz (the CH32V307's rated maximum); the wdap and v305 applications
 * 216 MHz (12 MHz x 18), which is ABOVE the rated 144 MHz of the CH32V307 and CH32V305.
 * That is an accepted risk of these builds: bit-banged SWD runs 1.5x faster. Temperature
 * and long-run stability tests are part of the hardware regression; setting
 * XRDAP_APP_SYSCLK_HZ to 144000000 in boards/wdap.cmake or boards/v305.cmake returns to
 * the rated clock.
 *
 * 各镜像的内核时钟由板级文件（boards/<名字>.cmake）选择，WCH 的系统文件保持复位时钟
 * （HSI）。PLL 把板上晶振（HSE_VALUE）倍频到 XRDAP_SYSCLK_HZ；HCLK、PCLK2 等于内核
 * 时钟，PCLK1 为其一半。HSE 保持开启：USBHS PHY 的 PLL 以它为参考。
 *
 * 使用的时钟：bootloader 96 MHz（其 flash 驱动要求 SYSCLK <= 120 MHz）；evt 应用 144 MHz
 * （CH32V307 的标称上限）；wdap 和 v305 应用 216 MHz（12 MHz x 18），**超过** CH32V307、
 * CH32V305 标称的 144 MHz。这是这两种构建接受的风险：软件 SWD 快 1.5 倍。温度和长时间
 * 稳定性测试列在硬件回归里；把 boards/wdap.cmake 或 boards/v305.cmake 的
 * XRDAP_APP_SYSCLK_HZ 改为 144000000 即回到标称时钟。
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Switch the core clock from HSI to the PLL at XRDAP_SYSCLK_HZ / 把内核时钟从 HSI
 *        切到 XRDAP_SYSCLK_HZ 的 PLL.
 * @return 1 on success; 0 when the crystal did not start, the clock then stays on HSI /
 *         成功时为 1；晶振未起振时为 0，时钟保持 HSI。
 * @note Call once after SystemInit() and before SystemCoreClockUpdate(), with interrupts
 *       off / 在 SystemInit() 之后、SystemCoreClockUpdate() 之前调用一次，此时中断关闭。
 */
int XrDapSysClockInit(void);

#ifdef __cplusplus
}
#endif

#endif /* XRDAP_SYSCLK_H_ */
