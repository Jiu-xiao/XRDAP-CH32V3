#ifndef CUSTOM_BOOTLOADER_PROTOCOL_H_
#define CUSTOM_BOOTLOADER_PROTOCOL_H_

#include <stdint.h>

/*
 * SRAM boot-request mailbox.
 * Kept in top RAM area so it survives NVIC_SystemReset and avoids .bss/.data.
 */
#define CH32_CUSTOM_BL_REQ_ADDR0 0x20007FF0u
#define CH32_CUSTOM_BL_REQ_ADDR1 0x20007FF4u

#define CH32_CUSTOM_BL_REQ_MAGIC0 0x5842424Cu /* "LBBX" */
#define CH32_CUSTOM_BL_REQ_MAGIC1 0x30335643u /* "CV30" */

/*
 * BL-only SRAM layout near the top of RAM.
 * Keep enough room between the runtime diag, hard-fault trace, and request mailbox so
 * post-reset evidence survives without self-corruption.
 */
#define CH32_CUSTOM_BL_RT_DIAG_BASE 0x20007F60u
#define CH32_CUSTOM_BL_RT_DIAG_MAGIC 0x424C4447u /* "GDLB" */

#endif /* CUSTOM_BOOTLOADER_PROTOCOL_H_ */
