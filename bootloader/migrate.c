// SPDX-License-Identifier: Apache-2.0
// Writes the boot chain from the application window.
//
// migrate-to-ab: linked at 0x10000, the application address of the old single-bootloader
// layout, so the old bootloader takes it over DFU as an application and starts it.
// rewrite-boot-chain: the same program at the application address of the A/B layout, for
// a board that already runs it (repair and bring-up). It writes boot-chain.bin
// (tools/boot_chain.py: stage 1 and bootloader slot A with its record), carried in the
// image, over flash 0x0..0xDFFF: the slot first, page by page with a read-back check,
// stage 1's sector last; then it erases its own seal record, so that no bootloader starts
// it again, and restarts. Stage 1 starts the new bootloader, which stays in DFU mode
// until an application is written. The settings at the end of the flash stay. A power cut
// while it runs (about two seconds) leaves no working bootloader: the chip's own USB
// loader (BOOT0) or a WCH-Link then writes boot-chain.hex.
//
// 从应用区写启动链。
// migrate-to-ab：链接在旧的单 bootloader 布局的应用地址 0x10000，旧 bootloader 把它当应用经
// DFU 写入并启动。rewrite-boot-chain：同一个程序链接在 A/B 布局的应用地址，给已经用这套布局
// 的板子（修复和调试）。它把镜像里带的 boot-chain.bin（tools/boot_chain.py：stage1 和带槽记录
// 的引导槽 A）写到 flash 0x0..0xDFFF：先写槽，逐页回读校验，最后写 stage1 所在扇区；然后擦掉
// 自己的 seal 记录，免得引导再次启动它，再重启。stage1 启动新引导，新引导停在 DFU 等待应用。
// flash 末尾的设置保留。运行期间（约 2 s）断电会没有可用的引导，只能用芯片自带的 USB 下载
// （BOOT0）或 WCH-Link 写 boot-chain.hex。

#include <stdint.h>
#include <string.h>

#include "ch32v30x.h"

extern const uint8_t migrate_payload[];
extern const uint8_t migrate_payload_end[];

#define FLASH_ALIAS 0x08000000u
#define PAGE 256u
#define STAGE1_SECTOR 0x1000u
// The application seal record: the same flash address in both layouts (0x3FF00).
#define APP_SEAL_PAGE (XRDAP_APP_BASE + XRDAP_APP_SIZE - XRDAP_SEAL_SIZE)

static int PageMatches(uint32_t offset, const uint8_t* data)
{
  return memcmp((const void*)(FLASH_ALIAS + offset), data, PAGE) == 0;
}

// Erases and programs one 256-byte page, then reads it back; three tries.
static int WritePage(uint32_t offset, const uint8_t* data)
{
  static uint32_t buffer[PAGE / 4u];
  memcpy(buffer, data, PAGE);
  for (int attempt = 0; attempt < 3; ++attempt)
  {
    FLASH_ErasePage_Fast(FLASH_ALIAS + offset);
    FLASH_ProgramPage_Fast(FLASH_ALIAS + offset, buffer);
    if (PageMatches(offset, data))
    {
      return 1;
    }
  }
  return 0;
}

static int WriteRange(uint32_t from, uint32_t to, const uint8_t* payload)
{
  for (uint32_t offset = from; offset < to; offset += PAGE)
  {
    if (!PageMatches(offset, payload + offset) && !WritePage(offset, payload + offset))
    {
      return 0;
    }
  }
  return 1;
}

int main(void)
{
  __disable_irq();
  SystemInit();  // HSI: flash programming wants SYSCLK <= 120 MHz
  const uint32_t size = (uint32_t)(migrate_payload_end - migrate_payload);
  if (size % PAGE == 0u && size > STAGE1_SECTOR)
  {
    FLASH_Unlock_Fast();  // writes the FPEC keys and then the fast-mode keys
    const int ok = WriteRange(STAGE1_SECTOR, size, migrate_payload) &&
                   WriteRange(0u, STAGE1_SECTOR, migrate_payload);
    if (ok)
    {
      FLASH_ErasePage_Fast(FLASH_ALIAS + APP_SEAL_PAGE);
    }
    FLASH_Lock_Fast();
    FLASH_Lock();
    if (ok)
    {
      NVIC_SystemReset();
    }
  }
  for (;;)
  {
  }
}
