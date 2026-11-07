// SPDX-License-Identifier: Apache-2.0
// Writes the boot chain from the application window.
//
// migrate-to-ab: linked in the application window of the old bootloader
// (XRDAP_MIGRATE_BASE: 0x10000 on the CH32V307, 0x2000 on the CH32V305), so the old
// bootloader takes it over DFU as an application and starts it. rewrite-boot-chain: the
// same program at the application address of the A/B layout, for a board that already
// runs it (repair and bring-up). It carries boot-chain-parts.bin (tools/boot_chain.py):
// stage 1's sector and the board's first bootloader slot with its record (slot A on the
// CH32V307, slot B on the CH32V305). It writes the slot first, page by page with a
// read-back check, then stage 1's sector from its last page to its first, so that the
// entry page changes last; then it erases its own seal record, so that no bootloader
// starts it again, and restarts. Stage 1 starts the new bootloader, which stays in DFU
// mode until an application is written. The settings at the end of the application window
// stay. If a page does not verify it erases its seal and restarts as well: the old
// bootloader is still in place when the slot does not overlap it (CH32V305). A power cut
// while the CH32V305's stage 1 page is written, or while the CH32V307's slot A (over the
// old bootloader) is written, leaves no working bootloader: the chip's own USB loader
// (BOOT0, CH32V307) or a WCH-Link then writes boot-chain.hex.
//
// 从应用区写启动链。
// migrate-to-ab：链接在旧 bootloader 的应用区（XRDAP_MIGRATE_BASE：CH32V307 为 0x10000，
// CH32V305 为 0x2000），旧 bootloader 把它当应用经 DFU 写入并启动。
// rewrite-boot-chain：同一个程序链接在 A/B 布局的应用地址，给已经用这套布局的板子
// （修复和调试）。它带着 boot-chain-parts.bin（tools/boot_chain.py）：stage1 所在扇区
// 和板子的首个引导槽及其槽记录（CH32V307 为槽 A，CH32V305 为槽 B）。先写槽，逐页回读
// 校验；再从后往前写 stage1 扇区，入口页最后改；然后擦掉自己的 seal 记录，免得引导
// 再次启动它，再重启。stage1 启动新引导，新引导停在 DFU 等待应用。应用区末尾的设置
// 保留。某页校验失败时同样擦掉 seal 并重启：槽不与旧 bootloader 重叠时（CH32V305）
// 旧 bootloader 仍然可用。写 CH32V305 的 stage1 页时，或写 CH32V307 的槽 A（覆盖旧
// bootloader）时断电，就没有可用的引导，只能用芯片自带的 USB 下载（BOOT0，CH32V307）
// 或 WCH-Link 写 boot-chain.hex。

#include <stdint.h>
#include <string.h>

#include "ch32v30x.h"

extern const uint8_t migrate_payload[];
extern const uint8_t migrate_payload_end[];

#define FLASH_ALIAS 0x08000000u
#define PAGE 256u
#define STAGE1_SECTOR XRDAP_STAGE1_SIZE
#define SLOT_SIZE XRDAP_BL_SLOT_SIZE
// The application seal record: the same flash address in the old and the A/B layout.
#define APP_SEAL_PAGE (XRDAP_APP_BASE + XRDAP_APP_SIZE - XRDAP_SEAL_SIZE)

static int PageMatches(uint32_t address, const uint8_t* data)
{
  return memcmp((const void*)(FLASH_ALIAS + address), data, PAGE) == 0;
}

// Erases and programs one 256-byte page, then reads it back; three tries.
static int WritePage(uint32_t address, const uint8_t* data)
{
  static uint32_t buffer[PAGE / 4u];
  memcpy(buffer, data, PAGE);
  for (int attempt = 0; attempt < 3; ++attempt)
  {
    FLASH_ErasePage_Fast(FLASH_ALIAS + address);
    FLASH_ProgramPage_Fast(FLASH_ALIAS + address, buffer);
    if (PageMatches(address, data))
    {
      return 1;
    }
  }
  return 0;
}

// Writes `size` bytes at `address`, skipping pages that already match; from the last page
// to the first when `backwards`.
static int WriteRange(uint32_t address, const uint8_t* data, uint32_t size, int backwards)
{
  for (uint32_t i = 0; i < size; i += PAGE)
  {
    const uint32_t offset = backwards ? size - PAGE - i : i;
    if (!PageMatches(address + offset, data + offset) &&
        !WritePage(address + offset, data + offset))
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
  if (size == STAGE1_SECTOR + SLOT_SIZE)
  {
    FLASH_Unlock_Fast();  // writes the FPEC keys and then the fast-mode keys
    // Stage 1 only after the whole slot verified; either way the restart reaches a
    // bootloader that does not start this program again.
    if (WriteRange(MIGRATE_SLOT_BASE, migrate_payload + STAGE1_SECTOR, SLOT_SIZE, 0))
    {
      WriteRange(0u, migrate_payload, STAGE1_SECTOR, 1);
    }
    FLASH_ErasePage_Fast(FLASH_ALIAS + APP_SEAL_PAGE);
    FLASH_Lock_Fast();
    FLASH_Lock();
    NVIC_SystemReset();
  }
  for (;;)
  {
  }
}
