# CH32V305 board (CH32V305FBP6, 12 MHz crystal, the circuit of the WCH-LinkE): the board
# of the earlier application of this repository (tag legacy-v305-app). Wired only.
set(XRDAP_HSE_HZ 12000000)
# The application runs at 216 MHz (12 MHz x 18), ABOVE the rated 144 MHz, as on wdap and
# for the same reason: bit-banged SWD 1.5x faster (common/
# sysclk.h). The flash has no wait states to set (code runs from the zero-wait area),
# flash writes halve HCLK (LibXR CH32Flash), and the USBHS PLL runs from the
# crystal. At 216 MHz the chip can run too hot, and restarts and similar problems can
# occur; boards/v305-144.cmake is the same board at the rated clock.
set(XRDAP_APP_SYSCLK_HZ 216000000)
# Boot chain (bootloader/): stage 1 at 0, the application window 0x2000..0x20000 in the
# 128 KB zero-wait flash (settings at 0x1F700, seal at 0x1FF00, as with the earlier
# three-stage bootloader), and the two bootloader slots above it, at
# 0x20000 and 0x2D000. The CH32V305FB data sheet gives 224 KB of code flash, 128 KB of it
# zero-wait (CH32V307DS0 p.4, table 2-1-1); the end of slot B, with its record at 0x39F00,
# lies beyond the 224 KB. The earlier stage-3 bootloader ran at 0x20000 as well. The first
# installation and the migration write slot B, which leaves the earlier bootloader at
# 0x20000 in place until stage 1 is written. The boot key is the IAP key (PC7). The boot request mailbox keeps the magics of
# the earlier bootloader, "LBBX" "CV30", so an application of either layout reaches DFU
# with either bootloader.
set(XRDAP_APP_BASE 0x00002000)
set(XRDAP_APP_SIZE 0x0001E000)
set(XRDAP_BL_SLOT_A_BASE 0x00020000)
set(XRDAP_BL_SLOT_B_BASE 0x0002D000)
set(XRDAP_BL_FIRST_SLOT b)
set(XRDAP_BOOT_KEY_PORT C)
set(XRDAP_BOOT_KEY_PIN 7)
set(XRDAP_BOOT_REQUEST_MAGIC0 0x5842424C)
set(XRDAP_BOOT_REQUEST_MAGIC1 0x30335643)
# The application window of the earlier three-stage bootloader, where its DFU puts
# migrate-to-ab.
set(XRDAP_MIGRATE_BASE 0x00002000)
set(XRDAP_MIGRATE_SIZE 0x0001D700)
# The bootloader in 32 KB: its LibXR objects take about 10 KB of heap.
set(XRDAP_BL_HEAP_SIZE 0x3C00)
set(XRDAP_BL_PRODUCT "XR-DAP-V305 DFU")
# CH32V305FB: 128K zero-wait flash, 32K SRAM, no other split (CH32V307DS0 p.4, table
# 2-1-1). Smaller pools than the CH32V307 boards: 2 DAP packets, a 2 KB sampling ring.
set(XRDAP_RAM_SIZE 0x8000)
set(XRDAP_MAIN_STACK_SIZE 3072)
set(XRDAP_C_HEAP_SIZE 0x2000)
set(XRDAP_DAP_PACKET_COUNT 2)
set(XRDAP_RING_BYTES 2048)
set(XRDAP_HAS_LED ON)
# The board's composition (boards/v305.cpp) and its parts.
set(XRDAP_BOARD_APP v305)
set(XRDAP_BOARD_SOURCES app/status_led.cpp)
