# CH32V305 board (CH32V305FBP6, 12 MHz crystal, the circuit of the WCH-LinkE): the board
# of the earlier application of this repository (tag legacy-v305-app). Wired only.
set(XRDAP_HSE_HZ 12000000)
# The application runs at 216 MHz (12 MHz x 18), ABOVE the rated 144 MHz, as on wdap and
# for the same reason: bit-banged SWD 1.5x faster (common/
# sysclk.h). The flash has no wait states to set (code runs from the zero-wait area),
# settings writes halve HCLK (app/config.cpp SlowBus), and the USBHS PLL runs from the
# crystal. 144000000 returns to the rated clock, the clock of the earlier application.
set(XRDAP_APP_SYSCLK_HZ 216000000)
# The board keeps its three-stage bootflow (bootloader-ch32v3): stage-1 trampoline at 0, the
# stage-3 DFU bootloader at 0x20000 (USB 1209:3070), the application window
# 0x2000..0x20000 with the seal in its last 256 bytes, and the boot request mailbox at
# the top of the 32 KB RAM with the magics "LBBX" "CV30" (custom_bootloader_protocol.h).
set(XRDAP_EXTERNAL_BOOTLOADER ON)
set(XRDAP_APP_BASE 0x00002000)
set(XRDAP_APP_SIZE 0x0001E000)
set(XRDAP_BOOT_REQUEST_MAGIC0 0x5842424C)
set(XRDAP_BOOT_REQUEST_MAGIC1 0x30335643)
# CH32V305FB: 128K zero-wait flash, 32K SRAM, no other split (CH32V307DS0 p.4, table
# 2-1-1). Smaller pools than the CH32V307 boards: 2 DAP packets, a 2 KB sampling ring.
set(XRDAP_RAM_SIZE 0x8000)
set(XRDAP_RTOS_HEAP_SIZE 19456)
set(XRDAP_APP_STACK_WORDS 1024)
set(XRDAP_MAIN_STACK_SIZE 1024)
set(XRDAP_C_HEAP_SIZE 0x100)
set(XRDAP_DAP_PACKET_COUNT 2)
set(XRDAP_RING_BYTES 2048)
# The LED on PC9 shows the DAP status (v305.hpp).
set(XRDAP_HAS_STATUS_LED ON)
