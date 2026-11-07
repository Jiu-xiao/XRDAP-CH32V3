# WCH-LinkW-R0-1v1: CH32V208GBU6, 32 MHz crystal, full-speed USB, the chip's 2.4 GHz radio.
set(XRDAP_CHIP_FAMILY ch32v20x)
set(XRDAP_HSE_HZ 32000000)
# The rated maximum, 32 MHz / 4 x 18.
set(XRDAP_APP_SYSCLK_HZ 144000000)
# Boot chain (bootloader/): stage 1 at 0, the application window 0x1000..0x40000 (settings
# at 0x3F700, seal at 0x3FF00) and the two bootloader slots above it. The start of the
# flash runs without wait states (128, 144 or 160 KB by the user option word); the rest, up
# to 480 KB, is slower and holds the slots, which run during a DFU only.
set(XRDAP_APP_BASE 0x00001000)
set(XRDAP_APP_SIZE 0x0003F000)
set(XRDAP_BL_SLOT_A_BASE 0x00040000)
set(XRDAP_BL_SLOT_B_BASE 0x0004D000)
# The boot key is ModeS, low when held. BOOT (PB8, bonded to BOOT0) starts the chip's ISP.
set(XRDAP_BOOT_KEY_PORT A)
set(XRDAP_BOOT_KEY_PIN 4)
set(XRDAP_BOOT_MIGRATION OFF)
# The bootloader in 32 KB: its LibXR objects take about 10 KB of heap.
set(XRDAP_BL_HEAP_SIZE 0x3C00)
set(XRDAP_BL_PRODUCT "XR-DAP-V208 DFU")
# 32 KB of SRAM is there with every setting of the user option word (CH32V208DS0 p.4,
# table 2-1).
set(XRDAP_RAM_SIZE 0x8000)
set(XRDAP_MAIN_STACK_SIZE 3072)
set(XRDAP_C_HEAP_SIZE 0x3000)
set(XRDAP_DAP_PACKET_COUNT 2)
set(XRDAP_RING_BYTES 2048)
set(XRDAP_HAS_LED ON)
# The board's composition (boards/linkw.cpp) and its parts.
set(XRDAP_BOARD_APP linkw)
set(XRDAP_BOARD_SOURCES app/status_led.cpp app/settings_store.cpp)
