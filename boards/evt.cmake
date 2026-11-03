# CH32V307 evaluation board (CH32V307VCT6, the CH32V307_LibXR_Template board).
set(XRDAP_HSE_HZ 8000000)
# Boot key (boards/*.hpp kBootKey), read by stage 1 as well: PB3, low when held.
set(XRDAP_BOOT_KEY_PORT B)
set(XRDAP_BOOT_KEY_PIN 3)
# LED on PB4, blinking in the bootloader (evt.hpp).
set(XRDAP_HAS_LED ON)
set(XRDAP_C_HEAP_SIZE 0x4000)
# The board's composition (boards/evt.cpp); no parts beyond the common probe.
set(XRDAP_BOARD_APP evt)
