# "高速无线DAP调试器" board: CH32V307RCT6, 12 MHz crystal, SX1281 radio module.
set(XRDAP_HSE_HZ 12000000)
# The application runs at 216 MHz (12 MHz x18), above the CH32V307's 144 MHz rating:
# bit-banged SWD is 1.5x faster; verified by a 20-minute soak.
set(XRDAP_APP_SYSCLK_HZ 216000000)
set(XRDAP_HAS_RADIO ON)
