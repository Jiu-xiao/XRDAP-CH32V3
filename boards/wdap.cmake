# "高速无线DAP调试器" board: CH32V307RCT6, 12 MHz crystal, SX1281 radio module.
set(XRDAP_HSE_HZ 12000000)
# The application runs at 216 MHz (12 MHz x 18), ABOVE the CH32V307's rated 144 MHz: an
# accepted risk for 1.5x faster bit-banged SWD (common/sysclk.h). 144000000 returns to
# the rated clock.
set(XRDAP_APP_SYSCLK_HZ 216000000)
set(XRDAP_HAS_RADIO ON)
# WS2812 chain on PB15 (wdap.hpp).
set(XRDAP_HAS_STATUS_LIGHTS ON)
