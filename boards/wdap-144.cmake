# "高速无线DAP调试器" board at the CH32V307's rated 144 MHz (12 MHz x 12): the stable
# version of `wdap`. Everything else is the same as boards/wdap.cmake.
include("${CMAKE_CURRENT_LIST_DIR}/wdap.cmake")
set(XRDAP_APP_SYSCLK_HZ 144000000)
