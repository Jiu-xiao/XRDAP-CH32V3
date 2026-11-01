# CH32V305 board at the rated 144 MHz (12 MHz x 12): the stable version of `v305`.
# Everything else is the same as boards/v305.cmake.
include("${CMAKE_CURRENT_LIST_DIR}/v305.cmake")
set(XRDAP_APP_SYSCLK_HZ 144000000)
